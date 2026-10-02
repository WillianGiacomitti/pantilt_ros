#!/usr/bin/env python3
"""
Detecção YOLO (Ultralytics), filtro de classe e seleção do alvo
(docs/architecture.md, seção 4.2).

Tópicos:
  assina   /camera/image_raw        (sensor_msgs/Image)              - BEST_EFFORT, fila de 1
  publica  /perception/detections   (vision_msgs/Detection2DArray)   - a cada quadro processado
  publica  /perception/target       (pantilt_interfaces/VisualTarget) - só com filtro ativo
  publica  /perception/debug_image  (sensor_msgs/Image)              - só com assinante

Service:
  /perception/set_target  (pantilt_interfaces/SetTarget) - define o filtro; "" remove

Comportamento:
  - sem filtro, /perception/detections traz todas as classes do modelo e nada
    sai em /perception/target;
  - com filtro, só a classe escolhida; a cada quadro, a bbox de maior
    confiança x área normalizada vai para /perception/target (detected=false
    quando não há candidato);
  - classes identificadas por NOME (model.names), nunca por ID: o
    equipment.yaml é cruzado com o modelo ao iniciar, e um equipamento ausente
    do modelo gera aviso e fica indisponível para o filtro;
  - processa sempre o quadro mais recente: os que chegam durante uma
    inferência são descartados, para não acumular atraso na malha IBVS;
  - todas as saídas usam o header da imagem (instante da captura);
  - não baixa pesos: se model_path não existir, encerra com o comando de download.

Dependência fora do package.xml: ultralytics (pip, instalado pelo Dockerfile do
pantilt_dockerfile; sem chave no rosdep).

Parâmetros: ver declare_parameter abaixo e pantilt_bringup/config/params.yaml.
"""

import os
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from pantilt_interfaces.msg import VisualTarget
from pantilt_interfaces.srv import SetTarget
from pantilt_perception.detection import (
    Detection, load_equipment, match_equipment, select_target, target_error, validate_params,
)

# Comando para baixar os pesos de teste (o nó nunca baixa sozinho)
DOWNLOAD_HINT = ('mkdir -p /ros2_ws/models && cd /ros2_ws/models && '
                 'python3 -c "from ultralytics import YOLO; YOLO(\'yolo11n.pt\')"')

# Período do resumo de desempenho no log (taxa, inferência e atraso)
STATS_PERIOD_S = 10.0

# Intervalo mínimo entre avisos repetidos
LOG_THROTTLE_S = 5.0

# Tamanho da imagem de aquecimento do modelo (a primeira inferência é lenta)
WARMUP_SHAPE = (480, 640, 3)

# Cores da imagem de debug (BGR), as mesmas da página
COLOR_BOX = (255, 184, 92)       # detecções: azul
COLOR_TARGET = (61, 138, 255)    # alvo: laranja
COLOR_CENTER = (220, 220, 220)   # centro da imagem
COLOR_TEXT_BG = (26, 22, 20)


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        self.declare_parameter('model_path', '/ros2_ws/models/yolo11n.pt')
        self.declare_parameter(
            'equipment_file',
            '/ros2_ws/src/pantilt_ros/pantilt_bringup/config/equipment_coco_test.yaml')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_debug_image', True)

        self.model_path = self.get_parameter('model_path').value
        self.equipment_file = self.get_parameter('equipment_file').value
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.device = str(self.get_parameter('device').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        try:
            validate_params(self.conf_threshold, self.imgsz)
            self.equipment = load_equipment(self.equipment_file)
            self.model = self._load_model()
        except ValueError as e:
            self.get_logger().fatal(f'Parâmetro inválido: {e}')
            raise

        self.available, missing = match_equipment(self.equipment, self.model.names)
        for key in missing:
            self.get_logger().warn(
                f'Equipamento "{key}" ignorado: a classe "{self.equipment[key].class_name}" '
                'não existe no modelo')
        if self.available:
            self.get_logger().info('Equipamentos disponíveis: ' + ', '.join(
                f'{k} ({e.class_name})' for k, e in self.available.items()))
        else:
            self.get_logger().warn(
                f'Nenhum equipamento de {self.equipment_file} existe no modelo; '
                'o filtro fica indisponível')

        self.filter = None          # Equipment do filtro ativo, ou None
        self.bridge = CvBridge()

        # Fila de 1: sempre o quadro mais recente; os que chegam durante a inferência se perdem
        image_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                               history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, '/camera/image_raw', self.on_image, image_qos)
        self.detections_pub = self.create_publisher(
            Detection2DArray, '/perception/detections', 10)
        self.target_pub = self.create_publisher(VisualTarget, '/perception/target', 1)
        self.debug_pub = None
        if self.publish_debug:
            self.debug_pub = self.create_publisher(
                Image, '/perception/debug_image', qos_profile_sensor_data)
        self.create_service(SetTarget, '/perception/set_target', self.on_set_target)

        self._reset_stats()
        self.no_frames_warned = False
        self.create_timer(STATS_PERIOD_S, self.on_stats_timer)

        self.get_logger().info(
            f'Pronto: imgsz {self.imgsz} | conf {self.conf_threshold:.2f} | '
            f'device "{self.device}" | debug_image {"sim" if self.publish_debug else "não"} | '
            'sem filtro')

    # ---------------- Modelo ----------------
    def _load_model(self):
        """Carrega os pesos e faz o aquecimento. Levanta ValueError com o motivo."""
        if not os.path.isfile(self.model_path):
            raise ValueError(
                f'model_path "{self.model_path}" não encontrado. O nó não baixa pesos; '
                f'para o modelo de teste COCO, rode uma vez: {DOWNLOAD_HINT}')

        # Importado aqui: o ultralytics demora a carregar, e os erros de
        # parâmetro acima aparecem na hora
        from ultralytics import YOLO

        t0 = time.monotonic()
        try:
            model = YOLO(self.model_path)
            model.predict(np.zeros(WARMUP_SHAPE, dtype=np.uint8), imgsz=self.imgsz,
                          conf=self.conf_threshold, device=self.device, verbose=False)
        except Exception as e:  # pesos corrompidos, device inexistente etc.
            raise ValueError(f'falha ao carregar {self.model_path} em "{self.device}": {e}') from e

        self.get_logger().info(
            f'Modelo {self.model_path} carregado e aquecido em {time.monotonic() - t0:.1f} s '
            f'({len(model.names)} classes)')
        return model

    def _infer(self, frame):
        """Roda a YOLO no quadro e devolve a lista de Detection (coordenadas em px)."""
        result = self.model.predict(frame, imgsz=self.imgsz, conf=self.conf_threshold,
                                    device=self.device, verbose=False)[0]
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            return []
        xyxy = boxes.xyxy.cpu().numpy()
        conf = boxes.conf.cpu().numpy()
        cls = boxes.cls.cpu().numpy().astype(int)
        return [Detection(result.names[c], float(p), *(float(v) for v in box))
                for box, p, c in zip(xyxy, conf, cls)]

    # ---------------- Quadros ----------------
    def on_image(self, msg: Image):
        self.no_frames_warned = False
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(f'Imagem ignorada: {e}', throttle_duration_sec=LOG_THROTTLE_S)
            return
        h, w = frame.shape[:2]

        t0 = time.perf_counter()
        try:
            detections = self._infer(frame)
        except Exception as e:
            self.get_logger().error(f'Falha na inferência: {e}',
                                    throttle_duration_sec=LOG_THROTTLE_S)
            return
        infer_ms = (time.perf_counter() - t0) * 1000.0

        eq = self.filter
        if eq is not None:
            detections = [d for d in detections if d.class_name == eq.class_name]
        self.detections_pub.publish(self._detections_msg(msg.header, detections))

        target = None
        if eq is not None:
            target = select_target(detections, eq.class_name, w, h)
            self.target_pub.publish(self._target_msg(msg.header, eq.key, target, w, h))

        if self.debug_pub is not None and self.debug_pub.get_subscription_count() > 0:
            self._publish_debug(frame, msg.header, detections, target, infer_ms)

        latency = self.get_clock().now() - Time.from_msg(msg.header.stamp)
        self.stats_frames += 1
        self.stats_infer_ms += infer_ms
        self.stats_latency_s += latency.nanoseconds / 1e9

    # ---------------- Mensagens ----------------
    @staticmethod
    def _detections_msg(header, detections) -> Detection2DArray:
        out = Detection2DArray()
        out.header = header
        for d in detections:
            item = Detection2D()
            item.header = header
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.class_name
            hyp.hypothesis.score = d.confidence
            item.results.append(hyp)
            cx, cy = d.center
            bw, bh = d.size
            item.bbox.center.position.x = cx
            item.bbox.center.position.y = cy
            item.bbox.size_x = bw
            item.bbox.size_y = bh
            out.detections.append(item)
        return out

    @staticmethod
    def _target_msg(header, key, target, w, h) -> VisualTarget:
        msg = VisualTarget()
        msg.header = header
        msg.equipment = key
        msg.image_width = w
        msg.image_height = h
        if target is not None:
            msg.detected = True
            msg.confidence = target.confidence
            msg.center_x, msg.center_y = target.center
            msg.width, msg.height = target.size
            msg.error_x, msg.error_y = target_error(target, w, h)
        return msg

    # ---------------- Imagem de debug ----------------
    def _publish_debug(self, frame, header, detections, target, infer_ms):
        """Desenha as detecções, o alvo e o centro da imagem sobre o quadro e publica."""
        h, w = frame.shape[:2]
        center = (w // 2, h // 2)
        cv2.drawMarker(frame, center, COLOR_CENTER, cv2.MARKER_CROSS, 24, 1)

        for d in detections:
            if d is target:
                continue
            p1, p2 = (int(d.x1), int(d.y1)), (int(d.x2), int(d.y2))
            cv2.rectangle(frame, p1, p2, COLOR_BOX, 2)
            _put_label(frame, f'{d.class_name} {d.confidence:.2f}', p1, COLOR_BOX)

        if target is not None:
            p1, p2 = (int(target.x1), int(target.y1)), (int(target.x2), int(target.y2))
            tc = tuple(int(v) for v in target.center)
            cv2.rectangle(frame, p1, p2, COLOR_TARGET, 3)
            cv2.line(frame, center, tc, COLOR_TARGET, 2)
            cv2.circle(frame, tc, 5, COLOR_TARGET, -1)
            ex, ey = target_error(target, w, h)
            _put_label(frame, f'ALVO {target.class_name} {target.confidence:.2f} '
                       f'erro ({ex:+.0f}, {ey:+.0f}) px', p1, COLOR_TARGET)

        state = f'Filtro: {self.filter.label}' if self.filter else 'Sem filtro'
        _put_label(frame, f'{state} | {len(detections)} det. | {infer_ms:.0f} ms', (0, 0),
                   COLOR_CENTER, below=True)

        out = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out.header = header
        self.debug_pub.publish(out)

    # ---------------- Filtro ----------------
    def on_set_target(self, request, response):
        key = request.equipment.strip()
        if not key:
            self.filter = None
            response.success = True
            response.message = 'Filtro removido: publicando todas as classes'
            self.get_logger().info(response.message)
            return response

        eq = self.available.get(key)
        if eq is None:
            if key in self.equipment:
                reason = (f'"{key}" indisponível: a classe '
                          f'"{self.equipment[key].class_name}" não existe no modelo')
            else:
                reason = f'equipamento "{key}" desconhecido'
            response.success = False
            response.message = (f'{reason}. Disponíveis: '
                                f'{", ".join(self.available) or "nenhum"}')
            self.get_logger().warn(f'Filtro recusado: {response.message}')
            return response

        self.filter = eq
        response.success = True
        response.message = f'Filtro: {eq.label} ({eq.class_name})'
        self.get_logger().info(response.message)
        return response

    # ---------------- Desempenho ----------------
    def _reset_stats(self):
        self.stats_frames = 0
        self.stats_infer_ms = 0.0
        self.stats_latency_s = 0.0

    def on_stats_timer(self):
        n = self.stats_frames
        if n == 0:
            if not self.no_frames_warned:
                self.no_frames_warned = True
                self.get_logger().warn(
                    f'Nenhum quadro de /camera/image_raw em {STATS_PERIOD_S:.0f} s '
                    '(a câmera está rodando?)')
            return
        self.get_logger().info(
            f'{n / STATS_PERIOD_S:.1f} quadros/s processados | inferência média '
            f'{self.stats_infer_ms / n:.0f} ms | atraso médio desde a captura '
            f'{self.stats_latency_s / n * 1000.0:.0f} ms')
        self._reset_stats()


def _put_label(img, text, origin, color, below=False):
    """Texto com fundo escuro; acima de origin (ou abaixo, com below=True)."""
    font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
    (tw, th), base = cv2.getTextSize(text, font, scale, thick)
    x = max(0, min(origin[0], img.shape[1] - tw - 4))
    y = origin[1] + th + base + 4 if below else origin[1] - 4
    y = max(th + base + 4, y)
    cv2.rectangle(img, (x, y - th - base - 4), (x + tw + 4, y), COLOR_TEXT_BG, -1)
    cv2.putText(img, text, (x + 2, y - base - 2), font, scale, color, thick, cv2.LINE_AA)


def main():
    rclpy.init()
    try:
        node = DetectorNode()
    except ValueError:
        # Parâmetro inválido ou modelo ausente: o motivo já foi registrado no log
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
