#!/usr/bin/env python3
"""
Gravação de vídeos para o dataset de treino da YOLO (docs/architecture.md,
seção 4.9). Grava /camera/image_raw em MP4 e, opcionalmente, mantém o
pan-tilt varrendo com o scan_node enquanto grava.

Services:
  /capture/start  (pantilt_interfaces/StartCapture)  - inicia uma gravação
  /capture/stop   (std_srvs/Trigger)                 - encerra e finaliza o arquivo

Tópicos:
  assina   /camera/image_raw  (sensor_msgs/Image)              - só durante a gravação
  publica  /capture/status    (pantilt_interfaces/CaptureStatus) - 2 Hz e a cada mudança (transient_local)

Action (cliente):
  /control/scan  (pantilt_interfaces/Scan)

Comportamento:
  - um MP4 por sessão em output_dir, com o nome AAAAMMDD_HHMMSS_<sessao>.mp4,
    e um .json de metadados ao lado; uma gravação sem quadros não deixa arquivo;
  - com scan=true, envia goals Scan em sequência enquanto grava. Se a varredura
    for interrompida (goal recusado, abortado ou cancelado por outro cliente, ou
    o scan_node sair do ar), a gravação continua sem varrer. Com o scan_node fora do ar, o início é recusado;
  - recusa o início, ou encerra a gravação com o arquivo finalizado, se o espaço
    livre em output_dir ficar abaixo de min_free_gb;
  - a escrita roda numa thread com fila (video_recorder.py); quadros
    descartados por fila cheia são contados em CaptureStatus.dropped.

Limitação: o MP4 declara record_fps. Se a câmera entregar menos (ex.: pouca
luz), o vídeo toca acelerado; o fps_real do .json registra a taxa real.

Parâmetros: ver declare_parameter abaixo e pantilt_bringup/config/params.yaml.
"""

import json
import math
import os
import time
from dataclasses import dataclass
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge, CvBridgeError
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from pantilt_dataset.capture_session import (
    build_metadata, clean_session, free_gb, unique_path, validate_params, video_basename,
)
from pantilt_dataset.video_recorder import VideoRecorder
from pantilt_interfaces.action import Scan
from pantilt_interfaces.msg import CaptureStatus
from pantilt_interfaces.srv import StartCapture
from pantilt_perception.camera_source import FrameThrottle

# Período de publicação do /capture/status
STATUS_PERIOD_S = 0.5

# Intervalo entre verificações do espaço em disco durante a gravação
DISK_CHECK_PERIOD_S = 5.0

# Sem quadros da câmera por mais que isso -> aviso no status
NO_FRAMES_WARN_S = 2.0

# Velocidade máxima aceita no goal Scan (a mesma do scan_node)
MAX_SPEED_DEG_S = 30.0

# Timeout de cada volta do executor; limita a demora em atender o Ctrl+C
SPIN_TIMEOUT_S = 0.1

# Intervalo mínimo entre avisos repetidos
LOG_THROTTLE_S = 5.0

REASON_OPERATOR = 'Parada pelo operador'
REASON_SHUTDOWN = 'Nó encerrado'

_STATUS_NAMES = {
    GoalStatus.STATUS_ABORTED: 'abortada',
    GoalStatus.STATUS_CANCELED: 'cancelada',
}


@dataclass
class _Session:
    """Estado de uma gravação em andamento."""
    session: str
    path: str
    recorder: VideoRecorder
    throttle: FrameThrottle
    started_at: datetime
    start_mono: float
    scan: bool
    speed: float
    subscription: object = None
    last_frame_mono: float = None
    last_disk_check: float = 0.0
    no_frames_warned: bool = False
    scanning: bool = False
    scan_stop_reason: str = ''
    goal_handle: object = None
    goals_sent: int = 0
    goals_succeeded: int = 0


class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_node')

        self.declare_parameter('output_dir', '/ros2_ws/datasets')
        self.declare_parameter('record_fps', 15.0)
        self.declare_parameter('fourcc', 'mp4v')
        self.declare_parameter('min_free_gb', 1.0)
        self.declare_parameter('queue_size', 30)

        self.output_dir = self.get_parameter('output_dir').value
        self.record_fps = self.get_parameter('record_fps').value
        self.fourcc = self.get_parameter('fourcc').value
        self.min_free_gb = self.get_parameter('min_free_gb').value
        self.queue_size = self.get_parameter('queue_size').value

        try:
            validate_params(self.record_fps, self.fourcc, self.min_free_gb, self.queue_size)
        except ValueError as e:
            self.get_logger().fatal(f'Parâmetro inválido: {e}')
            raise

        self.get_logger().info(
            f'Saída: {self.output_dir} | {self.record_fps:.1f} fps | fourcc "{self.fourcc}" | '
            f'mínimo {self.min_free_gb:.1f} GB livres | fila {self.queue_size}'
        )

        self.bridge = CvBridge()

        # transient_local: a página recebe o último estado ao conectar
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = QoSReliabilityPolicy.RELIABLE
        status_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(CaptureStatus, '/capture/status', status_qos)

        self.create_service(StartCapture, '/capture/start', self.on_start)
        self.create_service(Trigger, '/capture/stop', self.on_stop)
        self.scan_client = ActionClient(self, Scan, '/control/scan')
        self.create_timer(STATUS_PERIOD_S, self.on_timer)

        # Todos os callbacks rodam na mesma thread (executor de uma thread): sem locks
        self._session = None
        self._message = 'Pronto'
        self._last_file = ''
        self._last_frames = 0
        self._last_dropped = 0
        self._publish_status()

    # ---------------- Services ----------------
    def on_start(self, request, response):
        if self._session is not None:
            return self._refuse(response, f'Já existe uma gravação ativa ({self._session.path})')

        speed = request.speed_deg_s
        if not (math.isfinite(speed) and 0.0 <= speed <= MAX_SPEED_DEG_S):
            return self._refuse(
                response, f'speed_deg_s={speed:.1f} fora de [0, {MAX_SPEED_DEG_S:.0f}]°/s')
        if request.scan and not self.scan_client.server_is_ready():
            return self._refuse(
                response, 'scan_node indisponível: suba o control.launch.py ou use scan=false')

        try:
            os.makedirs(self.output_dir, exist_ok=True)
        except OSError as e:
            return self._refuse(response, f'Não foi possível criar {self.output_dir}: {e}')
        free = free_gb(self.output_dir)
        if free < self.min_free_gb:
            return self._refuse(
                response, f'Pouco espaço em disco: {free:.1f} GB livres '
                f'(mínimo {self.min_free_gb:.1f} GB)')

        session = clean_session(request.session)
        started_at = datetime.now()
        path = unique_path(self.output_dir, video_basename(started_at, session), '.mp4')
        s = _Session(
            session=session,
            path=path,
            recorder=VideoRecorder(path, self.record_fps, self.fourcc, self.queue_size),
            throttle=FrameThrottle(self.record_fps),
            started_at=started_at,
            start_mono=time.monotonic(),
            scan=request.scan,
            speed=speed,
        )
        s.last_disk_check = s.start_mono
        s.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self._session = s

        modo = 'com varredura' if s.scan else 'sem varredura'
        self.get_logger().info(
            f'Gravação iniciada {modo}: {path} ({free:.1f} GB livres)')
        if s.scan:
            self._send_scan_goal(s)

        response.accepted = True
        response.message = f'Gravando {modo} em {path}'
        response.file = path
        self._publish_status()
        return response

    def _refuse(self, response, text):
        self.get_logger().warn(f'Início recusado: {text}')
        response.accepted = False
        response.message = text
        response.file = ''
        return response

    def on_stop(self, request, response):
        if self._session is None:
            response.success = False
            response.message = 'Nenhuma gravação ativa'
            return response
        response.success = True
        response.message = self._stop_recording(REASON_OPERATOR)
        return response

    # ---------------- Quadros ----------------
    def on_image(self, msg: Image):
        s = self._session
        if s is None:
            return
        now = time.monotonic()
        s.last_frame_mono = now
        if not s.throttle.ready(now):
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().warn(
                f'Quadro ignorado: conversão falhou ({e})', throttle_duration_sec=LOG_THROTTLE_S)
            return
        if not s.recorder.submit(frame):
            self.get_logger().warn(
                'Fila do escritor cheia: quadros descartados', throttle_duration_sec=LOG_THROTTLE_S)

    # ---------------- Varredura ----------------
    def _send_scan_goal(self, s: _Session):
        # speed_deg_s = 0 e timeout_s = 0 usam os parâmetros do scan_node
        goal = Scan.Goal(speed_deg_s=s.speed, timeout_s=0.0)
        s.goals_sent += 1
        s.scanning = True
        future = self.scan_client.send_goal_async(goal)
        future.add_done_callback(lambda f, s=s: self._on_goal_response(s, f))

    def _on_goal_response(self, s: _Session, future):
        handle = future.result()
        if s is not self._session or not s.scanning:
            # A gravação acabou (ou a varredura foi dada como interrompida) antes
            # da resposta: não deixa o goal varrendo
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._scan_interrupted(s, 'goal recusado pelo scan_node')
            return
        s.goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda f, s=s, h=handle: self._on_scan_result(s, h, f))

    def _on_scan_result(self, s: _Session, handle, future):
        if s is not self._session or handle is not s.goal_handle:
            return
        s.goal_handle = None
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            # Padrão completo ou tempo esgotado: segue com a próxima passada
            s.goals_succeeded += 1
            self._send_scan_goal(s)
            return
        status = _STATUS_NAMES.get(result.status, f'status {result.status}')
        motivo = result.result.message or 'sem mensagem'
        self._scan_interrupted(s, f'{status} ({motivo})')

    def _scan_interrupted(self, s: _Session, reason: str):
        s.scanning = False
        s.scan_stop_reason = reason
        self.get_logger().warn(f'Varredura interrompida: {reason}; a gravação continua')
        self._publish_status()

    # ---------------- Encerramento da gravação ----------------
    def _stop_recording(self, reason: str) -> str:
        """Finaliza o vídeo e o .json. Retorna o texto do resultado."""
        s = self._session
        # Antes de tudo: callbacks atrasados (quadros, goals) passam a ser ignorados
        self._session = None
        self.destroy_subscription(s.subscription)
        if s.goal_handle is not None:
            s.goal_handle.cancel_goal_async()
        s.recorder.close()
        ended_at = datetime.now()

        rec = s.recorder
        if rec.frames == 0:
            text = 'Nenhum quadro recebido; nada gravado'
            if rec.error:
                text += f' ({rec.error})'
            self._last_file = ''
        else:
            width, height = rec.size
            meta = build_metadata(
                s.session, s.path, s.started_at, ended_at, rec.frames, rec.dropped,
                self.record_fps, width, height, self.fourcc, rec.size_bytes,
                {'enabled': s.scan, 'speed_deg_s': s.speed, 'goals_sent': s.goals_sent,
                 'goals_succeeded': s.goals_succeeded},
                reason)
            json_path = os.path.splitext(s.path)[0] + '.json'
            try:
                with open(json_path, 'w', encoding='utf-8') as f:
                    json.dump(meta, f, ensure_ascii=False, indent=2)
            except OSError as e:
                self.get_logger().error(f'Falha ao gravar {json_path}: {e}')
            text = (f'Gravado: {rec.frames} quadros em {meta["duration_s"]:.0f} s '
                    f'({meta["fps_real"]:.1f} fps, {rec.size_bytes / 1e6:.1f} MB) -> {s.path}')
            if rec.dropped or rec.size_mismatch:
                text += (f' | descartados: {rec.dropped} por fila cheia, '
                         f'{rec.size_mismatch} por tamanho')
            self._last_file = s.path

        self._last_frames = rec.frames
        self._last_dropped = rec.dropped
        self._message = f'{reason}. {text}'
        self.get_logger().info(self._message)
        self._publish_status()
        return text

    # ---------------- Status ----------------
    def on_timer(self):
        s = self._session
        if s is not None:
            now = time.monotonic()
            if s.recorder.error:
                self._stop_recording(f'Erro na gravação: {s.recorder.error}')
                return
            if now - s.last_disk_check >= DISK_CHECK_PERIOD_S:
                s.last_disk_check = now
                free = free_gb(self.output_dir)
                if free < self.min_free_gb:
                    self._stop_recording(
                        f'Pouco espaço em disco: {free:.1f} GB livres '
                        f'(mínimo {self.min_free_gb:.1f} GB)')
                    return
            if s.scanning and not self.scan_client.server_is_ready():
                # O scan_node saiu do ar (ex.: Ctrl+C) sem entregar o resultado do goal
                s.goal_handle = None
                self._scan_interrupted(s, 'scan_node saiu do ar')
            silent = now - (s.last_frame_mono or s.start_mono)
            if silent > NO_FRAMES_WARN_S and not s.no_frames_warned:
                s.no_frames_warned = True
                self.get_logger().warn(
                    f'Sem quadros de /camera/image_raw há mais de {NO_FRAMES_WARN_S:.0f} s')
            elif silent <= NO_FRAMES_WARN_S:
                s.no_frames_warned = False
        self._publish_status()

    def _status_message(self, s: _Session) -> str:
        if s.no_frames_warned:
            return (f'Sem quadros de /camera/image_raw há mais de {NO_FRAMES_WARN_S:.0f} s '
                    '(a câmera está rodando?)')
        if s.scanning:
            return f'Gravando com varredura (passada {s.goals_sent})'
        if s.scan:
            return f'Gravando sem varredura: interrompida, {s.scan_stop_reason}'
        return 'Gravando'

    def _publish_status(self):
        msg = CaptureStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        s = self._session
        if s is None:
            msg.file = self._last_file
            msg.frames = self._last_frames
            msg.dropped = self._last_dropped
            msg.message = self._message
        else:
            msg.recording = True
            msg.scanning = s.scanning
            msg.session = s.session
            msg.file = s.path
            msg.elapsed_s = time.monotonic() - s.start_mono
            msg.frames = s.recorder.frames
            msg.dropped = s.recorder.dropped
            msg.message = self._status_message(s)
        self.status_pub.publish(msg)

    def destroy_node(self):
        # Finaliza o arquivo e cancela a varredura antes de sair
        if self._session is not None:
            self._stop_recording(REASON_SHUTDOWN)
        self.scan_client.destroy()
        super().destroy_node()


def main():
    # Sem o tratador de sinais do rclpy, o Ctrl+C vira KeyboardInterrupt com o
    # contexto ainda válido: o nó ainda consegue cancelar o goal e publicar o status
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = CaptureNode()
    except ValueError:
        # Parâmetro inválido: o motivo já foi registrado no log
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT_S)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
