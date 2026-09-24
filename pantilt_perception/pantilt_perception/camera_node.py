#!/usr/bin/env python3
"""
Captura quadros de uma câmera, stream ou arquivo de vídeo e publica em
/camera/image_raw (docs/architecture.md, seção 4.1).

Tópicos:
  publica  /camera/image_raw  (sensor_msgs/Image, bgr8)  - QoS sensor data

Fontes (parâmetro source, ver camera_source.parse_source):
  - índice ou caminho V4L2 ("0", "/dev/video0"): pede MJPG, a resolução e a
    taxa configuradas e buffer de 1 quadro;
  - URL (ex.: stream MJPEG do Windows, para quando o WSL2 não tem driver UVC);
  - arquivo de vídeo: lido na cadência de fps e reiniciado ao terminar, para
    ensaios repetíveis.

A leitura roda numa thread própria, que lê continuamente e publica no máximo
fps quadros por segundo, sempre o mais recente. Assim o buffer do OpenCV não
acumula atraso, o que prejudicaria a malha IBVS. Se a fonte falhar (câmera
desconectada, stream caiu), a captura é reaberta a cada RECONNECT_INTERVAL_S.

Parâmetros: ver declare_parameter abaixo e pantilt_bringup/config/params.yaml.
"""

import os
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from pantilt_perception.camera_source import (
    KIND_DEVICE, KIND_FILE, KIND_URL, FrameThrottle, fourcc_to_str, parse_source,
    validate_params,
)

# Espera entre tentativas de abrir a fonte
RECONNECT_INTERVAL_S = 1.0

# Leituras seguidas sem quadro até considerar a fonte perdida e reabri-la
MAX_READ_FAILURES = 10

# Intervalo mínimo entre avisos repetidos
LOG_THROTTLE_S = 5.0

# Nível de log do OpenCV (cv::utils::logging): 2 = só erros, sem os avisos do V4L2
OPENCV_LOG_LEVEL_ERROR = 2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # source aceita tipagem dinâmica: "-p source:=0" (ou source: 0 no YAML) chega
        # como inteiro, e um parâmetro só de texto seria rejeitado
        self.declare_parameter(
            'source', '0', ParameterDescriptor(dynamic_typing=True,
                                               description='Índice, URL ou caminho de vídeo'))
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera_optical_frame')

        source = str(self.get_parameter('source').value)
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = float(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value

        try:
            self.kind, self.target = parse_source(source)
            validate_params(self.width, self.height, self.fps)
        except ValueError as e:
            self.get_logger().fatal(f'Parâmetro inválido: {e}')
            raise

        self.get_logger().info(
            f'Fonte: {self.kind} "{self.target}" | pedido {self.width}x{self.height} '
            f'@ {self.fps:.1f} fps | frame_id "{self.frame_id}"'
        )

        # Os avisos internos do OpenCV se repetiriam a cada tentativa de reabrir a
        # fonte; o nó já registra a falha com limite de repetição
        cv2.setLogLevel(OPENCV_LOG_LEVEL_ERROR)

        self.bridge = CvBridge()
        self.throttle = FrameThrottle(self.fps)
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)

        self.stop_event = threading.Event()
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

    # ---------------- Abertura da fonte ----------------
    def open_capture(self):
        """Abre a fonte configurada. Retorna a VideoCapture ou None se falhar."""
        if self.kind == KIND_FILE and not os.path.isfile(self.target):
            self.get_logger().warn(
                f'Arquivo de vídeo não encontrado: {self.target}',
                throttle_duration_sec=LOG_THROTTLE_S)
            return None

        if self.kind == KIND_DEVICE:
            cap = cv2.VideoCapture(self.target, cv2.CAP_V4L2)
        elif self.kind == KIND_URL:
            cap = cv2.VideoCapture(self.target, cv2.CAP_FFMPEG)
        else:
            cap = cv2.VideoCapture(self.target)

        if not cap.isOpened():
            cap.release()
            self.get_logger().warn(
                f'Não foi possível abrir a fonte "{self.target}"; nova tentativa a cada '
                f'{RECONNECT_INTERVAL_S:.0f} s', throttle_duration_sec=LOG_THROTTLE_S)
            return None

        if self.kind == KIND_DEVICE:
            # MJPG permite 640x480 a 30 fps em USB 2.0; buffer de 1 quadro evita atraso
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.get_logger().info(
            f'Fonte aberta: {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x'
            f'{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f} @ {cap.get(cv2.CAP_PROP_FPS):.1f} fps '
            f'({fourcc_to_str(cap.get(cv2.CAP_PROP_FOURCC))})'
        )
        return cap

    # ---------------- Laço de captura ----------------
    def capture_loop(self):
        cap = None
        failures = 0
        size_checked = False
        try:
            while not self.stop_event.is_set():
                if cap is None:
                    cap = self.open_capture()
                    if cap is None:
                        self.stop_event.wait(RECONNECT_INTERVAL_S)
                        continue
                    failures = 0
                    size_checked = False
                    self.throttle.reset()

                if self.kind == KIND_FILE:
                    # Arquivo é lido tão rápido quanto possível: cadencia em fps
                    self.stop_event.wait(self.throttle.wait_time(time.monotonic()))

                ok, frame = cap.read()
                if not ok or frame is None:
                    failures += 1
                    if self.kind == KIND_FILE and failures == 1:
                        # Fim do vídeo: volta ao início
                        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        continue
                    if failures >= MAX_READ_FAILURES:
                        self.get_logger().warn(
                            f'Fonte "{self.target}" parou de entregar quadros; reabrindo')
                        cap.release()
                        cap = None
                        self.stop_event.wait(RECONNECT_INTERVAL_S)
                    continue
                failures = 0

                stamp = self.get_clock().now()
                if not self.throttle.ready(time.monotonic()):
                    continue

                if not size_checked:
                    self._check_size(frame)
                    size_checked = True
                self._publish(frame, stamp)
        except Exception:
            # Ctrl+C invalida o contexto antes do destroy_node; publicar falha nesse intervalo
            if self.context.ok():
                raise
        finally:
            # A VideoCapture só é usada e liberada nesta thread
            if cap is not None:
                cap.release()

    def _check_size(self, frame):
        h, w = frame.shape[:2]
        if (w, h) != (self.width, self.height):
            self.get_logger().warn(
                f'A fonte entrega {w}x{h} em vez de {self.width}x{self.height}; '
                'publicando no tamanho nativo')

    def _publish(self, frame, stamp):
        encoding = 'mono8' if frame.ndim == 2 else 'bgr8'
        msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)

    def destroy_node(self):
        self.stop_event.set()
        # Uma leitura de URL pode bloquear; a thread é daemon e não impede a saída
        self.capture_thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = CameraNode()
    except ValueError:
        # Parâmetro inválido: o motivo já foi registrado no log
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
