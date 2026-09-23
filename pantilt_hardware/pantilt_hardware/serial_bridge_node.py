#!/usr/bin/env python3
"""
Driver serial do pan-tilt: traduz tópicos ROS para o protocolo binário da
ESP32 (ver serial_protocol.py) e vice-versa. Não faz arbitração de fontes;
isso é papel do command_mux, que é o único publicador de /ptu/cmd_*.

Tópicos:
  publica  /joint_states   (sensor_msgs/JointState)  - pan_joint, tilt_joint em rad e rad/s
  publica  /ptu/errors     (std_msgs/String)         - log de erros/boot da ESP32 (transient_local)
  assina   /ptu/cmd_vel    (geometry_msgs/Twist)     - angular.z = pan, angular.y = tilt (rad/s)
  assina   /ptu/cmd_pos    (sensor_msgs/JointState)  - posição absoluta (rad)
  serviço  /ptu/set_zero   (std_srvs/Trigger)

Segurança:
  - heartbeat a heartbeat_hz, que precisa ser maior que 1 / FAILSAFE_TIMEOUT_S;
  - limites de ângulo por software: posições são recortadas e velocidades que
    empurram um eixo para fora do limite são zeradas naquele eixo, tanto ao
    receber o comando quanto a cada telemetria.

Parâmetros: ver declare_parameter abaixo e pantilt_bringup/config/params.yaml.
"""

import math
import struct
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pantilt_hardware.limits import clip_position, effective_limits, limit_velocity
from pantilt_hardware.serial_protocol import (
    ERROR_CODES, FAILSAFE_TIMEOUT_S, MSG_BOOT_INFO, MSG_CMD_POS, MSG_CMD_VEL,
    MSG_ERROR, MSG_HEARTBEAT, MSG_SET_ZERO_ACK, MSG_SET_ZERO_REQ, MSG_TELEMETRY,
    RESET_REASONS, TELEMETRY_RATE_HZ, FrameParser, build_frame,
)

# Intervalo mínimo entre avisos repetidos de comando limitado/rejeitado
LOG_THROTTLE_S = 2.0


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('heartbeat_hz', 5.0)
        self.declare_parameter('pan_limits_deg', [-30.0, 30.0])
        self.declare_parameter('tilt_limits_deg', [-90.0, 90.0])
        self.declare_parameter('limit_margin_deg', 1.0)
        self.declare_parameter('reconnect_interval_s', 1.0)
        self.declare_parameter('stale_timeout_s', 1.0)

        self.device = self.get_parameter('device').value
        self.baud = self.get_parameter('baud').value
        heartbeat_hz = self.get_parameter('heartbeat_hz').value
        self.reconnect_interval = self.get_parameter('reconnect_interval_s').value
        self.stale_timeout = self.get_parameter('stale_timeout_s').value

        # Heartbeat com período >= fail-safe faz o firmware parar os motores esporadicamente
        min_hz = 1.0 / FAILSAFE_TIMEOUT_S
        if heartbeat_hz <= min_hz:
            msg = (f'heartbeat_hz={heartbeat_hz} inválido: precisa ser maior que {min_hz:.1f} Hz '
                   f'(fail-safe do firmware = {FAILSAFE_TIMEOUT_S * 1000:.0f} ms). Use 5.0.')
            self.get_logger().fatal(msg)
            raise ValueError(msg)

        margin_deg = self.get_parameter('limit_margin_deg').value
        try:
            self.pan_limits = effective_limits(
                self.get_parameter('pan_limits_deg').value, margin_deg)
            self.tilt_limits = effective_limits(
                self.get_parameter('tilt_limits_deg').value, margin_deg)
        except ValueError as e:
            self.get_logger().fatal(f'Limites de ângulo inválidos: {e}')
            raise
        self.get_logger().info(
            f'Limites efetivos: pan [{math.degrees(self.pan_limits[0]):.1f}°, '
            f'{math.degrees(self.pan_limits[1]):.1f}°], tilt '
            f'[{math.degrees(self.tilt_limits[0]):.1f}°, {math.degrees(self.tilt_limits[1]):.1f}°]'
        )

        # RLock: _close_serial é chamado de dentro de send_frame, que já segura o lock
        self.ser = None
        self.ser_lock = threading.RLock()
        self.parser = FrameParser(self.handle_frame)
        self.last_rx_time = 0.0
        self._stale_reported = False

        self.zero_ack_event = threading.Event()
        self.zero_ack_success = False

        # Estado para os limites. Protegido por state_lock, porque a telemetria chega
        # pela thread de leitura e os comandos pelo executor. Ordem dos locks:
        # state_lock -> ser_lock.
        self.state_lock = threading.Lock()
        self.joint_pos = None               # (pan, tilt) em rad da última telemetria
        self.last_vel_sent = (0.0, 0.0)     # (pan, tilt) em rad/s efetivamente enviados
        self.lookahead_s = 1.0 / TELEMETRY_RATE_HZ

        # QoS com histórico (transient_local) para o painel web, ao (re)conectar,
        # já receber o(s) último(s) erro(s)/status sem precisar esperar um novo evento.
        diagnostics_qos = QoSProfile(depth=20)
        diagnostics_qos.reliability = QoSReliabilityPolicy.RELIABLE
        diagnostics_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.errors_pub = self.create_publisher(String, '/ptu/errors', diagnostics_qos)

        self.create_subscription(JointState, '/ptu/cmd_pos', self.on_cmd_pos, 10)
        self.create_subscription(Twist, '/ptu/cmd_vel', self.on_cmd_vel, 10)
        self.create_service(Trigger, '/ptu/set_zero', self.on_set_zero)

        self.create_timer(1.0, self.check_connection_health)
        self.create_timer(1.0 / heartbeat_hz, self.send_heartbeat)

        self.stop_event = threading.Event()
        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

    # ---------------- Conexão / reconexão ----------------
    def try_connect(self) -> bool:
        try:
            ser = serial.Serial(self.device, self.baud, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().warn(f'Falha ao abrir {self.device}: {e}')
            return False
        with self.ser_lock:
            self.ser = ser
        self.get_logger().info(f'Conectado a {self.device} @ {self.baud} bps')
        self._publish_diagnostic(f'Conectado a {self.device}')
        return True

    def reader_loop(self):
        while not self.stop_event.is_set():
            ser = self.ser
            if ser is None:
                if not self.try_connect():
                    time.sleep(self.reconnect_interval)
                continue
            try:
                data = ser.read(256)
                if data:
                    for b in data:
                        self.parser.feed(b)
            except (serial.SerialException, OSError, TypeError) as e:
                # TypeError: a porta foi fechada por outra thread durante o read
                if self.stop_event.is_set():
                    break
                self.get_logger().error(f'Erro de leitura serial, reconectando: {e}')
                self._publish_diagnostic('Conexão serial com a ESP32 perdida - reconectando...')
                self._close_serial()
                time.sleep(self.reconnect_interval)
            except Exception:
                # Ctrl+C invalida o contexto antes do destroy_node; publicar falha nesse intervalo
                if not self.context.ok():
                    break
                raise

    def _close_serial(self):
        with self.ser_lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None

    def check_connection_health(self):
        stale = self.last_rx_time and (time.time() - self.last_rx_time) > self.stale_timeout
        if stale and not self._stale_reported:
            self.get_logger().warn(f'Sem dados da ESP32 há mais de {self.stale_timeout:.1f}s')
            self._publish_diagnostic('Sem dados da ESP32 (conexão travada ou lenta)')
            self._stale_reported = True
        elif not stale:
            self._stale_reported = False

    def send_heartbeat(self):
        self.send_frame(MSG_HEARTBEAT, b'')

    # ---------------- Recepção de frames vindos da ESP32 ----------------
    def handle_frame(self, msg_type: int, payload: bytes):
        self.last_rx_time = time.time()

        if msg_type == MSG_TELEMETRY and len(payload) == 16:
            pan_pos, pan_vel, tilt_pos, tilt_vel = struct.unpack('<ffff', payload)
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['pan_joint', 'tilt_joint']
            msg.position = [pan_pos, tilt_pos]
            msg.velocity = [pan_vel, tilt_vel]
            self.joint_pub.publish(msg)
            self._enforce_limits_on_telemetry(pan_pos, tilt_pos)

        elif msg_type == MSG_SET_ZERO_ACK and len(payload) == 1:
            self.zero_ack_success = bool(payload[0])
            self.zero_ack_event.set()

        elif msg_type == MSG_ERROR and len(payload) == 1:
            code = payload[0]
            desc = ERROR_CODES.get(code, f'Código de erro desconhecido ({code})')
            self.get_logger().warn(f'[ESP32] Erro {code}: {desc}')
            self._publish_diagnostic(f'ERRO {code}: {desc}')

        elif msg_type == MSG_BOOT_INFO and len(payload) == 5:
            reset_reason, free_heap = struct.unpack('<BI', payload)
            desc = RESET_REASONS.get(reset_reason, f'Motivo desconhecido ({reset_reason})')
            self.get_logger().info(
                f'[ESP32] Boot detectado - motivo do reset: {desc} | heap livre: {free_heap} bytes'
            )
            self._publish_diagnostic(f'BOOT ({desc}) - heap livre: {free_heap} bytes')
            # Após o boot o firmware parte de velocidade zero
            with self.state_lock:
                self.last_vel_sent = (0.0, 0.0)

    def _enforce_limits_on_telemetry(self, pan_pos: float, tilt_pos: float):
        """
        O firmware mantém a última velocidade recebida. Se ela levar um eixo
        além do limite, reenvia o comando com aquele eixo zerado.
        """
        with self.state_lock:
            self.joint_pos = (pan_pos, tilt_pos)
            pan_vel, tilt_vel = self.last_vel_sent
            pan_lim = limit_velocity(pan_vel, pan_pos, self.pan_limits, self.lookahead_s)
            tilt_lim = limit_velocity(tilt_vel, tilt_pos, self.tilt_limits, self.lookahead_s)
            if (pan_lim, tilt_lim) != (pan_vel, tilt_vel):
                self.get_logger().warn(
                    f'Limite de ângulo atingido (pan={math.degrees(pan_pos):.1f}°, '
                    f'tilt={math.degrees(tilt_pos):.1f}°): eixo parado'
                )
                self._send_velocity_locked(pan_lim, tilt_lim)

    def _publish_diagnostic(self, text: str):
        stamp = time.strftime('%H:%M:%S')
        msg = String()
        msg.data = f'[{stamp}] {text}'
        self.errors_pub.publish(msg)

    # ---------------- Envio de comandos para a ESP32 ----------------
    def send_frame(self, msg_type: int, payload: bytes):
        frame = build_frame(msg_type, payload)
        with self.ser_lock:
            if self.ser is None:
                return
            try:
                self.ser.write(frame)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Falha ao escrever na serial: {e}')
                self._close_serial()

    def _send_velocity_locked(self, pan: float, tilt: float):
        """Envia CMD_VEL e registra o valor enviado. Exige state_lock."""
        self.last_vel_sent = (pan, tilt)
        self.send_frame(MSG_CMD_VEL, struct.pack('<ff', pan, tilt))

    def on_cmd_pos(self, msg: JointState):
        with self.state_lock:
            current = self.joint_pos
        if current is None:
            self.get_logger().warn(
                'Comando de posição rejeitado: ainda sem telemetria da ESP32',
                throttle_duration_sec=LOG_THROTTLE_S)
            return

        # Junta ausente na mensagem mantém a posição atual
        requested = dict(zip(msg.name, msg.position))
        pan = requested.get('pan_joint', current[0])
        tilt = requested.get('tilt_joint', current[1])
        if not (math.isfinite(pan) and math.isfinite(tilt)):
            self.get_logger().warn(
                'Comando de posição rejeitado: valor não finito',
                throttle_duration_sec=LOG_THROTTLE_S)
            return

        pan_clip = clip_position(pan, self.pan_limits)
        tilt_clip = clip_position(tilt, self.tilt_limits)
        if (pan_clip, tilt_clip) != (pan, tilt):
            self.get_logger().warn(
                f'Comando de posição recortado ao limite: pan {math.degrees(pan):.1f}° -> '
                f'{math.degrees(pan_clip):.1f}°, tilt {math.degrees(tilt):.1f}° -> '
                f'{math.degrees(tilt_clip):.1f}°',
                throttle_duration_sec=LOG_THROTTLE_S)

        with self.state_lock:
            # O firmware sai do modo de velocidade; sem isso a checagem na telemetria
            # poderia reenviar uma velocidade antiga e interromper o movimento
            self.last_vel_sent = (0.0, 0.0)
            self.send_frame(MSG_CMD_POS, struct.pack('<ff', pan_clip, tilt_clip))

    def on_cmd_vel(self, msg: Twist):
        # mantém a mesma convenção do firmware original: angular.z = pan, angular.y = tilt
        pan, tilt = msg.angular.z, msg.angular.y
        if not (math.isfinite(pan) and math.isfinite(tilt)):
            self.get_logger().warn(
                'Comando de velocidade rejeitado: valor não finito',
                throttle_duration_sec=LOG_THROTTLE_S)
            return

        with self.state_lock:
            if self.joint_pos is None:
                # Sem posição conhecida não há como checar limites: só permite parar
                pan_lim, tilt_lim = 0.0, 0.0
            else:
                pan_lim = limit_velocity(pan, self.joint_pos[0], self.pan_limits, self.lookahead_s)
                tilt_lim = limit_velocity(
                    tilt, self.joint_pos[1], self.tilt_limits, self.lookahead_s)
            if (pan_lim, tilt_lim) != (pan, tilt):
                motivo = 'sem telemetria' if self.joint_pos is None else 'limite de ângulo'
                self.get_logger().warn(
                    f'Comando de velocidade limitado ({motivo}): '
                    f'pan {pan:.3f} -> {pan_lim:.3f} rad/s, tilt {tilt:.3f} -> {tilt_lim:.3f} rad/s',
                    throttle_duration_sec=LOG_THROTTLE_S)
            self._send_velocity_locked(pan_lim, tilt_lim)

    def on_set_zero(self, request, response):
        self.zero_ack_event.clear()
        self.send_frame(MSG_SET_ZERO_REQ, b'')
        got_ack = self.zero_ack_event.wait(timeout=1.0)
        response.success = bool(got_ack and self.zero_ack_success)
        response.message = 'Eixos zerados' if response.success else 'Sem resposta do firmware (timeout)'
        return response

    def destroy_node(self):
        self.stop_event.set()
        # Para os motores antes de soltar a porta, sem esperar o fail-safe
        self.send_frame(MSG_CMD_VEL, struct.pack('<ff', 0.0, 0.0))
        self.reader_thread.join(timeout=1.0)
        self._close_serial()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = SerialBridgeNode()
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
