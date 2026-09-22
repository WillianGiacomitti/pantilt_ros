#!/usr/bin/env python3
"""
Nó ROS 2 que substitui o micro_ros_agent: fala com a ESP32 via serial usando
um protocolo binário simples e próprio, e republica/recebe nos mesmos
tópicos que o firmware micro-ROS usava, para não precisar mexer no
rosbridge/index.html.

Tópicos:
  publica  /joint_states           (sensor_msgs/JointState)
  publica  /ptu/errors             (std_msgs/String)   - log de erros/boot da ESP32
  publica  /ptu/active_source      (std_msgs/String)   - qual fonte está no controle agora
  assina   /ptu/cmd_pos_web        (sensor_msgs/JointState)
  assina   /ptu/cmd_vel_web        (geometry_msgs/Twist)
  assina   /ptu/cmd_vel_joystick   (geometry_msgs/Twist)
  serviço  /ptu/set_zero           (std_srvs/Trigger)

Arbitração: só uma fonte (web ou joystick) controla o PTU por vez. A primeira
fonte a mandar um comando "trava" o controle por ARBITRATION_TIMEOUT_S segundos;
comandos de outra fonte nesse intervalo são ignorados. Se a fonte ativa ficar
em silêncio por mais que esse tempo, qualquer fonte pode assumir o controle.

Parâmetros:
  device   (string, default /dev/ttyUSB0)
  baud     (int,    default 921600)
  reconnect_interval_s (float, default 1.0)
  stale_timeout_s      (float, default 1.0)
"""

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

SOURCE_WEB = 'web'
SOURCE_JOYSTICK = 'joystick'
ARBITRATION_TIMEOUT_S = 0.3  # fonte fica "dona" do controle por esse tempo após o último comando

SYNC1, SYNC2 = 0xA5, 0x5A
MSG_TELEMETRY = 0x01
MSG_CMD_POS = 0x02
MSG_CMD_VEL = 0x03
MSG_SET_ZERO_REQ = 0x04
MSG_SET_ZERO_ACK = 0x05
MSG_HEARTBEAT = 0x06
MSG_ERROR = 0x07
MSG_BOOT_INFO = 0x08

# Precisa ficar em sincronia com os #define ERR_* em SerialProtocol.h
ERROR_CODES = {
    1: 'Timeout de leitura I2C no encoder PAN',
    2: 'Timeout de leitura I2C no encoder TILT',
    3: 'Barramento I2C estava travado - recuperação automática acionada',
    4: 'Fail-safe acionado: motores parados por perda de comunicação',
    5: 'Comando /ptu/cmd_pos recebido com payload inválido',
    6: 'Comando /ptu/cmd_vel recebido com payload inválido',
    7: 'Heap livre baixo na ESP32 (possível vazamento de memória)',
}

# Precisa ficar em sincronia com enum esp_reset_reason_t do ESP-IDF
RESET_REASONS = {
    0: 'ESP_RST_UNKNOWN (desconhecido)',
    1: 'ESP_RST_POWERON (ligou/energizou normalmente)',
    2: 'ESP_RST_EXT (reset externo via pino)',
    3: 'ESP_RST_SW (reset via software, ex: esp_restart())',
    4: 'ESP_RST_PANIC (crash/exceção)',
    5: 'ESP_RST_INT_WDT (watchdog de interrupção)',
    6: 'ESP_RST_TASK_WDT (watchdog de task - nosso watchdog pegou uma trava)',
    7: 'ESP_RST_WDT (outro watchdog)',
    8: 'ESP_RST_DEEPSLEEP',
    9: 'ESP_RST_BROWNOUT (queda de tensão! checar fonte de alimentação)',
    10: 'ESP_RST_SDIO',
    12: 'ESP_RST_USB',
    14: 'ESP_RST_JTAG',
}


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_frame(msg_type: int, payload: bytes) -> bytes:
    header = bytes([msg_type, len(payload)])
    crc = crc8(header + payload)
    return bytes([SYNC1, SYNC2]) + header + payload + bytes([crc])


class FrameParser:
    """State machine não bloqueante, alimentada byte a byte."""

    (WAIT_SYNC1, WAIT_SYNC2, WAIT_TYPE,
     WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC) = range(6)

    def __init__(self, on_frame):
        self.state = self.WAIT_SYNC1
        self.on_frame = on_frame
        self.type = 0
        self.length = 0
        self.payload = bytearray()

    def feed(self, byte: int):
        if self.state == self.WAIT_SYNC1:
            if byte == SYNC1:
                self.state = self.WAIT_SYNC2

        elif self.state == self.WAIT_SYNC2:
            self.state = self.WAIT_TYPE if byte == SYNC2 else self.WAIT_SYNC1

        elif self.state == self.WAIT_TYPE:
            self.type = byte
            self.state = self.WAIT_LEN

        elif self.state == self.WAIT_LEN:
            self.length = byte
            self.payload = bytearray()
            self.state = self.WAIT_CRC if self.length == 0 else self.WAIT_PAYLOAD

        elif self.state == self.WAIT_PAYLOAD:
            self.payload.append(byte)
            if len(self.payload) >= self.length:
                self.state = self.WAIT_CRC

        elif self.state == self.WAIT_CRC:
            calc = crc8(bytes([self.type, self.length]) + bytes(self.payload))
            if calc == byte:
                self.on_frame(self.type, bytes(self.payload))
            # CRC errado -> descarta silenciosamente e resincroniza
            self.state = self.WAIT_SYNC1


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('reconnect_interval_s', 1.0)
        self.declare_parameter('stale_timeout_s', 1.0)

        self.device = self.get_parameter('device').value
        self.baud = self.get_parameter('baud').value
        self.reconnect_interval = self.get_parameter('reconnect_interval_s').value
        self.stale_timeout = self.get_parameter('stale_timeout_s').value

        self.ser = None
        self.ser_lock = threading.Lock()
        self.parser = FrameParser(self.handle_frame)
        self.last_rx_time = 0.0
        self._stale_reported = False

        self.zero_ack_event = threading.Event()
        self.zero_ack_success = False
        self._last_rejection_notice_time = 0.0

        # ---- Arbitração de fonte de comando ----
        self.active_source = None
        self.active_source_last_time = 0.0
        self.source_lock = threading.Lock()

        # QoS com histórico (transient_local) para o painel web, ao (re)conectar,
        # já receber o(s) último(s) erro(s)/status sem precisar esperar um novo evento.
        diagnostics_qos = QoSProfile(depth=20)
        diagnostics_qos.reliability = QoSReliabilityPolicy.RELIABLE
        diagnostics_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.errors_pub = self.create_publisher(String, '/ptu/errors', diagnostics_qos)
        self.active_source_pub = self.create_publisher(String, '/ptu/active_source', diagnostics_qos)

        self.create_subscription(JointState, '/ptu/cmd_pos_web',
                                  lambda m: self.on_cmd_pos(m, SOURCE_WEB), 10)
        self.create_subscription(Twist, '/ptu/cmd_vel_web',
                                  lambda m: self.on_cmd_vel(m, SOURCE_WEB), 10)
        self.create_subscription(Twist, '/ptu/cmd_vel_joystick',
                                  lambda m: self.on_cmd_vel(m, SOURCE_JOYSTICK), 10)
        self.create_service(Trigger, '/ptu/set_zero', self.on_set_zero)

        self.create_timer(1.0, self.check_connection_health)
        self.create_timer(0.5, self.send_heartbeat)

        self.stop_event = threading.Event()
        self.reader_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.reader_thread.start()

    # ---------------- Conexão / reconexão ----------------
    def try_connect(self) -> bool:
        try:
            self.ser = serial.Serial(self.device, self.baud, timeout=0.1)
            self.get_logger().info(f'Conectado a {self.device} @ {self.baud} bps')
            self._publish_diagnostic(f'Conectado a {self.device}')
            return True
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(f'Falha ao abrir {self.device}: {e}')
            return False

    def reader_loop(self):
        while not self.stop_event.is_set():
            if self.ser is None:
                if not self.try_connect():
                    time.sleep(self.reconnect_interval)
                    continue
            try:
                data = self.ser.read(256)
                if data:
                    for b in data:
                        self.parser.feed(b)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Erro de leitura serial, reconectando: {e}')
                self._publish_diagnostic('Conexão serial com a ESP32 perdida - reconectando...')
                self._close_serial()
                time.sleep(self.reconnect_interval)

    def _close_serial(self):
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

    def _publish_diagnostic(self, text: str):
        stamp = time.strftime('%H:%M:%S')
        msg = String()
        msg.data = f'[{stamp}] {text}'
        self.errors_pub.publish(msg)

    # ---------------- Envio de comandos para a ESP32 ----------------
    def send_frame(self, msg_type: int, payload: bytes):
        if self.ser is None:
            return
        frame = build_frame(msg_type, payload)
        with self.ser_lock:
            try:
                self.ser.write(frame)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Falha ao escrever na serial: {e}')
                self._close_serial()

    def _try_acquire(self, source: str) -> bool:
        """Retorna True se 'source' pode enviar o comando agora (é a fonte ativa
        ou a fonte ativa está em silêncio há tempo suficiente para trocar)."""
        now = time.time()
        with self.source_lock:
            timed_out = (now - self.active_source_last_time) > ARBITRATION_TIMEOUT_S
            if self.active_source is None or self.active_source == source or timed_out:
                if self.active_source != source:
                    self.get_logger().info(f'Controle assumido por: {source}')
                    msg = String()
                    msg.data = source
                    self.active_source_pub.publish(msg)
                self.active_source = source
                self.active_source_last_time = now
                return True

            # Rejeitado - avisa no log de diagnóstico, mas no máximo 1x a cada 2s
            if (now - self._last_rejection_notice_time) > 2.0:
                self._publish_diagnostic(
                    f'Comando de "{source}" ignorado - controle em uso por "{self.active_source}"'
                )
                self._last_rejection_notice_time = now
            return False

    def on_cmd_pos(self, msg: JointState, source: str):
        if not self._try_acquire(source):
            return
        pos = dict(zip(msg.name, msg.position))
        payload = struct.pack('<ff', pos.get('pan_joint', 0.0), pos.get('tilt_joint', 0.0))
        self.send_frame(MSG_CMD_POS, payload)

    def on_cmd_vel(self, msg: Twist, source: str):
        if not self._try_acquire(source):
            return
        # mantém a mesma convenção do firmware original: angular.z = pan, angular.y = tilt
        payload = struct.pack('<ff', msg.angular.z, msg.angular.y)
        self.send_frame(MSG_CMD_VEL, payload)

    def on_set_zero(self, request, response):
        self.zero_ack_event.clear()
        self.send_frame(MSG_SET_ZERO_REQ, b'')
        got_ack = self.zero_ack_event.wait(timeout=1.0)
        response.success = bool(got_ack and self.zero_ack_success)
        response.message = 'Eixos zerados' if response.success else 'Sem resposta do firmware (timeout)'
        return response

    def destroy_node(self):
        self.stop_event.set()
        self._close_serial()
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()