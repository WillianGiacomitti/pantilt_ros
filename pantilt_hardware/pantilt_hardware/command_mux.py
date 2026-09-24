#!/usr/bin/env python3
"""
Multiplexador de comandos do pan-tilt: único caminho de comandos até o
serial_bridge_node. Aplica a prioridade do operador e o watchdog de comando
(ver command_arbiter.py e docs/architecture.md, seção 4.6).

Tópicos:
  assina   /ptu/cmd_vel_auto    (geometry_msgs/Twist)     - scan_node, visual_servo_node
  assina   /ptu/cmd_vel_web     (geometry_msgs/Twist)     - interface web
  assina   /ptu/cmd_pos_web     (sensor_msgs/JointState)  - interface web
  publica  /ptu/cmd_vel         (geometry_msgs/Twist)     - angular.z = pan, angular.y = tilt (rad/s)
  publica  /ptu/cmd_pos         (sensor_msgs/JointState)  - posição absoluta (rad)
  publica  /ptu/control_source  (std_msgs/String)         - web, auto ou none (transient_local)

Segurança:
  - um comando da web assume o controle na hora; comandos automáticos são
    descartados enquanto a web tiver publicado nos últimos operator_hold_s;
  - se a fonte ativa ficar em silêncio por mais de cmd_timeout_s depois de uma
    velocidade não nula, envia velocidade zero uma vez. O firmware mantém a
    última velocidade e o heartbeat do bridge impede o fail-safe, então sem
    isso um nó de controle travado deixaria o eixo girando.

Parâmetros: ver declare_parameter abaixo e pantilt_bringup/config/params.yaml.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from pantilt_hardware.command_arbiter import CommandArbiter

# Período da checagem do watchdog e da fonte ativa. O zero sai no máximo
# cmd_timeout_s + CHECK_PERIOD_S depois do último comando.
CHECK_PERIOD_S = 0.05

# Intervalo mínimo entre avisos repetidos de comando descartado
LOG_THROTTLE_S = 2.0


def is_zero_velocity(msg: Twist) -> bool:
    """Só angular.z (pan) e angular.y (tilt) chegam ao firmware."""
    return msg.angular.z == 0.0 and msg.angular.y == 0.0


class CommandMuxNode(Node):
    def __init__(self):
        super().__init__('command_mux')

        self.declare_parameter('operator_hold_s', 1.0)
        self.declare_parameter('cmd_timeout_s', 0.3)

        try:
            self.arbiter = CommandArbiter(
                self.get_parameter('operator_hold_s').value,
                self.get_parameter('cmd_timeout_s').value,
            )
        except ValueError as e:
            self.get_logger().fatal(f'Parâmetro inválido: {e}')
            raise
        self.get_logger().info(
            f'Prioridade do operador: {self.arbiter.operator_hold_s:.2f} s | '
            f'watchdog de comando: {self.arbiter.cmd_timeout_s:.2f} s'
        )

        # transient_local: quem conectar depois (inspection_manager, web) recebe a fonte atual
        source_qos = QoSProfile(depth=1)
        source_qos.reliability = QoSReliabilityPolicy.RELIABLE
        source_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.vel_pub = self.create_publisher(Twist, '/ptu/cmd_vel', 10)
        self.pos_pub = self.create_publisher(JointState, '/ptu/cmd_pos', 10)
        self.source_pub = self.create_publisher(String, '/ptu/control_source', source_qos)

        self.create_subscription(Twist, '/ptu/cmd_vel_auto', self.on_cmd_vel_auto, 10)
        self.create_subscription(Twist, '/ptu/cmd_vel_web', self.on_cmd_vel_web, 10)
        self.create_subscription(JointState, '/ptu/cmd_pos_web', self.on_cmd_pos_web, 10)

        self.source = None
        self._update_source(time.monotonic())

        self.create_timer(CHECK_PERIOD_S, self.on_check)

    # ---------------- Comandos recebidos ----------------
    def on_cmd_vel_web(self, msg: Twist):
        now = time.monotonic()
        self.arbiter.on_web(now, is_velocity=True, is_zero=is_zero_velocity(msg))
        self.vel_pub.publish(msg)
        self._update_source(now)

    def on_cmd_pos_web(self, msg: JointState):
        now = time.monotonic()
        self.arbiter.on_web(now, is_velocity=False)
        self.pos_pub.publish(msg)
        self._update_source(now)

    def on_cmd_vel_auto(self, msg: Twist):
        now = time.monotonic()
        if not self.arbiter.on_auto(now, is_zero=is_zero_velocity(msg)):
            self.get_logger().info(
                'Comando automático descartado: operador no controle',
                throttle_duration_sec=LOG_THROTTLE_S)
            return
        self.vel_pub.publish(msg)
        self._update_source(now)

    # ---------------- Watchdog e fonte ativa ----------------
    def on_check(self):
        now = time.monotonic()
        silent = self.arbiter.check_watchdog(now)
        if silent is not None:
            self.get_logger().warn(
                f'Watchdog: fonte "{silent}" em silêncio há mais de '
                f'{self.arbiter.cmd_timeout_s:.2f} s, enviando velocidade zero'
            )
            self.vel_pub.publish(Twist())
        self._update_source(now)

    def _update_source(self, now: float):
        source = self.arbiter.source(now)
        if source == self.source:
            return
        if self.source is None:
            self.get_logger().info(f'Fonte de controle inicial: {source}')
        else:
            self.get_logger().info(f'Fonte de controle: {self.source} -> {source}')
        self.source = source
        self.source_pub.publish(String(data=source))

    def destroy_node(self):
        # Não deixa o eixo girando com a última velocidade repassada
        if self.arbiter.armed:
            self.get_logger().info('Encerrando: enviando velocidade zero')
            self.vel_pub.publish(Twist())
        super().destroy_node()


def main():
    # Sem o tratador de sinais do rclpy, o Ctrl+C vira KeyboardInterrupt com o
    # contexto ainda válido, e o destroy_node consegue publicar a velocidade zero
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = CommandMuxNode()
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
