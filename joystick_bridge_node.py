#!/usr/bin/env python3
"""
Nó ROS 2 que traduz o controle 8BitDo (lido via sensor_msgs/Joy, publicado
pelo nó padrão 'joy_node' do pacote ros-humble-joy) em comandos para o PTU.

Publica em /ptu/cmd_vel_joystick (geometry_msgs/Twist) para o
serial_bridge_node, que faz a arbitração entre essa fonte e a web.

Pré-requisito: rodar o joy_node apontando para o dispositivo do controle,
por exemplo:
    ros2 run joy joy_node --ros-args -p device_id:=0 -p deadzone:=0.1

Mapeamento de eixos/botões varia por controle/modo (X-input vs D-input) e
driver do SO. Para descobrir os índices certos do seu 8BitDo:
    ros2 topic echo /joy
e mexa nos manches/botões observando qual posição do array muda.

Parâmetros (ajuste conforme o /joy echo do seu controle):
  axis_pan            (int, default 0)  -> eixo do analógico esquerdo (X) = pan
  axis_tilt           (int, default 1)  -> eixo do analógico esquerdo (Y) = tilt
  invert_pan          (bool, default False)
  invert_tilt         (bool, default True)   -> Y do analógico geralmente vem invertido
  deadzone            (float, default 0.15)  -> ignora ruído perto do centro
  max_pan_deg_s        (float, default 60.0)
  max_tilt_deg_s       (float, default 40.0)
  zero_button          (int, default 0)   -> botão que aciona /ptu/set_zero
  publish_rate_hz      (float, default 20.0)
"""

import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

DEG_TO_RAD = 0.0174532925


class JoystickBridgeNode(Node):
    def __init__(self):
        super().__init__('joystick_bridge_node')

        self.declare_parameter('axis_pan', 0)
        self.declare_parameter('axis_tilt', 1)
        self.declare_parameter('invert_pan', False)
        self.declare_parameter('invert_tilt', True)
        self.declare_parameter('deadzone', 0.15)
        self.declare_parameter('max_pan_deg_s', 60.0)
        self.declare_parameter('max_tilt_deg_s', 40.0)
        self.declare_parameter('zero_button', 0)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.axis_pan = self.get_parameter('axis_pan').value
        self.axis_tilt = self.get_parameter('axis_tilt').value
        self.invert_pan = self.get_parameter('invert_pan').value
        self.invert_tilt = self.get_parameter('invert_tilt').value
        self.deadzone = self.get_parameter('deadzone').value
        self.max_pan_deg_s = self.get_parameter('max_pan_deg_s').value
        self.max_tilt_deg_s = self.get_parameter('max_tilt_deg_s').value
        self.zero_button = self.get_parameter('zero_button').value
        self.publish_period = 1.0 / self.get_parameter('publish_rate_hz').value

        self.cmd_vel_pub = self.create_publisher(Twist, '/ptu/cmd_vel_joystick', 10)
        self.set_zero_client = self.create_client(Trigger, '/ptu/set_zero')

        self.create_subscription(Joy, '/joy', self.on_joy, 10)

        self._last_publish_time = 0.0
        self._was_active = False       # se no ciclo anterior havia movimento (fora da deadzone)
        self._zero_button_was_pressed = False

    @staticmethod
    def _apply_deadzone(value: float, deadzone: float) -> float:
        if abs(value) < deadzone:
            return 0.0
        # remapeia o range [deadzone, 1.0] -> [0.0, 1.0] pra não ter um "salto" na saída
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)

    def on_joy(self, msg: Joy):
        # ---- Botão de zerar (borda de subida, pra não chamar o serviço em repeat) ----
        if len(msg.buttons) > self.zero_button:
            pressed = bool(msg.buttons[self.zero_button])
            if pressed and not self._zero_button_was_pressed:
                self.call_set_zero()
            self._zero_button_was_pressed = pressed

        # ---- Eixos de jog ----
        if len(msg.axes) <= max(self.axis_pan, self.axis_tilt):
            return  # controle não tem eixos suficientes / mapeamento errado

        raw_pan = msg.axes[self.axis_pan]
        raw_tilt = msg.axes[self.axis_tilt]

        pan_norm = self._apply_deadzone(raw_pan, self.deadzone)
        tilt_norm = self._apply_deadzone(raw_tilt, self.deadzone)

        if self.invert_pan:
            pan_norm = -pan_norm
        if self.invert_tilt:
            tilt_norm = -tilt_norm

        is_active = (pan_norm != 0.0) or (tilt_norm != 0.0)
        now = time.time()

        # Publica continuamente enquanto há movimento (a uma taxa limitada), e manda
        # UM comando de zero explícito na transição pra parado (solta o stick) -
        # depois disso para de publicar, liberando a arbitração pra outra fonte.
        if is_active:
            if (now - self._last_publish_time) < self.publish_period:
                return
            self._last_publish_time = now
            self._send_velocity(pan_norm * self.max_pan_deg_s, tilt_norm * self.max_tilt_deg_s)
            self._was_active = True
        elif self._was_active:
            self._send_velocity(0.0, 0.0)
            self._was_active = False

    def _send_velocity(self, pan_deg_s: float, tilt_deg_s: float):
        twist = Twist()
        twist.angular.z = pan_deg_s * DEG_TO_RAD
        twist.angular.y = tilt_deg_s * DEG_TO_RAD
        self.cmd_vel_pub.publish(twist)

    def call_set_zero(self):
        if not self.set_zero_client.service_is_ready():
            self.get_logger().warn('/ptu/set_zero indisponível no momento')
            return
        req = Trigger.Request()
        future = self.set_zero_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info(f'set_zero via joystick: {f.result().message}')
            if f.result() else None
        )


def main():
    rclpy.init()
    node = JoystickBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()