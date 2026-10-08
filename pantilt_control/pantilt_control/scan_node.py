#!/usr/bin/env python3
"""
Varredura em zigue-zague do pan-tilt: servidor da action /control/scan
(docs/architecture.md, seção 4.4). O padrão está em scan_pattern.py.

Action:
  /control/scan  (pantilt_interfaces/action/Scan)  - clientes: inspection_manager, capture_node

Tópicos:
  assina   /joint_states      (sensor_msgs/JointState)  - pan_joint e tilt_joint (rad)
  publica  /ptu/cmd_vel_auto  (geometry_msgs/Twist)     - angular.z = pan, angular.y = tilt (rad/s)

Comportamento:
  - publica somente enquanto há um goal ativo, a rate_hz, e velocidade zero
    uma vez ao terminar, ser cancelado, abortar ou o nó encerrar;
  - speed_deg_s e timeout_s iguais a 0 no goal usam os parâmetros do nó;
  - um goal novo substitui o anterior;
  - timeout esgotado termina com completed=false (sem abortar);
  - aborta sem /joint_states por mais de JOINT_STATES_TIMEOUT_S ou se nenhum
    eixo se mover com velocidade comandada (ver scan_pattern.py).

O command_mux continua aplicando a prioridade do operador e o watchdog: o jog
da web descarta estes comandos enquanto o operador estiver no controle.

Parâmetros: ver declare_parameter abaixo e pantilt_bringup/config/params.yaml.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from pantilt_control.scan_pattern import ScanPattern
from pantilt_interfaces.action import Scan

# Idade máxima da última posição recebida em /joint_states
JOINT_STATES_TIMEOUT_S = 0.5

# Velocidade máxima aceita (parâmetro ou goal); igual à saturação do visual_servo_node
MAX_SPEED_DEG_S = 30.0

# Período do feedback da action (o laço roda a rate_hz)
FEEDBACK_PERIOD_S = 0.2

# Espera máxima pelo fim do laço de um goal ao encerrar o nó
SHUTDOWN_WAIT_S = 1.0

# Timeout de cada volta do executor; limita a demora em atender o Ctrl+C
SPIN_TIMEOUT_S = 0.1

PAN_JOINT = 'pan_joint'
TILT_JOINT = 'tilt_joint'

OUTCOME_SUCCEED = 'succeed'
OUTCOME_CANCEL = 'cancel'
OUTCOME_ABORT = 'abort'


class ScanNode(Node):
    def __init__(self):
        super().__init__('scan_node')

        self.declare_parameter('pan_min_deg', -28.0)
        self.declare_parameter('pan_max_deg', 28.0)
        self.declare_parameter('tilt_levels_deg', [-20.0, 0.0, 20.0])
        self.declare_parameter('speed_deg_s', 15.0)
        self.declare_parameter('timeout_s', 60.0)
        self.declare_parameter('rate_hz', 20.0)

        self.pan_min = self.get_parameter('pan_min_deg').value
        self.pan_max = self.get_parameter('pan_max_deg').value
        self.tilt_levels = list(self.get_parameter('tilt_levels_deg').value)
        self.speed = self.get_parameter('speed_deg_s').value
        self.timeout = self.get_parameter('timeout_s').value
        self.rate_hz = self.get_parameter('rate_hz').value

        try:
            # Valida a faixa e as velocidades com a mesma regra usada nos goals
            ScanPattern(self.pan_min, self.pan_max, self.tilt_levels, self.speed)
            if self.speed > MAX_SPEED_DEG_S:
                raise ValueError(f'speed_deg_s deve ser no máximo {MAX_SPEED_DEG_S:.0f}°/s')
            if self.timeout <= 0.0:
                raise ValueError(f'timeout_s deve ser positivo, recebido {self.timeout}')
            if self.rate_hz <= 0.0:
                raise ValueError(f'rate_hz deve ser positivo, recebido {self.rate_hz}')
        except ValueError as e:
            self.get_logger().fatal(f'Parâmetro inválido: {e}')
            raise

        self.get_logger().info(
            f'Pan [{self.pan_min:.1f}°, {self.pan_max:.1f}°] | faixas de tilt '
            f'{[round(t, 1) for t in self.tilt_levels]}° | {self.speed:.1f}°/s | '
            f'timeout {self.timeout:.0f} s | {self.rate_hz:.0f} Hz'
        )

        self.cmd_pub = self.create_publisher(Twist, '/ptu/cmd_vel_auto', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)

        # Última posição (graus) e instante de chegada (monotônico)
        self._position_lock = threading.Lock()
        self._position = None
        self._position_time = 0.0

        # Goal ativo; as transições de estado dos goals passam por este lock
        self._goal_lock = threading.Lock()
        self._goal_handle = None

        self._stop_event = threading.Event()
        self._loop_idle = threading.Event()
        self._loop_idle.set()

        # Reentrante: o laço do goal roda numa thread do executor sem bloquear
        # o /joint_states nem novos goals e cancelamentos
        self._action_server = ActionServer(
            self, Scan, '/control/scan',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            handle_accepted_callback=self.on_accepted,
            cancel_callback=self.on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    # ---------------- Entradas ----------------
    def on_joint_states(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if PAN_JOINT not in positions or TILT_JOINT not in positions:
            self.get_logger().warn(
                f'/joint_states sem {PAN_JOINT}/{TILT_JOINT}: {list(msg.name)}',
                throttle_duration_sec=5.0)
            return
        with self._position_lock:
            self._position = (math.degrees(positions[PAN_JOINT]),
                              math.degrees(positions[TILT_JOINT]))
            self._position_time = time.monotonic()

    def _latest_position(self, now: float):
        """Retorna a última posição em graus, ou None se ausente ou velha demais."""
        with self._position_lock:
            if self._position is None or now - self._position_time > JOINT_STATES_TIMEOUT_S:
                return None
            return self._position

    # ---------------- Goals ----------------
    def on_goal(self, goal: Scan.Goal):
        if not 0.0 <= goal.speed_deg_s <= MAX_SPEED_DEG_S:
            self.get_logger().warn(
                f'Goal recusado: speed_deg_s={goal.speed_deg_s:.1f} fora de '
                f'[0, {MAX_SPEED_DEG_S:.0f}]°/s')
            return GoalResponse.REJECT
        if goal.timeout_s < 0.0:
            self.get_logger().warn(f'Goal recusado: timeout_s={goal.timeout_s:.1f} negativo')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def on_accepted(self, goal_handle):
        with self._goal_lock:
            previous = self._goal_handle
            if previous is not None and previous.is_active:
                # O laço do goal anterior vê que ele não está mais ativo e sai sem publicar zero
                previous.abort()
                self.get_logger().info('Goal anterior substituído por um novo')
            self._goal_handle = goal_handle
        goal_handle.execute()

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        self._loop_idle.clear()
        try:
            return self._run(goal_handle)
        finally:
            self._loop_idle.set()

    def _run(self, goal_handle):
        goal = goal_handle.request
        speed = goal.speed_deg_s or self.speed
        timeout = goal.timeout_s or self.timeout

        position = self._wait_position()
        if position is None:
            return self._finish(goal_handle, OUTCOME_ABORT, False,
                                f'Sem /joint_states há mais de {JOINT_STATES_TIMEOUT_S} s')

        pattern = ScanPattern(self.pan_min, self.pan_max, self.tilt_levels, speed)
        start = time.monotonic()
        pattern.start(start, *position)
        self.get_logger().info(
            f'Varredura iniciada em pan {position[0]:.1f}°, tilt {position[1]:.1f}° | '
            f'{speed:.1f}°/s | timeout {timeout:.0f} s')

        period = 1.0 / self.rate_hz
        next_tick = start
        last_feedback = -math.inf
        while True:
            now = time.monotonic()
            if self._stop_event.is_set():
                return self._finish(goal_handle, OUTCOME_ABORT, False, 'Nó encerrado')
            if not goal_handle.is_active:
                # Substituído por outro goal, que já está publicando
                return Scan.Result(completed=False, message='Substituído por um novo goal')
            if goal_handle.is_cancel_requested:
                return self._finish(goal_handle, OUTCOME_CANCEL, False, 'Varredura cancelada')

            position = self._latest_position(now)
            if position is None:
                return self._finish(goal_handle, OUTCOME_ABORT, False,
                                    f'Sem /joint_states há mais de {JOINT_STATES_TIMEOUT_S} s')
            if now - start > timeout:
                return self._finish(goal_handle, OUTCOME_SUCCEED, False,
                                    f'Tempo esgotado ({timeout:.0f} s) antes do fim do padrão')

            step = pattern.update(now, *position)
            if step.error:
                return self._finish(goal_handle, OUTCOME_ABORT, False, step.error)
            if step.done:
                return self._finish(goal_handle, OUTCOME_SUCCEED, True, 'Padrão completo')

            self._publish_velocity(step.pan_vel, step.tilt_vel)
            if now - last_feedback >= FEEDBACK_PERIOD_S:
                last_feedback = now
                goal_handle.publish_feedback(Scan.Feedback(
                    pan_deg=position[0], tilt_deg=position[1], pass_index=step.pass_index))

            # Cadência fixa; se atrasar (ex.: carga alta), recomeça a contagem
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay < 0.0:
                next_tick = time.monotonic()
                delay = 0.0
            self._stop_event.wait(delay)

    def _wait_position(self):
        """Espera até JOINT_STATES_TIMEOUT_S por uma posição recente."""
        deadline = time.monotonic() + JOINT_STATES_TIMEOUT_S
        while not self._stop_event.is_set():
            now = time.monotonic()
            position = self._latest_position(now)
            if position is not None or now >= deadline:
                return position
            self._stop_event.wait(0.05)
        return None

    def _finish(self, goal_handle, outcome, completed, message):
        """Leva o goal ao estado final e envia velocidade zero, se ele ainda estiver ativo."""
        with self._goal_lock:
            if goal_handle.is_active:
                self._publish_velocity(0.0, 0.0)
                if outcome == OUTCOME_SUCCEED:
                    goal_handle.succeed()
                    self.get_logger().info(f'Varredura terminada: {message}')
                elif outcome == OUTCOME_CANCEL:
                    goal_handle.canceled()
                    self.get_logger().info(message)
                else:
                    goal_handle.abort()
                    self.get_logger().warn(f'Varredura abortada: {message}')
            if self._goal_handle is goal_handle:
                self._goal_handle = None
        return Scan.Result(completed=completed, message=message)

    # ---------------- Saída ----------------
    def _publish_velocity(self, pan_deg_s: float, tilt_deg_s: float):
        msg = Twist()
        msg.angular.z = math.radians(pan_deg_s)
        msg.angular.y = math.radians(tilt_deg_s)
        self.cmd_pub.publish(msg)

    def destroy_node(self):
        # O laço do goal ativo vê o evento, envia o zero e aborta o goal
        self._stop_event.set()
        if not self._loop_idle.wait(SHUTDOWN_WAIT_S):
            self.get_logger().warn('Encerrando: laço da varredura não terminou; enviando velocidade zero')
            self._publish_velocity(0.0, 0.0)
        self._action_server.destroy()
        super().destroy_node()


def main():
    # Sem o tratador de sinais do rclpy, o Ctrl+C vira KeyboardInterrupt com o
    # contexto ainda válido, e o laço do goal consegue publicar a velocidade zero
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = ScanNode()
    except ValueError:
        # Parâmetro inválido: o motivo já foi registrado no log
        rclpy.shutdown()
        raise SystemExit(1)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        # O KeyboardInterrupt só é entregue quando a thread principal volta ao
        # Python. O spin() fica bloqueado em C até chegar um evento, e o nó
        # ocioso não tem timer: o timeout garante a saída em até SPIN_TIMEOUT_S
        while rclpy.ok():
            executor.spin_once(timeout_sec=SPIN_TIMEOUT_S)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
