#!/usr/bin/env python3
"""
Servo visual IBVS do pan-tilt: servidor da action /control/center
(docs/architecture.md, seção 4.5). Controladores em controllers/, conversão,
dt e supervisão em visual_servo_math.py.

Action:
  /control/center  (pantilt_interfaces/action/Center)  - cliente: inspection_manager

Tópicos:
  assina   /perception/target   (pantilt_interfaces/VisualTarget)  - erro do alvo em px
  assina   /camera/camera_info  (sensor_msgs/CameraInfo)           - f_x e f_y em px
  assina   /joint_states        (sensor_msgs/JointState)           - pan_joint e tilt_joint (rad)
  publica  /ptu/cmd_vel_auto    (geometry_msgs/Twist)              - angular.z = pan, angular.y = tilt (rad/s)

Pipeline (um ciclo por alvo novo, publicação a publish_rate_hz):
  1. conversão: θ = atan(error_px / f), com f_x e f_y fixados no início do goal;
  2. controlador: um por eixo, criado por criar_controlador(controller, ...);
     dt pelos header.stamp dos alvos válidos;
  3. saturação em max_vel_deg_s (o PID já satura e faz o anti-windup) e
     invert_pan / invert_tilt;
  4. supervisão: centrado, perdido, preso no limite e métricas t_c e e_r;
  5. publicação: repete o último comando por até cmd_hold_s após o último
     alvo válido; depois disso, zero.

Comportamento:
  - goal recusado sem /camera/camera_info válido (f_x e f_y > 0) ou com
    campos fora da faixa;
  - um goal novo substitui o anterior; os controladores reiniciam com reset();
  - publica somente enquanto há um goal ativo e velocidade zero antes de
    responder, em qualquer término (inclusive o nó encerrado);
  - MODE_CENTER termina ao centralizar; MODE_TRACK continua ativo com
    feedback centered=true até ser cancelado, perder o alvo ou prender;
  - um feedback por alvo recebido (é dele que sai o e_d, via ros2 bag).

O command_mux continua aplicando a prioridade do operador e o watchdog.

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
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, JointState

from pantilt_control.controllers.base import criar_controlador
from pantilt_control.visual_servo_math import (
    Supervisor, calcular_dt, px_para_angulo, saturar,
)
from pantilt_interfaces.action import Center
from pantilt_interfaces.msg import VisualTarget

# Idade máxima da última posição recebida em /joint_states
JOINT_STATES_TIMEOUT_S = 0.5

# Saturação máxima aceita; igual ao teto de velocidade do scan_node
MAX_SPEED_DEG_S = 30.0

# Espera máxima pelo fim do laço de um goal ao encerrar o nó
SHUTDOWN_WAIT_S = 1.0

# Timeout de cada volta do executor; limita a demora em atender o Ctrl+C
SPIN_TIMEOUT_S = 0.1

PAN_JOINT = 'pan_joint'
TILT_JOINT = 'tilt_joint'

OUTCOME_SUCCEED = 'succeed'
OUTCOME_CANCEL = 'cancel'
OUTCOME_ABORT = 'abort'


def _segundos(stamp) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


class VisualServoNode(Node):
    def __init__(self):
        super().__init__('visual_servo_node')

        self.declare_parameter('controller', 'pid')
        for eixo in ('pan', 'tilt'):
            self.declare_parameter(f'{eixo}.kp', 0.8)
            self.declare_parameter(f'{eixo}.ki', 0.0)
            self.declare_parameter(f'{eixo}.kd', 0.0)
        self.declare_parameter('derivative_filter_hz', 2.0)
        self.declare_parameter('max_vel_deg_s', 20.0)
        self.declare_parameter('integral_limit_deg_s', 10.0)
        self.declare_parameter('invert_pan', False)
        self.declare_parameter('invert_tilt', False)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('cmd_hold_s', 0.25)
        self.declare_parameter('limit_timeout_s', 2.0)

        p = lambda nome: self.get_parameter(nome).value  # noqa: E731
        self.controller = p('controller')
        self.max_vel_deg_s = p('max_vel_deg_s')
        self.integral_limit_deg_s = p('integral_limit_deg_s')
        self.invert_pan = p('invert_pan')
        self.invert_tilt = p('invert_tilt')
        self.publish_rate_hz = p('publish_rate_hz')
        self.cmd_hold_s = p('cmd_hold_s')
        self.limit_timeout_s = p('limit_timeout_s')

        try:
            if not 0.0 < self.max_vel_deg_s <= MAX_SPEED_DEG_S:
                raise ValueError(f'max_vel_deg_s deve estar em (0, {MAX_SPEED_DEG_S:.0f}]°/s, '
                                 f'recebido {self.max_vel_deg_s}')
            for nome in ('publish_rate_hz', 'cmd_hold_s', 'limit_timeout_s'):
                if p(nome) <= 0.0:
                    raise ValueError(f'{nome} deve ser positivo, recebido {p(nome)}')
            # Mesmas chaves para qualquer controlador; cada um usa as que precisa
            self.controladores = {}
            for eixo in ('pan', 'tilt'):
                try:
                    self.controladores[eixo] = criar_controlador(self.controller, {
                        'kp': p(f'{eixo}.kp'),
                        'ki': p(f'{eixo}.ki'),
                        'kd': p(f'{eixo}.kd'),
                        'derivative_filter_hz': p('derivative_filter_hz'),
                        'max_output': math.radians(self.max_vel_deg_s),
                        'integral_limit': math.radians(self.integral_limit_deg_s),
                    })
                except ValueError as e:
                    raise ValueError(f'{eixo}: {e}') from e
        except ValueError as e:
            self.get_logger().fatal(f'Parâmetro inválido: {e}')
            raise

        self.get_logger().info(
            f'Controlador {self.controller} | pan kp={p("pan.kp")} ki={p("pan.ki")} '
            f'kd={p("pan.kd")} | tilt kp={p("tilt.kp")} ki={p("tilt.ki")} kd={p("tilt.kd")} | '
            f'máx {self.max_vel_deg_s:.1f}°/s | invert pan={self.invert_pan} '
            f'tilt={self.invert_tilt} | {self.publish_rate_hz:.0f} Hz'
        )

        self.cmd_pub = self.create_publisher(Twist, '/ptu/cmd_vel_auto', 10)
        self.create_subscription(VisualTarget, '/perception/target', self.on_target, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_camera_info, qos_profile_sensor_data)

        # Últimas entradas. O alvo tem um contador para o laço saber se é novo
        self._data_lock = threading.Lock()
        self._alvo = None
        self._alvo_seq = 0
        self._position = None            # (pan, tilt) em graus
        self._position_time = 0.0        # instante de chegada (monotônico)
        self._camera_info = None

        # Goal ativo; as transições de estado dos goals passam por este lock
        self._goal_lock = threading.Lock()
        self._goal_handle = None

        self._stop_event = threading.Event()
        self._loop_idle = threading.Event()
        self._loop_idle.set()

        # Reentrante: o laço do goal roda numa thread do executor sem bloquear
        # as assinaturas nem novos goals e cancelamentos
        self._action_server = ActionServer(
            self, Center, '/control/center',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            handle_accepted_callback=self.on_accepted,
            cancel_callback=self.on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    # ---------------- Entradas ----------------
    def on_target(self, msg: VisualTarget):
        with self._data_lock:
            self._alvo = msg
            self._alvo_seq += 1

    def on_joint_states(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        if PAN_JOINT not in positions or TILT_JOINT not in positions:
            self.get_logger().warn(
                f'/joint_states sem {PAN_JOINT}/{TILT_JOINT}: {list(msg.name)}',
                throttle_duration_sec=5.0)
            return
        with self._data_lock:
            self._position = (math.degrees(positions[PAN_JOINT]),
                              math.degrees(positions[TILT_JOINT]))
            self._position_time = time.monotonic()

    def on_camera_info(self, msg: CameraInfo):
        with self._data_lock:
            self._camera_info = msg

    def _latest_position(self):
        """Retorna a última posição em graus, ou None se ausente ou velha demais."""
        with self._data_lock:
            if (self._position is None
                    or time.monotonic() - self._position_time > JOINT_STATES_TIMEOUT_S):
                return None
            return self._position

    def _focais(self):
        """(f_x, f_y, largura, altura) do último CameraInfo, ou None se ausente ou sem calibração."""
        with self._data_lock:
            info = self._camera_info
        if info is None or info.k[0] <= 0.0 or info.k[4] <= 0.0:
            return None
        return info.k[0], info.k[4], info.width, info.height

    def _agora(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------- Goals ----------------
    def on_goal(self, goal: Center.Goal):
        motivo = None
        if self._focais() is None:
            motivo = ('sem /camera/camera_info válido (f_x = 0 ou nenhuma mensagem). '
                      'Suba o camera_node com o arquivo de intrínsecos (calibration_node)')
        elif goal.mode not in (Center.Goal.MODE_CENTER, Center.Goal.MODE_TRACK):
            motivo = f'mode={goal.mode} inválido (0 = CENTER, 1 = TRACK)'
        elif goal.tolerance_px <= 0.0:
            motivo = f'tolerance_px={goal.tolerance_px:.1f} deve ser positivo'
        elif goal.hold_time_s < 0.0:
            motivo = f'hold_time_s={goal.hold_time_s:.1f} negativo'
        elif goal.lost_timeout_s <= 0.0:
            motivo = f'lost_timeout_s={goal.lost_timeout_s:.1f} deve ser positivo'
        if motivo:
            self.get_logger().warn(f'Goal recusado: {motivo}')
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
        track = goal.mode == Center.Goal.MODE_TRACK

        focais = self._focais()
        if focais is None:
            # CameraInfo perdeu a calibração entre o aceite e o início
            sup = Supervisor(goal.tolerance_px, goal.hold_time_s, goal.lost_timeout_s,
                             self.limit_timeout_s, self._agora())
            return self._finish(goal_handle, OUTCOME_ABORT, sup,
                                'sem /camera/camera_info válido')
        f_x, f_y, largura, altura = focais

        for controlador in self.controladores.values():
            controlador.reset()
        sup = Supervisor(goal.tolerance_px, goal.hold_time_s, goal.lost_timeout_s,
                         self.limit_timeout_s, self._agora())
        with self._data_lock:
            # Só alvos que chegarem depois do início do goal
            seq_visto = self._alvo_seq
        stamp_anterior = None
        comando = (0.0, 0.0)             # graus/s, já com invert_*
        comando_t = -math.inf            # chegada do último alvo válido (monotônico)
        avisou_juntas = False

        self.get_logger().info(
            f'Centralização iniciada ({"TRACK" if track else "CENTER"}) | f_x={f_x:.1f} '
            f'f_y={f_y:.1f} px | tolerância {goal.tolerance_px:.1f} px por '
            f'{goal.hold_time_s:.1f} s | perda em {goal.lost_timeout_s:.1f} s')

        period = 1.0 / self.publish_rate_hz
        next_tick = time.monotonic()
        while True:
            if self._stop_event.is_set():
                return self._finish(goal_handle, OUTCOME_ABORT, sup, 'nó encerrado')
            if not goal_handle.is_active:
                # Substituído por outro goal, que já está publicando
                return self._result(sup, False, 'substituído por um novo goal')
            if goal_handle.is_cancel_requested:
                return self._finish(goal_handle, OUTCOME_CANCEL, sup, 'cancelado')

            with self._data_lock:
                alvo, seq = self._alvo, self._alvo_seq
            if seq != seq_visto and alvo is not None:
                seq_visto = seq
                t = _segundos(alvo.header.stamp)
                error_px = math.hypot(alvo.error_x, alvo.error_y)
                if alvo.detected:
                    if (alvo.image_width, alvo.image_height) != (largura, altura):
                        self.get_logger().warn(
                            f'Alvo em {alvo.image_width}x{alvo.image_height}, mas o '
                            f'/camera/camera_info é de {largura}x{altura}: f não vale para '
                            'esta resolução (refaça a calibração)', throttle_duration_sec=5.0)
                    dt, lacuna = calcular_dt(t, stamp_anterior)
                    stamp_anterior = t
                    if lacuna:
                        for controlador in self.controladores.values():
                            controlador.reset()
                    comando = self._calcular_comando(alvo, f_x, f_y, dt)
                    comando_t = time.monotonic()
                sup.alvo(t, alvo.detected, error_px)
                goal_handle.publish_feedback(Center.Feedback(
                    error_x=alvo.error_x, error_y=alvo.error_y,
                    error_px=error_px if alvo.detected else 0.0,
                    target_visible=alvo.detected, centered=sup.centrado))

            # Estágio 5: repete o último comando por até cmd_hold_s
            if time.monotonic() - comando_t <= self.cmd_hold_s:
                publicado = comando
            else:
                publicado = (0.0, 0.0)

            agora = self._agora()
            position = self._latest_position()
            if position is None:
                sup.limpar_juntas()
                if not avisou_juntas:
                    avisou_juntas = True
                    self.get_logger().warn(
                        f'Sem /joint_states há mais de {JOINT_STATES_TIMEOUT_S} s: '
                        'checagem de limite desligada até a telemetria voltar')
            else:
                avisou_juntas = False
                sup.junta(agora, position[0], position[1], *publicado)

            estado = sup.checar(agora)
            if estado.centrado and not track:
                return self._finish(
                    goal_handle, OUTCOME_SUCCEED, sup,
                    f'centralizado em {estado.t_c:.2f} s, erro residual {estado.e_r:.1f} px')
            if estado.preso is not None:
                return self._finish(goal_handle, OUTCOME_ABORT, sup, estado.preso.mensagem())
            if estado.perdido:
                return self._finish(goal_handle, OUTCOME_ABORT, sup, 'alvo perdido')

            self._publish_velocity(*publicado)

            # Cadência fixa; se atrasar (ex.: carga alta), recomeça a contagem
            next_tick += period
            delay = next_tick - time.monotonic()
            if delay < 0.0:
                next_tick = time.monotonic()
                delay = 0.0
            self._stop_event.wait(delay)

    def _calcular_comando(self, alvo: VisualTarget, f_x: float, f_y: float, dt: float):
        """Estágios 1 a 3 para um alvo válido: retorna (pan, tilt) em graus/s."""
        theta_pan = px_para_angulo(alvo.error_x, f_x)
        theta_tilt = px_para_angulo(alvo.error_y, f_y)
        limite = math.radians(self.max_vel_deg_s)
        pan = saturar(self.controladores['pan'].compute(theta_pan, dt), limite)
        tilt = saturar(self.controladores['tilt'].compute(theta_tilt, dt), limite)
        if self.invert_pan:
            pan = -pan
        if self.invert_tilt:
            tilt = -tilt
        return math.degrees(pan), math.degrees(tilt)

    def _result(self, sup: Supervisor, success: bool, message: str):
        # Métricas da primeira convergência (0 se não houve)
        return Center.Result(success=success, message=message,
                             final_error_px=sup.e_r, convergence_time_s=sup.t_c)

    def _finish(self, goal_handle, outcome, sup, message):
        """Leva o goal ao estado final e envia velocidade zero, se ele ainda estiver ativo."""
        with self._goal_lock:
            if goal_handle.is_active:
                self._publish_velocity(0.0, 0.0)
                if outcome == OUTCOME_SUCCEED:
                    goal_handle.succeed()
                    self.get_logger().info(f'Centralização terminada: {message}')
                elif outcome == OUTCOME_CANCEL:
                    goal_handle.canceled()
                    self.get_logger().info('Centralização cancelada')
                else:
                    goal_handle.abort()
                    self.get_logger().warn(f'Centralização abortada: {message}')
            if self._goal_handle is goal_handle:
                self._goal_handle = None
        return self._result(sup, outcome == OUTCOME_SUCCEED, message)

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
            self.get_logger().warn('Encerrando: laço da centralização não terminou; '
                                   'enviando velocidade zero')
            self._publish_velocity(0.0, 0.0)
        self._action_server.destroy()
        super().destroy_node()


def main():
    # Sem o tratador de sinais do rclpy, o Ctrl+C vira KeyboardInterrupt com o
    # contexto ainda válido, e o laço do goal consegue publicar a velocidade zero
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = VisualServoNode()
    except ValueError:
        # Parâmetro inválido: o motivo já foi registrado no log
        rclpy.shutdown()
        raise SystemExit(1)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        # O KeyboardInterrupt só é entregue quando a thread principal volta ao
        # Python; o timeout garante a saída em até SPIN_TIMEOUT_S
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
