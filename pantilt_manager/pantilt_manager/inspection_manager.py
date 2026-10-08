#!/usr/bin/env python3
"""
Gerenciador da inspeção (docs/architecture.md, seções 4.3 e 6): máquina de
estados IDLE → SEARCHING → CENTERING → (TRACKING) → IDLE. As transições estão
em state_machine.py; este nó liga a máquina ao ROS e executa as ações dela.

Services:
  /inspection/start           (pantilt_interfaces/StartInspection) - inicia a partir do IDLE
  /inspection/stop            (std_srvs/Trigger)                   - interrompe e volta ao IDLE
  /inspection/list_equipment  (pantilt_interfaces/ListEquipment)   - equipamentos do yaml

Tópicos:
  assina   /perception/target   (pantilt_interfaces/VisualTarget)     - confirmação do alvo
  assina   /ptu/control_source  (std_msgs/String)                     - fonte ativa do command_mux
  publica  /inspection/status   (pantilt_interfaces/InspectionStatus) - 5 Hz e a cada mudança

As duas últimas usam QoS transient_local.

Clientes:
  /perception/set_target  (pantilt_interfaces/SetTarget)  - filtro do detector_node
  /control/scan           (pantilt_interfaces/Scan)       - action do scan_node
  /control/center         (pantilt_interfaces/Center)     - action do visual_servo_node

Comportamento:
  - o /inspection/start espera a resposta do set_target (até SET_TARGET_TIMEOUT_S):
    uma recusa do detector volta à interface com a mensagem dele;
  - resultados e feedbacks de goals que não são mais o atual são ignorados;
  - operador no controle (/ptu/control_source = web), stop, servidor de
    action fora do ar ou nó encerrado: cancela os goals, remove o filtro do
    detector e volta ao IDLE.

Concorrência: MultiThreadedExecutor. O /inspection/start espera a resposta do
set_target numa thread, com o cliente num grupo de callbacks próprio. O rclpy
roda os callbacks de conclusão dos futures (resposta e resultado dos goals)
como tasks do executor, fora dos callback groups: por isso a máquina e os goal
handles ficam sob um lock.

Parâmetros: ver declare_parameter abaixo e pantilt_bringup/config/params.yaml.
"""

import threading

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pantilt_interfaces.action import Center, Scan
from pantilt_interfaces.msg import InspectionStatus, VisualTarget
from pantilt_interfaces.srv import ListEquipment, SetTarget, StartInspection
from pantilt_manager.state_machine import (
    ACTION_CANCEL_CENTER, ACTION_CANCEL_SCAN, ACTION_CLEAR_TARGET, ACTION_SEND_CENTER,
    ACTION_SEND_SCAN, MODE_TRACK, InspectionMachine, Params, validate_params,
)
from pantilt_perception.detection import load_equipment

# Período de publicação do /inspection/status
STATUS_PERIOD_S = 0.2

# Espera máxima pela resposta do /perception/set_target no /inspection/start
SET_TARGET_TIMEOUT_S = 2.0

# Timeout de cada volta do executor; limita a demora em atender o Ctrl+C
SPIN_TIMEOUT_S = 0.1

# Fonte publicada pelo command_mux quando o operador está no controle
SOURCE_WEB = 'web'

SCAN_SERVER = 'scan_node (/control/scan)'
CENTER_SERVER = 'visual_servo_node (/control/center)'
DETECTOR_SERVICE = 'detector_node (/perception/set_target)'


def _segundos(stamp) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


class InspectionManager(Node):
    def __init__(self):
        super().__init__('inspection_manager')

        self.declare_parameter(
            'equipment_file',
            '/ros2_ws/src/pantilt_ros/pantilt_bringup/config/equipment_coco_test.yaml')
        self.declare_parameter('confirm_frames', 3)
        self.declare_parameter('scan_timeout_s', 60.0)
        self.declare_parameter('max_reacquire', 2)
        self.declare_parameter('tolerance_px', 20.0)
        self.declare_parameter('hold_time_s', 1.0)
        self.declare_parameter('lost_timeout_s', 1.0)

        p = lambda nome: self.get_parameter(nome).value  # noqa: E731
        self.equipment_file = p('equipment_file')
        self.scan_timeout_s = p('scan_timeout_s')
        self.tolerance_px = p('tolerance_px')
        self.hold_time_s = p('hold_time_s')
        self.lost_timeout_s = p('lost_timeout_s')

        try:
            validate_params(p('confirm_frames'), self.scan_timeout_s, p('max_reacquire'),
                            self.tolerance_px, self.hold_time_s, self.lost_timeout_s)
            self.equipment = load_equipment(self.equipment_file)
        except ValueError as e:
            self.get_logger().fatal(f'Parâmetro inválido: {e}')
            raise

        params = Params(confirm_frames=p('confirm_frames'), scan_timeout_s=self.scan_timeout_s,
                        max_reacquire=p('max_reacquire'))
        self.machine = InspectionMachine(params, self.equipment.keys())

        self.get_logger().info(
            f'Equipamentos de {self.equipment_file}: {", ".join(self.equipment)} | '
            f'confirmação em {params.confirm_frames} quadros | varredura até '
            f'{self.scan_timeout_s:.0f} s | até {params.max_reacquire} readquisições | '
            f'Center: {self.tolerance_px:.1f} px por {self.hold_time_s:.1f} s, perda em '
            f'{self.lost_timeout_s:.1f} s'
        )

        # transient_local: quem conectar depois (a página) recebe o último estado / a fonte atual
        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.status_pub = self.create_publisher(InspectionStatus, '/inspection/status', latched)
        self.create_subscription(VisualTarget, '/perception/target', self.on_target, 10)
        self.create_subscription(String, '/ptu/control_source', self.on_control_source, latched)

        self.create_service(StartInspection, '/inspection/start', self.on_start)
        self.create_service(Trigger, '/inspection/stop', self.on_stop)
        self.create_service(ListEquipment, '/inspection/list_equipment', self.on_list)

        # Grupo próprio: a resposta chega enquanto o /inspection/start espera no grupo padrão
        self.set_target_client = self.create_client(
            SetTarget, '/perception/set_target',
            callback_group=MutuallyExclusiveCallbackGroup())
        self.scan_client = ActionClient(self, Scan, '/control/scan')
        self.center_client = ActionClient(self, Center, '/control/center')

        self.create_timer(STATUS_PERIOD_S, self.on_timer)

        # Máquina e goals: tocados também pelos callbacks dos futures (ver docstring)
        self._lock = threading.RLock()
        self._source = ''
        # Token do goal atual de cada action (None = nenhum); o handle chega com a resposta
        self._scan_token = None
        self._scan_handle = None
        self._center_token = None
        self._center_handle = None
        self._last_status = None
        self._publish_status()

    # ---------------- Services ----------------
    def on_start(self, request, response):
        key = request.equipment.strip()
        with self._lock:
            motivo = self.machine.check_start(
                key, request.mode, self._source == SOURCE_WEB, self._unavailable())
        if motivo:
            return self._refuse(response, motivo)

        # Fora do lock: os callbacks dos goals continuam livres. O grupo padrão
        # segue ocupado, então nenhum outro start ou stop entra durante a espera
        ok, texto = self._set_target_and_wait(key)
        if not ok:
            return self._refuse(response, texto)

        with self._lock:
            # Alvos capturados antes desta resposta não confirmam
            since = self.get_clock().now().nanoseconds * 1e-9
            self._apply(self.machine.start(key, request.mode, since))

        modo = 'TRACK' if request.mode == MODE_TRACK else 'CENTER'
        response.accepted = True
        response.message = f'Inspeção iniciada: {self.equipment[key].label} ({modo})'
        self.get_logger().info(f'{response.message} | {texto}')
        return response

    def _refuse(self, response, text):
        self.get_logger().warn(f'Início recusado: {text}')
        response.accepted = False
        response.message = text
        return response

    def _unavailable(self) -> list:
        """Servidores fora do ar, com o launch a subir."""
        out = []
        if not self.scan_client.server_is_ready():
            out.append(f'{SCAN_SERVER}: suba o control.launch.py')
        if not self.center_client.server_is_ready():
            out.append(f'{CENTER_SERVER}: suba o control.launch.py')
        if not self.set_target_client.service_is_ready():
            out.append(f'{DETECTOR_SERVICE}: suba o perception.launch.py')
        return out

    def _set_target_and_wait(self, key: str):
        """Aplica o filtro no detector. Retorna (sucesso, mensagem)."""
        done = threading.Event()
        future = self.set_target_client.call_async(SetTarget.Request(equipment=key))
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(SET_TARGET_TIMEOUT_S):
            # Se a resposta chegar depois, o filtro não fica ativo sem inspeção
            self._clear_target()
            return False, (f'detector_node não respondeu ao /perception/set_target em '
                           f'{SET_TARGET_TIMEOUT_S:.0f} s')
        result = future.result()
        if not result.success:
            return False, f'Filtro recusado pelo detector_node: {result.message}'
        return True, result.message

    def on_stop(self, request, response):
        with self._lock:
            if not self.machine.active:
                response.success = False
                response.message = 'Nenhuma inspeção ativa'
                return response
            self._apply(self.machine.stop())
            response.success = True
            response.message = self.machine.message
        return response

    def on_list(self, request, response):
        response.keys = list(self.equipment)
        response.labels = [e.label for e in self.equipment.values()]
        return response

    # ---------------- Tópicos ----------------
    def on_target(self, msg: VisualTarget):
        with self._lock:
            self._apply(self.machine.target(msg.detected, msg.equipment,
                                            _segundos(msg.header.stamp)))

    def on_control_source(self, msg: String):
        with self._lock:
            self._source = msg.data
            if msg.data == SOURCE_WEB:
                self._apply(self.machine.operator())

    def on_timer(self):
        with self._lock:
            # Servidor que saiu do ar (ex.: Ctrl+C) sem entregar o resultado do goal
            if self.machine.scan_active and not self.scan_client.server_is_ready():
                self._forget_scan()
                self._apply(self.machine.server_lost(SCAN_SERVER))
            elif self.machine.center_active and not self.center_client.server_is_ready():
                self._forget_center()
                self._apply(self.machine.server_lost(CENTER_SERVER))
            self._publish_status()

    # ---------------- Ações da máquina ----------------
    def _apply(self, actions):
        """Executa as ações na ordem e publica o status se algo visível mudou. Sob o lock."""
        for action in actions:
            if action == ACTION_SEND_SCAN:
                self._send_scan()
            elif action == ACTION_CANCEL_SCAN:
                self._cancel_scan()
            elif action == ACTION_SEND_CENTER:
                self._send_center()
            elif action == ACTION_CANCEL_CENTER:
                self._cancel_center()
            elif action == ACTION_CLEAR_TARGET:
                self._clear_target()

        status = self.machine.status()
        last = self._last_status
        if last is None or (status.state, status.equipment, status.message) != (
                last.state, last.equipment, last.message):
            if last is not None and last.state != status.state:
                self.get_logger().info(f'{last.state} -> {status.state}: {status.message}')
            elif last is not None:
                self.get_logger().info(f'{status.state}: {status.message}')
            self._publish_status()

    def _clear_target(self):
        if not self.set_target_client.service_is_ready():
            self.get_logger().warn(
                f'{DETECTOR_SERVICE} fora do ar: filtro do detector não removido')
            return
        future = self.set_target_client.call_async(SetTarget.Request(equipment=''))
        future.add_done_callback(self._on_clear_target)

    def _on_clear_target(self, future):
        result = future.result()
        if result is None or not result.success:
            motivo = result.message if result is not None else 'sem resposta'
            self.get_logger().warn(f'Falha ao remover o filtro do detector: {motivo}')

    # ---------------- Varredura ----------------
    def _send_scan(self):
        # speed_deg_s = 0 usa o parâmetro do scan_node; uma passada por SEARCHING
        goal = Scan.Goal(speed_deg_s=0.0, timeout_s=float(self.scan_timeout_s))
        token = object()
        self._scan_token = token
        self._scan_handle = None
        future = self.scan_client.send_goal_async(goal)
        future.add_done_callback(lambda f, t=token: self._on_scan_response(t, f))

    def _on_scan_response(self, token, future):
        handle = future.result()
        with self._lock:
            if token is not self._scan_token:
                # Cancelado (alvo confirmado, stop...) antes da resposta: não deixa varrendo
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self._forget_scan()
                self._apply(self.machine.scan_rejected())
                return
            self._scan_handle = handle
            handle.get_result_async().add_done_callback(
                lambda f, t=token: self._on_scan_result(t, f))

    def _on_scan_result(self, token, future):
        with self._lock:
            if token is not self._scan_token:
                return
            self._forget_scan()
            response = future.result()
            self._apply(self.machine.scan_finished(
                response.status == GoalStatus.STATUS_SUCCEEDED,
                response.result.completed, response.result.message))

    def _cancel_scan(self):
        if self._scan_handle is not None:
            self._scan_handle.cancel_goal_async()
        self._forget_scan()

    def _forget_scan(self):
        self._scan_token = None
        self._scan_handle = None

    # ---------------- Centralização ----------------
    def _send_center(self):
        goal = Center.Goal(mode=self.machine.mode, tolerance_px=float(self.tolerance_px),
                           hold_time_s=float(self.hold_time_s),
                           lost_timeout_s=float(self.lost_timeout_s))
        token = object()
        self._center_token = token
        self._center_handle = None
        future = self.center_client.send_goal_async(
            goal, feedback_callback=lambda m, t=token: self._on_center_feedback(t, m))
        future.add_done_callback(lambda f, t=token: self._on_center_response(t, f))

    def _on_center_response(self, token, future):
        handle = future.result()
        with self._lock:
            if token is not self._center_token:
                if handle.accepted:
                    handle.cancel_goal_async()
                return
            if not handle.accepted:
                self._forget_center()
                self._apply(self.machine.center_rejected())
                return
            self._center_handle = handle
            handle.get_result_async().add_done_callback(
                lambda f, t=token: self._on_center_result(t, f))

    def _on_center_feedback(self, token, msg):
        with self._lock:
            if token is not self._center_token:
                return
            fb = msg.feedback
            self._apply(self.machine.center_feedback(fb.error_px, fb.centered))

    def _on_center_result(self, token, future):
        with self._lock:
            if token is not self._center_token:
                return
            self._forget_center()
            response = future.result()
            r = response.result
            self._apply(self.machine.center_finished(
                response.status == GoalStatus.STATUS_SUCCEEDED, r.message,
                r.convergence_time_s, r.final_error_px))

    def _cancel_center(self):
        if self._center_handle is not None:
            self._center_handle.cancel_goal_async()
        self._forget_center()

    def _forget_center(self):
        self._center_token = None
        self._center_handle = None

    # ---------------- Status ----------------
    def _publish_status(self):
        status = self.machine.status()
        msg = InspectionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = status.state
        msg.equipment = status.equipment
        msg.autonomous = status.autonomous
        msg.error_px = float(status.error_px)
        msg.message = status.message
        self.status_pub.publish(msg)
        self._last_status = status

    def destroy_node(self):
        # Cancela os goals, remove o filtro e publica o IDLE antes de sair
        with self._lock:
            self._apply(self.machine.shutdown())
        self.scan_client.destroy()
        self.center_client.destroy()
        super().destroy_node()


def main():
    # Sem o tratador de sinais do rclpy, o Ctrl+C vira KeyboardInterrupt com o
    # contexto ainda válido: o nó ainda consegue cancelar os goals e publicar o IDLE
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = InspectionManager()
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
