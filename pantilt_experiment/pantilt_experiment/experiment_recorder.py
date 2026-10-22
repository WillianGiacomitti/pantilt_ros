#!/usr/bin/env python3
"""
Gravação de cada inspeção para os ensaios do TCC (docs/architecture.md,
seção 4.11). Só observa: assina tópicos, consulta resultados de actions e
parâmetros dos nós, e nunca comanda nada.

Gatilhos e lógica das pastas em session.py; escrita dos CSVs em csv_writer.py.

Service:
  /experiment/configure  (pantilt_interfaces/ConfigureExperiment) - rótulo, observação, imagens

Tópicos:
  publica  /experiment/status  (pantilt_interfaces/ExperimentStatus) - 2 Hz e a cada mudança (transient_local)
  assina   /inspection/status, /perception/target, /perception/detections,
           /perception/timing, /control/servo_debug, /joint_states,
           /ptu/cmd_vel_auto, /ptu/cmd_vel, /ptu/control_source, /ptu/errors,
           /camera/camera_info e os tópicos ocultos status e feedback de
           /control/center e /control/scan

Clientes:
  /control/center/_action/get_result, /control/scan/_action/get_result - resultado dos goals
  /<nó>/list_parameters e /<nó>/get_parameters dos nós de param_nodes

Comportamento:
  - uma gravação por inspeção (sai do IDLE -> volta ao IDLE) ou por goal
    Center direto (com o gerenciador em IDLE), mais tail_s de cauda e pre_s de
    pré-gravação;
  - pasta <output_dir>/<rótulo>/NNN_AAAAMMDD_HHMMSS, renomeada ao fechar para
    ..._<equipamento>_<modo>, com um CSV por sinal e o resumo.json;
  - ao fechar, espera até RESULT_WAIT_S pelos resultados e parâmetros pendentes;
  - não abre (ou fecha a atual) com espaço livre abaixo de min_free_gb;
  - Ctrl+C no meio de uma gravação: fecha a pasta com desfecho "interrompido".

Concorrência: executor de uma thread (spin_once). Os callbacks dos futures
também rodam nessa thread: sem locks.

Parâmetros: ver declare_parameter abaixo e pantilt_bringup/config/params.yaml.
"""

import json
import math
import os
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import (
    QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_action_status_default,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from pantilt_experiment.csv_writer import CsvWriter
from pantilt_experiment.session import (
    ACTION_CLOSE, MODE_TRACK, SOURCE_DIRECT, SOURCE_INSPECTION, RecordingTrigger,
    check_image_mode, clean_label, csv_columns, final_basename, folder_basename, free_gb,
    git_info, infer_mode, next_repetition, outcome, validate_params,
)
from pantilt_interfaces.action import Center, Scan
from pantilt_interfaces.msg import (
    DetectionTiming, ExperimentStatus, InspectionStatus, ServoDebug, VisualTarget,
)
from pantilt_interfaces.srv import ConfigureExperiment

# Período do laço de supervisão (cauda, pendências, status, disco)
TICK_PERIOD_S = 0.1

# Período de publicação do /experiment/status
STATUS_PERIOD_S = 0.5

# Intervalo entre verificações do espaço em disco durante a gravação
DISK_CHECK_PERIOD_S = 5.0

# Espera máxima pelos resultados e parâmetros pendentes ao fechar
RESULT_WAIT_S = 2.0

# Goal visto pela primeira vez já finalizado: só conta se foi aceito há menos disso
RECENT_GOAL_S = 5.0

# Fila das assinaturas dos sinais (o executor de uma thread pode atrasar)
SIGNAL_QUEUE = 50

# Timeout de cada volta do executor; limita a demora em atender o Ctrl+C
SPIN_TIMEOUT_S = 0.1

# Ações observadas
ACTION_CENTER = 'center'
ACTION_SCAN = 'scan'

ACTIVE_STATUSES = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                   GoalStatus.STATUS_CANCELING)
TERMINAL_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'sucesso',
    GoalStatus.STATUS_CANCELED: 'cancelado',
    GoalStatus.STATUS_ABORTED: 'abortado',
}

STATE_CENTERING = 'CENTERING'
STATE_TRACKING = 'TRACKING'

REASON_SHUTDOWN = 'nó encerrado'


def _seconds(stamp) -> float:
    return Time.from_msg(stamp).nanoseconds * 1e-9


def _goal_key(goal_id) -> str:
    return bytes(goal_id.uuid).hex()


def _json_default(value):
    # Arrays dos parâmetros (array.array, numpy) e o que mais o json não conhece
    try:
        return list(value)
    except TypeError:
        return str(value)


@dataclass
class _Recording:
    """Estado de uma gravação em andamento."""
    source: str
    label: str
    repetition: int
    notes: str
    image_mode: str
    folder: str                  # caminho provisório (NNN_AAAAMMDD_HHMMSS)
    base: str
    writer: CsvWriter
    started_at: datetime
    t0: float                    # início, relógio do ROS (s)
    equipment: str = ''
    servo_mode: object = None    # Center.Goal.mode visto no /control/servo_debug
    saw_tracking: bool = False
    centered_seen: bool = False
    centering_entries: int = 0
    last_message: str = ''
    center_goals: dict = field(default_factory=dict)
    scan_goals: dict = field(default_factory=dict)
    params: dict = field(default_factory=dict)
    pending: int = 0             # respostas de services ainda não recebidas
    closing_since: float = None  # fim pedido; espera as pendências
    interrupted: str = ''        # motivo da interrupção, se houve
    write_error_logged: bool = False

    def rel(self, t: float) -> float:
        return round(t - self.t0, 4)


class ExperimentRecorder(Node):
    def __init__(self):
        super().__init__('experiment_recorder')

        self.declare_parameter('output_dir', '/ros2_ws/ensaios')
        self.declare_parameter('default_label', 'sem_rotulo')
        self.declare_parameter('image_mode', 'none')
        self.declare_parameter('pre_s', 1.0)
        self.declare_parameter('tail_s', 1.0)
        self.declare_parameter('min_free_gb', 1.0)
        # Imagens (snapshots e video): usados a partir da etapa 3
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('video_fps', 10.0)
        self.declare_parameter('fourcc', 'mp4v')
        self.declare_parameter('queue_size', 2000)
        self.declare_parameter(
            'param_nodes', ['visual_servo_node', 'inspection_manager', 'detector_node', 'scan_node'])
        self.declare_parameter('repo_dir', '/ros2_ws/src/pantilt_ros')

        p = lambda nome: self.get_parameter(nome).value  # noqa: E731
        self.output_dir = p('output_dir')
        self.default_label = clean_label(p('default_label')) or 'sem_rotulo'
        self.image_mode = p('image_mode')
        self.pre_s = p('pre_s')
        self.tail_s = p('tail_s')
        self.min_free_gb = p('min_free_gb')
        self.queue_size = p('queue_size')
        self.param_nodes = [n.strip('/') for n in p('param_nodes') if n.strip('/')]

        try:
            validate_params(self.image_mode, self.pre_s, self.tail_s, self.min_free_gb,
                            self.queue_size)
        except ValueError as e:
            self.get_logger().fatal(f'Parâmetro inválido: {e}')
            raise

        self.commit = git_info(p('repo_dir'))
        if self.commit is None:
            self.get_logger().warn(f'{p("repo_dir")} não é um repositório git: o commit não '
                                   'vai para o resumo.json')

        # Configuração das próximas gravações (/experiment/configure)
        self.label = self.default_label
        self.notes = ''
        self.trigger = RecordingTrigger(self.tail_s)
        self._rec = None
        self._pre = deque()              # (t_rx, arquivo, t, valores) fora de gravação
        self._goal_status = {ACTION_CENTER: {}, ACTION_SCAN: {}}
        self._last_inspection = None     # (estado, mensagem) do último /inspection/status
        self._camera = None
        self._last_folder = ''
        self._message = 'Pronto'
        self._next_rep = next_repetition(self._label_dir())
        self._last_status_t = 0.0
        self._last_disk_t = 0.0

        latched = QoSProfile(depth=1)
        latched.reliability = QoSReliabilityPolicy.RELIABLE
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.status_pub = self.create_publisher(ExperimentStatus, '/experiment/status', latched)
        self.create_service(ConfigureExperiment, '/experiment/configure', self.on_configure)

        sub = self.create_subscription
        sub(InspectionStatus, '/inspection/status', self.on_inspection_status, latched)
        sub(String, '/ptu/control_source', self.on_control_source, latched)
        sub(String, '/ptu/errors', self.on_errors, latched)
        sub(VisualTarget, '/perception/target', self.on_target, SIGNAL_QUEUE)
        sub(Detection2DArray, '/perception/detections', self.on_detections, SIGNAL_QUEUE)
        sub(DetectionTiming, '/perception/timing', self.on_timing, SIGNAL_QUEUE)
        sub(ServoDebug, '/control/servo_debug', self.on_servo_debug, SIGNAL_QUEUE)
        sub(JointState, '/joint_states', self.on_joint_states, SIGNAL_QUEUE)
        sub(Twist, '/ptu/cmd_vel_auto', lambda m: self._on_twist('cmd_auto', m), SIGNAL_QUEUE)
        sub(Twist, '/ptu/cmd_vel', lambda m: self._on_twist('cmd_mux', m), SIGNAL_QUEUE)
        sub(CameraInfo, '/camera/camera_info', self.on_camera_info, qos_profile_sensor_data)

        self._actions = {ACTION_CENTER: Center, ACTION_SCAN: Scan}
        self._result_clients = {}
        for name, action in self._actions.items():
            prefix = f'/control/{name}/_action'
            sub(action.Impl.FeedbackMessage, f'{prefix}/feedback',
                lambda m, n=name: self.on_feedback(n, m), SIGNAL_QUEUE)
            sub(GoalStatusArray, f'{prefix}/status',
                lambda m, n=name: self.on_goal_status(n, m), qos_profile_action_status_default)
            self._result_clients[name] = self.create_client(
                action.Impl.GetResultService, f'{prefix}/get_result')

        self._param_clients = {
            n: (self.create_client(ListParameters, f'/{n}/list_parameters'),
                self.create_client(GetParameters, f'/{n}/get_parameters'))
            for n in self.param_nodes
        }

        self.create_timer(TICK_PERIOD_S, self.on_tick)

        self.get_logger().info(
            f'Saída: {self.output_dir} | rótulo "{self.label}" (próxima #{self._next_rep:03d}) | '
            f'imagens: {self.image_mode} | pré {self.pre_s:.1f} s, cauda {self.tail_s:.1f} s | '
            f'mínimo {self.min_free_gb:.1f} GB livres | commit '
            f'{self.commit["commit"][:8] if self.commit else "?"}')
        self._publish_status()

    # ---------------- Tempo e linhas ----------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _label_dir(self, label: str = None) -> str:
        return os.path.join(self.output_dir, label or self.label)

    def _row(self, name: str, t, values):
        """Grava uma linha (t = stamp da mensagem, ou None para usar a recepção)."""
        now = self._now()
        if t is None:
            t = now
        r = self._rec
        if r is None or r.closing_since is not None:
            # Fora de gravação: guarda os últimos pre_s para a próxima
            if self.pre_s > 0.0:
                self._pre.append((now, name, t, values))
                while self._pre and self._pre[0][0] < now - self.pre_s:
                    self._pre.popleft()
            return
        r.writer.write(name, [r.rel(t), r.rel(now), *values])

    def _event(self, source: str, event: str, detail: str = '', t=None):
        self._row('events', t, [source, event, detail])

    # ---------------- Sinais ----------------
    def on_target(self, msg: VisualTarget):
        r = self._rec
        if r is not None and r.source == SOURCE_DIRECT and msg.equipment and not r.equipment:
            r.equipment = msg.equipment
        self._row('target', _seconds(msg.header.stamp), [
            msg.detected, msg.equipment, msg.confidence, msg.center_x, msg.center_y,
            msg.width, msg.height, msg.error_x, msg.error_y, msg.image_width, msg.image_height])

    def on_detections(self, msg: Detection2DArray):
        t = _seconds(msg.header.stamp)
        for i, d in enumerate(msg.detections):
            hyp = d.results[0].hypothesis if d.results else None
            self._row('detections', t, [
                i, hyp.class_id if hyp else '', hyp.score if hyp else math.nan,
                d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size_x, d.bbox.size_y])

    def on_timing(self, msg: DetectionTiming):
        self._row('timing', _seconds(msg.header.stamp),
                  [msg.inference_ms, msg.latency_ms, msg.n_detections])

    def on_servo_debug(self, msg: ServoDebug):
        r = self._rec
        if r is not None and r.servo_mode is None:
            r.servo_mode = msg.mode
        self._row('servo', _seconds(msg.header.stamp), [
            msg.controller, msg.mode, msg.target_visible, msg.error_x, msg.error_y, msg.error_px,
            msg.theta_pan, msg.theta_tilt, msg.dt, msg.gap_reset,
            msg.p_pan, msg.i_pan, msg.d_pan, msg.p_tilt, msg.i_tilt, msg.d_tilt,
            msg.u_raw_pan, msg.u_raw_tilt, msg.saturated_pan, msg.saturated_tilt,
            msg.cmd_pan, msg.cmd_tilt, msg.centered, msg.convergence_time_s,
            msg.final_error_px])

    def on_joint_states(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        vel = dict(zip(msg.name, msg.velocity))
        self._row('joints', _seconds(msg.header.stamp), [
            pos.get('pan_joint', math.nan), pos.get('tilt_joint', math.nan),
            vel.get('pan_joint', math.nan), vel.get('tilt_joint', math.nan)])

    def _on_twist(self, name: str, msg: Twist):
        self._row(name, None, [msg.angular.z, msg.angular.y])

    def on_camera_info(self, msg: CameraInfo):
        self._camera = {'fx': msg.k[0], 'fy': msg.k[4], 'cx': msg.k[2], 'cy': msg.k[5],
                        'width': msg.width, 'height': msg.height}

    def on_control_source(self, msg: String):
        self._event('mux', 'fonte', msg.data)

    def on_errors(self, msg: String):
        self._event('bridge', 'erro', msg.data)

    def on_feedback(self, action: str, msg):
        fb = msg.feedback
        goal = _goal_key(msg.goal_id)[:8]
        if action == ACTION_CENTER:
            r = self._rec
            if r is not None and fb.centered:
                r.centered_seen = True
            self._row('center_feedback', None, [
                goal, fb.error_x, fb.error_y, fb.error_px, fb.target_visible, fb.centered])
        else:
            self._row('scan_feedback', None, [goal, fb.pan_deg, fb.tilt_deg, fb.pass_index])

    # ---------------- Inspeção ----------------
    def on_inspection_status(self, msg: InspectionStatus):
        now = self._now()
        previous = self._last_inspection
        self._last_inspection = (msg.state, msg.message)
        self._apply(self.trigger.inspection_state(msg.state, now))

        r = self._rec
        if r is not None and r.source == SOURCE_INSPECTION and msg.equipment:
            r.equipment = msg.equipment
        if previous == self._last_inspection:
            return                       # repetição a 5 Hz
        if r is not None and r.source == SOURCE_INSPECTION:
            if msg.state == STATE_TRACKING:
                r.saw_tracking = True
            if msg.state == STATE_CENTERING and (previous is None
                                                 or previous[0] != STATE_CENTERING):
                r.centering_entries += 1
            if msg.message:
                r.last_message = msg.message
        self._event('inspecao', msg.state, msg.message, _seconds(msg.header.stamp))

    # ---------------- Goals ----------------
    def on_goal_status(self, action: str, msg: GoalStatusArray):
        now = self._now()
        known = self._goal_status[action]
        current = {}
        for st in msg.status_list:
            key = _goal_key(st.goal_info.goal_id)
            current[key] = st.status
            previous = known.get(key)
            if previous == st.status:
                continue
            accepted_t = _seconds(st.goal_info.stamp)
            if previous is None:
                if st.status in ACTIVE_STATUSES:
                    self._goal_started(action, key, accepted_t, now)
                elif st.status in TERMINAL_NAMES and now - accepted_t < RECENT_GOAL_S:
                    # Aceito e finalizado entre duas mensagens de status
                    self._goal_started(action, key, accepted_t, now)
                    self._goal_finished(action, key, st, now)
                # Finalizado há mais tempo (ex.: histórico ao subir): ignorado
            elif st.status in TERMINAL_NAMES and previous in ACTIVE_STATUSES:
                self._goal_finished(action, key, st, now)
        self._goal_status[action] = current

    def _goals(self, r: _Recording, action: str) -> dict:
        return r.center_goals if action == ACTION_CENTER else r.scan_goals

    def _goal_started(self, action: str, key: str, accepted_t: float, now: float):
        if action == ACTION_CENTER:
            self._apply(self.trigger.center_goal_active(key, now))
        r = self._rec
        if r is not None and r.closing_since is None:
            goal = {'goal': key[:8], 't_inicio': r.rel(accepted_t), 't_fim': None,
                    'status': 'ativo', 'message': ''}
            if action == ACTION_CENTER:
                goal.update(success=None, convergence_time_s=None, final_error_px=None)
            else:
                goal.update(completed=None)
            self._goals(r, action)[key] = goal
        self._event(action, 'goal_ativo', key[:8], accepted_t)

    def _goal_finished(self, action: str, key: str, st, now: float):
        name = TERMINAL_NAMES[st.status]
        self._event(action, 'goal_fim', f'{key[:8]} {name}')
        r = self._rec
        goal = self._goals(r, action).get(key) if r is not None else None
        if goal is not None:
            goal['t_fim'] = r.rel(now)
            goal['status'] = name
            client = self._result_clients[action]
            if client.service_is_ready():
                request = self._actions[action].Impl.GetResultService.Request()
                request.goal_id = st.goal_info.goal_id
                r.pending += 1
                client.call_async(request).add_done_callback(
                    lambda f, r=r, a=action, k=key: self._on_result(r, a, k, f))
            else:
                goal['message'] = 'resultado indisponível (servidor fora do ar)'
        if action == ACTION_CENTER:
            self._apply(self.trigger.center_goal_finished(key, now))

    def _on_result(self, r: _Recording, action: str, key: str, future):
        r.pending -= 1
        response = future.result()
        if response is None:
            return
        res = response.result
        goal = self._goals(r, action)[key]
        goal['message'] = res.message
        if action == ACTION_CENTER:
            goal.update(success=bool(res.success),
                        convergence_time_s=round(float(res.convergence_time_s), 4),
                        final_error_px=round(float(res.final_error_px), 3))
            if r.source == SOURCE_DIRECT:
                r.last_message = res.message
        else:
            goal['completed'] = bool(res.completed)
        if r is self._rec:
            self._event(action, 'resultado', f'{key[:8]} {res.message}')

    # ---------------- Parâmetros dos nós ----------------
    def _snapshot_params(self, r: _Recording):
        for node, (list_client, get_client) in self._param_clients.items():
            if not (list_client.service_is_ready() and get_client.service_is_ready()):
                r.params[node] = 'ausente'
                continue
            r.pending += 1
            list_client.call_async(ListParameters.Request()).add_done_callback(
                lambda f, r=r, n=node: self._on_param_names(r, n, f))

    def _on_param_names(self, r: _Recording, node: str, future):
        response = future.result()
        if response is None:
            r.pending -= 1
            r.params[node] = 'sem resposta'
            return
        names = list(response.result.names)
        self._param_clients[node][1].call_async(GetParameters.Request(names=names)) \
            .add_done_callback(lambda f, r=r, n=node, names=names:
                               self._on_param_values(r, n, names, f))

    def _on_param_values(self, r: _Recording, node: str, names: list, future):
        r.pending -= 1
        response = future.result()
        if response is None:
            r.params[node] = 'sem resposta'
            return
        r.params[node] = {name: parameter_value_to_python(value)
                          for name, value in zip(names, response.values)}

    # ---------------- Abertura e fechamento ----------------
    def _apply(self, actions):
        for i, action in enumerate(actions):
            if action[0] == ACTION_CLOSE:
                # Seguido de outra abertura: fecha na hora, sem esperar pendências
                self._close(force=i + 1 < len(actions))
            else:
                self._open(action[1])

    def _open(self, source: str):
        if self._rec is not None:
            self._close(force=True)
        label_dir = self._label_dir()
        try:
            os.makedirs(label_dir, exist_ok=True)
            free = free_gb(label_dir)
        except OSError as e:
            return self._refuse_open(f'não foi possível criar {label_dir}: {e}')
        if free < self.min_free_gb:
            return self._refuse_open(f'pouco espaço em disco: {free:.1f} GB livres '
                                     f'(mínimo {self.min_free_gb:.1f} GB)')

        now = self._now()
        started_at = datetime.now()
        repetition = next_repetition(label_dir)
        base = folder_basename(repetition, started_at)
        folder = os.path.join(label_dir, base)
        try:
            os.makedirs(folder)
        except OSError as e:
            return self._refuse_open(f'não foi possível criar {folder}: {e}')

        r = _Recording(source=source, label=self.label, repetition=repetition, notes=self.notes,
                       image_mode=self.image_mode, folder=folder, base=base,
                       writer=CsvWriter(folder, csv_columns(), self.queue_size),
                       started_at=started_at, t0=now)
        self._rec = r
        self._last_disk_t = now
        # Pré-gravação: as linhas guardadas entram com tempo negativo
        for t_rx, name, t, values in self._pre:
            r.writer.write(name, [r.rel(t), r.rel(t_rx), *values])
        self._pre.clear()

        origem = 'inspeção' if source == SOURCE_INSPECTION else 'goal Center direto'
        self._event('gravador', 'inicio', f'{source} | {r.label} #{repetition:03d}')
        self._snapshot_params(r)
        self.get_logger().info(f'Gravação iniciada ({origem}): {folder}')
        self._publish_status()

    def _refuse_open(self, reason: str):
        self._message = f'Gravação não iniciada: {reason}'
        self.get_logger().warn(self._message)
        self._publish_status()

    def _close(self, force: bool = False, reason: str = ''):
        """Pede o fim da gravação; fecha quando não houver pendências (ou force/reason)."""
        r = self._rec
        if r is None:
            return
        now = self._now()
        if reason:
            r.interrupted = reason
        if r.closing_since is None:
            self._event('gravador', 'fim', reason)
            r.closing_since = now
        if force or reason or r.pending <= 0 or now - r.closing_since >= RESULT_WAIT_S:
            self._finish(r)

    def _finish(self, r: _Recording):
        self._rec = None
        rows = r.writer.close()
        now = self._now()
        ended_at = datetime.now()

        mode = infer_mode(r.servo_mode, r.saw_tracking)
        tracked = r.centered_seen and mode == MODE_TRACK
        result = outcome(r.source, list(r.center_goals.values()), tracked, bool(r.interrupted))
        for node in self.param_nodes:
            r.params.setdefault(node, 'sem resposta')

        folder = os.path.join(os.path.dirname(r.folder),
                              final_basename(r.base, r.equipment, mode))
        try:
            os.rename(r.folder, folder)
        except OSError as e:
            self.get_logger().warn(f'Não foi possível renomear {r.folder}: {e}')
            folder = r.folder

        summary = {
            'rotulo': r.label,
            'repeticao': r.repetition,
            'observacao': r.notes,
            'modo_imagem': r.image_mode,
            'origem': r.source,
            'equipamento': r.equipment,
            'modo': mode,
            'inicio': r.started_at.isoformat(timespec='milliseconds'),
            'inicio_ros_s': r.t0,
            'fim': ended_at.isoformat(timespec='milliseconds'),
            'duracao_s': round(now - r.t0, 3),
            'desfecho': result,
            'mensagem_final': r.last_message,
            'motivo_interrupcao': r.interrupted,
            'readquisicoes': (max(0, r.centering_entries - 1)
                              if r.source == SOURCE_INSPECTION else None),
            'center': list(r.center_goals.values()),
            'scan': list(r.scan_goals.values()),
            'camera': self._camera,
            'parametros': r.params,
            'commit': self.commit,
            'pre_s': self.pre_s,
            'tail_s': self.tail_s,
            'linhas': rows,
            'linhas_descartadas': r.writer.dropped,
            'erro_escrita': r.writer.error,
        }
        path = os.path.join(folder, 'resumo.json')
        try:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(summary, f, ensure_ascii=False, indent=2, default=_json_default)
        except OSError as e:
            self.get_logger().error(f'Falha ao gravar {path}: {e}')

        self._last_folder = folder
        self._next_rep = next_repetition(self._label_dir())
        self._message = (f'Gravado: {r.label}/{os.path.basename(folder)} ({result}, '
                         f'{summary["duracao_s"]:.1f} s)')
        if r.writer.dropped or r.writer.error:
            self._message += (f' | {r.writer.dropped} linhas descartadas'
                              + (f', {r.writer.error}' if r.writer.error else ''))
            self.get_logger().warn(self._message)
        else:
            self.get_logger().info(self._message)
        self._publish_status()

    # ---------------- Configuração ----------------
    def on_configure(self, request, response):
        if self._rec is not None:
            response.success = False
            response.message = (f'Gravação em andamento ({self._rec.label} '
                                f'#{self._rec.repetition:03d}): configure depois do fim')
            response.next_repetition = self._rec.repetition
            self.get_logger().warn(f'Configuração recusada: {response.message}')
            return response
        image_mode = request.image_mode.strip() or self.image_mode
        try:
            check_image_mode(image_mode)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            response.next_repetition = self._next_rep
            self.get_logger().warn(f'Configuração recusada: {response.message}')
            return response

        self.label = clean_label(request.label) or self.default_label
        self.notes = request.notes.strip()
        self.image_mode = image_mode
        self._next_rep = next_repetition(self._label_dir())
        response.success = True
        response.next_repetition = self._next_rep
        response.message = (f'Próxima gravação: {self.label} #{self._next_rep:03d} | '
                            f'imagens: {self.image_mode}'
                            + (f' | obs.: {self.notes}' if self.notes else ''))
        self._message = response.message
        self.get_logger().info(response.message)
        self._publish_status()
        return response

    # ---------------- Supervisão ----------------
    def on_tick(self):
        now = self._now()
        self._apply(self.trigger.tick(now))
        r = self._rec
        if r is not None:
            if r.closing_since is not None:
                self._close()
            elif now - self._last_disk_t >= DISK_CHECK_PERIOD_S:
                self._last_disk_t = now
                free = free_gb(r.folder)
                if free < self.min_free_gb:
                    self._close(reason=f'pouco espaço em disco: {free:.1f} GB livres')
            if r.writer.error and not r.write_error_logged:
                r.write_error_logged = True
                self.get_logger().error(f'Gravação {r.folder}: {r.writer.error}')
        if now - self._last_status_t >= STATUS_PERIOD_S:
            self._publish_status()

    def _publish_status(self):
        now = self._now()
        self._last_status_t = now
        msg = ExperimentStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.image_mode = self.image_mode
        r = self._rec
        if r is None:
            msg.label = self.label
            msg.repetition = self._next_rep
            msg.notes = self.notes
            msg.folder = self._last_folder
            msg.message = self._message
        else:
            msg.recording = True
            msg.label = r.label
            msg.repetition = r.repetition
            msg.notes = r.notes
            msg.image_mode = r.image_mode
            msg.folder = r.folder
            msg.elapsed_s = now - r.t0
            if r.closing_since is not None:
                msg.message = 'Finalizando'
            else:
                origem = 'inspeção' if r.source == SOURCE_INSPECTION else 'goal Center direto'
                msg.message = (f'Gravando {origem}: {r.label} #{r.repetition:03d}'
                               + (' (cauda)' if self.trigger.in_tail else ''))
        self.status_pub.publish(msg)

    def destroy_node(self):
        # Fecha a pasta (desfecho "interrompido") antes de sair
        if self._rec is not None:
            self._close(reason=REASON_SHUTDOWN)
        super().destroy_node()


def main():
    # Sem o tratador de sinais do rclpy, o Ctrl+C vira KeyboardInterrupt com o
    # contexto ainda válido: o nó ainda consegue fechar a gravação
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = ExperimentRecorder()
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
