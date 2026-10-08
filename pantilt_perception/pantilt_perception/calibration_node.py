#!/usr/bin/env python3
"""
Calibração da distância focal por rotação (docs/architecture.md, seção 4.10):
gira a câmera em ângulos conhecidos, mede o deslocamento do alvo na imagem e
grava os intrínsecos no arquivo que o camera_node publica em /camera/camera_info.
O modelo e o ajuste estão em calibration_math.py.

Services:
  /calibration/run    (std_srvs/Trigger)  - executa a rotina e grava o arquivo; responde ao final
  /calibration/abort  (std_srvs/Trigger)  - interrompe a rotina e devolve os eixos ao home

Tópicos:
  assina   /perception/target   (pantilt_interfaces/VisualTarget)  - erro do alvo em px
  assina   /joint_states        (sensor_msgs/JointState)           - pan_joint e tilt_joint (rad, rad/s)
  assina   /ptu/control_source  (std_msgs/String)                  - aborta se virar "web"
  assina   /camera/camera_info  (sensor_msgs/CameraInfo)           - só para o log (calibração anterior)
  publica  /ptu/cmd_pos_auto    (sensor_msgs/JointState)           - posições, via command_mux

Uso (o operador antes centraliza o alvo pela página e zera os eixos):
  ros2 service call /perception/set_target pantilt_interfaces/srv/SetTarget "{equipment: 'garrafa'}"
  ros2 service call /calibration/run std_srvs/srv/Trigger

Rotina: sondagem de ±probe_deg no pan para um f grosseiro; grade em cruz (pan
com o tilt no home, depois tilt com o pan no home); em cada ponto, espera a
junta parar e coleta alvos com header.stamp posterior à parada, por causa do
atraso da imagem. No fim, volta ao home, ajusta os dois eixos e grava.

Segurança:
  - publica apenas posições, nunca velocidade: um zero de velocidade logo
    depois de uma posição interromperia o movimento no firmware;
  - se o operador assumir (control_source = web) ou o nó for encerrado
    (Ctrl+C), para de comandar e NÃO volta ao home: nada se move depois disso;
  - nas demais saídas (sucesso, abort, erro, pontos insuficientes), volta ao home.

Parâmetros: ver declare_parameter abaixo e pantilt_bringup/config/params.yaml.
"""

import math
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pantilt_interfaces.msg import VisualTarget
from pantilt_perception.calibration_math import (
    DetectorParada, ajustar_eixo, alvo_centralizado, fov_deg, grade_eixo, k_sondagem,
    validar_parametros,
)
from pantilt_perception.camera_info_file import make_intrinsics, save_camera_info

# Idade máxima do último alvo para a pré-condição (stamp da captura)
TARGET_MAX_AGE_S = 1.0

# Distância máxima de zero dos eixos na pré-condição (o operador usou /ptu/set_zero)
HOME_TOL_DEG = 1.0

# Velocidade abaixo da qual a junta é considerada parada
STOPPED_VEL_DEG_S = 0.5

# Tempo parado no alvo para confirmar a parada
SETTLE_HOLD_S = 0.15

# Tempo máximo para uma junta chegar a um ponto e parar
MOVE_TIMEOUT_S = 10.0

# Tempo máximo para coletar as amostras de um ponto (o detector roda a ~10 Hz)
SAMPLE_TIMEOUT_S = 3.0

# Idade máxima da última posição recebida em /joint_states
JOINT_STATES_TIMEOUT_S = 0.5

# Faixa plausível do f grosseiro da sondagem (px); fora dela, alvo ou encoder com problema
F_MIN_PX = 100.0
F_MAX_PX = 5000.0

# Fração da meia-dimensão do quadro alcançada pelo maior ângulo da grade
GRID_FRACTION = 0.35

# Fração da meia-dimensão do quadro aceita como "alvo centralizado"
CENTER_FRACTION = 0.2

# Período dos laços de espera da rotina
POLL_S = 0.02

# Alvos guardados para a coleta (~10 s a 10 Hz)
TARGET_BUFFER = 100

# Espera máxima pelo fim da rotina ao encerrar o nó
SHUTDOWN_WAIT_S = 2.0

# Timeout de cada volta do executor; limita a demora em atender o Ctrl+C
SPIN_TIMEOUT_S = 0.1

PAN_JOINT = 'pan_joint'
TILT_JOINT = 'tilt_joint'
PAN = 'pan'
TILT = 'tilt'
SOURCE_WEB = 'web'


class CalibracaoInterrompida(Exception):
    """Interrompe a rotina. voltar_home=False quando nada mais deve se mover."""

    def __init__(self, motivo: str, voltar_home: bool):
        super().__init__(motivo)
        self.motivo = motivo
        self.voltar_home = voltar_home


@dataclass(frozen=True)
class Junta:
    """Última leitura de /joint_states, em graus e graus/s."""
    pan: float
    tilt: float
    pan_vel: float
    tilt_vel: float
    recebido: float     # instante de chegada (monotônico)


@dataclass(frozen=True)
class Ponto:
    """Medida de um ponto da grade: medianas do ângulo do eixo e do erro."""
    theta_deg: float
    erro_px: float
    amostras: int


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        self.declare_parameter('probe_deg', 3.0)
        self.declare_parameter('max_angle_deg', 10.0)
        self.declare_parameter('n_points', 7)
        self.declare_parameter('samples_per_point', 5)
        self.declare_parameter('settle_tol_deg', 0.2)
        self.declare_parameter('settle_extra_s', 0.4)
        self.declare_parameter('min_points', 4)
        self.declare_parameter(
            'output_file', '/ros2_ws/src/pantilt_ros/pantilt_bringup/config/camera_intrinsics.yaml')
        self.declare_parameter('camera_name', 'pantilt_cam')

        self.probe = float(self.get_parameter('probe_deg').value)
        self.max_angle = float(self.get_parameter('max_angle_deg').value)
        self.n_points = int(self.get_parameter('n_points').value)
        self.samples = int(self.get_parameter('samples_per_point').value)
        self.settle_tol = float(self.get_parameter('settle_tol_deg').value)
        self.settle_extra = float(self.get_parameter('settle_extra_s').value)
        self.min_points = int(self.get_parameter('min_points').value)
        self.output_file = self.get_parameter('output_file').value
        self.camera_name = self.get_parameter('camera_name').value

        try:
            validar_parametros(self.probe, self.max_angle, self.n_points, self.samples,
                               self.settle_tol, self.settle_extra, self.min_points)
        except ValueError as e:
            self.get_logger().fatal(f'Parâmetro inválido: {e}')
            raise

        self.get_logger().info(
            f'Sondagem ±{self.probe:.1f}° | grade de {self.n_points} pontos até '
            f'±{self.max_angle:.1f}° | {self.samples} amostras por ponto | parada em '
            f'±{self.settle_tol:.2f}° + {self.settle_extra:.2f} s | mínimo {self.min_points} '
            f'pontos | saída {self.output_file}')

        # Dados das assinaturas, protegidos por _data_lock
        self._data_lock = threading.Lock()
        self._junta = None
        self._alvos = deque(maxlen=TARGET_BUFFER)
        self._fonte = None
        self._camera_info = None

        # Estado da rotina
        self._run_lock = threading.Lock()
        self._running = False
        self._abort_event = threading.Event()
        self._stop_event = threading.Event()
        self._idle = threading.Event()
        self._idle.set()

        self.pos_pub = self.create_publisher(JointState, '/ptu/cmd_pos_auto', 10)

        # Mesmo QoS do publisher do command_mux: recebe a fonte atual ao conectar
        source_qos = QoSProfile(depth=1)
        source_qos.reliability = QoSReliabilityPolicy.RELIABLE
        source_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(VisualTarget, '/perception/target', self.on_target, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.create_subscription(String, '/ptu/control_source', self.on_control_source, source_qos)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self.on_camera_info, qos_profile_sensor_data)

        # Reentrante: o run bloqueia durante a rotina, e o abort precisa ser
        # atendido nesse intervalo; as assinaturas ficam no grupo padrão
        services = ReentrantCallbackGroup()
        self.create_service(Trigger, '/calibration/run', self.on_run, callback_group=services)
        self.create_service(Trigger, '/calibration/abort', self.on_abort, callback_group=services)

    # ---------------- Entradas ----------------
    def on_target(self, msg: VisualTarget):
        with self._data_lock:
            self._alvos.append(msg)

    def on_joint_states(self, msg: JointState):
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        if PAN_JOINT not in positions or TILT_JOINT not in positions:
            self.get_logger().warn(
                f'/joint_states sem {PAN_JOINT}/{TILT_JOINT}: {list(msg.name)}',
                throttle_duration_sec=5.0)
            return
        junta = Junta(
            pan=math.degrees(positions[PAN_JOINT]),
            tilt=math.degrees(positions[TILT_JOINT]),
            pan_vel=math.degrees(velocities.get(PAN_JOINT, 0.0)),
            tilt_vel=math.degrees(velocities.get(TILT_JOINT, 0.0)),
            recebido=time.monotonic(),
        )
        with self._data_lock:
            self._junta = junta

    def on_control_source(self, msg: String):
        with self._data_lock:
            self._fonte = msg.data

    def on_camera_info(self, msg: CameraInfo):
        with self._data_lock:
            self._camera_info = msg

    def _ultima_junta(self):
        """Última posição, ou None se ausente ou mais velha que JOINT_STATES_TIMEOUT_S."""
        with self._data_lock:
            junta = self._junta
        if junta is None or time.monotonic() - junta.recebido > JOINT_STATES_TIMEOUT_S:
            return None
        return junta

    def _ultimo_alvo(self):
        with self._data_lock:
            return self._alvos[-1] if self._alvos else None

    def _alvos_detectados_apos(self, stamp_ns: int) -> list:
        """Alvos detectados com header.stamp posterior a stamp_ns, em ordem de chegada."""
        with self._data_lock:
            alvos = list(self._alvos)
        return [a for a in alvos if a.detected and _stamp_ns(a) > stamp_ns]

    def _fonte_atual(self):
        with self._data_lock:
            return self._fonte

    # ---------------- Services ----------------
    def on_run(self, request, response):
        with self._run_lock:
            if self._running:
                response.success = False
                response.message = 'Calibração já em andamento'
                return response
            self._running = True
            self._abort_event.clear()
            self._idle.clear()
        try:
            motivo = self._checar_precondicoes()
            if motivo:
                self.get_logger().warn(f'Calibração recusada: {motivo}')
                response.success, response.message = False, f'Recusada: {motivo}'
            else:
                response.success, response.message = self._executar()
        finally:
            with self._run_lock:
                self._running = False
            self._idle.set()
        return response

    def on_abort(self, request, response):
        with self._run_lock:
            running = self._running
        if not running:
            response.success = False
            response.message = 'Nenhuma calibração em andamento'
            return response
        self._abort_event.set()
        self.get_logger().warn('Abort pedido: interrompendo e voltando ao home')
        response.success = True
        response.message = 'Abortando: os eixos voltam ao home'
        return response

    def _checar_precondicoes(self) -> str:
        """Retorna o motivo da recusa, ou '' se a rotina pode começar."""
        if self._fonte_atual() == SOURCE_WEB:
            return 'operador no controle (control_source = web); solte o jog e tente de novo'

        junta = self._ultima_junta()
        if junta is None:
            return 'sem /joint_states recente: o hardware.launch.py está rodando?'
        if abs(junta.pan) > HOME_TOL_DEG or abs(junta.tilt) > HOME_TOL_DEG:
            return (f'eixos fora de zero (pan {junta.pan:.1f}°, tilt {junta.tilt:.1f}°; '
                    f'máximo ±{HOME_TOL_DEG:.0f}°): centralize o alvo pelo jog e use /ptu/set_zero')
        if abs(junta.pan_vel) >= STOPPED_VEL_DEG_S or abs(junta.tilt_vel) >= STOPPED_VEL_DEG_S:
            return 'eixos em movimento'

        alvo = self._ultimo_alvo()
        if alvo is None:
            return 'nenhum /perception/target recebido: o filtro foi definido (/perception/set_target)?'
        idade = (self.get_clock().now().nanoseconds - _stamp_ns(alvo)) / 1e9
        if idade > TARGET_MAX_AGE_S:
            return (f'último /perception/target tem {idade:.1f} s: o detector está rodando '
                    'e o filtro está definido?')
        if not alvo.detected:
            return f'alvo "{alvo.equipment}" não detectado no quadro atual'
        if not alvo_centralizado(alvo.error_x, alvo.error_y, alvo.image_width,
                                 alvo.image_height, CENTER_FRACTION):
            return (f'alvo fora do centro (erro {alvo.error_x:+.0f}, {alvo.error_y:+.0f} px; '
                    f'máximo ±{CENTER_FRACTION * alvo.image_width / 2:.0f}, '
                    f'±{CENTER_FRACTION * alvo.image_height / 2:.0f} px): centralize pelo jog')
        return ''

    # ---------------- Rotina ----------------
    def _executar(self):
        """Executa a rotina completa. Retorna (sucesso, mensagem)."""
        junta = self._ultima_junta()
        alvo = self._ultimo_alvo()
        home = (junta.pan, junta.tilt)
        largura, altura = alvo.image_width, alvo.image_height
        self._log_calibracao_anterior(largura, altura)
        self.get_logger().info(
            f'Calibração iniciada: home pan {home[0]:.2f}°, tilt {home[1]:.2f}° | '
            f'alvo "{alvo.equipment}" | quadro {largura}x{altura}')

        pontos = {PAN: [], TILT: []}
        erro = ''
        voltar_home = True
        try:
            f_grosseiro = self._sondar(home)
            grades = {
                PAN: grade_eixo(f_grosseiro, largura / 2.0, self.n_points, self.max_angle,
                                GRID_FRACTION),
                TILT: grade_eixo(f_grosseiro, altura / 2.0, self.n_points, self.max_angle,
                                 GRID_FRACTION),
            }
            for eixo in (PAN, TILT):
                self.get_logger().info(
                    f'Grade do {eixo}: ' + ', '.join(f'{g:+.2f}°' for g in grades[eixo]))
                for offset in grades[eixo]:
                    if eixo == PAN:
                        destino = (home[0] + offset, home[1])
                    else:
                        destino = (home[0], home[1] + offset)
                    ponto = self._medir(eixo, destino)
                    if ponto is not None:
                        pontos[eixo].append(ponto)
            for eixo in (PAN, TILT):
                if len(pontos[eixo]) < self.min_points:
                    raise CalibracaoInterrompida(
                        f'só {len(pontos[eixo])} pontos válidos no {eixo} '
                        f'(mínimo {self.min_points}): o alvo saiu do quadro ou não foi detectado?',
                        voltar_home=True)
        except CalibracaoInterrompida as e:
            erro, voltar_home = e.motivo, e.voltar_home
        except Exception as e:
            # Qualquer falha inesperada ainda devolve os eixos ao home
            self.get_logger().error(f'Erro inesperado na calibração: {e!r}')
            erro, voltar_home = f'erro interno: {e}', True
        finally:
            if voltar_home:
                self._voltar_home(home)
            else:
                self.get_logger().warn('Os eixos não serão movidos: ficam onde estão')

        if erro:
            destino = 'eixos de volta ao home' if voltar_home else 'eixos não foram movidos de volta'
            self.get_logger().error(f'Calibração interrompida: {erro} ({destino})')
            return False, f'Calibração interrompida: {erro} ({destino})'
        return self._ajustar_e_gravar(pontos, largura, altura)

    def _sondar(self, home) -> float:
        """Move o pan ±probe_deg e devolve o f grosseiro (px)."""
        medidas = []
        for offset in (-self.probe, self.probe):
            ponto = self._medir(PAN, (home[0] + offset, home[1]))
            if ponto is None:
                raise CalibracaoInterrompida(
                    'alvo não detectado na sondagem: confira o filtro e a iluminação',
                    voltar_home=True)
            medidas.append(ponto)
        a, b = medidas
        try:
            f_grosseiro = abs(k_sondagem(a.theta_deg, a.erro_px, b.theta_deg, b.erro_px, home[0]))
        except ValueError as e:
            raise CalibracaoInterrompida(f'sondagem inválida: {e}', voltar_home=True) from e
        self.get_logger().info(f'Sondagem: f grosseiro = {f_grosseiro:.0f} px')
        if not F_MIN_PX <= f_grosseiro <= F_MAX_PX:
            raise CalibracaoInterrompida(
                f'f grosseiro de {f_grosseiro:.0f} px fora de [{F_MIN_PX:.0f}, {F_MAX_PX:.0f}]: '
                'o alvo se moveu ou o encoder não acompanha o eixo?', voltar_home=True)
        return f_grosseiro

    def _medir(self, eixo: str, destino):
        """Leva os eixos ao destino e mede o ponto. Retorna Ponto, ou None sem detecção."""
        parada_ns = self._mover(destino, permitir_abort=True)
        self._esperar(self.settle_extra)

        angulos = []
        inicio = time.monotonic()
        alvos = []
        while time.monotonic() - inicio < SAMPLE_TIMEOUT_S:
            self._verificar(permitir_abort=True)
            junta = self._junta_ou_interrompe()
            angulos.append(junta.pan if eixo == PAN else junta.tilt)
            alvos = self._alvos_detectados_apos(parada_ns)
            if len(alvos) >= self.samples:
                break
            self._stop_event.wait(POLL_S)

        comandado = destino[0] if eixo == PAN else destino[1]
        if len(alvos) < self.samples:
            self.get_logger().warn(
                f'{eixo} {comandado:+.2f}°: só {len(alvos)} de {self.samples} detecções em '
                f'{SAMPLE_TIMEOUT_S:.0f} s; ponto descartado')
            return None

        erros = [a.error_x if eixo == PAN else a.error_y for a in alvos[:self.samples]]
        ponto = Ponto(statistics.median(angulos), statistics.median(erros), len(erros))
        self.get_logger().info(
            f'{eixo} {comandado:+.2f}° (medido {ponto.theta_deg:+.2f}°): '
            f'erro {ponto.erro_px:+.1f} px ({ponto.amostras} amostras)')
        return ponto

    def _mover(self, destino, permitir_abort: bool) -> int:
        """
        Publica a posição e espera as duas juntas pararem nela. Retorna o
        instante da parada (ns, relógio do nó, comparável ao header.stamp).
        """
        msg = JointState()
        msg.name = [PAN_JOINT, TILT_JOINT]
        msg.position = [math.radians(destino[0]), math.radians(destino[1])]
        self.pos_pub.publish(msg)

        detectores = [DetectorParada(self.settle_tol, STOPPED_VEL_DEG_S, SETTLE_HOLD_S)
                      for _ in range(2)]
        inicio = time.monotonic()
        while True:
            self._verificar(permitir_abort)
            junta = self._junta_ou_interrompe()
            now = time.monotonic()
            pan_ok = detectores[0].update(now, junta.pan, junta.pan_vel, destino[0])
            tilt_ok = detectores[1].update(now, junta.tilt, junta.tilt_vel, destino[1])
            if pan_ok is not None and tilt_ok is not None:
                return self.get_clock().now().nanoseconds
            if now - inicio > MOVE_TIMEOUT_S:
                raise CalibracaoInterrompida(
                    self._motivo_sem_parada(detectores, destino, now), voltar_home=True)
            self._stop_event.wait(POLL_S)

    def _motivo_sem_parada(self, detectores, destino, now) -> str:
        partes = []
        for nome, det, alvo in zip((PAN, TILT), detectores, destino):
            if abs(det.posicao - alvo) < self.settle_tol and abs(det.velocidade) < STOPPED_VEL_DEG_S:
                continue
            if det.parado_fora_do_alvo(now):
                partes.append(
                    f'o {nome} parou em {det.posicao:.2f}°, a {abs(det.posicao - alvo):.2f}° do '
                    f'alvo {alvo:.2f}° (tolerância {self.settle_tol:.2f}°): aumente settle_tol_deg')
            else:
                partes.append(
                    f'o {nome} não parou em {alvo:.2f}° em {MOVE_TIMEOUT_S:.0f} s (posição '
                    f'{det.posicao:.2f}°, velocidade {det.velocidade:.2f}°/s)')
        return '; '.join(partes) or f'eixos não estabilizaram em {MOVE_TIMEOUT_S:.0f} s'

    def _voltar_home(self, home):
        """Leva os eixos ao home. Só o operador e o encerramento interrompem a volta."""
        self.get_logger().info(f'Voltando ao home (pan {home[0]:.2f}°, tilt {home[1]:.2f}°)')
        try:
            self._mover(home, permitir_abort=False)
            self.get_logger().info('Eixos no home')
        except CalibracaoInterrompida as e:
            self.get_logger().error(f'Não foi possível voltar ao home: {e.motivo}')

    def _esperar(self, duracao: float):
        fim = time.monotonic() + duracao
        while True:
            self._verificar(permitir_abort=True)
            restante = fim - time.monotonic()
            if restante <= 0.0:
                return
            self._stop_event.wait(min(POLL_S, restante))

    def _verificar(self, permitir_abort: bool):
        """Lança CalibracaoInterrompida se a rotina deve parar."""
        if self._stop_event.is_set():
            raise CalibracaoInterrompida('nó encerrado', voltar_home=False)
        if self._fonte_atual() == SOURCE_WEB:
            raise CalibracaoInterrompida(
                'operador assumiu o controle (control_source = web)', voltar_home=False)
        if permitir_abort and self._abort_event.is_set():
            raise CalibracaoInterrompida('abortada por /calibration/abort', voltar_home=True)

    def _junta_ou_interrompe(self) -> Junta:
        junta = self._ultima_junta()
        if junta is None:
            # Sem posição conhecida, mover de volta seria às cegas
            raise CalibracaoInterrompida(
                f'sem /joint_states há mais de {JOINT_STATES_TIMEOUT_S} s', voltar_home=False)
        return junta

    # ---------------- Resultado ----------------
    def _ajustar_e_gravar(self, pontos, largura: int, altura: int):
        ajustes = {}
        for eixo in (PAN, TILT):
            theta = [p.theta_deg for p in pontos[eixo]]
            erros = [p.erro_px for p in pontos[eixo]]
            try:
                ajustes[eixo] = ajustar_eixo(theta, erros)
            except ValueError as e:
                self.get_logger().error(f'Ajuste do {eixo} falhou: {e}')
                return False, f'Ajuste do {eixo} falhou: {e}'
            ajuste = ajustes[eixo]
            for p, residuo in zip(pontos[eixo], ajuste.residuos):
                self.get_logger().info(
                    f'  {eixo} {p.theta_deg:+7.2f}°  erro {p.erro_px:+7.1f} px  '
                    f'resíduo {residuo:+5.2f} px')

        pan, tilt = ajustes[PAN], ajustes[TILT]
        fov_h, fov_v = fov_deg(pan.f, largura), fov_deg(tilt.f, altura)
        linhas = [
            f'pan:  fx = {pan.f:.1f} px | RMS {pan.rms:.2f} px | {pan.n} pontos | '
            f'FOV_h {fov_h:.1f}° | {_sentido(PAN, pan.k)}',
            f'tilt: fy = {tilt.f:.1f} px | RMS {tilt.rms:.2f} px | {tilt.n} pontos | '
            f'FOV_v {fov_v:.1f}° | {_sentido(TILT, tilt.k)}',
        ]
        comentario = '\n'.join([
            f'Gerado pelo calibration_node em {datetime.now():%Y-%m-%d %H:%M:%S}',
            *linhas,
            'cx e cy fixos no centro geométrico (docs/architecture.md, seção 4.10)',
        ])
        intrinsics = make_intrinsics(largura, altura, self.camera_name, pan.f, tilt.f,
                                     largura / 2.0, altura / 2.0)
        try:
            save_camera_info(self.output_file, intrinsics, comentario)
        except ValueError as e:
            self.get_logger().error(f'Falha ao gravar a calibração: {e}')
            return False, f'Ajuste feito, mas a gravação falhou: {e}\n' + '\n'.join(linhas)

        for linha in linhas:
            self.get_logger().info(linha)
        self.get_logger().info(f'Calibração gravada em {self.output_file}')
        mensagem = '\n'.join([
            f'Calibração gravada em {self.output_file}',
            *linhas,
            f'cx = {largura / 2.0:.1f}, cy = {altura / 2.0:.1f} (centro). '
            'Reinicie o camera_node para publicar a K nova.',
        ])
        return True, mensagem

    def _log_calibracao_anterior(self, largura: int, altura: int):
        with self._data_lock:
            info = self._camera_info
        if info is None:
            self.get_logger().info('Sem /camera/camera_info: calibração anterior desconhecida')
        elif info.k[0] <= 0.0:
            self.get_logger().info('Calibração anterior: nenhuma (K zerada)')
        else:
            self.get_logger().info(
                f'Calibração anterior: fx={info.k[0]:.1f}, fy={info.k[4]:.1f} px')
        if info is not None and (info.width, info.height) != (largura, altura):
            self.get_logger().warn(
                f'/camera/camera_info é de {info.width}x{info.height}, mas o alvo é '
                f'reportado em {largura}x{altura}')

    def destroy_node(self):
        # A rotina vê o evento e sai sem mover mais nada
        self._stop_event.set()
        if not self._idle.wait(SHUTDOWN_WAIT_S):
            self.get_logger().warn('Encerrando: a rotina de calibração não terminou a tempo')
        super().destroy_node()


def _stamp_ns(msg) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def _sentido(eixo: str, k: float) -> str:
    """Para que lado o alvo anda na imagem com o eixo no sentido positivo."""
    if eixo == PAN:
        lado = 'a direita' if k > 0 else 'a esquerda'
    else:
        lado = 'baixo' if k > 0 else 'cima'
    return f'{eixo}+ desloca o alvo para {lado} na imagem'


def main():
    # Sem o tratador de sinais do rclpy, o Ctrl+C vira KeyboardInterrupt com o
    # contexto ainda válido, e a rotina sai pelo caminho normal
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = CalibrationNode()
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
