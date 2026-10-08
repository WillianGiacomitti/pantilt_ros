"""
Máquina de estados do inspection_manager sem dependência do ROS (testável
isoladamente): estados, transições e métricas na mensagem de status
(docs/architecture.md, seções 4.3 e 6).

O nó traduz services, tópicos e resultados das actions em chamadas de
InspectionMachine e executa, na ordem, as ações devolvidas (ACTION_*). A
máquina não conhece goal handles: o nó descarta os resultados e feedbacks de
goals que não são mais o atual antes de chamar os eventos.
"""

from dataclasses import dataclass

# Estados: os mesmos textos de InspectionStatus.STATE_*
STATE_IDLE = 'IDLE'
STATE_SEARCHING = 'SEARCHING'
STATE_CENTERING = 'CENTERING'
STATE_TRACKING = 'TRACKING'

# Modos: os mesmos valores de StartInspection.MODE_* e Center.Goal.MODE_*
MODE_CENTER = 0
MODE_TRACK = 1

# Ações que o nó executa
ACTION_SEND_SCAN = 'send_scan'          # goal Scan(speed_deg_s=0, timeout_s=scan_timeout_s)
ACTION_CANCEL_SCAN = 'cancel_scan'
ACTION_SEND_CENTER = 'send_center'      # goal Center com o mode da inspeção e os parâmetros do nó
ACTION_CANCEL_CENTER = 'cancel_center'
ACTION_CLEAR_TARGET = 'clear_target'    # /perception/set_target(""), sem esperar a resposta

# Mensagem do resultado da Center quando o visual_servo_node aborta por perda do
# alvo (visual_servo_node.py, _run). É a única que leva a uma nova varredura
LOST_MESSAGE = 'alvo perdido'

MSG_READY = 'Pronto'
REASON_STOP = 'Inspeção interrompida (/inspection/stop)'
REASON_OPERATOR = 'Inspeção abortada pelo operador'
REASON_SHUTDOWN = 'Nó encerrado'


@dataclass(frozen=True)
class Params:
    """Parâmetros do inspection_manager que a máquina usa."""
    confirm_frames: int = 3
    scan_timeout_s: float = 60.0
    max_reacquire: int = 2


@dataclass(frozen=True)
class Status:
    """Conteúdo do /inspection/status (sem o header)."""
    state: str
    equipment: str
    autonomous: bool
    error_px: float
    message: str


def validate_params(confirm_frames: int, scan_timeout_s: float, max_reacquire: int,
                    tolerance_px: float, hold_time_s: float, lost_timeout_s: float):
    """Levanta ValueError se algum parâmetro do inspection_manager for inválido."""
    if confirm_frames < 1:
        raise ValueError(f'confirm_frames={confirm_frames} inválido: deve ser pelo menos 1')
    if scan_timeout_s <= 0.0:
        raise ValueError(f'scan_timeout_s={scan_timeout_s} inválido: deve ser positivo')
    if max_reacquire < 0:
        raise ValueError(f'max_reacquire={max_reacquire} inválido: não pode ser negativo')
    # Mesmas regras do visual_servo_node para o goal Center: um valor fora delas
    # faria toda centralização ser recusada
    if tolerance_px <= 0.0:
        raise ValueError(f'tolerance_px={tolerance_px} inválido: deve ser positivo')
    if hold_time_s < 0.0:
        raise ValueError(f'hold_time_s={hold_time_s} inválido: não pode ser negativo')
    if lost_timeout_s <= 0.0:
        raise ValueError(f'lost_timeout_s={lost_timeout_s} inválido: deve ser positivo')


class InspectionMachine:
    """
    Estados IDLE → SEARCHING → CENTERING → (TRACKING) → IDLE.

    Cada evento devolve a lista de ações a executar (vazia se nada muda no
    mundo). Eventos fora do estado em que fazem sentido são ignorados.
    """

    def __init__(self, params: Params, equipment_keys):
        self.params = params
        self.equipment_keys = list(equipment_keys)
        self.state = STATE_IDLE
        self.equipment = ''
        self.mode = MODE_CENTER
        self.error_px = 0.0
        self.message = MSG_READY
        self.scan_active = False         # goal Scan enviado e ainda sem resultado
        self.center_active = False       # goal Center enviado e ainda sem resultado
        self.losses = 0                  # perdas consecutivas do alvo
        self._confirm_count = 0
        self._since = 0.0                # stamp (s) da resposta do set_target

    # ---------------- Consulta ----------------
    @property
    def active(self) -> bool:
        return self.state != STATE_IDLE

    def status(self) -> Status:
        return Status(self.state, self.equipment, self.active, self.error_px, self.message)

    # ---------------- Início ----------------
    def check_start(self, equipment: str, mode: int, operator_active: bool,
                    unavailable=()) -> str:
        """
        Motivo da recusa de um /inspection/start, ou '' se pode iniciar.
        unavailable: textos dos servidores fora do ar (ex.: "scan_node (/control/scan)").
        """
        if self.active:
            return f'Inspeção em andamento ({self.state}): use /inspection/stop antes'
        if equipment not in self.equipment_keys:
            disponiveis = ', '.join(self.equipment_keys) or 'nenhum'
            return f'Equipamento "{equipment}" desconhecido. Disponíveis: {disponiveis}'
        if mode not in (MODE_CENTER, MODE_TRACK):
            return f'mode={mode} inválido (0 = CENTER, 1 = TRACK)'
        if operator_active:
            return 'Operador no controle (comando recente da web): aguarde ~1 s e tente de novo'
        if unavailable:
            return f'Indisponível: {"; ".join(unavailable)}'
        return ''

    def start(self, equipment: str, mode: int, since: float) -> list:
        """
        Inicia a inspeção, depois de check_start sem motivo e do set_target aceito.
        since: stamp (s) da resposta do set_target; alvos anteriores não confirmam.
        """
        self.equipment = equipment
        self.mode = mode
        self.losses = 0
        self._since = since
        self.message = 'Varrendo'
        return self._enter_searching()

    def _enter_searching(self) -> list:
        self.state = STATE_SEARCHING
        self.error_px = 0.0
        self._confirm_count = 0
        self.scan_active = True
        return [ACTION_SEND_SCAN]

    # ---------------- SEARCHING ----------------
    def target(self, detected: bool, equipment: str, stamp: float) -> list:
        """Mensagem de /perception/target (stamp do header, em s)."""
        if self.state != STATE_SEARCHING:
            return []
        if equipment != self.equipment or stamp <= self._since:
            # Filtro anterior ou quadro de antes do set_target: não conta nem zera
            return []
        if not detected:
            self._confirm_count = 0
            return []
        self._confirm_count += 1
        if self._confirm_count < self.params.confirm_frames:
            return []
        self.state = STATE_CENTERING
        self.scan_active = False
        self.center_active = True
        if self.losses:
            self.message = (f'Centralizando (readquisição '
                            f'{self.losses}/{self.params.max_reacquire})')
        else:
            self.message = 'Centralizando'
        return [ACTION_CANCEL_SCAN, ACTION_SEND_CENTER]

    def scan_rejected(self) -> list:
        if self.state != STATE_SEARCHING:
            return []
        self.scan_active = False
        return self._go_idle('Varredura recusada pelo scan_node')

    def scan_finished(self, succeeded: bool, completed: bool, message: str = '') -> list:
        """
        Resultado do goal Scan atual. succeeded: status SUCCEEDED (padrão completo
        ou tempo esgotado); senão abortado ou cancelado por outro cliente.
        """
        if self.state != STATE_SEARCHING:
            return []
        self.scan_active = False
        if not succeeded:
            return self._go_idle(f'Varredura interrompida: {message or "sem mensagem"}')
        if completed:
            return self._go_idle('Alvo não encontrado (varredura completa)')
        return self._go_idle(f'Alvo não encontrado em {self.params.scan_timeout_s:.0f} s')

    # ---------------- CENTERING / TRACKING ----------------
    def _centering(self) -> bool:
        return self.state in (STATE_CENTERING, STATE_TRACKING)

    def center_rejected(self) -> list:
        if not self._centering():
            return []
        self.center_active = False
        return self._go_idle('Centralização recusada pelo visual_servo_node '
                             '(veja o log dele; sem /camera/camera_info?)')

    def center_feedback(self, error_px: float, centered: bool) -> list:
        if not self._centering():
            return []
        self.error_px = error_px
        if centered:
            self.losses = 0
            if self.mode == MODE_TRACK and self.state == STATE_CENTERING:
                self.state = STATE_TRACKING
                self.message = 'Rastreando'
        return []

    def center_finished(self, succeeded: bool, message: str = '',
                        t_c: float = 0.0, e_r: float = 0.0) -> list:
        """Resultado do goal Center atual (succeeded: status SUCCEEDED)."""
        if not self._centering():
            return []
        self.center_active = False
        if succeeded:
            return self._go_idle(f'Centralizado em {t_c:.2f} s, erro residual {e_r:.1f} px')
        if message != LOST_MESSAGE:
            return self._go_idle(f'Centralização abortada: {message or "sem mensagem"}')
        self.losses += 1
        maximo = self.params.max_reacquire
        if self.losses > maximo:
            return self._go_idle(
                f'Alvo perdido {self.losses} vezes seguidas (máximo de readquisições: {maximo})')
        self.message = f'Alvo perdido: nova varredura (readquisição {self.losses}/{maximo})'
        return self._enter_searching()

    # ---------------- Saídas para o IDLE ----------------
    def stop(self) -> list:
        return self.abort(REASON_STOP)

    def operator(self) -> list:
        return self.abort(REASON_OPERATOR)

    def server_lost(self, name: str) -> list:
        return self.abort(f'{name} saiu do ar')

    def shutdown(self) -> list:
        return self.abort(REASON_SHUTDOWN)

    def abort(self, reason: str) -> list:
        """Interrompe a inspeção de qualquer estado; em IDLE, nada muda."""
        if not self.active:
            return []
        return self._go_idle(reason)

    def _go_idle(self, reason: str) -> list:
        actions = []
        if self.scan_active:
            actions.append(ACTION_CANCEL_SCAN)
        if self.center_active:
            actions.append(ACTION_CANCEL_CENTER)
        actions.append(ACTION_CLEAR_TARGET)
        self.state = STATE_IDLE
        self.equipment = ''
        self.error_px = 0.0
        self.scan_active = False
        self.center_active = False
        self._confirm_count = 0
        self.message = reason
        return actions
