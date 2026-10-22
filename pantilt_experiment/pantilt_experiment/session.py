"""
Lógica das gravações do experiment_recorder sem dependência do ROS (testável
isoladamente; docs/architecture.md, seção 4.11): rótulo e numeração das
repetições, nomes das pastas e dos snapshots, gatilhos de início e fim,
desfecho, colunas dos CSVs, validação dos parâmetros e commit do repositório.
"""

import os
import re
import shutil
import subprocess
import unicodedata

# Tamanho máximo do rótulo e do equipamento no nome da pasta
MAX_LABEL_LEN = 40

# Modos de imagem aceitos (docs/architecture.md, seção 4.11)
IMAGE_MODE_NONE = 'none'
IMAGE_MODE_SNAPSHOTS = 'snapshots'
IMAGE_MODE_VIDEO = 'video'
IMAGE_MODES = (IMAGE_MODE_NONE, IMAGE_MODE_SNAPSHOTS, IMAGE_MODE_VIDEO)

# Eventos dos snapshots (parte do nome do arquivo NN_<evento>.jpg)
SNAP_START = 'inicio'        # cada entrada em CENTERING ou início do goal direto
SNAP_CENTERED = 'centrado'   # primeiro centered=true da gravação
SNAP_END = 'fim'             # último quadro, ao fechar a gravação

# Estado do inspection_manager fora de inspeção
STATE_IDLE = 'IDLE'

# Origem de uma gravação
SOURCE_INSPECTION = 'inspecao'
SOURCE_DIRECT = 'direto'

# Ações devolvidas pelo gatilho
ACTION_OPEN = 'abrir'
ACTION_CLOSE = 'fechar'

# Desfechos gravados no resumo.json
OUTCOME_CENTERED = 'centralizado'      # um goal Center terminou com sucesso
OUTCOME_TRACKED = 'rastreou'           # TRACK chegou a centered=true
OUTCOME_NO_TARGET = 'sem_alvo'         # inspeção sem nenhum goal Center
OUTCOME_FAILED = 'falhou'              # nenhum dos anteriores
OUTCOME_INTERRUPTED = 'interrompido'   # nó encerrado ou pouco espaço em disco

# Modos no nome da pasta
MODE_CENTER = 'center'
MODE_TRACK = 'track'
MODE_UNKNOWN = 'nd'

# Center.Goal.mode
GOAL_MODE_CENTER = 0
GOAL_MODE_TRACK = 1

# Colunas de cada CSV (sempre precedidas por t e t_rx, em s desde o início)
TIME_COLUMNS = ('t', 't_rx')
COLUMNS = {
    'target': ('detected', 'equipment', 'confidence', 'center_x', 'center_y', 'width',
               'height', 'error_x', 'error_y', 'image_width', 'image_height'),
    'detections': ('index', 'class_name', 'score', 'center_x', 'center_y', 'width', 'height'),
    'timing': ('inference_ms', 'latency_ms', 'n_detections'),
    'servo': ('controller', 'mode', 'target_visible', 'error_x', 'error_y', 'error_px',
              'theta_pan', 'theta_tilt', 'dt', 'gap_reset',
              'p_pan', 'i_pan', 'd_pan', 'p_tilt', 'i_tilt', 'd_tilt',
              'u_raw_pan', 'u_raw_tilt', 'saturated_pan', 'saturated_tilt',
              'cmd_pan', 'cmd_tilt', 'centered', 'convergence_time_s', 'final_error_px'),
    'center_feedback': ('goal', 'error_x', 'error_y', 'error_px', 'target_visible', 'centered'),
    'scan_feedback': ('goal', 'pan_deg', 'tilt_deg', 'pass_index'),
    'joints': ('pan_pos', 'tilt_pos', 'pan_vel', 'tilt_vel'),
    'cmd_auto': ('pan', 'tilt'),
    'cmd_mux': ('pan', 'tilt'),
    'events': ('source', 'event', 'detail'),
    'video_frames': ('frame',),
}


def csv_columns() -> dict:
    """Colunas completas de cada CSV (t, t_rx e as de COLUMNS)."""
    return {name: TIME_COLUMNS + cols for name, cols in COLUMNS.items()}


_INVALID_CHARS = re.compile(r'[^a-z0-9_.-]')
_REPEATED_UNDERSCORE = re.compile(r'_+')
_REPETITION = re.compile(r'^(\d{3,})_')


def clean_label(text: str) -> str:
    """
    Limpa o rótulo para [a-z0-9_.-]: minúsculas, sem acentos, espaços viram "_"
    e o resto é removido. Ex.: "Degrau Kp 0.8" -> "degrau_kp_0.8".
    """
    ascii_text = unicodedata.normalize('NFKD', str(text)).encode('ascii', 'ignore').decode()
    cleaned = _INVALID_CHARS.sub('', '_'.join(ascii_text.lower().split()))
    cleaned = _REPEATED_UNDERSCORE.sub('_', cleaned).strip('_-.')
    return cleaned[:MAX_LABEL_LEN].rstrip('_-.')


def next_repetition(label_dir: str) -> int:
    """Maior NNN entre as pastas NNN_... de label_dir, mais 1 (1 se não houver)."""
    try:
        names = os.listdir(label_dir)
    except OSError:
        return 1
    numbers = [int(m.group(1)) for m in map(_REPETITION.match, names) if m]
    return max(numbers, default=0) + 1


def folder_basename(repetition: int, started_at) -> str:
    """Nome provisório da pasta: NNN_AAAAMMDD_HHMMSS."""
    return f'{repetition:03d}_{started_at.strftime("%Y%m%d_%H%M%S")}'


def final_basename(base: str, equipment: str, mode: str) -> str:
    """Nome final da pasta: <base>_<equipamento>_<modo>."""
    return f'{base}_{clean_label(equipment) or SOURCE_DIRECT}_{mode}'


def snapshot_name(seq: int, event: str) -> str:
    """Nome do snapshot: NN_<evento>.jpg (NN = ordem na gravação, a partir de 1)."""
    return f'{seq:02d}_{event}.jpg'


def infer_mode(servo_mode, saw_tracking: bool) -> str:
    """
    Modo da gravação: pelo Center.Goal.mode visto no /control/servo_debug; sem
    ele, TRACK se o gerenciador passou por TRACKING; senão, desconhecido.
    """
    if servo_mode == GOAL_MODE_CENTER:
        return MODE_CENTER
    if servo_mode == GOAL_MODE_TRACK or saw_tracking:
        return MODE_TRACK
    return MODE_UNKNOWN


def outcome(source: str, center_goals: list, tracked: bool, interrupted: bool) -> str:
    """
    Desfecho de uma gravação. center_goals: dicts com a chave success (True,
    False ou None se o resultado não chegou); tracked: centered=true visto no
    modo TRACK.
    """
    if interrupted:
        return OUTCOME_INTERRUPTED
    if any(g.get('success') for g in center_goals):
        return OUTCOME_CENTERED
    if tracked:
        return OUTCOME_TRACKED
    if source == SOURCE_INSPECTION and not center_goals:
        return OUTCOME_NO_TARGET
    return OUTCOME_FAILED


def validate_params(image_mode: str, pre_s: float, tail_s: float, min_free_gb: float,
                    queue_size: int, jpeg_quality: int, video_fps: float, fourcc: str):
    """Levanta ValueError se algum parâmetro do experiment_recorder for inválido."""
    check_image_mode(image_mode)
    if pre_s < 0.0:
        raise ValueError(f'pre_s={pre_s} inválido: não pode ser negativo')
    if tail_s < 0.0:
        raise ValueError(f'tail_s={tail_s} inválido: não pode ser negativo')
    if min_free_gb < 0.0:
        raise ValueError(f'min_free_gb={min_free_gb} inválido: não pode ser negativo')
    if queue_size < 1:
        raise ValueError(f'queue_size={queue_size} inválido: deve ser pelo menos 1')
    if not 1 <= jpeg_quality <= 100:
        raise ValueError(f'jpeg_quality={jpeg_quality} inválido: deve estar entre 1 e 100')
    if video_fps <= 0.0:
        raise ValueError(f'video_fps={video_fps} inválido: deve ser positivo')
    if len(fourcc) != 4:
        raise ValueError(f'fourcc="{fourcc}" inválido: deve ter 4 caracteres (ex.: "mp4v")')


def check_image_mode(image_mode: str):
    """Levanta ValueError se o modo de imagem for desconhecido."""
    if image_mode not in IMAGE_MODES:
        raise ValueError(f'image_mode="{image_mode}" desconhecido (opções: '
                         f'{", ".join(IMAGE_MODES)})')


def free_gb(path: str) -> float:
    """Espaço livre, em GB, no disco que contém path."""
    return shutil.disk_usage(path).free / 1e9


def git_info(repo_dir: str, timeout_s: float = 10.0):
    """
    Commit, branch e alterações locais (arquivos versionados) do repositório,
    ou None se repo_dir não for um repositório git ou o git falhar.
    """
    def git(*args):
        out = subprocess.run(['git', '-C', repo_dir, *args], capture_output=True, text=True,
                             timeout=timeout_s, check=True)
        return out.stdout.strip()

    try:
        return {
            'commit': git('rev-parse', 'HEAD'),
            'branch': git('rev-parse', '--abbrev-ref', 'HEAD'),
            'alteracoes_locais': bool(git('status', '--porcelain', '--untracked-files=no')),
        }
    except (OSError, subprocess.SubprocessError):
        return None


class RecordingTrigger:
    """
    Decide quando uma gravação abre e fecha (docs/architecture.md, seção 4.11).

    Os métodos recebem os eventos observados e devolvem a lista de ações, na
    ordem: (ACTION_CLOSE,) e/ou (ACTION_OPEN, origem). O nó executa as ações.

      - inspeção: abre quando o estado do gerenciador sai do IDLE e entra na
        cauda quando ele volta ao IDLE;
      - goal Center direto (gerenciador em IDLE): abre quando o goal fica
        ativo e entra na cauda quando ele termina;
      - cauda: tail_s depois do fim, tick() fecha. Uma gravação nova durante a
        cauda (ou um goal direto que substitui outro) fecha a atual na hora.
    """

    def __init__(self, tail_s: float):
        self.tail_s = tail_s
        self.manager_state = STATE_IDLE
        self._clear()

    def _clear(self):
        self.active = False
        self.source = None
        self.goal = None              # goal direto que abriu a gravação
        self.tail_until = None        # instante em que a cauda acaba

    @property
    def in_tail(self) -> bool:
        return self.active and self.tail_until is not None

    def _open(self, source: str, goal=None) -> list:
        actions = [(ACTION_CLOSE,)] if self.active else []
        self._clear()
        self.active = True
        self.source = source
        self.goal = goal
        return actions + [(ACTION_OPEN, source)]

    def inspection_state(self, state: str, now: float) -> list:
        """Estado publicado em /inspection/status (a cada mudança e a 5 Hz)."""
        self.manager_state = state
        if state != STATE_IDLE:
            if self.active and self.source == SOURCE_INSPECTION and not self.in_tail:
                return []
            return self._open(SOURCE_INSPECTION)
        if self.active and self.source == SOURCE_INSPECTION and not self.in_tail:
            self.tail_until = now + self.tail_s
        return []

    def center_goal_active(self, goal, now: float) -> list:
        """Goal Center aceito ou em execução em /control/center/_action/status."""
        if self.manager_state != STATE_IDLE:
            return []                 # pertence à inspeção em andamento
        if self.active and self.source == SOURCE_DIRECT and self.goal == goal:
            return []
        return self._open(SOURCE_DIRECT, goal)

    def center_goal_finished(self, goal, now: float) -> list:
        """Goal Center em estado final (sucesso, cancelado ou abortado)."""
        if (self.active and self.source == SOURCE_DIRECT and self.goal == goal
                and not self.in_tail):
            self.tail_until = now + self.tail_s
        return []

    def tick(self, now: float) -> list:
        """Chamado periodicamente: fecha a gravação quando a cauda acaba."""
        if self.in_tail and now >= self.tail_until:
            self._clear()
            return [(ACTION_CLOSE,)]
        return []

    def stop(self) -> list:
        """Fecha a gravação ativa sem esperar a cauda (ex.: nó encerrado)."""
        if not self.active:
            return []
        self._clear()
        return [(ACTION_CLOSE,)]
