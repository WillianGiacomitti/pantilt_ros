"""
Lógica da gravação sem dependência do ROS nem do OpenCV (testável isoladamente):
nome da sessão e do arquivo, validação dos parâmetros, espaço em disco e
metadados do .json (docs/architecture.md, seção 4.9).
"""

import os
import re
import shutil
import unicodedata

# Tamanho máximo do nome da sessão no arquivo
MAX_SESSION_LEN = 40

_INVALID_CHARS = re.compile(r'[^a-z0-9_-]')
_REPEATED_UNDERSCORE = re.compile(r'_+')


def clean_session(text: str) -> str:
    """
    Limpa o nome da sessão para [a-z0-9_-]: minúsculas, sem acentos, espaços
    viram "_" e o resto é removido. Ex.: "Mesa Janela (tarde)" -> "mesa_janela_tarde".
    """
    ascii_text = unicodedata.normalize('NFKD', str(text)).encode('ascii', 'ignore').decode()
    words = ascii_text.lower().split()
    cleaned = _INVALID_CHARS.sub('', '_'.join(words))
    cleaned = _REPEATED_UNDERSCORE.sub('_', cleaned).strip('_-')
    return cleaned[:MAX_SESSION_LEN].rstrip('_-')


def video_basename(start, session: str) -> str:
    """Nome do arquivo sem extensão: AAAAMMDD_HHMMSS_<sessao>, ou só a data sem sessão."""
    stamp = start.strftime('%Y%m%d_%H%M%S')
    return f'{stamp}_{session}' if session else stamp


def unique_path(directory: str, base: str, ext: str) -> str:
    """Caminho em directory que ainda não existe, acrescentando _1, _2... se preciso."""
    path = os.path.join(directory, base + ext)
    n = 1
    while os.path.exists(path):
        path = os.path.join(directory, f'{base}_{n}{ext}')
        n += 1
    return path


def validate_params(record_fps: float, fourcc: str, min_free_gb: float, queue_size: int):
    """Levanta ValueError se algum parâmetro do capture_node for inválido."""
    if record_fps <= 0.0:
        raise ValueError(f'record_fps={record_fps} inválido: deve ser positivo')
    if len(fourcc) != 4:
        raise ValueError(f'fourcc="{fourcc}" inválido: deve ter 4 caracteres (ex.: "mp4v")')
    if min_free_gb < 0.0:
        raise ValueError(f'min_free_gb={min_free_gb} inválido: não pode ser negativo')
    if queue_size < 1:
        raise ValueError(f'queue_size={queue_size} inválido: deve ser pelo menos 1')


def free_gb(path: str) -> float:
    """Espaço livre, em GB, no disco que contém path."""
    return shutil.disk_usage(path).free / 1e9


def build_metadata(session: str, file: str, started_at, ended_at, frames: int, dropped: int,
                   record_fps: float, width: int, height: int, fourcc: str, size_bytes: int,
                   scan: dict, end_reason: str) -> dict:
    """
    Conteúdo do .json gravado ao lado do vídeo. started_at e ended_at são
    datetime; fps_real é a taxa efetivamente gravada (quadros / duração).
    """
    duration = max(0.0, (ended_at - started_at).total_seconds())
    return {
        'session': session,
        'file': os.path.basename(file),
        'started_at': started_at.isoformat(timespec='seconds'),
        'ended_at': ended_at.isoformat(timespec='seconds'),
        'duration_s': round(duration, 2),
        'frames': frames,
        'dropped': dropped,
        'fps_real': round(frames / duration, 2) if duration > 0.0 else 0.0,
        'record_fps': record_fps,
        'width': width,
        'height': height,
        'fourcc': fourcc,
        'size_bytes': size_bytes,
        'scan': dict(scan),
        'end_reason': end_reason,
    }
