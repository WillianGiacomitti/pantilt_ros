"""
Arquivo de intrínsecos da câmera no formato padrão do ROS (o mesmo do
camera_calibration_parsers), sem dependência do ROS (testável isoladamente).

Lido pelo camera_node, que publica /camera/camera_info, e gravado pelo
calibration_node (docs/architecture.md, seções 4.1 e 4.10).

Formato:
  image_width: 640
  image_height: 480
  camera_name: pantilt_cam
  camera_matrix:            {rows: 3, cols: 3, data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]}
  distortion_model: plumb_bob
  distortion_coefficients:  {rows: 1, cols: 5, data: [...]}
  rectification_matrix:     {rows: 3, cols: 3, data: [...]}
  projection_matrix:        {rows: 3, cols: 4, data: [...]}

Obrigatórios: image_width, image_height e camera_matrix. Sem os demais, vale
o modelo pinhole sem distorção: D nulo, R identidade e P = [K | 0].
"""

import math
import os
import re
import tempfile
from dataclasses import dataclass

import yaml

DISTORTION_MODEL = 'plumb_bob'
IDENTITY_3X3 = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
ZERO_DISTORTION = (0.0, 0.0, 0.0, 0.0, 0.0)

# Nome de câmera que pode ir sem aspas no YAML
_PLAIN_NAME = re.compile(r'^[A-Za-z0-9_\-]+$')


@dataclass(frozen=True)
class CameraIntrinsics:
    """Conteúdo do arquivo. Matrizes por linhas, como no sensor_msgs/CameraInfo."""
    width: int
    height: int
    camera_name: str
    k: tuple                  # 3x3
    d: tuple                  # coeficientes de distorção
    distortion_model: str
    r: tuple                  # 3x3
    p: tuple                  # 3x4

    @property
    def fx(self) -> float:
        return self.k[0]

    @property
    def fy(self) -> float:
        return self.k[4]

    @property
    def cx(self) -> float:
        return self.k[2]

    @property
    def cy(self) -> float:
        return self.k[5]


def make_intrinsics(width: int, height: int, camera_name: str,
                    fx: float, fy: float, cx: float, cy: float) -> CameraIntrinsics:
    """Modelo pinhole sem distorção: D nulo, R identidade e P = [K | 0]."""
    k = (float(fx), 0.0, float(cx), 0.0, float(fy), float(cy), 0.0, 0.0, 1.0)
    return CameraIntrinsics(int(width), int(height), str(camera_name), k, ZERO_DISTORTION,
                            DISTORTION_MODEL, IDENTITY_3X3, _projection_from_k(k))


def _projection_from_k(k) -> tuple:
    return (k[0], k[1], k[2], 0.0,
            k[3], k[4], k[5], 0.0,
            k[6], k[7], k[8], 0.0)


# ---------------- Leitura ----------------

def load_camera_info(path: str):
    """
    Lê o arquivo de intrínsecos. Retorna None se ele não existir (câmera sem
    calibração) e levanta ValueError com o motivo se ele for inválido.
    """
    if not os.path.exists(path):
        return None
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except OSError as e:
        raise ValueError(f'não foi possível ler {path}: {e.strerror or e}') from e
    except yaml.YAMLError as e:
        raise ValueError(f'YAML inválido em {path}: {e}') from e
    try:
        return parse_camera_info(data)
    except ValueError as e:
        raise ValueError(f'{path}: {e}') from e


def parse_camera_info(data) -> CameraIntrinsics:
    """Valida o conteúdo já lido do YAML. Levanta ValueError com o motivo."""
    if not isinstance(data, dict):
        raise ValueError('o conteúdo deve ser um mapa (chave: valor)')

    width = _positive_int(data, 'image_width')
    height = _positive_int(data, 'image_height')

    k = _matrix(data, 'camera_matrix', 3, 3)
    if not (k[0] > 0.0 and k[4] > 0.0):
        raise ValueError(f'fx e fy devem ser positivos, recebido fx={k[0]}, fy={k[4]}')

    if 'distortion_coefficients' in data:
        d = _matrix(data, 'distortion_coefficients', 1, None)
    else:
        d = ZERO_DISTORTION
    r = _matrix(data, 'rectification_matrix', 3, 3) if 'rectification_matrix' in data \
        else IDENTITY_3X3
    p = _matrix(data, 'projection_matrix', 3, 4) if 'projection_matrix' in data \
        else _projection_from_k(k)

    return CameraIntrinsics(
        width=width,
        height=height,
        camera_name=str(data.get('camera_name', '')),
        k=k,
        d=d,
        distortion_model=str(data.get('distortion_model', DISTORTION_MODEL)),
        r=r,
        p=p,
    )


def _positive_int(data: dict, key: str) -> int:
    if key not in data:
        raise ValueError(f'"{key}" ausente')
    value = data[key]
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f'"{key}" deve ser um inteiro positivo, recebido {value!r}')
    return value


def _matrix(data: dict, key: str, rows: int, cols) -> tuple:
    """
    Lê uma matriz {rows, cols, data}. cols=None aceita qualquer número de
    colunas (coeficientes de distorção).
    """
    entry = data.get(key)
    if not isinstance(entry, dict) or not isinstance(entry.get('data'), list):
        raise ValueError(f'"{key}" ausente ou sem a lista "data"')
    try:
        values = tuple(float(v) for v in entry['data'])
    except (TypeError, ValueError) as e:
        raise ValueError(f'"{key}" tem valores não numéricos') from e
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f'"{key}" tem valores não finitos')

    n_rows = entry.get('rows', rows)
    n_cols = entry.get('cols', cols if cols is not None else len(values))
    if not (isinstance(n_rows, int) and isinstance(n_cols, int)):
        raise ValueError(f'"{key}": rows e cols devem ser inteiros')
    if n_rows != rows or (cols is not None and n_cols != cols):
        expected = f'{rows}x{cols}' if cols is not None else f'{rows}xN'
        raise ValueError(f'"{key}" deve ser {expected}, recebido {n_rows}x{n_cols}')
    if n_rows * n_cols != len(values):
        raise ValueError(
            f'"{key}": {n_rows}x{n_cols} exige {n_rows * n_cols} valores, recebido {len(values)}')
    return values


# ---------------- Escrita ----------------

def format_camera_info(intrinsics: CameraIntrinsics, comentario: str = '') -> str:
    """Texto do arquivo, com o comentário (várias linhas) como cabeçalho #."""
    values = intrinsics.k + intrinsics.d + intrinsics.r + intrinsics.p
    if not all(math.isfinite(v) for v in values):
        raise ValueError('intrínsecos com valores não finitos')

    name = intrinsics.camera_name
    name_text = name if _PLAIN_NAME.match(name) else '"' + name.replace('"', '\\"') + '"'

    lines = [f'# {line}'.rstrip() for line in comentario.splitlines()]
    lines += [
        f'image_width: {intrinsics.width}',
        f'image_height: {intrinsics.height}',
        f'camera_name: {name_text}',
        *_matrix_lines('camera_matrix', 3, 3, intrinsics.k),
        f'distortion_model: {intrinsics.distortion_model}',
        *_matrix_lines('distortion_coefficients', 1, len(intrinsics.d), intrinsics.d),
        *_matrix_lines('rectification_matrix', 3, 3, intrinsics.r),
        *_matrix_lines('projection_matrix', 3, 4, intrinsics.p),
    ]
    return '\n'.join(lines) + '\n'


def _matrix_lines(key: str, rows: int, cols: int, values) -> list:
    data = ', '.join(_number(v) for v in values)
    return [f'{key}:', f'  rows: {rows}', f'  cols: {cols}', f'  data: [{data}]']


def _number(value: float) -> str:
    """repr exato do float, com ponto na mantissa: o PyYAML lê '1e-05' como texto."""
    text = repr(float(value))
    mantissa, sep, exponent = text.partition('e')
    if sep and '.' not in mantissa:
        text = f'{mantissa}.0e{exponent}'
    return text


def save_camera_info(path: str, intrinsics: CameraIntrinsics, comentario: str = ''):
    """
    Grava o arquivo de forma atômica: escreve um temporário na mesma pasta e
    o renomeia, para nunca deixar um arquivo pela metade. Levanta ValueError
    com o motivo se não conseguir gravar.
    """
    text = format_camera_info(intrinsics, comentario)
    directory = os.path.dirname(os.path.abspath(path))
    try:
        fd, tmp_path = tempfile.mkstemp(prefix='.camera_info_', suffix='.tmp', dir=directory)
    except OSError as e:
        raise ValueError(f'não foi possível gravar em {directory}: {e.strerror or e}') from e
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(text)
        # mkstemp cria com 0600; o arquivo de configuração deve ser legível por todos
        os.chmod(tmp_path, 0o644)
        os.replace(tmp_path, path)
    except OSError as e:
        try:
            os.remove(tmp_path)
        except OSError:
            pass
        raise ValueError(f'não foi possível gravar {path}: {e.strerror or e}') from e
