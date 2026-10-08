"""
Lógica do detector_node sem dependência do ROS nem do ultralytics (testável
isoladamente): leitura do equipment.yaml, cruzamento com as classes do modelo,
seleção do alvo e erro em pixels (docs/architecture.md, seções 4.2 e 7).

As classes são identificadas sempre por NOME (model.names), nunca por ID.
"""

from dataclasses import dataclass

import yaml


@dataclass(frozen=True)
class Equipment:
    """Entrada do equipment.yaml."""
    key: str           # chave usada nos services (ex.: "garrafa")
    label: str         # texto para a interface
    class_name: str    # nome da classe no modelo (ex.: "bottle")


@dataclass(frozen=True)
class Detection:
    """Uma bbox detectada, em pixels da imagem original (x1, y1 = canto superior esquerdo)."""
    class_name: str
    confidence: float
    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def center(self):
        return (self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0

    @property
    def size(self):
        return self.x2 - self.x1, self.y2 - self.y1

    @property
    def area(self) -> float:
        w, h = self.size
        return max(0.0, w) * max(0.0, h)


def parse_equipment(data) -> dict:
    """
    Valida o conteúdo de um equipment.yaml já carregado. Retorna {chave: Equipment}.

    Formato:
      equipment:
        <chave>:
          label: "<texto>"
          class_name: "<nome da classe no modelo>"
    """
    if not isinstance(data, dict) or not isinstance(data.get('equipment'), dict):
        raise ValueError('o arquivo deve ter um mapeamento "equipment:" com os equipamentos')
    entries = data['equipment']
    if not entries:
        raise ValueError('nenhum equipamento em "equipment:"')

    result = {}
    for key, entry in entries.items():
        if not isinstance(entry, dict):
            raise ValueError(f'equipamento "{key}": esperado um mapeamento com label e class_name')
        class_name = entry.get('class_name')
        if not isinstance(class_name, str) or not class_name.strip():
            raise ValueError(f'equipamento "{key}": class_name ausente ou vazio')
        label = entry.get('label')
        label = str(label).strip() if label is not None and str(label).strip() else str(key)
        result[str(key)] = Equipment(str(key), label, class_name.strip())
    return result


def load_equipment(path: str) -> dict:
    """Lê e valida um equipment.yaml. Levanta ValueError com o motivo."""
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except OSError as e:
        raise ValueError(f'não foi possível ler {path}: {e.strerror or e}') from e
    except yaml.YAMLError as e:
        raise ValueError(f'YAML inválido em {path}: {e}') from e
    try:
        return parse_equipment(data)
    except ValueError as e:
        raise ValueError(f'{path}: {e}') from e


def match_equipment(equipment: dict, model_names) -> tuple:
    """
    Cruza os equipamentos com as classes do modelo, por nome.

    model_names: dict {id: nome} (model.names do ultralytics) ou lista de nomes.
    Retorna (disponíveis {chave: Equipment}, chaves ausentes do modelo [lista]).
    """
    names = set(model_names.values()) if isinstance(model_names, dict) else set(model_names)
    available = {k: e for k, e in equipment.items() if e.class_name in names}
    missing = [k for k in equipment if k not in available]
    return available, missing


def target_score(det: Detection, img_w: int, img_h: int) -> float:
    """Score do alvo: confiança x área normalizada (área da bbox / área da imagem)."""
    return det.confidence * det.area / float(img_w * img_h)


def select_target(detections, class_name: str, img_w: int, img_h: int):
    """Bbox da classe class_name com o maior score, ou None se não houver nenhuma."""
    candidates = [d for d in detections if d.class_name == class_name]
    if not candidates:
        return None
    return max(candidates, key=lambda d: target_score(d, img_w, img_h))


def target_error(det: Detection, img_w: int, img_h: int) -> tuple:
    """
    Erro em pixels: centro da bbox menos centro da imagem.
    error_x > 0: alvo à direita do centro; error_y > 0: alvo abaixo do centro.
    """
    cx, cy = det.center
    return cx - img_w / 2.0, cy - img_h / 2.0


def validate_params(conf_threshold: float, imgsz: int):
    """Levanta ValueError se algum parâmetro do detector_node for inválido."""
    if not 0.0 < conf_threshold <= 1.0:
        raise ValueError(f'conf_threshold={conf_threshold} inválido: deve estar em (0, 1]')
    if imgsz <= 0 or imgsz % 32 != 0:
        raise ValueError(f'imgsz={imgsz} inválido: deve ser positivo e múltiplo de 32')
