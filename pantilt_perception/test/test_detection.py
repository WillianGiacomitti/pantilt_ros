"""Testes da lógica do detector_node (sem ROS nem modelo YOLO)."""

import pytest

from pantilt_perception.detection import (
    Detection, Equipment, load_equipment, match_equipment, parse_equipment, select_target,
    target_error, target_score, validate_params,
)

COCO_NAMES = {0: 'person', 39: 'bottle', 41: 'cup', 67: 'cell phone'}


def det(name, conf, x1, y1, x2, y2):
    return Detection(name, conf, x1, y1, x2, y2)


# ---------------- equipment.yaml ----------------
def test_parse_equipment_ok():
    eq = parse_equipment({'equipment': {
        'garrafa': {'label': 'Garrafa', 'class_name': 'bottle'},
        'celular': {'class_name': ' cell phone '},
    }})
    assert eq['garrafa'] == Equipment('garrafa', 'Garrafa', 'bottle')
    # sem label, usa a chave; class_name sem espaços nas pontas
    assert eq['celular'] == Equipment('celular', 'celular', 'cell phone')


@pytest.mark.parametrize('data', [
    None,
    [],
    {'outra_chave': {}},
    {'equipment': {}},
    {'equipment': ['garrafa']},
    {'equipment': {'garrafa': 'bottle'}},
    {'equipment': {'garrafa': {'label': 'Garrafa'}}},
    {'equipment': {'garrafa': {'label': 'Garrafa', 'class_name': '  '}}},
])
def test_parse_equipment_invalido(data):
    with pytest.raises(ValueError):
        parse_equipment(data)


def test_load_equipment_arquivo(tmp_path):
    path = tmp_path / 'equipment.yaml'
    path.write_text('equipment:\n  copo:\n    label: "Copo"\n    class_name: "cup"\n',
                    encoding='utf-8')
    assert load_equipment(str(path)) == {'copo': Equipment('copo', 'Copo', 'cup')}


def test_load_equipment_ausente(tmp_path):
    with pytest.raises(ValueError, match='não foi possível ler'):
        load_equipment(str(tmp_path / 'nao_existe.yaml'))


def test_load_equipment_yaml_invalido(tmp_path):
    path = tmp_path / 'ruim.yaml'
    path.write_text('equipment: [garrafa\n', encoding='utf-8')
    with pytest.raises(ValueError, match='YAML inválido'):
        load_equipment(str(path))


# ---------------- Cruzamento com o modelo ----------------
def test_match_equipment_remove_ausentes():
    eq = parse_equipment({'equipment': {
        'garrafa': {'class_name': 'bottle'},
        'esfera': {'class_name': 'esfera'},
    }})
    available, missing = match_equipment(eq, COCO_NAMES)
    assert list(available) == ['garrafa']
    assert missing == ['esfera']


def test_match_equipment_lista_de_nomes():
    eq = parse_equipment({'equipment': {'copo': {'class_name': 'cup'}}})
    available, missing = match_equipment(eq, ['cup', 'bottle'])
    assert list(available) == ['copo'] and missing == []


# ---------------- Seleção do alvo ----------------
def test_target_score_area_normalizada():
    # bbox de 64x48 numa imagem 640x480 = 1% da área
    assert target_score(det('bottle', 0.5, 0, 0, 64, 48), 640, 480) == pytest.approx(0.005)


def test_select_target_maior_score():
    grande_pouco_confiavel = det('bottle', 0.55, 0, 0, 200, 200)    # 0.55 * 40000
    pequena_confiavel = det('bottle', 0.95, 300, 300, 350, 350)     # 0.95 * 2500
    alvo = select_target([pequena_confiavel, grande_pouco_confiavel], 'bottle', 640, 480)
    assert alvo is grande_pouco_confiavel


def test_select_target_filtra_classe():
    pessoa = det('person', 0.99, 0, 0, 640, 480)
    copo = det('cup', 0.6, 10, 10, 50, 50)
    assert select_target([pessoa, copo], 'cup', 640, 480) is copo


def test_select_target_sem_candidato():
    assert select_target([det('person', 0.9, 0, 0, 10, 10)], 'bottle', 640, 480) is None
    assert select_target([], 'bottle', 640, 480) is None


# ---------------- Erro em pixels ----------------
def test_target_error_centrado():
    assert target_error(det('cup', 0.9, 300, 220, 340, 260), 640, 480) == (0.0, 0.0)


def test_target_error_sinais():
    # à direita e abaixo do centro: erros positivos
    ex, ey = target_error(det('cup', 0.9, 500, 400, 540, 440), 640, 480)
    assert ex > 0 and ey > 0
    # à esquerda e acima: negativos
    ex, ey = target_error(det('cup', 0.9, 0, 0, 40, 40), 640, 480)
    assert ex == pytest.approx(-300.0) and ey == pytest.approx(-220.0)


# ---------------- Parâmetros ----------------
@pytest.mark.parametrize('conf, imgsz', [(0.5, 640), (1.0, 320), (0.01, 480)])
def test_validate_params_ok(conf, imgsz):
    validate_params(conf, imgsz)


@pytest.mark.parametrize('conf, imgsz', [(0.0, 640), (1.5, 640), (-0.1, 640),
                                         (0.5, 0), (0.5, 600), (0.5, -32)])
def test_validate_params_invalido(conf, imgsz):
    with pytest.raises(ValueError):
        validate_params(conf, imgsz)
