"""Testes do arquivo de intrínsecos da câmera (sem ROS)."""

import os
from dataclasses import replace

import pytest
import yaml

from pantilt_perception.camera_info_file import (
    DISTORTION_MODEL, IDENTITY_3X3, ZERO_DISTORTION, format_camera_info, load_camera_info,
    make_intrinsics, save_camera_info,
)

MINIMO = """\
image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [600.0, 0.0, 320.0, 0.0, 605.0, 240.0, 0.0, 0.0, 1.0]
"""


def escrever(tmp_path, texto, nome='camera.yaml'):
    path = tmp_path / nome
    path.write_text(texto, encoding='utf-8')
    return str(path)


def test_ida_e_volta(tmp_path):
    intr = make_intrinsics(640, 480, 'pantilt_cam', 602.37, 598.1, 320.0, 240.0)
    path = str(tmp_path / 'intr.yaml')
    save_camera_info(path, intr, 'Gerado pelo teste\nRMS: 1.2 px')
    lido = load_camera_info(path)
    assert lido == intr
    assert (lido.fx, lido.fy, lido.cx, lido.cy) == (602.37, 598.1, 320.0, 240.0)
    assert lido.k == (602.37, 0.0, 320.0, 0.0, 598.1, 240.0, 0.0, 0.0, 1.0)
    assert lido.p == (602.37, 0.0, 320.0, 0.0, 0.0, 598.1, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    assert lido.r == IDENTITY_3X3
    assert lido.d == ZERO_DISTORTION


def test_arquivo_gravado_no_formato_padrao(tmp_path):
    path = str(tmp_path / 'intr.yaml')
    save_camera_info(path, make_intrinsics(640, 480, 'pantilt_cam', 600, 600, 320, 240),
                     'linha 1\nlinha 2')
    texto = open(path, encoding='utf-8').read()
    assert texto.startswith('# linha 1\n# linha 2\n')
    data = yaml.safe_load(texto)
    assert set(data) == {
        'image_width', 'image_height', 'camera_name', 'camera_matrix', 'distortion_model',
        'distortion_coefficients', 'rectification_matrix', 'projection_matrix'}
    assert data['camera_matrix']['rows'] == 3 and data['camera_matrix']['cols'] == 3
    assert data['projection_matrix']['rows'] == 3 and data['projection_matrix']['cols'] == 4
    assert data['distortion_coefficients']['cols'] == 5
    assert data['distortion_model'] == DISTORTION_MODEL
    # nenhum temporário sobra na pasta
    assert os.listdir(tmp_path) == ['intr.yaml']


def test_save_substitui_arquivo_existente(tmp_path):
    path = str(tmp_path / 'intr.yaml')
    save_camera_info(path, make_intrinsics(640, 480, 'cam', 500, 500, 320, 240))
    save_camera_info(path, make_intrinsics(640, 480, 'cam', 700, 700, 320, 240))
    assert load_camera_info(path).fx == 700.0


def test_numeros_pequenos_continuam_numeros(tmp_path):
    # O PyYAML lê '1e-05' como texto; o gravador precisa escrever '1.0e-05'
    intr = make_intrinsics(640, 480, 'cam', 600, 600, 320, 240)
    intr = replace(intr, d=(1e-05, -2.5e-07, 0.0, 0.0, 0.0))
    path = str(tmp_path / 'intr.yaml')
    save_camera_info(path, intr)
    assert load_camera_info(path).d == (1e-05, -2.5e-07, 0.0, 0.0, 0.0)


def test_nome_com_espaco_entre_aspas(tmp_path):
    path = str(tmp_path / 'intr.yaml')
    save_camera_info(path, make_intrinsics(640, 480, 'webcam USB', 600, 600, 320, 240))
    assert load_camera_info(path).camera_name == 'webcam USB'


def test_arquivo_inexistente_retorna_none(tmp_path):
    assert load_camera_info(str(tmp_path / 'nao_existe.yaml')) is None


def test_arquivo_minimo_assume_pinhole(tmp_path):
    intr = load_camera_info(escrever(tmp_path, MINIMO))
    assert (intr.width, intr.height) == (640, 480)
    assert (intr.fx, intr.fy, intr.cx, intr.cy) == (600.0, 605.0, 320.0, 240.0)
    assert intr.d == ZERO_DISTORTION
    assert intr.distortion_model == DISTORTION_MODEL
    assert intr.r == IDENTITY_3X3
    assert intr.p == (600.0, 0.0, 320.0, 0.0, 0.0, 605.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0)
    assert intr.camera_name == ''


def test_distorcao_de_tabuleiro_preservada(tmp_path):
    texto = MINIMO + """\
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.31, 0.12, 0.001, -0.002, 0.0]
"""
    intr = load_camera_info(escrever(tmp_path, texto))
    assert intr.d == (-0.31, 0.12, 0.001, -0.002, 0.0)


@pytest.mark.parametrize('texto, trecho', [
    ('image_width: [640\n', 'YAML inválido'),
    ('- 1\n- 2\n', 'mapa'),
    ('image_width: 640\nimage_height: 480\n', 'camera_matrix'),
    (MINIMO.replace('image_width: 640', 'image_width: 0'), 'image_width'),
    (MINIMO.replace('image_height: 480\n', ''), 'image_height'),
    (MINIMO.replace('600.0, 0.0, 320.0', '0.0, 0.0, 320.0'), 'fx e fy'),
    (MINIMO.replace(', 0.0, 0.0, 1.0]', ', 0.0, 0.0]'), 'exige 9 valores'),
    (MINIMO.replace('rows: 3', 'rows: 2'), 'deve ser 3x3'),
    (MINIMO.replace('600.0, 0.0', 'abc, 0.0'), 'não numéricos'),
    (MINIMO.replace('600.0, 0.0', '.nan, 0.0'), 'não finitos'),
])
def test_arquivo_invalido(tmp_path, texto, trecho):
    with pytest.raises(ValueError, match=trecho):
        load_camera_info(escrever(tmp_path, texto))


def test_save_em_pasta_inexistente(tmp_path):
    intr = make_intrinsics(640, 480, 'cam', 600, 600, 320, 240)
    with pytest.raises(ValueError, match='não foi possível gravar'):
        save_camera_info(str(tmp_path / 'nao_existe' / 'intr.yaml'), intr)


def test_save_recusa_valor_nao_finito(tmp_path):
    intr = make_intrinsics(640, 480, 'cam', float('nan'), 600, 320, 240)
    with pytest.raises(ValueError, match='não finitos'):
        format_camera_info(intr)
