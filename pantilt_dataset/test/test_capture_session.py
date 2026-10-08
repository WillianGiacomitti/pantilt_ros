"""Testes da lógica de gravação do capture_node (sem ROS nem OpenCV)."""

from datetime import datetime, timedelta

import pytest

from pantilt_dataset.capture_session import (
    MAX_SESSION_LEN, build_metadata, clean_session, free_gb, unique_path, validate_params,
    video_basename,
)


@pytest.mark.parametrize('texto, esperado', [
    ('Mesa Janela (tarde)', 'mesa_janela_tarde'),
    ('Ação no Pátio', 'acao_no_patio'),
    ('  a   b  ', 'a_b'),
    ('ABC_123-z', 'abc_123-z'),
    ('isolador/disco#2', 'isoladordisco2'),
    ('_mesa_', 'mesa'),
    ('!!!', ''),
    ('', ''),
])
def test_clean_session(texto, esperado):
    assert clean_session(texto) == esperado


def test_clean_session_limita_o_tamanho():
    assert clean_session('a' * 60) == 'a' * MAX_SESSION_LEN
    # O corte não deixa "_" sobrando no fim
    assert clean_session('a' * (MAX_SESSION_LEN - 1) + ' b') == 'a' * (MAX_SESSION_LEN - 1)


def test_video_basename():
    inicio = datetime(2026, 9, 29, 14, 5, 3)
    assert video_basename(inicio, 'mesa') == '20260929_140503_mesa'
    assert video_basename(inicio, '') == '20260929_140503'


def test_unique_path(tmp_path):
    primeiro = unique_path(str(tmp_path), 'video', '.mp4')
    assert primeiro == str(tmp_path / 'video.mp4')
    (tmp_path / 'video.mp4').touch()
    assert unique_path(str(tmp_path), 'video', '.mp4') == str(tmp_path / 'video_1.mp4')
    (tmp_path / 'video_1.mp4').touch()
    assert unique_path(str(tmp_path), 'video', '.mp4') == str(tmp_path / 'video_2.mp4')


def test_validate_params_ok():
    validate_params(15.0, 'mp4v', 1.0, 30)
    validate_params(15.0, 'MJPG', 0.0, 1)


@pytest.mark.parametrize('record_fps, fourcc, min_free_gb, queue_size', [
    (0.0, 'mp4v', 1.0, 30),
    (-1.0, 'mp4v', 1.0, 30),
    (15.0, 'mp4', 1.0, 30),
    (15.0, 'mp4v1', 1.0, 30),
    (15.0, 'mp4v', -0.5, 30),
    (15.0, 'mp4v', 1.0, 0),
])
def test_validate_params_invalidos(record_fps, fourcc, min_free_gb, queue_size):
    with pytest.raises(ValueError):
        validate_params(record_fps, fourcc, min_free_gb, queue_size)


def test_free_gb(tmp_path):
    assert free_gb(str(tmp_path)) > 0.0


def _metadata(duracao_s, quadros):
    inicio = datetime(2026, 9, 29, 14, 0, 0)
    scan = {'enabled': True, 'speed_deg_s': 0.0, 'goals_sent': 2, 'goals_succeeded': 1}
    return build_metadata(
        'mesa', '/ros2_ws/datasets/20260929_140000_mesa.mp4', inicio,
        inicio + timedelta(seconds=duracao_s), quadros, 3, 15.0, 640, 480, 'mp4v',
        1234, scan, 'Parada pelo operador')


def test_build_metadata():
    meta = _metadata(10.0, 150)
    assert meta['file'] == '20260929_140000_mesa.mp4'
    assert meta['started_at'] == '2026-09-29T14:00:00'
    assert meta['ended_at'] == '2026-09-29T14:00:10'
    assert meta['duration_s'] == 10.0
    assert meta['fps_real'] == 15.0
    assert meta['frames'] == 150
    assert meta['dropped'] == 3
    assert (meta['width'], meta['height']) == (640, 480)
    assert meta['scan']['goals_sent'] == 2
    assert meta['end_reason'] == 'Parada pelo operador'


def test_build_metadata_duracao_zero():
    meta = _metadata(0.0, 1)
    assert meta['duration_s'] == 0.0
    assert meta['fps_real'] == 0.0
