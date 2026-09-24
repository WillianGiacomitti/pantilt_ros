"""Testes da lógica de captura do camera_node (sem câmera nem ROS)."""

import pytest

from pantilt_perception.camera_source import (
    KIND_DEVICE, KIND_FILE, KIND_URL, FrameThrottle, fourcc_to_str, parse_source,
    validate_params,
)


@pytest.mark.parametrize('source, esperado', [
    ('0', (KIND_DEVICE, 0)),
    ('2', (KIND_DEVICE, 2)),
    (' 1 ', (KIND_DEVICE, 1)),
    ('/dev/video0', (KIND_DEVICE, '/dev/video0')),
    ('http://192.168.0.10:8081/video', (KIND_URL, 'http://192.168.0.10:8081/video')),
    ('rtsp://camera/stream', (KIND_URL, 'rtsp://camera/stream')),
    ('/ros2_ws/videos/ensaio.mp4', (KIND_FILE, '/ros2_ws/videos/ensaio.mp4')),
    ('ensaio.avi', (KIND_FILE, 'ensaio.avi')),
])
def test_parse_source(source, esperado):
    assert parse_source(source) == esperado


@pytest.mark.parametrize('source', ['', '   '])
def test_parse_source_vazio(source):
    with pytest.raises(ValueError):
        parse_source(source)


def test_validate_params_ok():
    validate_params(640, 480, 30.0)


@pytest.mark.parametrize('width, height, fps', [
    (0, 480, 30.0),
    (640, -1, 30.0),
    (640, 480, 0.0),
    (640, 480, -5.0),
])
def test_validate_params_invalidos(width, height, fps):
    with pytest.raises(ValueError):
        validate_params(width, height, fps)


def test_fourcc_to_str():
    mjpg = ord('M') | ord('J') << 8 | ord('P') << 16 | ord('G') << 24
    assert fourcc_to_str(float(mjpg)) == 'MJPG'
    assert fourcc_to_str(0.0) == '?'


def publicados(throttle, instantes):
    return [t for t in instantes if throttle.ready(t)]


def test_throttle_mesma_taxa_com_jitter_nao_descarta():
    # Fonte a 30 fps com jitter de +-1,5 ms, limite em 30 fps: nenhum quadro perdido
    instantes = [i / 30.0 + (0.0015 if i % 2 else -0.0015) for i in range(1, 91)]
    assert len(publicados(FrameThrottle(30.0), instantes)) == 90


@pytest.mark.parametrize('fps, esperado', [(10.0, 10), (15.0, 15), (20.0, 20)])
def test_throttle_reduz_taxa(fps, esperado):
    # 1 s de fonte a 30 fps
    instantes = [i / 30.0 for i in range(30)]
    assert len(publicados(FrameThrottle(fps), instantes)) == esperado


def test_throttle_realinha_apos_atraso_sem_rajada():
    th = FrameThrottle(10.0)
    assert th.ready(0.0)
    # Fonte parada por 2 s: o próximo quadro sai, mas o seguinte respeita o período
    assert th.ready(2.0)
    assert not th.ready(2.03)
    assert th.ready(2.1)


def test_throttle_wait_time():
    th = FrameThrottle(10.0)
    assert th.wait_time(0.0) == 0.0
    th.ready(0.0)
    assert th.wait_time(0.04) == pytest.approx(0.06)
    assert th.wait_time(0.5) == 0.0


def test_throttle_fps_invalido():
    with pytest.raises(ValueError):
        FrameThrottle(0.0)
