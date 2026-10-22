"""Testes do escritor de snapshots do experiment_recorder (OpenCV real e falso, sem ROS)."""

import threading

import cv2
import numpy as np

from pantilt_experiment.snapshot_writer import SnapshotWriter


def quadro(largura=64, altura=48, valor=0):
    return np.full((altura, largura, 3), valor, dtype=np.uint8)


def test_grava_jpeg_e_le_de_volta(tmp_path):
    w = SnapshotWriter(quality=85, queue_size=10)
    for i, nome in enumerate(('01_inicio.jpg', '02_centrado.jpg', '03_fim.jpg')):
        assert w.submit(str(tmp_path / nome), quadro(valor=50 * i))
    w.close()
    assert w.error == ''
    assert w.written == 3
    assert w.dropped == 0
    img = cv2.imread(str(tmp_path / '02_centrado.jpg'))
    assert img.shape == (48, 64, 3)
    assert abs(int(img.mean()) - 50) <= 2


def test_qualidade_repassada_ao_escritor(tmp_path):
    chamadas = []
    w = SnapshotWriter(quality=42, queue_size=10,
                       write_fn=lambda path, frame, q: chamadas.append((path, q)) or True)
    w.submit('a.jpg', quadro())
    w.close()
    assert chamadas == [('a.jpg', 42)]


def test_falha_do_escritor_vira_erro():
    w = SnapshotWriter(quality=85, queue_size=10, write_fn=lambda path, frame, q: False)
    w.submit('/inexistente/01_inicio.jpg', quadro())
    w.close()
    assert w.written == 0
    assert '01_inicio.jpg' in w.error


def test_excecao_do_escritor_vira_erro():
    def explode(path, frame, q):
        raise OSError('disco cheio')

    w = SnapshotWriter(quality=85, queue_size=10, write_fn=explode)
    w.submit('a.jpg', quadro())
    w.close()
    assert 'disco cheio' in w.error


def test_fila_cheia_descarta():
    liberar = threading.Event()

    def lento(path, frame, q):
        liberar.wait(5.0)
        return True

    w = SnapshotWriter(quality=85, queue_size=1, write_fn=lento)
    resultados = [w.submit(f'{i}.jpg', quadro()) for i in range(5)]
    liberar.set()
    w.close()
    assert resultados[0] is True
    assert w.dropped >= 1
    assert w.written + w.dropped == 5


def test_close_idempotente_e_submit_depois_recusado(tmp_path):
    w = SnapshotWriter(quality=85, queue_size=10)
    w.close()
    w.close()
    assert w.submit(str(tmp_path / 'x.jpg'), quadro()) is False
    assert not (tmp_path / 'x.jpg').exists()
