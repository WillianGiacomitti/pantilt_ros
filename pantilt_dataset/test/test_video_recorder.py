"""Testes do escritor de vídeo do capture_node (OpenCV real e writer falso, sem ROS)."""

import threading

import cv2
import numpy as np

from pantilt_dataset.video_recorder import VideoRecorder


def quadro(largura=64, altura=48, valor=0):
    return np.full((altura, largura, 3), valor, dtype=np.uint8)


def contar_quadros(path):
    cap = cv2.VideoCapture(path)
    n = 0
    while cap.read()[0]:
        n += 1
    cap.release()
    return n


def test_grava_e_le_de_volta(tmp_path):
    path = str(tmp_path / 'video.mp4')
    rec = VideoRecorder(path, 15.0, 'mp4v', queue_size=100)
    for i in range(20):
        assert rec.submit(quadro(valor=i * 10))
    rec.close()
    assert rec.error == ''
    assert rec.frames == 20
    assert rec.dropped == 0
    assert rec.size == (64, 48)
    assert rec.size_bytes > 0
    assert contar_quadros(path) == 20


def test_sem_quadros_nao_cria_arquivo(tmp_path):
    path = tmp_path / 'vazio.mp4'
    rec = VideoRecorder(str(path), 15.0, 'mp4v', queue_size=10)
    rec.close()
    assert rec.frames == 0
    assert not path.exists()


def test_tamanho_diferente_e_descartado(tmp_path):
    path = str(tmp_path / 'video.mp4')
    rec = VideoRecorder(path, 15.0, 'mp4v', queue_size=100)
    for _ in range(5):
        rec.submit(quadro(64, 48))
    for _ in range(2):
        rec.submit(quadro(32, 24))
    rec.close()
    assert rec.frames == 5
    assert rec.size_mismatch == 2
    assert contar_quadros(path) == 5


def test_close_duas_vezes(tmp_path):
    rec = VideoRecorder(str(tmp_path / 'video.mp4'), 15.0, 'mp4v', queue_size=10)
    rec.submit(quadro())
    rec.close()
    rec.close()
    assert rec.frames == 1
    assert not rec.submit(quadro())


class WriterLento:
    """Writer falso: cada write espera a liberação do teste."""

    def __init__(self):
        self.escrevendo = threading.Event()
        self.liberado = threading.Event()
        self.escritos = 0

    def isOpened(self):
        return True

    def write(self, frame):
        self.escrevendo.set()
        self.liberado.wait(timeout=5.0)
        self.escritos += 1

    def release(self):
        pass


def test_fila_cheia_descarta_e_conta(tmp_path):
    writer = WriterLento()
    rec = VideoRecorder(str(tmp_path / 'video.mp4'), 15.0, 'mp4v', queue_size=2,
                        writer_factory=lambda *args: writer)
    assert rec.submit(quadro())
    # A thread está presa no primeiro write: a fila aceita só mais 2 quadros
    assert writer.escrevendo.wait(timeout=5.0)
    aceitos = [rec.submit(quadro()) for _ in range(5)]
    assert aceitos == [True, True, False, False, False]
    assert rec.dropped == 3
    writer.liberado.set()
    rec.close()
    assert rec.frames == 3
    assert writer.escritos == 3


class WriterQueFalha:
    def isOpened(self):
        return False

    def release(self):
        pass


def test_writer_que_nao_abre(tmp_path):
    path = tmp_path / 'video.mp4'
    rec = VideoRecorder(str(path), 15.0, 'xxxx', queue_size=10,
                        writer_factory=lambda *args: WriterQueFalha())
    rec.submit(quadro())
    rec.close()
    assert 'não foi possível criar o vídeo' in rec.error
    assert rec.frames == 0
    assert not rec.submit(quadro())
    assert not path.exists()
