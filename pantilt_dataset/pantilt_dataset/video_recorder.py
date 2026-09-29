"""
Escritor de vídeo do capture_node, numa thread própria com fila.

O callback de imagem só enfileira o quadro (submit) e nunca espera a
codificação. Com a fila cheia, o quadro é descartado e contado em dropped.

O arquivo só é criado no primeiro quadro, que define o tamanho do vídeo: uma
gravação sem quadros não deixa arquivo. Quadros de tamanho diferente do
primeiro são descartados (o VideoWriter não aceita mudança de tamanho).
"""

import os
import queue
import threading

import cv2

# Sentinela que encerra a thread escritora
_STOP = object()


def opencv_writer(path: str, fourcc: str, fps: float, size):
    """Cria o cv2.VideoWriter; size = (largura, altura)."""
    return cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*fourcc), fps, size)


class VideoRecorder:
    def __init__(self, path: str, fps: float, fourcc: str, queue_size: int,
                 writer_factory=opencv_writer):
        self.path = path
        self.fps = fps
        self.fourcc = fourcc
        self._writer_factory = writer_factory

        self.frames = 0             # quadros gravados
        self.dropped = 0            # descartados por fila cheia
        self.size_mismatch = 0      # descartados por tamanho diferente do primeiro
        self.size = None            # (largura, altura) do vídeo
        self.error = ''             # não vazio = falha ao criar o vídeo

        self._queue = queue.Queue(maxsize=queue_size)
        self._closed = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def submit(self, frame) -> bool:
        """Enfileira um quadro (BGR). Retorna False se ele foi descartado."""
        if self._closed or self.error:
            return False
        try:
            self._queue.put_nowait(frame)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def close(self):
        """Grava o que resta na fila, finaliza o arquivo e espera a thread terminar."""
        if not self._closed:
            self._closed = True
            self._queue.put(_STOP)
        self._thread.join()

    @property
    def size_bytes(self) -> int:
        return os.path.getsize(self.path) if os.path.isfile(self.path) else 0

    # ---------------- Thread escritora ----------------
    def _loop(self):
        writer = None
        try:
            while True:
                frame = self._queue.get()
                if frame is _STOP:
                    break
                if self.error:
                    continue    # esvazia a fila sem gravar
                height, width = frame.shape[:2]
                if writer is None:
                    writer = self._writer_factory(self.path, self.fourcc, self.fps, (width, height))
                    if writer is None or not writer.isOpened():
                        self.error = (f'não foi possível criar o vídeo {self.path} '
                                      f'(fourcc "{self.fourcc}", {width}x{height})')
                        writer = None
                        continue
                    self.size = (width, height)
                elif (width, height) != self.size:
                    self.size_mismatch += 1
                    continue
                writer.write(frame)
                self.frames += 1
        finally:
            if writer is not None:
                writer.release()
            # Um writer que falhou pode ter deixado um arquivo vazio
            if self.frames == 0 and os.path.isfile(self.path):
                os.remove(self.path)
