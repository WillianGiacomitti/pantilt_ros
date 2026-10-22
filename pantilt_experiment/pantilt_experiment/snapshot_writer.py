"""
Escritor dos snapshots (JPEG) do experiment_recorder, numa thread própria com
fila.

O callback só enfileira o quadro (submit) e nunca espera a codificação nem o
disco. Com a fila cheia, o snapshot é descartado e contado em dropped.
"""

import queue
import threading

import cv2

# Sentinela que encerra a thread escritora
_STOP = object()


def opencv_jpeg(path: str, frame, quality: int) -> bool:
    """Grava o quadro (BGR) em JPEG; retorna False se o OpenCV falhar."""
    return cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])


class SnapshotWriter:
    def __init__(self, quality: int, queue_size: int, write_fn=opencv_jpeg):
        self.quality = quality
        self._write_fn = write_fn

        self.written = 0            # snapshots gravados
        self.dropped = 0            # descartados por fila cheia
        self.error = ''             # não vazio = última falha de escrita

        self._queue = queue.Queue(maxsize=queue_size)
        self._closed = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def submit(self, path: str, frame) -> bool:
        """Enfileira um snapshot. Retorna False se ele foi descartado."""
        if self._closed:
            return False
        try:
            self._queue.put_nowait((path, frame))
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def close(self):
        """Grava o que resta na fila e espera a thread terminar."""
        if not self._closed:
            self._closed = True
            self._queue.put(_STOP)
        self._thread.join()

    # ---------------- Thread escritora ----------------
    def _loop(self):
        while True:
            item = self._queue.get()
            if item is _STOP:
                break
            path, frame = item
            detail = ''
            try:
                ok = self._write_fn(path, frame, self.quality)
            except (OSError, cv2.error) as e:
                ok = False
                detail = f': {e}'
            if ok:
                self.written += 1
            else:
                self.error = f'falha ao gravar {path}{detail}'
