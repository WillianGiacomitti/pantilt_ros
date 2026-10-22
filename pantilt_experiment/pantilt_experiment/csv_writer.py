"""
Escritor dos CSVs do experiment_recorder, numa thread própria com fila.

Os callbacks só enfileiram a linha (write) e nunca esperam o disco. Com a
fila cheia, a linha é descartada e contada em dropped.

Cada arquivo só é criado na primeira linha, com o cabeçalho: um CSV sem
linhas não deixa arquivo. Booleanos viram 1/0, para o numpy ler direto, e
floats saem com 9 algarismos significativos.
"""

import csv
import os
import queue
import threading

# Sentinela que encerra a thread escritora
_STOP = object()


def _cell(value):
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, float):
        # 9 algarismos: preserva um float32 e corta o ruído de representação
        return format(value, '.9g')
    return value


class CsvWriter:
    def __init__(self, folder: str, columns: dict, queue_size: int):
        """columns: nome do arquivo (sem .csv) -> sequência com os nomes das colunas."""
        self.folder = folder
        self.columns = {name: tuple(cols) for name, cols in columns.items()}
        self.rows = {}              # linhas gravadas por arquivo
        self.dropped = 0            # linhas descartadas por fila cheia
        self.error = ''             # não vazio = falha de escrita (as linhas seguintes se perdem)

        self._queue = queue.Queue(maxsize=queue_size)
        self._closed = False
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def write(self, name: str, row) -> bool:
        """Enfileira uma linha do arquivo name. Retorna False se ela foi descartada."""
        if name not in self.columns:
            raise KeyError(f'CSV desconhecido: {name}')
        if self._closed:
            return False
        try:
            self._queue.put_nowait((name, row))
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def close(self) -> dict:
        """Grava o que resta na fila, fecha os arquivos e devolve as linhas por arquivo."""
        if not self._closed:
            self._closed = True
            self._queue.put(_STOP)
        self._thread.join()
        return dict(self.rows)

    # ---------------- Thread escritora ----------------
    def _loop(self):
        files = {}
        try:
            while True:
                item = self._queue.get()
                if item is _STOP:
                    break
                if self.error:
                    continue    # esvazia a fila sem gravar
                name, row = item
                try:
                    if name not in files:
                        f = open(os.path.join(self.folder, f'{name}.csv'), 'w',
                                 newline='', encoding='utf-8')
                        writer = csv.writer(f)
                        writer.writerow(self.columns[name])
                        files[name] = (f, writer)
                        self.rows[name] = 0
                    files[name][1].writerow([_cell(v) for v in row])
                    self.rows[name] += 1
                except OSError as e:
                    self.error = f'falha ao gravar {name}.csv: {e}'
        finally:
            for f, _ in files.values():
                f.close()
