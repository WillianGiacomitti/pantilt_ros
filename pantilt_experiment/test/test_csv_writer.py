"""Testes do escritor de CSV do experiment_recorder (sem ROS)."""

import csv
import threading

import pytest

from pantilt_experiment import csv_writer
from pantilt_experiment.csv_writer import CsvWriter

COLUNAS = {'a': ('t', 'x', 'ok'), 'b': ('t', 'nome')}


def ler(caminho):
    with open(caminho, newline='', encoding='utf-8') as f:
        return list(csv.reader(f))


def test_grava_cabecalho_e_linhas(tmp_path):
    w = CsvWriter(str(tmp_path), COLUNAS, queue_size=100)
    assert w.write('a', [0.1, 1.5, True])
    assert w.write('a', [0.2, -2.0, False])
    assert w.write('b', [0.3, 'cell phone, 2'])
    assert w.close() == {'a': 2, 'b': 1}
    assert ler(tmp_path / 'a.csv') == [['t', 'x', 'ok'], ['0.1', '1.5', '1'], ['0.2', '-2', '0']]
    assert ler(tmp_path / 'b.csv') == [['t', 'nome'], ['0.3', 'cell phone, 2']]


def test_floats_com_9_algarismos(tmp_path):
    import numpy as np
    w = CsvWriter(str(tmp_path), COLUNAS, queue_size=10)
    f32 = float(np.float32(-191.9634))              # float32 em float64: ruído nas casas finais
    w.write('a', [1792682528.9675686, f32, False])
    w.write('a', [1e-12, 3, True])                   # inteiro continua inteiro
    w.close()
    linhas = ler(tmp_path / 'a.csv')
    assert linhas[1][:2] == ['1.79268253e+09', '-191.963394']
    assert linhas[2][:2] == ['1e-12', '3']
    assert float(linhas[1][1]) == pytest.approx(f32, rel=1e-8)


def test_csv_sem_linhas_nao_cria_arquivo(tmp_path):
    w = CsvWriter(str(tmp_path), COLUNAS, queue_size=10)
    w.write('a', [0.0, 1.0, True])
    assert w.close() == {'a': 1}
    assert not (tmp_path / 'b.csv').exists()


def test_arquivo_desconhecido(tmp_path):
    w = CsvWriter(str(tmp_path), COLUNAS, queue_size=10)
    with pytest.raises(KeyError):
        w.write('c', [1])
    w.close()


def test_depois_de_fechar_descarta(tmp_path):
    w = CsvWriter(str(tmp_path), COLUNAS, queue_size=10)
    w.close()
    assert not w.write('a', [0.0, 1.0, True])
    assert w.close() == {}                       # fechar duas vezes não trava


def test_fila_cheia_conta_descartes(tmp_path, monkeypatch):
    # Segura a thread escritora na abertura do primeiro arquivo
    liberar = threading.Event()
    abrir = open

    def open_lento(*args, **kwargs):
        liberar.wait(5.0)
        return abrir(*args, **kwargs)

    monkeypatch.setattr(csv_writer, 'open', open_lento, raising=False)
    w = CsvWriter(str(tmp_path), COLUNAS, queue_size=2)
    resultados = [w.write('a', [i, 0.0, True]) for i in range(10)]
    liberar.set()
    linhas = w.close()
    assert w.dropped == resultados.count(False) > 0
    assert linhas['a'] == resultados.count(True)


def test_falha_de_escrita(tmp_path):
    w = CsvWriter(str(tmp_path / 'nao_existe'), COLUNAS, queue_size=10)
    w.write('a', [0.0, 1.0, True])
    w.write('a', [0.1, 1.0, True])
    assert w.close() == {}
    assert 'a.csv' in w.error
