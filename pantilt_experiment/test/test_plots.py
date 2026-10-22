"""Testes das figuras dos ensaios (matplotlib com backend Agg, sem ROS)."""

from conftest import summary, write_recording

from pantilt_experiment import analisar_ensaios
from pantilt_experiment.analysis import analyze_recording
from pantilt_experiment.plots import _tick, number, plot_comparisons, plot_recording


def test_virgula_decimal():
    assert _tick(1.5) == '1,5'
    assert _tick(-0.25) == '−0,25'
    assert _tick(-0.0) == '0'
    assert _tick(20.0) == '20'
    assert number(3.14159) == '3,14'
    assert number(-2.0, 1) == '−2,0'
    assert number(float('nan')) == '—'


def test_figura_por_gravacao_e_comparativas(ensaios, tmp_path):
    analyses = [analyze_recording(str(p)) for p in sorted((ensaios / 'kp0.8').iterdir())]
    paths = plot_recording(analyses[0], str(tmp_path / 'fig' / 'kp0.8' / 'rec1'))
    assert sorted(p.rsplit('.', 1)[1] for p in paths) == ['pdf', 'png']
    assert all((tmp_path / 'fig' / 'kp0.8' / f'rec1.{e}').stat().st_size > 0
               for e in ('png', 'pdf'))
    paths = plot_comparisons(analyses, str(tmp_path / 'fig'))
    names = sorted(p.rsplit('/', 1)[1] for p in paths)
    assert names == ['comparativo_metricas.pdf', 'comparativo_metricas.png',
                     'comparativo_respostas.pdf', 'comparativo_respostas.png']


def test_gravacao_sem_servo_nao_gera_figura(tmp_path):
    folder = write_recording(str(tmp_path / 'r' / '001_x'), summary(outcome='sem_alvo'))
    assert plot_recording(analyze_recording(folder), str(tmp_path / 'fig')) == []


def test_comparativas_sem_gravacoes_validas(tmp_path):
    assert plot_comparisons([], str(tmp_path)) == []


def test_linha_de_comando_com_figuras(ensaios):
    assert analisar_ensaios.main([str(ensaios)]) == 0
    fig = ensaios / '_analise' / 'figuras'
    assert len(list((fig / 'kp0.8').glob('*.png'))) == 2
    assert (fig / 'comparativo_metricas.pdf').is_file()
