"""Testes das métricas dos ensaios (analysis.py e linha de comando, sem figuras e sem ROS)."""

import csv
import math

import numpy as np
import pytest
from conftest import F_PX, center_recording, summary, write_recording

from pantilt_experiment import analisar_ensaios
from pantilt_experiment.analysis import (
    METRICS_CSV_COLUMNS, analyze_recording, detector_stats, find_recordings, iae, load_csv,
    overshoot, principal_goal, search_times, summarize, tracking_error, write_csv,
)


def deg(px):
    return math.degrees(math.atan(px / F_PX))


# ---------------- Funções isoladas ----------------
def test_iae_trapezio_e_janela():
    t = [0.0, 1.0, 2.0, 3.0]
    assert iae(t, [2.0, 2.0, 2.0, 2.0], 0.0, 3.0) == pytest.approx(6.0)
    assert iae(t, [0.0, -2.0, 0.0, 9.0], 0.0, 2.0) == pytest.approx(2.0)   # |erro|, até t=2
    assert math.isnan(iae(t, [1.0] * 4, 2.5, 3.0))                        # uma amostra só


def test_iae_ordena_por_tempo():
    assert iae([1.0, 0.0, 2.0], [2.0, 0.0, 2.0], 0.0, 2.0) == pytest.approx(3.0)


def test_sobressinal_com_cruzamento():
    mp, pct = overshoot(np.radians([10.0, 5.0, 0.0, -3.0, -1.0, 2.0]))
    assert mp == pytest.approx(3.0)
    assert pct == pytest.approx(30.0)


def test_sobressinal_negativo_e_sem_cruzamento():
    mp, pct = overshoot(np.radians([-8.0, -4.0, 2.0, 0.0]))
    assert (mp, pct) == (pytest.approx(2.0), pytest.approx(25.0))
    assert overshoot(np.radians([10.0, 5.0, 1.0])) == (0.0, 0.0)


def test_sobressinal_com_erro_inicial_pequeno_nao_tem_porcentagem():
    mp, pct = overshoot(np.radians([0.5, -0.2]))
    assert mp == pytest.approx(0.2)
    assert math.isnan(pct)
    assert overshoot(np.zeros(3))[0] == 0.0
    assert all(math.isnan(v) for v in overshoot([]))


def test_goal_principal_e_o_da_primeira_convergencia():
    goals = [{'goal': 'g1', 't_inicio': 0.0}, {'goal': 'g2', 't_inicio': 2.0}]
    fb = {'goal': np.array(['g1', 'g2', 'g2'], dtype=object),
          'centered': np.array([0.0, 0.0, 1.0])}
    assert principal_goal(goals, fb)['goal'] == 'g2'
    fb['centered'][:] = 0.0
    assert principal_goal(goals, fb)['goal'] == 'g1'
    assert principal_goal([], fb) is None


def test_erro_de_rastreamento_depois_do_primeiro_centered():
    fb = {'goal': np.array(['g'] * 6, dtype=object),
          'error_px': np.array([50.0, 40.0, 6.0, 0.0, 8.0, 10.0]),
          'target_visible': np.array([1.0, 1.0, 1.0, 0.0, 1.0, 1.0]),
          'centered': np.array([0.0, 0.0, 1.0, 0.0, 1.0, 1.0])}
    assert tracking_error(fb, {'goal': 'g'}) == pytest.approx(8.0)
    fb['centered'][:] = 0.0
    assert math.isnan(tracking_error(fb, {'goal': 'g'}))


def test_tempos_de_busca():
    events = {'t': np.array([-0.5, 0.2, 1.0, 3.0]),
              'source': np.array(['inspecao', 'inspecao', 'inspecao', 'inspecao'], dtype=object),
              'event': np.array(['IDLE', 'SEARCHING', 'CENTERING', 'IDLE'], dtype=object)}
    servo = {'t': np.array([1.1, 2.0, 2.2]), 'centered': np.array([0.0, 1.0, 1.0])}
    assert search_times(events, servo) == (pytest.approx(0.8), pytest.approx(1.8))
    assert all(math.isnan(v) for v in search_times(None, servo))


def test_estatisticas_do_detector():
    timing = {'t': np.array([0.0, 0.1, 0.2, 0.3, 0.4]),
              'inference_ms': np.array([70.0, 70.0, 70.0, 70.0, 90.0]),
              'latency_ms': np.array([100.0] * 5)}
    s = detector_stats(timing)
    assert s['det_fps'] == pytest.approx(10.0)
    assert s['det_inf_ms'] == pytest.approx(74.0)
    assert s['det_inf_p95_ms'] == pytest.approx(86.0)
    assert s['det_lat_p95_ms'] == pytest.approx(100.0)
    assert all(math.isnan(v) for v in detector_stats(None).values())


# ---------------- Gravações completas ----------------
def test_gravacao_center(tmp_path):
    folder = center_recording(str(tmp_path / 'kp0.8' / '001_x_garrafa_center'))
    a = analyze_recording(folder)
    r = a.row
    assert (r['rotulo'], r['repeticao'], r['modo'], r['desfecho']) == \
        ('kp0.8', 1, 'center', 'centralizado')
    assert r['sucesso'] is True
    assert (r['pan_kp'], r['controller'], r['f_x']) == (0.8, 'pid', F_PX)
    assert r['t_c_s'] == pytest.approx(0.7)
    assert r['e_r_px'] == pytest.approx(5.0)
    assert a.t0 == pytest.approx(1.1)
    assert a.hold_start == pytest.approx(1.8)
    assert a.t_centered == pytest.approx(2.8)
    assert r['iae_px_s'] == pytest.approx(27.25, rel=1e-3)
    assert r['mp_pan_deg'] == pytest.approx(deg(20.0), rel=1e-4)
    assert r['mp_pan_pct'] == pytest.approx(100.0 * deg(20.0) / deg(100.0), rel=1e-4)
    assert r['mp_tilt_deg'] == 0.0 and math.isnan(r['mp_tilt_pct'])
    assert math.isnan(r['e_d_px'])                           # só no TRACK
    assert r['t_b_s'] == pytest.approx(0.85)
    assert r['t_i_s'] == pytest.approx(2.75)
    assert r['saturacao_pct'] == pytest.approx(100.0 * 2 / 29)
    assert r['det_fps'] == pytest.approx(10.0)
    assert a.tolerance_px == 20.0


def test_gravacao_track_usa_o_goal_que_convergiu(ensaios):
    folder = sorted((ensaios / 'kp0.8').iterdir())[1]
    a = analyze_recording(str(folder))
    r = a.row
    assert a.goal['goal'] == 'bbbb0002'
    assert r['readquisicoes'] == 1
    assert r['goals_center'] == 2
    assert a.t0 == pytest.approx(2.1)
    assert r['t_c_s'] == pytest.approx(0.4)
    assert r['e_d_px'] == pytest.approx(5.0)
    assert r['t_b_s'] == pytest.approx(0.3)
    assert r['t_i_s'] == pytest.approx(3.4)
    assert r['mp_pan_deg'] == 0.0
    # TRACK: IAE até o fim do goal, só com o alvo visível
    assert r['iae_px_s'] > 0.0


def test_goal_sem_convergencia_tem_t_c_nan_e_iae_ate_o_fim(tmp_path):
    from conftest import goal, servo_row
    servo = [servo_row(round(1.1 + 0.1 * i, 2), 40.0) for i in range(11)]
    s = summary(outcome='falhou', goals=[goal('cccc0001', 1.0, 2.5, t_c=0.0, e_r=0.0,
                                              success=False)])
    folder = write_recording(str(tmp_path / 'r' / '001_x'), s, servo=servo)
    r = analyze_recording(folder).row
    assert math.isnan(r['t_c_s']) and math.isnan(r['e_r_px'])
    assert r['iae_px_s'] == pytest.approx(40.0)              # 40 px por 1,0 s
    assert r['sucesso'] is False
    assert math.isnan(r['det_fps'])                          # sem timing.csv


def test_sem_alvo_sem_servo(tmp_path):
    s = summary(outcome='sem_alvo', mode='nd')
    folder = write_recording(str(tmp_path / 'r' / '001_x'), s,
                             events=[[0.1, 0.1, 'inspecao', 'SEARCHING', '']])
    r = analyze_recording(folder).row
    assert r['goals_center'] == 0
    assert all(math.isnan(r[k]) for k in ('t_c_s', 'iae_px_s', 'mp_pan_deg', 't_b_s'))


# ---------------- Resumo, listagem e CSV ----------------
def test_resumo_por_rotulo_exclui_interrompidas():
    base = {k: math.nan for k in METRICS_CSV_COLUMNS}
    rows = [dict(base, rotulo='a', desfecho='centralizado', sucesso=True, t_c_s=1.0),
            dict(base, rotulo='a', desfecho='falhou', sucesso=False, t_c_s=2.0),
            dict(base, rotulo='a', desfecho='centralizado', sucesso=True, t_c_s=3.0),
            dict(base, rotulo='a', desfecho='interrompido', sucesso=False, t_c_s=100.0),
            dict(base, rotulo='b', desfecho='centralizado', sucesso=True, t_c_s=5.0)]
    lines = {(x['rotulo'], x['metrica']): x for x in summarize(rows)}
    assert lines[('a', 'gravacoes')]['n'] == 4
    assert lines[('a', 'interrompidas')]['n'] == 1
    t_c = lines[('a', 't_c_s')]
    assert (t_c['n'], t_c['media'], t_c['desvio'], t_c['minimo'], t_c['maximo']) == \
        (3, 2.0, 1.0, 1.0, 3.0)
    assert lines[('a', 'sucesso')]['media'] == pytest.approx(2 / 3)
    assert math.isnan(lines[('b', 't_c_s')]['desvio'])      # uma amostra: sem desvio
    assert lines[('a', 'e_r_px')]['n'] == 0


def test_listagem_ignora_analise_e_gravacoes_em_andamento(ensaios):
    (ensaios / '_analise').mkdir()
    (ensaios / 'kp0.8' / '003_em_andamento').mkdir()
    found, pending = find_recordings(str(ensaios))
    assert len(found) == 2
    assert [p.split('/')[-1] for p in pending] == ['003_em_andamento']
    assert find_recordings(str(ensaios), ['outro']) == ([], [])


def test_csv_com_nan_vazio_e_booleano(tmp_path):
    path = str(tmp_path / 'x.csv')
    write_csv(path, ('a', 'b', 'c'), [{'a': 1.5, 'b': math.nan, 'c': True}, {'a': 'x'}])
    with open(path, encoding='utf-8') as f:
        assert list(csv.reader(f)) == [['a', 'b', 'c'], ['1.5', '', '1'], ['x', '', '']]
    table = load_csv(path)
    assert math.isnan(table['b'][0])


def test_linha_de_comando_sem_figuras(ensaios, capsys):
    assert analisar_ensaios.main([str(ensaios), '--sem-figuras']) == 0
    out = ensaios / '_analise'
    metricas = load_csv(str(out / 'metricas.csv'))
    assert len(metricas['t_c_s']) == 2
    assert metricas['t_c_s'][0] == pytest.approx(0.7)
    assert (out / 'resumo_por_rotulo.csv').is_file()
    assert not (out / 'figuras').exists()
    assert 'kp0.8: 2 gravações' in capsys.readouterr().out


def test_linha_de_comando_sem_gravacoes(tmp_path):
    assert analisar_ensaios.main([str(tmp_path), '--sem-figuras']) == 1
