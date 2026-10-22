"""Testes da lógica das gravações do experiment_recorder (sem ROS)."""

from datetime import datetime

import pytest

from pantilt_experiment.session import (
    ACTION_CLOSE, ACTION_OPEN, COLUMNS, MAX_LABEL_LEN, MODE_CENTER, MODE_TRACK, MODE_UNKNOWN,
    OUTCOME_CENTERED, OUTCOME_FAILED, OUTCOME_INTERRUPTED, OUTCOME_NO_TARGET, OUTCOME_TRACKED,
    SOURCE_DIRECT, SOURCE_INSPECTION, RecordingTrigger, check_image_mode, clean_label,
    csv_columns, final_basename, folder_basename, git_info, infer_mode, next_repetition, outcome,
    validate_params,
)

OPEN_INSPECTION = (ACTION_OPEN, SOURCE_INSPECTION)
OPEN_DIRECT = (ACTION_OPEN, SOURCE_DIRECT)
CLOSE = (ACTION_CLOSE,)


# ---------------- Rótulo e nomes ----------------
@pytest.mark.parametrize('texto, esperado', [
    ('Degrau Kp 0.8', 'degrau_kp_0.8'),
    ('pid_kp0.8', 'pid_kp0.8'),
    ('Ação Rápida', 'acao_rapida'),
    ('  a   b  ', 'a_b'),
    ('fuzzy/v2#teste', 'fuzzyv2teste'),
    ('_.x.-', 'x'),
    ('!!!', ''),
    ('', ''),
])
def test_clean_label(texto, esperado):
    assert clean_label(texto) == esperado


def test_clean_label_limita_o_tamanho():
    assert clean_label('a' * 60) == 'a' * MAX_LABEL_LEN
    assert clean_label('a' * (MAX_LABEL_LEN - 1) + ' b') == 'a' * (MAX_LABEL_LEN - 1)


def test_next_repetition(tmp_path):
    assert next_repetition(str(tmp_path / 'nao_existe')) == 1
    assert next_repetition(str(tmp_path)) == 1
    for nome in ('001_20261022_100000_garrafa_center', '007_20261022_100500',
                 'abc', '12_curto', '_analise'):
        (tmp_path / nome).mkdir()
    (tmp_path / '050_arquivo.txt').write_text('')   # arquivo também conta: evita colisão
    assert next_repetition(str(tmp_path)) == 51


def test_nomes_da_pasta():
    base = folder_basename(3, datetime(2026, 10, 22, 14, 5, 9))
    assert base == '003_20261022_140509'
    assert final_basename(base, 'garrafa', MODE_CENTER) == '003_20261022_140509_garrafa_center'
    assert final_basename(base, 'Isolador Disco', MODE_TRACK) == \
        '003_20261022_140509_isolador_disco_track'
    assert final_basename(base, '', MODE_UNKNOWN) == '003_20261022_140509_direto_nd'


def test_colunas_comecam_pelo_tempo():
    cols = csv_columns()
    assert set(cols) == set(COLUMNS)
    for nome, c in cols.items():
        assert c[:2] == ('t', 't_rx')
        assert len(set(c)) == len(c), nome


# ---------------- Modo e desfecho ----------------
@pytest.mark.parametrize('servo_mode, tracking, esperado', [
    (0, False, MODE_CENTER),
    (1, False, MODE_TRACK),
    (0, True, MODE_CENTER),       # o servo manda
    (None, True, MODE_TRACK),
    (None, False, MODE_UNKNOWN),
])
def test_infer_mode(servo_mode, tracking, esperado):
    assert infer_mode(servo_mode, tracking) == esperado


def test_outcome():
    ok = [{'success': False}, {'success': True}]
    falha = [{'success': False}]
    sem_resultado = [{'success': None}]
    assert outcome(SOURCE_INSPECTION, ok, False, True) == OUTCOME_INTERRUPTED
    assert outcome(SOURCE_INSPECTION, ok, False, False) == OUTCOME_CENTERED
    assert outcome(SOURCE_INSPECTION, falha, True, False) == OUTCOME_TRACKED
    assert outcome(SOURCE_INSPECTION, [], False, False) == OUTCOME_NO_TARGET
    assert outcome(SOURCE_INSPECTION, falha, False, False) == OUTCOME_FAILED
    assert outcome(SOURCE_DIRECT, [], False, False) == OUTCOME_FAILED
    assert outcome(SOURCE_DIRECT, sem_resultado, False, False) == OUTCOME_FAILED


# ---------------- Parâmetros ----------------
def test_parametros_validos():
    validate_params('none', 1.0, 1.0, 1.0, 2000)
    validate_params('none', 0.0, 0.0, 0.0, 1)


@pytest.mark.parametrize('args', [
    ('foto', 1.0, 1.0, 1.0, 10),
    ('none', -0.1, 1.0, 1.0, 10),
    ('none', 1.0, -0.1, 1.0, 10),
    ('none', 1.0, 1.0, -1.0, 10),
    ('none', 1.0, 1.0, 1.0, 0),
])
def test_parametros_invalidos(args):
    with pytest.raises(ValueError):
        validate_params(*args)


def test_modos_de_imagem_da_etapa_3_ainda_recusados():
    check_image_mode('none')
    for modo in ('snapshots', 'video'):
        with pytest.raises(ValueError, match='ainda não implementado'):
            check_image_mode(modo)


# ---------------- Gatilho: inspeção ----------------
def test_inspecao_abre_e_fecha_depois_da_cauda():
    g = RecordingTrigger(tail_s=1.0)
    assert g.inspection_state('IDLE', 0.0) == []
    assert g.inspection_state('SEARCHING', 1.0) == [OPEN_INSPECTION]
    # status repetido a 5 Hz e transições internas não reabrem
    assert g.inspection_state('SEARCHING', 1.2) == []
    assert g.inspection_state('CENTERING', 3.0) == []
    assert g.inspection_state('TRACKING', 4.0) == []
    assert g.inspection_state('IDLE', 10.0) == []
    assert g.in_tail
    assert g.inspection_state('IDLE', 10.2) == []           # não estende a cauda
    assert g.tick(10.9) == []
    assert g.tick(11.0) == [CLOSE]
    assert not g.active
    assert g.tick(12.0) == []


def test_nova_inspecao_durante_a_cauda_fecha_a_anterior():
    g = RecordingTrigger(tail_s=1.0)
    g.inspection_state('SEARCHING', 0.0)
    g.inspection_state('IDLE', 5.0)
    assert g.inspection_state('SEARCHING', 5.5) == [CLOSE, OPEN_INSPECTION]
    assert g.active and not g.in_tail
    assert g.tick(7.0) == []


def test_inspecao_ja_em_andamento_ao_subir():
    # transient_local: o primeiro status recebido pode já ser fora do IDLE
    g = RecordingTrigger(tail_s=1.0)
    assert g.inspection_state('TRACKING', 0.0) == [OPEN_INSPECTION]


def test_cauda_zero_fecha_no_proximo_tick():
    g = RecordingTrigger(tail_s=0.0)
    g.inspection_state('SEARCHING', 0.0)
    g.inspection_state('IDLE', 1.0)
    assert g.tick(1.0) == [CLOSE]


# ---------------- Gatilho: goal Center direto ----------------
def test_goal_direto_abre_e_fecha():
    g = RecordingTrigger(tail_s=1.0)
    assert g.center_goal_active('a', 0.0) == [OPEN_DIRECT]
    assert g.center_goal_active('a', 0.1) == []             # ACCEPTED -> EXECUTING
    assert g.center_goal_finished('b', 2.0) == []           # outro goal: ignorado
    assert not g.in_tail
    assert g.center_goal_finished('a', 3.0) == []
    assert g.in_tail
    assert g.tick(4.0) == [CLOSE]


def test_goal_da_inspecao_nao_abre_outra_gravacao():
    g = RecordingTrigger(tail_s=1.0)
    g.inspection_state('CENTERING', 0.0)
    assert g.center_goal_active('a', 0.5) == []
    g.center_goal_finished('a', 2.0)
    assert not g.in_tail                                     # quem fecha é o IDLE
    assert g.source == SOURCE_INSPECTION


def test_goal_direto_substituido_abre_nova_gravacao():
    g = RecordingTrigger(tail_s=1.0)
    g.center_goal_active('a', 0.0)
    assert g.center_goal_active('b', 1.0) == [CLOSE, OPEN_DIRECT]
    assert g.goal == 'b'


def test_goal_direto_na_cauda_da_inspecao():
    g = RecordingTrigger(tail_s=1.0)
    g.inspection_state('SEARCHING', 0.0)
    g.inspection_state('IDLE', 5.0)
    assert g.center_goal_active('x', 5.3) == [CLOSE, OPEN_DIRECT]


def test_inspecao_durante_goal_direto():
    g = RecordingTrigger(tail_s=1.0)
    g.center_goal_active('a', 0.0)
    assert g.inspection_state('SEARCHING', 1.0) == [CLOSE, OPEN_INSPECTION]


def test_stop():
    g = RecordingTrigger(tail_s=1.0)
    assert g.stop() == []
    g.center_goal_active('a', 0.0)
    assert g.stop() == [CLOSE]
    assert not g.active


# ---------------- Commit ----------------
def test_git_info_fora_de_repositorio(tmp_path):
    assert git_info(str(tmp_path)) is None


def test_git_info_no_repositorio(tmp_path):
    import subprocess
    repo = str(tmp_path)
    subprocess.run(['git', 'init', '-q', repo], check=True)
    (tmp_path / 'a.txt').write_text('1')
    subprocess.run(['git', '-C', repo, 'add', 'a.txt'], check=True)
    subprocess.run(['git', '-C', repo, '-c', 'user.name=t', '-c', 'user.email=t@t',
                    'commit', '-q', '-m', 'x'], check=True)
    info = git_info(repo)
    assert len(info['commit']) == 40
    assert info['alteracoes_locais'] is False
    (tmp_path / 'a.txt').write_text('2')
    assert git_info(repo)['alteracoes_locais'] is True
