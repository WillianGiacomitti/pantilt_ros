"""Testes da máquina de estados do inspection_manager (sem ROS, com stamps sintéticos)."""

import pytest

from pantilt_manager.state_machine import (
    ACTION_CANCEL_CENTER, ACTION_CANCEL_SCAN, ACTION_CLEAR_TARGET, ACTION_SEND_CENTER,
    ACTION_SEND_SCAN, LOST_MESSAGE, MODE_CENTER, MODE_TRACK, REASON_OPERATOR, REASON_SHUTDOWN,
    REASON_STOP, STATE_CENTERING, STATE_IDLE, STATE_SEARCHING, STATE_TRACKING,
    InspectionMachine, Params, validate_params,
)

KEYS = ['garrafa', 'copo']
SINCE = 100.0


def machine(**kw):
    return InspectionMachine(Params(**kw), KEYS)


def searching(mode=MODE_CENTER, **kw):
    m = machine(**kw)
    assert m.start('garrafa', mode, SINCE) == [ACTION_SEND_SCAN]
    return m


def confirm(m, t0=SINCE + 1.0):
    """Envia confirm_frames alvos válidos; retorna as ações do último."""
    actions = []
    for i in range(m.params.confirm_frames):
        actions = m.target(True, 'garrafa', t0 + 0.1 * i)
    return actions


def centering(mode=MODE_CENTER, **kw):
    m = searching(mode, **kw)
    assert confirm(m) == [ACTION_CANCEL_SCAN, ACTION_SEND_CENTER]
    return m


# ---------------- Parâmetros ----------------
def test_parametros_validos():
    validate_params(3, 60.0, 2, 20.0, 1.0, 1.0)
    validate_params(1, 0.1, 0, 0.1, 0.0, 0.1)


@pytest.mark.parametrize('args', [
    (0, 60.0, 2, 20.0, 1.0, 1.0),
    (3, 0.0, 2, 20.0, 1.0, 1.0),
    (3, 60.0, -1, 20.0, 1.0, 1.0),
    (3, 60.0, 2, 0.0, 1.0, 1.0),
    (3, 60.0, 2, 20.0, -0.1, 1.0),
    (3, 60.0, 2, 20.0, 1.0, 0.0),
])
def test_parametros_invalidos(args):
    with pytest.raises(ValueError):
        validate_params(*args)


# ---------------- Início ----------------
def test_estado_inicial():
    s = machine().status()
    assert (s.state, s.equipment, s.autonomous, s.error_px) == (STATE_IDLE, '', False, 0.0)
    assert s.message == 'Pronto'


def test_start_permitido():
    assert machine().check_start('garrafa', MODE_TRACK, False, []) == ''


@pytest.mark.parametrize('equipment, mode, operator, unavailable, trecho', [
    ('pneu', MODE_CENTER, False, [], 'desconhecido'),
    ('garrafa', 2, False, [], 'mode=2'),
    ('garrafa', MODE_CENTER, True, [], 'Operador'),
    ('garrafa', MODE_CENTER, False, ['scan_node (/control/scan)'], 'scan_node'),
])
def test_start_recusado(equipment, mode, operator, unavailable, trecho):
    motivo = machine().check_start(equipment, mode, operator, unavailable)
    assert trecho in motivo


def test_start_recusado_fora_do_idle():
    m = searching()
    assert 'em andamento' in m.check_start('copo', MODE_CENTER, False, [])


def test_start_entra_em_searching():
    m = searching()
    s = m.status()
    assert (s.state, s.equipment, s.autonomous) == (STATE_SEARCHING, 'garrafa', True)
    assert m.scan_active and not m.center_active


# ---------------- Confirmação ----------------
def test_confirma_com_frames_consecutivos():
    m = searching(confirm_frames=3)
    assert m.target(True, 'garrafa', 101.0) == []
    assert m.target(True, 'garrafa', 101.1) == []
    assert m.target(True, 'garrafa', 101.2) == [ACTION_CANCEL_SCAN, ACTION_SEND_CENTER]
    assert m.state == STATE_CENTERING
    assert m.center_active and not m.scan_active


def test_deteccao_falsa_zera_a_contagem():
    m = searching(confirm_frames=3)
    m.target(True, 'garrafa', 101.0)
    m.target(True, 'garrafa', 101.1)
    m.target(False, 'garrafa', 101.2)
    m.target(True, 'garrafa', 101.3)
    assert m.target(True, 'garrafa', 101.4) == []
    assert m.target(True, 'garrafa', 101.5) == [ACTION_CANCEL_SCAN, ACTION_SEND_CENTER]


def test_outra_chave_e_stamp_antigo_sao_ignorados():
    m = searching(confirm_frames=2)
    m.target(True, 'garrafa', 101.0)
    # Não contam nem zeram a contagem
    assert m.target(True, 'copo', 101.1) == []
    assert m.target(False, 'copo', 101.1) == []
    assert m.target(True, 'garrafa', SINCE) == []
    assert m.target(False, 'garrafa', SINCE - 1.0) == []
    assert m.target(True, 'garrafa', 101.2) == [ACTION_CANCEL_SCAN, ACTION_SEND_CENTER]


def test_alvo_fora_do_searching_e_ignorado():
    m = machine()
    assert m.target(True, 'garrafa', 101.0) == []
    m = centering()
    assert m.target(True, 'garrafa', 102.0) == []
    assert m.state == STATE_CENTERING


# ---------------- Varredura ----------------
def test_passada_completa_sem_alvo():
    m = searching()
    assert m.scan_finished(True, True) == [ACTION_CLEAR_TARGET]
    s = m.status()
    assert (s.state, s.equipment, s.autonomous) == (STATE_IDLE, '', False)
    assert 'varredura completa' in s.message


def test_passada_por_tempo_sem_alvo():
    m = searching(scan_timeout_s=45.0)
    assert m.scan_finished(True, False) == [ACTION_CLEAR_TARGET]
    assert 'em 45 s' in m.message


def test_varredura_abortada():
    m = searching()
    assert m.scan_finished(False, False, 'sem /joint_states') == [ACTION_CLEAR_TARGET]
    assert m.message == 'Varredura interrompida: sem /joint_states'


def test_varredura_recusada():
    m = searching()
    assert m.scan_rejected() == [ACTION_CLEAR_TARGET]
    assert m.state == STATE_IDLE


def test_resultado_da_varredura_fora_do_searching_e_ignorado():
    m = centering()
    assert m.scan_finished(False, False, 'cancelado') == []
    assert m.scan_rejected() == []
    assert m.state == STATE_CENTERING


# ---------------- Centralização ----------------
def test_center_com_sucesso():
    m = centering()
    m.center_feedback(35.0, False)
    assert m.status().error_px == 35.0
    assert m.center_feedback(8.0, True) == []
    assert m.state == STATE_CENTERING        # no modo CENTER não há TRACKING
    assert m.center_finished(True, 'centralizado', 2.345, 6.78) == [ACTION_CLEAR_TARGET]
    s = m.status()
    assert (s.state, s.autonomous, s.error_px) == (STATE_IDLE, False, 0.0)
    assert s.message == 'Centralizado em 2.35 s, erro residual 6.8 px'


def test_track_entra_em_tracking_e_nao_volta():
    m = centering(MODE_TRACK)
    m.center_feedback(30.0, False)
    assert m.state == STATE_CENTERING
    m.center_feedback(5.0, True)
    assert m.state == STATE_TRACKING
    assert m.message == 'Rastreando'
    m.center_feedback(40.0, False)
    assert m.state == STATE_TRACKING
    assert m.status().error_px == 40.0


def test_center_recusada():
    m = centering()
    assert m.center_rejected() == [ACTION_CLEAR_TARGET]
    assert m.state == STATE_IDLE
    assert 'recusada' in m.message


def test_aborto_por_limite_volta_ao_idle():
    m = centering(MODE_TRACK)
    msg = 'pan preso no limite positivo (29.0°): comando empurrando há 2.0 s sem movimento'
    assert m.center_finished(False, msg) == [ACTION_CLEAR_TARGET]
    assert m.state == STATE_IDLE
    assert m.message == f'Centralização abortada: {msg}'


def test_eventos_da_center_fora_do_estado_sao_ignorados():
    m = searching()
    assert m.center_feedback(5.0, True) == []
    assert m.center_finished(True) == []
    assert m.center_rejected() == []
    assert m.state == STATE_SEARCHING and m.error_px == 0.0


# ---------------- Readquisição ----------------
def test_perda_leva_a_nova_varredura():
    m = centering(max_reacquire=2)
    m.center_feedback(20.0, False)
    assert m.center_finished(False, LOST_MESSAGE) == [ACTION_SEND_SCAN]
    assert m.state == STATE_SEARCHING
    assert m.scan_active and not m.center_active
    assert m.status().error_px == 0.0
    assert m.message == 'Alvo perdido: nova varredura (readquisição 1/2)'
    # Nova confirmação completa, e a mensagem indica a readquisição
    assert confirm(m, 105.0) == [ACTION_CANCEL_SCAN, ACTION_SEND_CENTER]
    assert m.message == 'Centralizando (readquisição 1/2)'


def test_confirmacao_reinicia_apos_a_perda():
    m = searching(confirm_frames=3)
    m.target(True, 'garrafa', 101.0)
    m.target(True, 'garrafa', 101.1)
    m.target(True, 'garrafa', 101.2)
    m.center_finished(False, LOST_MESSAGE)
    assert m.target(True, 'garrafa', 103.0) == []
    assert m.target(True, 'garrafa', 103.1) == []


def test_estouro_de_readquisicoes():
    m = centering(max_reacquire=2)
    for _ in range(2):
        assert m.center_finished(False, LOST_MESSAGE) == [ACTION_SEND_SCAN]
        confirm(m)
    assert m.center_finished(False, LOST_MESSAGE) == [ACTION_CLEAR_TARGET]
    assert m.state == STATE_IDLE
    assert m.message == 'Alvo perdido 3 vezes seguidas (máximo de readquisições: 2)'


def test_sem_readquisicao():
    m = centering(max_reacquire=0)
    assert m.center_finished(False, LOST_MESSAGE) == [ACTION_CLEAR_TARGET]
    assert m.state == STATE_IDLE


def test_centralizar_zera_as_perdas():
    m = centering(MODE_TRACK, max_reacquire=1)
    m.center_finished(False, LOST_MESSAGE)
    confirm(m)
    m.center_feedback(4.0, True)              # recentralizou: perdas zeradas
    assert m.losses == 0
    assert m.center_finished(False, LOST_MESSAGE) == [ACTION_SEND_SCAN]
    confirm(m)
    assert m.center_finished(False, LOST_MESSAGE) == [ACTION_CLEAR_TARGET]


def test_nova_inspecao_zera_as_perdas():
    m = centering(max_reacquire=1)
    m.center_finished(False, LOST_MESSAGE)
    m.stop()
    m.start('copo', MODE_CENTER, 200.0)
    assert m.losses == 0
    assert m.equipment == 'copo'


# ---------------- Interrupções ----------------
@pytest.mark.parametrize('evento, motivo', [
    (lambda m: m.stop(), REASON_STOP),
    (lambda m: m.operator(), REASON_OPERATOR),
    (lambda m: m.shutdown(), REASON_SHUTDOWN),
    (lambda m: m.server_lost('scan_node (/control/scan)'), 'scan_node (/control/scan) saiu do ar'),
])
def test_interrupcao_em_searching(evento, motivo):
    m = searching()
    assert evento(m) == [ACTION_CANCEL_SCAN, ACTION_CLEAR_TARGET]
    s = m.status()
    assert (s.state, s.autonomous, s.equipment, s.message) == (STATE_IDLE, False, '', motivo)
    assert not m.scan_active


@pytest.mark.parametrize('mode', [MODE_CENTER, MODE_TRACK])
def test_interrupcao_em_centering_e_tracking(mode):
    m = centering(mode)
    if mode == MODE_TRACK:
        m.center_feedback(3.0, True)
    assert m.operator() == [ACTION_CANCEL_CENTER, ACTION_CLEAR_TARGET]
    assert m.state == STATE_IDLE and not m.center_active
    assert m.message == REASON_OPERATOR


def test_interrupcao_em_idle_nao_faz_nada():
    m = machine()
    for evento in (m.stop, m.operator, m.shutdown):
        assert evento() == []
    assert m.message == 'Pronto'


def test_resultados_atrasados_depois_do_idle_sao_ignorados():
    m = centering()
    m.stop()
    assert m.center_finished(False, 'cancelado') == []
    assert m.scan_finished(False, False, 'cancelado') == []
    assert m.message == REASON_STOP
