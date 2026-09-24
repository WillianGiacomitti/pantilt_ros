"""Testes da arbitragem do command_mux (sem ROS, com tempos sintéticos)."""

import pytest

from pantilt_hardware.command_arbiter import (
    SOURCE_AUTO, SOURCE_NONE, SOURCE_WEB, CommandArbiter,
)

HOLD = 1.0
TIMEOUT = 0.3


@pytest.fixture
def arb():
    return CommandArbiter(operator_hold_s=HOLD, cmd_timeout_s=TIMEOUT)


@pytest.mark.parametrize('hold, timeout', [(0.0, 0.3), (1.0, 0.0), (-1.0, 0.3), (1.0, -0.1)])
def test_parametros_invalidos(hold, timeout):
    with pytest.raises(ValueError):
        CommandArbiter(hold, timeout)


def test_sem_comandos_fonte_none(arb):
    assert arb.source(0.0) == SOURCE_NONE
    assert arb.check_watchdog(10.0) is None


def test_auto_repassado_sem_web(arb):
    assert arb.on_auto(0.0, is_zero=False)
    assert arb.source(0.1) == SOURCE_AUTO


def test_web_descarta_auto_dentro_da_janela(arb):
    arb.on_web(0.0, is_velocity=True)
    assert not arb.on_auto(0.5, is_zero=False)
    assert not arb.on_auto(HOLD - 0.01, is_zero=False)
    assert arb.on_auto(HOLD + 0.01, is_zero=False)


def test_posicao_web_tambem_abre_janela(arb):
    arb.on_web(0.0, is_velocity=False)
    assert arb.source(0.5) == SOURCE_WEB
    assert not arb.on_auto(0.5, is_zero=False)


def test_auto_descartado_nao_renova_fonte_auto(arb):
    arb.on_web(0.0, is_velocity=True)
    arb.on_auto(0.9, is_zero=False)       # descartado
    assert arb.source(HOLD + 0.01) == SOURCE_NONE


@pytest.mark.parametrize('fonte', [SOURCE_AUTO, SOURCE_WEB])
def test_watchdog_dispara_uma_vez(arb, fonte):
    if fonte == SOURCE_AUTO:
        arb.on_auto(0.0, is_zero=False)
    else:
        arb.on_web(0.0, is_velocity=True, is_zero=False)
    assert arb.check_watchdog(TIMEOUT - 0.01) is None
    assert arb.check_watchdog(TIMEOUT + 0.01) == fonte
    assert arb.check_watchdog(TIMEOUT + 0.5) is None
    assert not arb.armed


def test_fluxo_continuo_nao_dispara(arb):
    for i in range(20):
        arb.on_auto(i * 0.1, is_zero=False)
        assert arb.check_watchdog(i * 0.1 + 0.05) is None


def test_watchdog_nao_dispara_apos_velocidade_zero(arb):
    arb.on_auto(0.0, is_zero=False)
    arb.on_auto(0.1, is_zero=True)
    assert arb.check_watchdog(1.0) is None


def test_watchdog_nao_dispara_apos_posicao(arb):
    arb.on_auto(0.0, is_zero=False)
    arb.on_web(0.1, is_velocity=False)    # posição da web desarma
    assert arb.check_watchdog(1.0) is None


def test_novo_comando_rearma(arb):
    arb.on_auto(0.0, is_zero=False)
    assert arb.check_watchdog(0.4) == SOURCE_AUTO
    arb.on_auto(1.0, is_zero=False)
    assert arb.armed
    assert arb.check_watchdog(1.4) == SOURCE_AUTO


def test_web_interrompe_auto_e_watchdog_segue_a_web(arb):
    arb.on_auto(0.0, is_zero=False)
    arb.on_web(0.1, is_velocity=True, is_zero=False)
    arb.on_auto(0.2, is_zero=False)       # descartado: não renova o watchdog
    assert arb.check_watchdog(0.1 + TIMEOUT + 0.01) == SOURCE_WEB


def test_transicoes_de_fonte(arb):
    assert arb.source(0.0) == SOURCE_NONE
    arb.on_auto(0.0, is_zero=False)
    assert arb.source(0.1) == SOURCE_AUTO
    arb.on_web(0.2, is_velocity=True)
    assert arb.source(0.2) == SOURCE_WEB
    assert arb.source(0.2 + HOLD - 0.01) == SOURCE_WEB
    assert arb.source(0.2 + HOLD + 0.01) == SOURCE_NONE
    arb.on_auto(2.0, is_zero=False)
    assert arb.source(2.0) == SOURCE_AUTO
    assert arb.source(2.0 + TIMEOUT + 0.01) == SOURCE_NONE
