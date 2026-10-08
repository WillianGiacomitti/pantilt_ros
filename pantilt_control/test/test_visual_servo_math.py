"""Testes da lógica do visual_servo_node (sem ROS, com tempos sintéticos)."""

import math

import pytest

from pantilt_control.visual_servo_math import (
    LIMIT_MIN_CMD_DEG_S, MAX_DT_S, MOVE_EPS_DEG, Supervisor, calcular_dt, px_para_angulo,
    saturar,
)

TOL = 20.0
HOLD = 1.0
LOST = 1.0
LIMIT = 2.0


def supervisor(inicio=0.0, hold=HOLD):
    return Supervisor(TOL, hold, LOST, LIMIT, inicio)


# ---------------- Conversão, dt e saturação ----------------
def test_px_para_angulo():
    assert px_para_angulo(0.0, 860.0) == 0.0
    assert px_para_angulo(860.0, 860.0) == pytest.approx(math.pi / 4)
    assert px_para_angulo(-100.0, 500.0) == pytest.approx(-math.atan(0.2))


def test_calcular_dt():
    assert calcular_dt(10.0, None) == (0.0, False)
    assert calcular_dt(10.1, 10.0) == (pytest.approx(0.1), False)
    assert calcular_dt(10.0, 10.0) == (0.0, False)
    assert calcular_dt(9.9, 10.0) == (0.0, False)
    assert calcular_dt(10.0 + MAX_DT_S + 0.01, 10.0) == (0.0, True)


def test_saturar():
    assert saturar(5.0, 2.0) == 2.0
    assert saturar(-5.0, 2.0) == -2.0
    assert saturar(1.0, 2.0) == 1.0


# ---------------- Centrado e métricas ----------------
def test_centrado_so_apos_hold_continuo():
    s = supervisor()
    s.alvo(0.0, True, 100.0)         # primeira detecção (início de t_c)
    s.alvo(0.5, True, 50.0)
    s.alvo(1.0, True, 15.0)          # entra na tolerância
    s.alvo(1.5, True, 10.0)
    assert not s.checar(1.5).centrado
    s.alvo(2.0, True, 5.0)           # 1 s dentro
    estado = s.checar(2.0)
    assert estado.centrado and estado.convergiu
    assert estado.t_c == pytest.approx(1.0)
    assert estado.e_r == pytest.approx((15.0 + 10.0 + 5.0) / 3)


def test_saida_da_tolerancia_reinicia_janela():
    s = supervisor()
    s.alvo(0.0, True, 10.0)
    s.alvo(0.8, True, 10.0)
    s.alvo(0.9, True, 30.0)          # sai
    s.alvo(1.0, True, 10.0)          # volta: nova janela
    s.alvo(1.9, True, 10.0)
    assert not s.checar(1.9).centrado
    s.alvo(2.0, True, 4.0)
    estado = s.checar(2.0)
    assert estado.centrado
    assert estado.t_c == pytest.approx(1.0)
    assert estado.e_r == pytest.approx(8.0)


def test_alvo_ausente_reinicia_janela():
    s = supervisor()
    s.alvo(0.0, True, 10.0)
    s.alvo(0.5, False, 0.0)
    s.alvo(1.0, True, 10.0)
    assert not s.checar(1.0).centrado


def test_t_c_conta_da_primeira_deteccao_valida():
    s = supervisor(inicio=0.0)
    s.alvo(0.3, False, 0.0)
    s.alvo(0.5, True, 80.0)          # primeira detecção válida
    s.alvo(1.5, True, 10.0)
    s.alvo(2.5, True, 10.0)
    assert s.checar(2.5).t_c == pytest.approx(1.0)


def test_metricas_da_primeira_convergencia_ficam():
    s = supervisor(hold=0.5)
    s.alvo(0.0, True, 10.0)
    s.alvo(0.5, True, 6.0)
    primeira = s.checar(0.5)
    assert primeira.centrado and primeira.e_r == pytest.approx(8.0)
    s.alvo(0.6, True, 50.0)          # sai (TRACK)
    assert not s.checar(0.6).centrado
    s.alvo(1.0, True, 2.0)
    s.alvo(1.5, True, 2.0)
    estado = s.checar(1.5)
    assert estado.centrado
    assert estado.t_c == primeira.t_c and estado.e_r == primeira.e_r


def test_hold_zero_centra_na_primeira_amostra():
    s = supervisor(hold=0.0)
    s.alvo(0.0, True, 5.0)
    assert s.checar(0.0).centrado


# ---------------- Perdido ----------------
def test_perdido_sem_nenhuma_mensagem():
    s = supervisor(inicio=10.0)
    assert not s.checar(10.9).perdido
    assert s.checar(11.1).perdido


def test_perdido_com_detected_false():
    s = supervisor(inicio=0.0)
    s.alvo(0.5, True, 30.0)
    for t in (0.6, 0.8, 1.0, 1.2, 1.4):
        s.alvo(t, False, 0.0)
    assert not s.checar(1.4).perdido
    s.alvo(1.6, False, 0.0)
    assert s.checar(1.6).perdido


def test_alvo_valido_renova_o_prazo():
    s = supervisor(inicio=0.0)
    s.alvo(0.9, True, 30.0)
    assert not s.checar(1.8).perdido
    assert s.checar(2.0).perdido


# ---------------- Preso no limite ----------------
def simular_junta(s, t0, t1, pan, tilt, cmd_pan, cmd_tilt, dt=0.05, mover_pan=0.0):
    t = t0
    while t <= t1 + 1e-9:
        s.junta(t, pan, tilt, cmd_pan, cmd_tilt)
        pan += mover_pan * dt
        t += dt
    return pan


def test_preso_no_limite_positivo():
    s = supervisor()
    simular_junta(s, 0.0, 2.0, 29.0, 0.0, 5.0, 0.0)
    assert s.checar(2.0).preso is None
    simular_junta(s, 2.05, 2.2, 29.0, 0.0, 5.0, 0.0)
    preso = s.checar(2.2).preso
    assert preso is not None
    assert (preso.eixo, preso.sentido) == ('pan', 1)
    assert preso.posicao_deg == pytest.approx(29.0)
    assert 'pan preso no limite positivo (29.0°)' in preso.mensagem()


def test_preso_tilt_negativo():
    s = supervisor()
    simular_junta(s, 0.0, 2.5, 0.0, -89.0, 0.0, -3.0)
    preso = s.checar(2.5).preso
    assert (preso.eixo, preso.sentido) == ('tilt', -1)


def test_eixo_se_movendo_nao_fica_preso():
    s = supervisor()
    simular_junta(s, 0.0, 5.0, 0.0, 0.0, 5.0, 0.0, mover_pan=5.0)
    assert s.checar(5.0).preso is None


def test_comando_pequeno_nao_conta():
    s = supervisor()
    simular_junta(s, 0.0, 5.0, 29.0, 0.0, LIMIT_MIN_CMD_DEG_S * 0.5, 0.0)
    assert s.checar(5.0).preso is None


def test_trocar_sentido_reinicia():
    s = supervisor()
    simular_junta(s, 0.0, 1.5, 29.0, 0.0, 5.0, 0.0)
    simular_junta(s, 1.55, 3.0, 29.0, 0.0, -5.0, 0.0)
    assert s.checar(3.0).preso is None


def test_movimento_reinicia_contagem():
    s = supervisor()
    simular_junta(s, 0.0, 1.5, 10.0, 0.0, 5.0, 0.0)
    s.junta(1.6, 10.0 + 2 * MOVE_EPS_DEG, 0.0, 5.0, 0.0)   # andou
    simular_junta(s, 1.65, 3.5, 10.0 + 2 * MOVE_EPS_DEG, 0.0, 5.0, 0.0)
    assert s.checar(3.5).preso is None


def test_limpar_juntas():
    s = supervisor()
    simular_junta(s, 0.0, 1.5, 29.0, 0.0, 5.0, 0.0)
    s.limpar_juntas()
    simular_junta(s, 1.55, 3.0, 29.0, 0.0, 5.0, 0.0)
    assert s.checar(3.0).preso is None
