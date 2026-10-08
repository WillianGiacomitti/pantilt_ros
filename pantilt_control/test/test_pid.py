"""Testes dos controladores do visual_servo_node (sem ROS)."""

import math

import pytest

from pantilt_control.controllers.base import Controlador, criar_controlador
from pantilt_control.controllers.pid import ControladorPID

MAX = math.radians(20.0)
I_MAX = math.radians(10.0)


def pid(kp=1.0, ki=0.0, kd=0.0, fc=2.0, max_output=MAX, integral_limit=I_MAX):
    return ControladorPID(kp, ki, kd, fc, max_output, integral_limit)


# ---------------- Fábrica e parâmetros ----------------
def test_fabrica_pid():
    c = criar_controlador('pid', dict(kp=0.8, ki=0.0, kd=0.0, derivative_filter_hz=2.0,
                                      max_output=MAX, integral_limit=I_MAX))
    assert isinstance(c, ControladorPID)
    assert isinstance(c, Controlador)


@pytest.mark.parametrize('nome', ['fuzzy', 'lqr', ''])
def test_fabrica_recusa(nome):
    with pytest.raises(ValueError):
        criar_controlador(nome, {})


@pytest.mark.parametrize('kwargs', [
    dict(kp=-0.1), dict(ki=-0.1), dict(kd=-0.1), dict(fc=0.0),
    dict(max_output=0.0), dict(integral_limit=-1.0),
])
def test_parametros_invalidos(kwargs):
    with pytest.raises(ValueError):
        pid(**kwargs)


# ---------------- Degrau ----------------
def test_degrau_proporcional():
    c = pid(kp=0.8)
    e = math.radians(5.0)
    assert c.compute(e, 0.0) == pytest.approx(0.8 * e)
    for _ in range(10):
        assert c.compute(e, 0.1) == pytest.approx(0.8 * e)


def test_degrau_pi_cresce():
    c = pid(kp=0.5, ki=0.5)
    e = math.radians(2.0)
    saidas = [c.compute(e, 0.0)] + [c.compute(e, 0.1) for _ in range(10)]
    assert saidas[0] == pytest.approx(0.5 * e)          # 1ª amostra: só P
    assert all(b > a for a, b in zip(saidas, saidas[1:]))
    assert saidas[-1] == pytest.approx(0.5 * e + 0.5 * e * 1.0)


def test_sinal_segue_o_erro():
    c = pid(kp=0.8, ki=0.2)
    assert c.compute(-0.05, 0.0) < 0.0
    assert c.compute(-0.05, 0.1) < 0.0


# ---------------- dt variável ----------------
def test_dt_variavel_mesmo_integral():
    e = math.radians(1.0)
    a = pid(kp=0.0, ki=1.0)
    a.compute(e, 0.0)
    a.compute(e, 0.2)
    b = pid(kp=0.0, ki=1.0)
    b.compute(e, 0.0)
    for dt in (0.05, 0.08, 0.03, 0.04):
        b.compute(e, dt)
    assert a.integral == pytest.approx(b.integral)
    assert a.integral == pytest.approx(e * 0.2)


def test_dt_nao_positivo_nao_integra():
    c = pid(kp=0.0, ki=1.0)
    c.compute(0.1, 0.0)
    c.compute(0.1, 0.0)
    c.compute(0.1, -0.05)
    assert c.integral == 0.0


# ---------------- Saturação e anti-windup ----------------
@pytest.mark.parametrize('e', [math.radians(60.0), -math.radians(60.0)])
def test_saturacao(e):
    c = pid(kp=3.0, ki=2.0, kd=0.5)
    assert abs(c.compute(e, 0.0)) == pytest.approx(MAX)
    for _ in range(50):
        u = c.compute(e, 0.1)
        assert abs(u) <= MAX + 1e-12
    assert c.saturado


def test_integral_nao_acumula_saturado():
    # P sozinho já satura: o integrador fica congelado em zero
    c = pid(kp=3.0, ki=1.0)
    e = math.radians(30.0)
    c.compute(e, 0.0)
    for _ in range(100):
        c.compute(e, 0.1)
    assert c.integral == 0.0
    # Erro invertido: sai da saturação na hora, sem integral acumulado segurando
    u = c.compute(-math.radians(1.0), 0.1)
    assert u < 0.0
    assert u == pytest.approx(3.0 * -math.radians(1.0) + c.integral)


def test_integral_desacumula_com_saida_saturada():
    # Integral acima da saturação (teto maior que a saída): saída saturada em +MAX
    c = pid(kp=1.0, ki=1.0, max_output=MAX, integral_limit=2.0 * MAX)
    c.integral = 1.5 * MAX
    c.compute(-0.01, 0.0)
    antes = c.integral
    u = c.compute(-0.01, 0.1)
    assert u == pytest.approx(MAX)
    # O erro empurra para fora da saturação: o integrador não fica congelado
    assert c.integral == pytest.approx(antes - 0.01 * 0.1)


def test_teto_do_integral():
    c = pid(kp=0.0, ki=5.0, max_output=MAX, integral_limit=I_MAX)
    e = math.radians(1.0)
    c.compute(e, 0.0)
    for _ in range(1000):
        u = c.compute(e, 0.1)
    assert c.integral == pytest.approx(I_MAX)
    assert u == pytest.approx(I_MAX)


# ---------------- Derivada ----------------
def test_derivada_sem_pico_na_primeira_amostra():
    c = pid(kp=0.0, kd=1.0)
    assert c.compute(math.radians(10.0), 0.0) == 0.0
    assert c.compute(math.radians(10.0), 0.1) == pytest.approx(0.0)


def test_derivada_filtrada():
    fc, dt = 2.0, 0.1
    c = pid(kp=0.0, kd=1.0, fc=fc, max_output=100.0)
    c.compute(0.0, 0.0)
    salto = 0.1
    u = c.compute(salto, dt)
    tau = 1.0 / (2.0 * math.pi * fc)
    alfa = dt / (tau + dt)
    assert u == pytest.approx(alfa * salto / dt)
    assert u < salto / dt                         # menor que a derivada crua
    # Erro constante depois do salto: a derivada decai até zero
    anteriores = [u]
    for _ in range(30):
        anteriores.append(c.compute(salto, dt))
    assert all(b < a for a, b in zip(anteriores, anteriores[1:]))
    assert anteriores[-1] == pytest.approx(0.0, abs=1e-4)


# ---------------- reset ----------------
def test_reset_entrada_nula_saida_nula():
    c = pid(kp=0.8, ki=1.0, kd=0.3)
    c.compute(0.2, 0.0)
    for _ in range(20):
        c.compute(0.2, 0.1)
    assert c.compute(0.0, 0.1) != 0.0
    c.reset()
    assert c.compute(0.0, 0.0) == 0.0
    assert c.compute(0.0, 0.1) == 0.0
    assert c.integral == 0.0
