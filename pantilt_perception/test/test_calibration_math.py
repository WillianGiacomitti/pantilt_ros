"""Testes da lógica da calibração por rotação (sem ROS, com dados sintéticos)."""

import math

import numpy as np
import pytest

from pantilt_perception.calibration_math import (
    DetectorParada, ajustar_eixo, alvo_centralizado, fov_deg, grade_eixo, k_sondagem,
    validar_parametros,
)

F = 600.0
FRACAO_GRADE = 0.35


def erros_sinteticos(theta_deg, k, phi_deg, ruido_px=0.0, semente=0):
    rng = np.random.default_rng(semente)
    theta = np.radians(theta_deg)
    return k * np.tan(theta - math.radians(phi_deg)) + rng.normal(0.0, ruido_px, len(theta))


GRADE = list(np.linspace(-10.0, 10.0, 7))


# ---------------- Ajuste ----------------

@pytest.mark.parametrize('k', [-F, F])
def test_ajuste_sem_ruido_exato(k):
    ajuste = ajustar_eixo(GRADE, erros_sinteticos(GRADE, k, 1.5))
    assert ajuste.f == pytest.approx(F, rel=1e-6)
    assert ajuste.k == pytest.approx(k, rel=1e-6)
    assert ajuste.phi_deg == pytest.approx(1.5, abs=1e-6)
    assert ajuste.rms == pytest.approx(0.0, abs=1e-6)
    assert ajuste.n == 7
    assert len(ajuste.residuos) == 7


@pytest.mark.parametrize('semente', range(5))
@pytest.mark.parametrize('k', [-F, F])
def test_ajuste_com_ruido(semente, k):
    erros = erros_sinteticos(GRADE, k, -2.0, ruido_px=1.0, semente=semente)
    ajuste = ajustar_eixo(GRADE, erros)
    assert ajuste.f == pytest.approx(F, rel=0.02)
    assert math.copysign(1.0, ajuste.k) == math.copysign(1.0, k)
    assert ajuste.phi_deg == pytest.approx(-2.0, abs=0.5)
    assert 0.3 < ajuste.rms < 2.0


def test_ajuste_grade_deslocada_do_zero():
    # home fora do zero do encoder: a grade inteira deslocada de 0,8°
    grade = [g + 0.8 for g in GRADE]
    ajuste = ajustar_eixo(grade, erros_sinteticos(grade, -590.0, 1.0))
    assert ajuste.f == pytest.approx(590.0, rel=1e-6)


@pytest.mark.parametrize('theta, erro, trecho', [
    ([0.0, 1.0], [0.0, -10.0], 'pelo menos 3'),
    ([2.0, 2.0, 2.0], [1.0, 2.0, 3.0], 'mesmo ângulo'),
    ([-5.0, 0.0, 5.0], [3.0, 3.0, 3.0], 'não se deslocou'),
    ([-5.0, 0.0, 5.0], [1.0, float('nan'), 3.0], 'não finitos'),
    ([-5.0, 0.0], [1.0, 2.0, 3.0], 'mesmo tamanho'),
])
def test_ajuste_invalido(theta, erro, trecho):
    with pytest.raises(ValueError, match=trecho):
        ajustar_eixo(theta, erro)


# ---------------- Sondagem e grade ----------------

def test_sondagem_recupera_k():
    erros = erros_sinteticos([-3.0, 3.0], -F, 0.0)
    assert k_sondagem(-3.0, erros[0], 3.0, erros[1], 0.0) == pytest.approx(-F)


def test_sondagem_angulos_iguais():
    with pytest.raises(ValueError):
        k_sondagem(1.0, 0.0, 1.0, 5.0, 0.0)


def test_grade_simetrica_e_crescente():
    grade = grade_eixo(F, 320.0, 7, 10.0, FRACAO_GRADE)
    assert len(grade) == 7
    assert grade == sorted(grade)
    assert grade[0] == pytest.approx(-grade[-1])
    assert grade[3] == pytest.approx(0.0)


def test_grade_limitada_por_max_angle():
    # 35% de 320 px com f = 600 daria 10,6°
    assert grade_eixo(F, 320.0, 7, 10.0, FRACAO_GRADE)[-1] == pytest.approx(10.0)


def test_grade_limitada_pela_fracao():
    # 35% de 240 px com f = 600: atan(84/600) = 7,97°
    assert grade_eixo(F, 240.0, 7, 10.0, FRACAO_GRADE)[-1] == pytest.approx(
        math.degrees(math.atan(84.0 / 600.0)))


def test_grade_f_invalido():
    with pytest.raises(ValueError):
        grade_eixo(0.0, 320.0, 7, 10.0, FRACAO_GRADE)


# ---------------- Pré-condição e FOV ----------------

@pytest.mark.parametrize('ex, ey, esperado', [
    (0.0, 0.0, True),
    (64.0, 48.0, True),      # exatamente 20% de 320 e de 240
    (64.1, 0.0, False),
    (0.0, -48.1, False),
])
def test_alvo_centralizado(ex, ey, esperado):
    assert alvo_centralizado(ex, ey, 640, 480, 0.2) is esperado


def test_fov():
    assert fov_deg(320.0, 640.0) == pytest.approx(90.0)


# ---------------- Detecção de parada ----------------

def test_parada_confirmada_apos_hold():
    det = DetectorParada(tol_deg=0.2, vel_max_deg_s=0.5, hold_s=0.15)
    assert det.update(0.00, 9.95, 0.0, 10.0) is None
    assert det.update(0.10, 9.95, 0.0, 10.0) is None
    assert det.update(0.15, 9.95, 0.0, 10.0) == pytest.approx(0.15)


def test_parada_reinicia_se_voltar_a_mover():
    det = DetectorParada(0.2, 0.5, 0.15)
    det.update(0.00, 10.0, 0.0, 10.0)
    det.update(0.10, 10.0, 2.0, 10.0)       # ainda se movendo
    assert det.update(0.20, 10.0, 0.0, 10.0) is None
    assert det.update(0.36, 10.0, 0.0, 10.0) == pytest.approx(0.36)


def test_parado_fora_do_alvo():
    det = DetectorParada(0.2, 0.5, 0.15)
    for t in (0.0, 0.1, 0.2, 0.3):
        assert det.update(t, 9.5, 0.0, 10.0) is None
    assert det.parado_fora_do_alvo(0.3)


def test_em_movimento_nao_esta_parado_fora_do_alvo():
    det = DetectorParada(0.2, 0.5, 0.15)
    for t in (0.0, 0.1, 0.2, 0.3):
        det.update(t, 5.0 + t, 10.0, 10.0)
    assert not det.parado_fora_do_alvo(0.3)


# ---------------- Parâmetros ----------------

VALIDOS = dict(probe_deg=3.0, max_angle_deg=10.0, n_points=7, samples_per_point=5,
               settle_tol_deg=0.2, settle_extra_s=0.4, min_points=4)


def test_parametros_validos():
    validar_parametros(**VALIDOS)


@pytest.mark.parametrize('campo, valor', [
    ('probe_deg', 0.0),
    ('probe_deg', 12.0),         # maior que max_angle_deg
    ('max_angle_deg', 0.0),
    ('max_angle_deg', 25.0),
    ('min_points', 2),
    ('n_points', 3),             # menor que min_points
    ('samples_per_point', 0),
    ('settle_tol_deg', 0.0),
    ('settle_extra_s', -0.1),
])
def test_parametros_invalidos(campo, valor):
    with pytest.raises(ValueError):
        validar_parametros(**{**VALIDOS, campo: valor})
