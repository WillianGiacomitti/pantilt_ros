"""Testes do controlador fuzzy do visual_servo_node (sem ROS)."""

import math

import pytest

from pantilt_control.controllers.base import Controlador, criar_controlador
from pantilt_control.controllers.fuzzy import (
    TABELA_PADRAO, ControladorFuzzy, _pertinencias,
)
from pantilt_control.controllers.pid import ControladorPID

MAX = math.radians(20.0)
I_MAX = math.radians(10.0)
KP = 0.8
DT = 0.1

# Tabela linear no erro e sem efeito da variação: reproduz o controlador P
TABELA_LINEAR = tuple((c, c, c) for c in (-1.0, -0.5, 0.0, 0.5, 1.0))


def fuzzy(kp=KP, fc=2.0, max_output=MAX, tabela=TABELA_PADRAO):
    return ControladorFuzzy(kp, 0.0, 0.0, fc, max_output, I_MAX, tabela=tabela)


def estatica(x: float) -> float:
    """Saída normalizada (u / max_output) para o erro normalizado x, sem variação."""
    c = fuzzy()
    return c.compute(x * c.erro_max, 0.0) / MAX


# ---------------- Fábrica ----------------
def test_fabrica_fuzzy():
    c = criar_controlador('fuzzy', dict(kp=0.8, ki=0.0, kd=0.0, derivative_filter_hz=2.0,
                                        max_output=MAX, integral_limit=I_MAX))
    assert isinstance(c, ControladorFuzzy)
    assert isinstance(c, Controlador)


# ---------------- 1. Partição da unidade ----------------
@pytest.mark.parametrize('n', [3, 5])
@pytest.mark.parametrize('x', [-3.0, -1.0, -0.73, -0.5, 0.0, 0.2, 0.5, 0.999, 1.0, 2.5])
def test_particao_da_unidade(n, x):
    graus = _pertinencias(x, n)
    assert len(graus) == n
    assert sum(graus) == pytest.approx(1.0, abs=1e-12)
    assert all(g >= 0.0 for g in graus)


# ---------------- 2. Simetria da tabela ----------------
def test_simetria_da_tabela():
    for i in range(5):
        for j in range(3):
            assert TABELA_PADRAO[i][j] == -TABELA_PADRAO[4 - i][2 - j]


# ---------------- 3. Equivalência com o P ----------------
@pytest.mark.parametrize('e', [-0.6, -0.3, -0.1, -0.01, 0.0, 0.02, 0.15, 0.4, 0.7])
def test_tabela_linear_equivale_ao_p(e):
    f = fuzzy(tabela=TABELA_LINEAR)
    p = ControladorPID(KP, 0.0, 0.0, 2.0, MAX, I_MAX)
    # Várias amostras: a variação do erro não pode mudar nada com a tabela linear
    for k in range(4):
        erro = e * (1.0 - 0.2 * k)
        assert f.compute(erro, DT) == pytest.approx(p.compute(erro, DT), abs=1e-12)


# ---------------- 4. Não degeneração ----------------
def test_curva_estatica_nao_e_linear():
    xs = [i / 50.0 for i in range(-50, 51)]
    desvio = max(abs(estatica(x) - x) for x in xs)
    assert desvio > 0.05


# ---------------- 5. Monotonicidade ----------------
def test_curva_estatica_monotona():
    xs = [i / 100.0 for i in range(-150, 151)]
    ys = [estatica(x) for x in xs]
    assert all(b >= a for a, b in zip(ys, ys[1:]))


# ---------------- 6. Simetria da saída ----------------
@pytest.mark.parametrize('e', [0.0, 0.01, 0.05, 0.12, 0.3, 0.6])
def test_saida_simetrica(e):
    c = fuzzy()
    positivo = c.compute(e, 0.0)
    c.reset()
    negativo = c.compute(-e, 0.0)
    assert positivo == pytest.approx(-negativo, abs=1e-15)


# ---------------- 7. Saturação ----------------
@pytest.mark.parametrize('e', [1.0, -1.0, 10.0, -10.0])
def test_saturacao(e):
    c = fuzzy()
    u = c.compute(e, 0.0)
    assert u == pytest.approx(math.copysign(MAX, e))
    assert c.saturado


# ---------------- 8. dt <= 0 ----------------
@pytest.mark.parametrize('dt', [0.0, -0.1])
def test_dt_nao_positivo_sem_derivada(dt):
    c = fuzzy()
    e = 0.1
    esperado = fuzzy().compute(e, 0.0)        # primeira amostra: só a ação sobre o erro
    c.compute(0.2, DT)                        # inicializa o filtro com outro erro
    u = c.compute(e, dt)
    assert u != 0.0
    assert u == pytest.approx(esperado)


def test_primeira_amostra_sem_derivada():
    c = fuzzy()
    assert c.compute(0.1, DT) == pytest.approx(fuzzy().compute(0.1, 0.0))


# ---------------- 9. Freio e reforço ----------------
def dinamica(e_anterior: float, e: float) -> float:
    c = fuzzy()
    c.compute(e_anterior, DT)
    return c.compute(e, DT)


def test_freio_com_erro_diminuindo():
    e = 0.1
    assert dinamica(0.12, e) < fuzzy().compute(e, 0.0)


def test_reforco_com_erro_aumentando():
    e = 0.1
    assert dinamica(0.08, e) > fuzzy().compute(e, 0.0)


# ---------------- 10. Validações ----------------
@pytest.mark.parametrize('kwargs', [
    dict(kp=0.0), dict(kp=-0.5), dict(max_output=0.0), dict(max_output=-1.0),
    dict(fc=0.0), dict(fc=-2.0),
])
def test_parametros_invalidos(kwargs):
    with pytest.raises(ValueError):
        fuzzy(**kwargs)


@pytest.mark.parametrize('tabela', [
    (),                                        # vazia
    ((0.0, 0.5), (0.1,)),                      # linhas de tamanhos diferentes
    ((0.0,), (1.0,)),                          # uma coluna
    ((-1.0, 0.0, 1.0),),                       # uma linha
    ((-1.0, 'a'), (0.0, 1.0)),                 # valor não numérico
    ((-1.0, 0.0), (0.0, 1.5)),                 # valor fora de [-1, 1]
    ((-1.0, 0.0), (0.0, math.nan)),            # valor não finito
])
def test_tabela_malformada(tabela):
    with pytest.raises(ValueError):
        fuzzy(tabela=tabela)


# ---------------- 11. Interface de diagnóstico ----------------
def test_termos_zerados():
    c = fuzzy()
    c.compute(0.2, 0.0)
    assert c.termos() == (0.0, 0.0, 0.0)


def test_saida_bruta_e_saturado():
    c = fuzzy()
    u = c.compute(0.05, 0.0)
    assert c.saida_bruta() == pytest.approx(u)
    assert not c.saturado
    c.compute(1.0, DT)
    assert c.saturado
    assert abs(c.saida_bruta()) <= MAX * (1.0 + 1e-12)
    c.compute(0.05, DT)
    assert not c.saturado


def test_reset_limpa_o_estado():
    c = fuzzy()
    c.compute(1.0, 0.0)
    c.reset()
    assert not c.saturado
    assert c.saida_bruta() == 0.0
    assert c.compute(0.0, 0.0) == 0.0
