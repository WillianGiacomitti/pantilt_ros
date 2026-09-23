"""Testes dos limites de ângulo por software (sem hardware)."""

import math

import pytest

from pantilt_hardware.limits import clip_position, effective_limits, limit_velocity

PAN = effective_limits([-30.0, 30.0], 1.0)


def test_limites_efetivos_com_margem():
    assert PAN == pytest.approx((math.radians(-29.0), math.radians(29.0)))


@pytest.mark.parametrize('limites, margem', [
    ([-30.0], 1.0),              # quantidade errada de valores
    ([30.0, -30.0], 1.0),        # mín > máx
    ([-1.0, 1.0], 1.0),          # margem consome a faixa
])
def test_limites_invalidos(limites, margem):
    with pytest.raises(ValueError):
        effective_limits(limites, margem)


def test_clip_position():
    assert clip_position(1.0, PAN) == pytest.approx(math.radians(29.0))
    assert clip_position(-1.0, PAN) == pytest.approx(math.radians(-29.0))
    assert clip_position(0.1, PAN) == 0.1


def test_velocidade_livre_dentro_da_faixa():
    assert limit_velocity(0.3, 0.0, PAN) == 0.3
    assert limit_velocity(-0.3, 0.0, PAN) == -0.3


def test_velocidade_bloqueada_so_no_sentido_do_limite():
    no_limite = math.radians(29.0)
    assert limit_velocity(0.3, no_limite, PAN) == 0.0
    assert limit_velocity(-0.3, no_limite, PAN) == -0.3
    assert limit_velocity(-0.3, -no_limite, PAN) == 0.0
    assert limit_velocity(0.3, -no_limite, PAN) == 0.3


def test_velocidade_projetada_para_frente():
    # 28.5° + 0.5 rad/s * 0.05 s (~1.43°) cruzaria 29°
    pos = math.radians(28.5)
    assert limit_velocity(0.5, pos, PAN) == 0.5
    assert limit_velocity(0.5, pos, PAN, lookahead_s=0.05) == 0.0


def test_velocidade_zero_passa():
    assert limit_velocity(0.0, math.radians(35.0), PAN) == 0.0
