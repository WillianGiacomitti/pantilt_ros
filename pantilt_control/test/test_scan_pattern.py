"""Testes do padrão de varredura do scan_node (sem ROS, com tempos sintéticos)."""

import pytest

from pantilt_control.scan_pattern import (
    MIN_SPEED_DEG_S, STALL_TIMEOUT_S, TOLERANCE_DEG, ScanPattern,
)

PAN_MIN, PAN_MAX = -28.0, 28.0
LEVELS = [-20.0, 0.0, 20.0]
SPEED = 15.0


@pytest.fixture
def pattern():
    return ScanPattern(PAN_MIN, PAN_MAX, LEVELS, SPEED)


def simulate(pattern, pan, tilt, dt=0.05, t_max=120.0, latency_steps=0):
    """
    Integra as velocidades pedidas numa junta ideal e devolve a lista de
    (t, pan, tilt, step) até o padrão terminar ou falhar. latency_steps atrasa
    a posição observada, como o atraso da telemetria.
    """
    t = 0.0
    pattern.start(t, pan, tilt)
    history = [(pan, tilt)]
    steps = []
    while t < t_max:
        observed = history[max(0, len(history) - 1 - latency_steps)]
        step = pattern.update(t, *observed)
        steps.append((t, pan, tilt, step))
        if step.done or step.error:
            return steps
        pan += step.pan_vel * dt
        tilt += step.tilt_vel * dt
        history.append((pan, tilt))
        t += dt
    raise AssertionError('o padrão não terminou no tempo máximo')


# ---------------- Parâmetros ----------------
@pytest.mark.parametrize('pan_min, pan_max, levels, speed', [
    (28.0, -28.0, LEVELS, SPEED),      # mín > máx
    (10.0, 10.0, LEVELS, SPEED),       # faixa vazia
    (PAN_MIN, PAN_MAX, [], SPEED),     # sem faixas de tilt
    (PAN_MIN, PAN_MAX, LEVELS, 0.0),   # velocidade nula
    (PAN_MIN, PAN_MAX, LEVELS, -5.0),  # velocidade negativa
])
def test_parametros_invalidos(pan_min, pan_max, levels, speed):
    with pytest.raises(ValueError):
        ScanPattern(pan_min, pan_max, levels, speed)


def test_update_antes_do_start_falha(pattern):
    with pytest.raises(RuntimeError):
        pattern.update(0.0, 0.0, 0.0)


# ---------------- Waypoints ----------------
def test_comeca_pela_ponta_mais_proxima_do_pan(pattern):
    pattern.start(0.0, 10.0, 5.0)
    wps = [(w.pan, w.tilt, w.pass_index) for w in pattern.waypoints]
    assert wps == [
        (PAN_MAX, 5.0, 0),       # só o pan até a ponta; tilt atual mantido
        (PAN_MAX, -20.0, 0),     # só o tilt até a primeira faixa
        (PAN_MIN, -20.0, 0),
        (PAN_MIN, 0.0, 1),
        (PAN_MAX, 0.0, 1),
        (PAN_MAX, 20.0, 2),
        (PAN_MIN, 20.0, 2),
    ]

    pattern.start(0.0, -5.0, 0.0)
    assert pattern.waypoints[0].pan == PAN_MIN


def test_cada_trecho_move_um_eixo(pattern):
    pattern.start(0.0, 3.0, 7.0)
    wps = pattern.waypoints
    for a, b in zip(wps, wps[1:]):
        assert a.pan == b.pan or a.tilt == b.tilt


# ---------------- Execução ----------------
def test_padrao_completo_em_simulacao(pattern):
    steps = simulate(pattern, 0.0, 0.0)
    t_end, pan, tilt, last = steps[-1]

    assert last.done and not last.error
    assert t_end < 30.0                     # ~17 s de movimento a 15°/s + aproximações
    assert pan == pytest.approx(PAN_MAX, abs=1.0)   # 3 faixas: termina na ponta oposta à inicial
    assert tilt == pytest.approx(LEVELS[-1], abs=1.0)

    for _, _, _, step in steps:
        assert step.pan_vel == 0.0 or step.tilt_vel == 0.0     # um eixo por vez
        assert abs(step.pan_vel) <= SPEED and abs(step.tilt_vel) <= SPEED

    passes = [step.pass_index for _, _, _, step in steps]
    assert passes == sorted(passes) and passes[-1] == len(LEVELS) - 1

    pans = [p for _, p, _, _ in steps]
    assert min(pans) >= PAN_MIN - 1.0 and max(pans) <= PAN_MAX + 1.0


def test_atraso_da_telemetria_nao_inverte_nem_ultrapassa_muito(pattern):
    # 3 ciclos a 20 Hz = 150 ms de atraso
    steps = simulate(pattern, 0.0, 0.0, latency_steps=3)
    assert steps[-1][3].done

    pans = [p for _, p, _, _ in steps]
    assert min(pans) >= PAN_MIN - 1.5 and max(pans) <= PAN_MAX + 1.5

    # Dentro de um trecho de pan, a velocidade nunca troca de sinal
    for (_, _, _, a), (_, _, _, b) in zip(steps, steps[1:]):
        assert a.pan_vel * b.pan_vel >= 0.0


def test_ultrapassar_o_alvo_conta_como_atingido():
    pattern = ScanPattern(PAN_MIN, PAN_MAX, [-20.0], SPEED)
    pattern.start(0.0, PAN_MIN, -20.0)       # já na ponta inicial e na faixa
    assert pattern.update(0.05, PAN_MIN, -20.0).pan_vel > 0.0

    # Passou do alvo além da tolerância: não volta, termina
    step = pattern.update(0.10, PAN_MAX + 3 * TOLERANCE_DEG, -20.0)
    assert step.done and step.pan_vel == 0.0


def test_desacelera_perto_do_alvo():
    pattern = ScanPattern(PAN_MIN, PAN_MAX, [0.0], SPEED)
    pattern.start(0.0, PAN_MIN, 0.0)
    pattern.update(0.05, PAN_MIN, 0.0)      # já na ponta inicial: avança para o trecho até PAN_MAX
    assert pattern.update(0.1, 0.0, 0.0).pan_vel == pytest.approx(SPEED)
    assert pattern.update(0.2, PAN_MAX - 1.0, 0.0).pan_vel == pytest.approx(2.0)     # 1° / 0,5 s
    assert pattern.update(0.3, PAN_MAX - 0.6, 0.0).pan_vel == pytest.approx(MIN_SPEED_DEG_S)


def test_velocidade_baixa_nao_passa_do_pedido():
    pattern = ScanPattern(PAN_MIN, PAN_MAX, [0.0], 0.5)
    steps = simulate(pattern, 0.0, 0.0, t_max=400.0)
    assert steps[-1][3].done
    assert all(abs(s.pan_vel) <= 0.5 and abs(s.tilt_vel) <= 0.5 for _, _, _, s in steps)


# ---------------- Falhas ----------------
def test_eixo_parado_falha_depois_do_tempo_limite(pattern):
    pattern.start(0.0, 0.0, 0.0)
    assert not pattern.update(STALL_TIMEOUT_S - 0.1, 0.0, 0.0).error
    step = pattern.update(STALL_TIMEOUT_S + 0.1, 0.0, 0.0)
    assert step.error and step.pan_vel == 0.0 and step.tilt_vel == 0.0


def test_movimento_do_operador_evita_a_falha(pattern):
    # O operador move o tilt pelo jog enquanto o mux descarta a varredura
    pattern.start(0.0, 0.0, 0.0)
    for i in range(1, 11):
        step = pattern.update(float(i), 0.0, 0.5 * i)
        assert not step.error
