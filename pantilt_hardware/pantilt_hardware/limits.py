"""
Limites de ângulo por software aplicados pelo serial_bridge_node.

Funções puras (sem ROS) para poderem ser testadas sem hardware.
Ângulos em rad e velocidades em rad/s, exceto onde o nome indica _deg.
"""

import math


def effective_limits(limits_deg, margin_deg: float):
    """Converte [mín, máx] em graus para (mín, máx) em rad, já descontada a margem."""
    if len(limits_deg) != 2:
        raise ValueError(f'limites devem ter 2 valores [mín, máx], recebido {list(limits_deg)}')
    low = math.radians(limits_deg[0] + margin_deg)
    high = math.radians(limits_deg[1] - margin_deg)
    if low >= high:
        raise ValueError(
            f'faixa vazia após aplicar a margem de {margin_deg}° aos limites {list(limits_deg)}'
        )
    return low, high


def clip_position(position: float, limits) -> float:
    """Recorta uma posição absoluta para dentro dos limites."""
    low, high = limits
    return min(max(position, low), high)


def limit_velocity(velocity: float, position: float, limits, lookahead_s: float = 0.0) -> float:
    """
    Zera a velocidade que empurra o eixo para fora do limite.

    A posição é projetada lookahead_s à frente (normalmente o período da
    telemetria), para o eixo parar antes de cruzar o limite e não depois.
    Velocidades que afastam o eixo do limite nunca são bloqueadas.
    """
    low, high = limits
    predicted = position + velocity * lookahead_s
    if velocity > 0.0 and predicted >= high:
        return 0.0
    if velocity < 0.0 and predicted <= low:
        return 0.0
    return velocity
