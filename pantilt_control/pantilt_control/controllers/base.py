"""
Interface comum dos controladores do visual_servo_node (docs/architecture.md, seção 4.5).

Lógica pura (sem ROS). Cada eixo usa uma instância independente (SISO):
o pan recebe o erro horizontal e o tilt o vertical. A entrada é o erro
ANGULAR da linha de visada em rad (θ = atan(e_px / f)) e a saída é a
velocidade angular do eixo em rad/s. O sinal físico (invert_pan /
invert_tilt) é aplicado pelo nó, fora do controlador.

Trocar de método é só mudar o parâmetro `controller`: o nó cria os
controladores por criar_controlador() e não conhece as classes concretas.
"""

import math
from abc import ABC, abstractmethod

CONTROLADORES = ('pid', 'fuzzy')


class Controlador(ABC):
    """Controlador SISO: erro angular (rad) -> velocidade angular (rad/s)."""

    @abstractmethod
    def compute(self, erro: float, dt: float) -> float:
        """
        Calcula a velocidade para o erro atual.

        dt é o intervalo desde a amostra anterior, em s, calculado pelos
        header.stamp dos alvos. dt <= 0 indica a primeira amostra (após
        reset ou após uma lacuna): nada que dependa do tempo é atualizado.
        """

    @abstractmethod
    def reset(self):
        """Volta ao estado inicial (goal novo ou alvo reencontrado após lacuna)."""

    # Diagnóstico para o /control/servo_debug (ensaios). Valem para a última
    # chamada de compute(); os padrões servem a controladores sem P/I/D.
    saturado = False

    def termos(self) -> tuple:
        """(p, i, d) em rad/s. Padrão: zeros."""
        return 0.0, 0.0, 0.0

    def saida_bruta(self) -> float:
        """Saída antes da saturação, em rad/s. Padrão: nan (não informada)."""
        return math.nan


def criar_controlador(nome: str, params: dict) -> Controlador:
    """
    Cria o controlador pelo nome. params são os argumentos do construtor da
    classe concreta, já em unidades SI. O nó passa sempre as mesmas chaves
    (kp, ki, kd, derivative_filter_hz, max_output, integral_limit); cada
    controlador usa as que precisa. Levanta ValueError para nome ou
    parâmetro inválido.
    """
    if nome == 'pid':
        # Import local: pid.py importa este módulo
        from pantilt_control.controllers.pid import ControladorPID
        return ControladorPID(**params)
    if nome == 'fuzzy':
        from pantilt_control.controllers.fuzzy import ControladorFuzzy
        return ControladorFuzzy(**params)
    raise ValueError(f'controlador desconhecido: "{nome}" (opções: {", ".join(CONTROLADORES)})')
