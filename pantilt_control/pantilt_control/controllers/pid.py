"""
PID posicional com anti-windup usado pelo visual_servo_node (docs/architecture.md, seção 4.5).

Lógica pura (sem ROS). Erro em rad, saída em rad/s. Unidades dos ganhos:
kp [1/s], ki [1/s²], kd [adimensional].

  - P: kp · e;
  - I: guardado já como contribuição na saída (rad/s). Recortado em
    ±integral_limit e congelado enquanto a saída está saturada e o erro
    empurra no mesmo sentido da saturação (integração condicional): o
    integrador não acumula, mas pode desacumular assim que o erro inverte;
  - D: derivada do erro passado por um filtro passa-baixas de 1ª ordem com
    corte em derivative_filter_hz. A referência é sempre zero (alvo no
    centro), então a derivada do erro é a derivada da medida e não há
    "kick" de referência. A primeira amostra só inicializa o filtro;
  - saída recortada em ±max_output.
"""

import math

from pantilt_control.controllers.base import Controlador


def _sign(x: float) -> int:
    return (x > 0) - (x < 0)


class ControladorPID(Controlador):
    def __init__(self, kp: float, ki: float, kd: float, derivative_filter_hz: float,
                 max_output: float, integral_limit: float):
        for nome, valor in (('kp', kp), ('ki', ki), ('kd', kd)):
            if valor < 0.0:
                raise ValueError(f'{nome} deve ser maior ou igual a zero, recebido {valor}')
        if derivative_filter_hz <= 0.0:
            raise ValueError(
                f'derivative_filter_hz deve ser positivo, recebido {derivative_filter_hz}')
        if max_output <= 0.0:
            raise ValueError(f'max_output deve ser positivo, recebido {max_output}')
        if integral_limit < 0.0:
            raise ValueError(
                f'integral_limit deve ser maior ou igual a zero, recebido {integral_limit}')

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral_limit = integral_limit
        # Constante de tempo do filtro da derivada
        self.tau = 1.0 / (2.0 * math.pi * derivative_filter_hz)
        self.reset()

    def reset(self):
        self.integral = 0.0          # contribuição integral, rad/s
        self._filtrado = None        # erro filtrado da amostra anterior
        self.saturado = False

    def compute(self, erro: float, dt: float) -> float:
        p = self.kp * erro

        d = 0.0
        if self._filtrado is None:
            # Primeira amostra: só inicializa o filtro (sem pico na derivada)
            self._filtrado = erro
        elif dt > 0.0:
            alfa = dt / (self.tau + dt)
            filtrado = self._filtrado + alfa * (erro - self._filtrado)
            d = self.kd * (filtrado - self._filtrado) / dt
            self._filtrado = filtrado

        integral = self.integral
        if dt > 0.0 and self.ki > 0.0:
            candidato = integral + self.ki * erro * dt
            candidato = min(max(candidato, -self.integral_limit), self.integral_limit)
            u = p + candidato + d
            # Congela se a saída satura e o erro empurra para dentro da saturação
            if not (abs(u) > self.max_output and _sign(erro) == _sign(u)):
                integral = candidato
        self.integral = integral

        u = p + self.integral + d
        self.saturado = abs(u) > self.max_output
        return min(max(u, -self.max_output), self.max_output)
