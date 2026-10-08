"""
Lógica do visual_servo_node fora dos controladores (docs/architecture.md, seção 4.5).

Lógica pura (sem ROS) para poder ser testada sem hardware. Erros de imagem em
pixels, ângulos de junta em graus e comandos em graus/s; os instantes em
segundos de um único relógio (o nó usa o tempo ROS: header.stamp dos alvos e
o relógio do nó para o resto).

Estágios do pipeline que ficam aqui:
  1. conversão do erro em pixels para erro angular, θ = atan(e_px / f);
  3. saturação da saída e cálculo do dt pelos header.stamp;
  4. supervisão: centrado, perdido e preso no limite, mais as métricas
     t_c e e_r (seção 10).
"""

import math
from dataclasses import dataclass
from typing import Optional

# Intervalo máximo entre dois alvos válidos para usar o dt na integral e na
# derivada. Acima dele (alvo sumiu e voltou), os controladores reiniciam.
MAX_DT_S = 0.5

# Preso no limite: comando mínimo, em módulo, para contar como "empurrando"
LIMIT_MIN_CMD_DEG_S = 1.0

# Preso no limite: deslocamento mínimo para considerar que o eixo se moveu
MOVE_EPS_DEG = 0.2


def px_para_angulo(erro_px: float, f_px: float) -> float:
    """Erro angular da linha de visada, em rad, para um erro em pixels e a focal f em pixels."""
    return math.atan(erro_px / f_px)


def calcular_dt(stamp: float, stamp_anterior: Optional[float]):
    """
    Intervalo entre dois alvos válidos pelos header.stamp.

    Retorna (dt, lacuna). Sem amostra anterior ou com dt <= 0, dt = 0
    (primeira amostra para o controlador). Acima de MAX_DT_S, dt = 0 e
    lacuna=True: o nó reinicia os controladores antes de usar a amostra.
    """
    if stamp_anterior is None:
        return 0.0, False
    dt = stamp - stamp_anterior
    if dt <= 0.0:
        return 0.0, False
    if dt > MAX_DT_S:
        return 0.0, True
    return dt, False


def saturar(valor: float, limite: float) -> float:
    return min(max(valor, -limite), limite)


def _sign(x: float) -> int:
    return (x > 0) - (x < 0)


@dataclass(frozen=True)
class Preso:
    eixo: str           # 'pan' ou 'tilt'
    sentido: int        # +1: comando empurrando para o lado positivo; -1: negativo
    posicao_deg: float
    duracao_s: float

    def mensagem(self) -> str:
        lado = 'positivo' if self.sentido > 0 else 'negativo'
        return (f'{self.eixo} preso no limite {lado} ({self.posicao_deg:.1f}°): '
                f'comando empurrando há {self.duracao_s:.1f} s sem movimento')


@dataclass(frozen=True)
class EstadoSupervisao:
    centrado: bool
    perdido: bool
    preso: Optional[Preso]
    convergiu: bool         # já centralizou alguma vez neste goal
    t_c: float              # tempo de convergência (s); 0 se não convergiu
    e_r: float              # erro residual (px); 0 se não convergiu


class _Eixo:
    """Detecção de eixo parado com comando empurrando num sentido."""

    def __init__(self):
        self.limpar()

    def limpar(self):
        self.sentido = 0
        self.inicio = 0.0
        self.referencia = 0.0
        self.posicao = 0.0

    def atualizar(self, t: float, posicao_deg: float, cmd_deg_s: float):
        self.posicao = posicao_deg
        sentido = _sign(cmd_deg_s) if abs(cmd_deg_s) >= LIMIT_MIN_CMD_DEG_S else 0
        if sentido == 0:
            self.limpar()
        elif sentido != self.sentido or abs(posicao_deg - self.referencia) > MOVE_EPS_DEG:
            # Começou a empurrar, trocou de sentido ou se moveu: recomeça a contagem
            self.sentido = sentido
            self.inicio = t
            self.referencia = posicao_deg

    def parado_ha(self, now: float) -> float:
        return now - self.inicio if self.sentido else 0.0


class Supervisor:
    """
    Estágio 4 do pipeline: condições de término e métricas de um goal.

      - centrado: error_px <= tolerance_px em todas as amostras com alvo de
        uma janela contínua de pelo menos hold_time_s (pelos header.stamp).
        Uma amostra fora da tolerância ou sem alvo reinicia a janela;
      - perdido: nenhum alvo válido há mais de lost_timeout_s (contado do
        header.stamp do último alvo válido, ou do início do goal se ainda
        não houve nenhum);
      - preso: um eixo com comando de pelo menos LIMIT_MIN_CMD_DEG_S no mesmo
        sentido e sem se mover mais que MOVE_EPS_DEG por mais de
        limit_timeout_s. Cobre o limite de ângulo do bridge, que zera o eixo
        que empurra para fora, e o eixo travado.

    Métricas da primeira convergência (seção 10): t_c vai da primeira detecção
    válida até o início da janela que completou hold_time_s (o instante em que
    o erro entrou na tolerância para ficar); e_r é a média de error_px nessa
    janela.
    """

    def __init__(self, tolerance_px: float, hold_time_s: float, lost_timeout_s: float,
                 limit_timeout_s: float, inicio: float):
        self.tolerance_px = tolerance_px
        self.hold_time_s = hold_time_s
        self.lost_timeout_s = lost_timeout_s
        self.limit_timeout_s = limit_timeout_s

        self._ultimo_valido = inicio
        self._primeira_deteccao = None
        self._janela_inicio = None
        self._janela_erros = []
        self.centrado = False

        self.convergiu = False
        self.t_c = 0.0
        self.e_r = 0.0

        self._eixos = {'pan': _Eixo(), 'tilt': _Eixo()}

    def alvo(self, t: float, detected: bool, error_px: float):
        """Registra um alvo de /perception/target; t é o header.stamp em s."""
        if not detected:
            self._fechar_janela()
            return
        self._ultimo_valido = max(self._ultimo_valido, t)
        if self._primeira_deteccao is None:
            self._primeira_deteccao = t

        if error_px > self.tolerance_px:
            self._fechar_janela()
            return
        if self._janela_inicio is None:
            self._janela_inicio = t
        self._janela_erros.append(error_px)
        self.centrado = t - self._janela_inicio >= self.hold_time_s
        if self.centrado and not self.convergiu:
            self.convergiu = True
            self.t_c = self._janela_inicio - self._primeira_deteccao
            self.e_r = sum(self._janela_erros) / len(self._janela_erros)

    def _fechar_janela(self):
        self._janela_inicio = None
        self._janela_erros = []
        self.centrado = False

    def junta(self, t: float, pan_deg: float, tilt_deg: float,
              cmd_pan_deg_s: float, cmd_tilt_deg_s: float):
        """Registra a posição das juntas e o comando publicado no instante t."""
        self._eixos['pan'].atualizar(t, pan_deg, cmd_pan_deg_s)
        self._eixos['tilt'].atualizar(t, tilt_deg, cmd_tilt_deg_s)

    def limpar_juntas(self):
        """Sem /joint_states recente: a checagem de limite recomeça do zero."""
        for eixo in self._eixos.values():
            eixo.limpar()

    def checar(self, now: float) -> EstadoSupervisao:
        preso = None
        for nome, eixo in self._eixos.items():
            duracao = eixo.parado_ha(now)
            if duracao > self.limit_timeout_s:
                preso = Preso(nome, eixo.sentido, eixo.posicao, duracao)
                break
        return EstadoSupervisao(
            centrado=self.centrado,
            perdido=now - self._ultimo_valido > self.lost_timeout_s,
            preso=preso,
            convergiu=self.convergiu,
            t_c=self.t_c,
            e_r=self.e_r,
        )
