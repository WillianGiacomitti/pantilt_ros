"""
Controlador nebuloso (fuzzy) usado pelo visual_servo_node (docs/architecture.md, seção 4.5).

Lógica pura (sem ROS). Erro em rad, saída em rad/s, como o PID.

Inferência de Sugeno de ordem zero, equivalente a um Mamdani com conjuntos de
saída simétricos e defuzzificação por centroide, mas sem integração numérica:

  - 2 entradas: erro normalizado (5 conjuntos) e variação do erro (3 conjuntos);
  - funções de pertinência triangulares uniformes formando partição da unidade
    (a soma das pertinências é sempre 1, e no máximo 4 regras pesam por ciclo);
  - 15 regras, cada uma com uma saída singleton em [-1, 1];
  - defuzzificação por média ponderada (o denominador é 1 por construção).

NORMALIZAÇÃO SEM PARÂMETROS NOVOS. O nó passa as mesmas chaves do PID. Daí:

    erro_max     = max_output / kp                    erro em que a ação proporcional saturaria
    derivada_max = max_output * FATOR_DERIVADA        taxa máxima com que o erro pode fechar

Com isso, uma tabela LINEAR reproduziria exatamente o controlador P de ganho kp.
A comparação com o PID isola, portanto, apenas a não linearidade da base de
regras. `kp` passa a ser o fator de escala do fuzzy, e não um ganho proporcional.

`ki`, `kd` e `integral_limit` são aceitos para manter a interface, mas NÃO são
usados: este controlador não tem ação integral. Nos ensaios com `controller:
fuzzy`, deixe `ki` e `kd` em zero para não haver dúvida na leitura do
`resumo.json`. `derivative_filter_hz` é usado, com o mesmo filtro do PID, para
que as duas malhas vejam a mesma derivada.

SATURAÇÃO. Como a saída é uma média de valores em [-1, 1] vezes max_output, ela
nunca passa do teto: a saturação acontece no recorte da entrada em ±erro_max.
`saturado` indica o comando no teto (|u| = max_output), que é o que a métrica de
saturação dos ensaios mede.
"""

import math

from pantilt_control.controllers.base import Controlador

# Multiplica a escala da variação do erro. Em 1,0, a pertinência plena ocorre
# quando o erro fecha na velocidade máxima do eixo. Aumentar este fator SUAVIZA
# a ação derivativa (a mesma variação passa a pesar menos). É o único ajuste de
# contingência do fuzzy, e vive aqui em vez de virar parâmetro ROS para manter
# a promessa de que só `controller` muda entre os dois métodos.
FATOR_DERIVADA = 1.0

# Tolerância relativa para considerar a saída no teto (erro de arredondamento
# da média ponderada)
TOL_SATURACAO = 1e-9

# Conjuntos do erro:      NG     NP     Z     PP     PG   (centros -1 .. +1)
# Conjuntos da variação:  N      Z      P                 (centros -1, 0, +1)
#
# TABELA_PADRAO[i][j]: i = conjunto do erro, j = conjunto da variação.
#
# A coluna central (-1,0 / -0,4 / 0 / +0,4 / +1,0) é deliberadamente NÃO linear:
# a versão linear seria -1,0 / -0,5 / 0 / +0,5 / +1,0. A inclinação resulta 0,8
# perto do centro e 1,2 longe, ou seja, cerca de 30% de variação de ganho ao
# longo do quadro: menos reação ao tremor da caixa delimitadora quando o alvo
# já está centralizado e aproximação mais firme quando está longe.
#
# Propriedades a preservar em qualquer ajuste:
#   1. simetria: tabela[i][j] == -tabela[4-i][2-j];
#   2. coluna central não linear (sem isso o controlador vira um PD);
#   3. monotonicidade: a saída não diminui quando o erro aumenta.
TABELA_PADRAO = (
    (-1.00, -1.00, -0.60),   # erro NG
    (-0.70, -0.40, -0.10),   # erro NP
    (-0.25,  0.00,  0.25),   # erro Z
    (0.10,  0.40,  0.70),    # erro PP
    (0.60,  1.00,  1.00),    # erro PG
)


def _saturar(valor: float, limite: float) -> float:
    return max(-limite, min(limite, valor))


def _pertinencias(x: float, n_conjuntos: int) -> list:
    """
    Pertinências de x a n_conjuntos triângulos uniformes em [-1, 1].

    Os centros são igualmente espaçados de -1 a +1 e as bases são adjacentes,
    de modo que a soma das pertinências é sempre 1 (partição da unidade). Isso
    dispensa a divisão na defuzzificação.
    """
    x = _saturar(x, 1.0)
    passo = 2.0 / (n_conjuntos - 1)
    posicao = (x + 1.0) / passo                    # posição contínua entre os centros
    i = min(int(posicao), n_conjuntos - 2)         # índice do centro à esquerda
    fracao = posicao - i
    graus = [0.0] * n_conjuntos
    graus[i] = 1.0 - fracao
    graus[i + 1] = fracao
    return graus


def _validar_tabela(tabela):
    """Levanta ValueError se a tabela de regras não for uma matriz >= 2x2 com valores em [-1, 1]."""
    try:
        linhas = [tuple(float(v) for v in linha) for linha in tabela]
    except (TypeError, ValueError) as e:
        raise ValueError('tabela de regras malformada: valores não numéricos') from e
    if len(linhas) < 2 or len({len(linha) for linha in linhas}) != 1 or len(linhas[0]) < 2:
        raise ValueError('tabela de regras malformada: precisa de pelo menos 2x2, '
                         'com linhas do mesmo tamanho')
    if not all(math.isfinite(v) and -1.0 <= v <= 1.0 for linha in linhas for v in linha):
        raise ValueError('tabela de regras malformada: valores devem estar em [-1, 1]')
    return tuple(linhas)


class ControladorFuzzy(Controlador):
    def __init__(self, kp: float, ki: float, kd: float, derivative_filter_hz: float,
                 max_output: float, integral_limit: float, tabela=TABELA_PADRAO):
        if kp <= 0.0:
            raise ValueError(
                f'kp deve ser positivo no fuzzy (é o fator de escala), recebido {kp}')
        if derivative_filter_hz <= 0.0:
            raise ValueError(
                f'derivative_filter_hz deve ser positivo, recebido {derivative_filter_hz}')
        if max_output <= 0.0:
            raise ValueError(f'max_output deve ser positivo, recebido {max_output}')

        self.kp = kp
        self.max_output = max_output
        self.erro_max = max_output / kp          # rad
        self.derivada_max = max_output * FATOR_DERIVADA   # rad/s
        self.tabela = _validar_tabela(tabela)
        self.n_erro = len(self.tabela)
        self.n_derivada = len(self.tabela[0])
        # Mesma constante de tempo do filtro da derivada do PID
        self.tau = 1.0 / (2.0 * math.pi * derivative_filter_hz)
        self.reset()

    def reset(self):
        self._filtrado = None        # erro filtrado da amostra anterior
        self.saturado = False
        self._u_bruto = 0.0

    def compute(self, erro: float, dt: float) -> float:
        # Derivada do erro, com o mesmo filtro do PID. A primeira amostra (ou a
        # primeira após uma lacuna, com dt <= 0) só inicializa o filtro.
        derivada = 0.0
        if self._filtrado is None:
            self._filtrado = erro
        elif dt > 0.0:
            alfa = dt / (self.tau + dt)
            filtrado = self._filtrado + alfa * (erro - self._filtrado)
            derivada = (filtrado - self._filtrado) / dt
            self._filtrado = filtrado

        # Fuzzificação
        mu_erro = _pertinencias(erro / self.erro_max, self.n_erro)
        mu_derivada = _pertinencias(derivada / self.derivada_max, self.n_derivada)

        # Inferência e defuzzificação por média ponderada (denominador = 1)
        saida = 0.0
        for i, me in enumerate(mu_erro):
            if me == 0.0:
                continue
            for j, md in enumerate(mu_derivada):
                if md == 0.0:
                    continue
                saida += me * md * self.tabela[i][j]

        u_bruto = saida * self.max_output
        u = _saturar(u_bruto, self.max_output)
        self._u_bruto = u_bruto
        self.saturado = abs(u) >= self.max_output * (1.0 - TOL_SATURACAO)
        return u

    def saida_bruta(self) -> float:
        """Saída antes da saturação, em rad/s (para o /control/servo_debug)."""
        return self._u_bruto
