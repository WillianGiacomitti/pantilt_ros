"""
Lógica da calibração por rotação usada pelo calibration_node.

Lógica pura (sem ROS) para poder ser testada sem hardware. Ângulos em graus
na interface, exceto onde o nome indica rad; erros em pixels; o instante
`now` em segundos de um relógio monotônico.

Modelo (docs/architecture.md, seção 4.10): com o alvo fixo no mundo e o eixo
no ângulo θ, o erro do alvo na imagem, medido em relação ao centro
geométrico, é

    e = k · tan(θ − φ)        f = |k|

em que φ é a direção do alvo em relação ao zero do encoder. O ponto principal
fica fixo no centro: sob rotação pura, um deslocamento de c e um de φ quase
não se distinguem, e deixar c livre degrada f. O sinal de k diz para que lado
o alvo anda na imagem quando o eixo gira no sentido positivo.
"""

import math
from dataclasses import dataclass

import numpy as np

# Mínimo de pontos para o ajuste: 2 incógnitas + 1 para haver resíduo
MIN_FIT_POINTS = 3

# Levenberg-Marquardt: iterações máximas, amortecimento inicial e teto do
# amortecimento (acima dele nenhum passo reduz o custo: já está no mínimo)
LM_MAX_ITER = 100
LM_LAMBDA0 = 1e-3
LM_LAMBDA_MAX = 1e10

# Redução relativa do custo abaixo da qual o ajuste é considerado convergido
LM_TOL = 1e-12

# Distância máxima entre φ e o centro da grade; além disso o ajuste caiu em
# outro ramo da tangente e não tem sentido físico
MAX_PHI_OFFSET_DEG = 45.0

# Ângulo máximo aceito em max_angle_deg (a distorção radial cresce nas bordas)
MAX_GRID_ANGLE_DEG = 20.0


@dataclass(frozen=True)
class AjusteEixo:
    """Resultado do ajuste de um eixo."""
    f: float            # px, sempre positivo
    k: float            # px, com sinal: k < 0 -> eixo positivo leva o alvo para erro menor
    phi_deg: float      # direção do alvo em relação ao zero do encoder
    rms: float          # px
    n: int              # pontos usados
    residuos: tuple     # px, na ordem dos pontos


def ajustar_eixo(theta_deg, erro_px) -> AjusteEixo:
    """
    Ajusta e = k·tan(θ − φ) por mínimos quadrados (Levenberg-Marquardt).
    Levanta ValueError se os dados não permitirem o ajuste.
    """
    theta = np.radians(np.asarray(theta_deg, dtype=float))
    erro = np.asarray(erro_px, dtype=float)
    if theta.ndim != 1 or theta.shape != erro.shape:
        raise ValueError('ângulos e erros devem ser listas do mesmo tamanho')
    n = len(theta)
    if n < MIN_FIT_POINTS:
        raise ValueError(f'são precisos pelo menos {MIN_FIT_POINTS} pontos, recebido {n}')
    if not (np.all(np.isfinite(theta)) and np.all(np.isfinite(erro))):
        raise ValueError('ângulos ou erros não finitos')
    if np.ptp(theta) < 1e-9:
        raise ValueError('todos os pontos têm o mesmo ângulo')

    # Chute inicial: fórmula de dois pontos nos extremos, com φ no centro da grade
    centro = float(np.mean(theta))
    i_min, i_max = int(np.argmin(theta)), int(np.argmax(theta))
    k0 = (erro[i_max] - erro[i_min]) / (math.tan(theta[i_max] - centro)
                                        - math.tan(theta[i_min] - centro))
    if abs(k0) < 1e-6:
        raise ValueError('o alvo não se deslocou na imagem com o giro do eixo')

    def residuos(p):
        return p[0] * np.tan(theta - p[1]) - erro

    p = np.array([k0, centro])
    r = residuos(p)
    custo = float(r @ r)
    lam = LM_LAMBDA0
    for _ in range(LM_MAX_ITER):
        t = np.tan(theta - p[1])
        jac = np.column_stack([t, -p[0] * (1.0 + t * t)])
        a = jac.T @ jac
        g = jac.T @ r
        try:
            passo = np.linalg.solve(a + lam * np.diag(np.diag(a)), -g)
        except np.linalg.LinAlgError as e:
            raise ValueError('ajuste mal condicionado') from e
        p_novo = p + passo
        r_novo = residuos(p_novo)
        custo_novo = float(r_novo @ r_novo)
        if np.all(np.isfinite(r_novo)) and custo_novo < custo:
            reducao = custo - custo_novo
            p, r, custo = p_novo, r_novo, custo_novo
            lam = max(lam / 10.0, 1e-12)
            if reducao <= LM_TOL * max(custo, 1e-30) or custo == 0.0:
                break
        else:
            lam *= 10.0
            if lam > LM_LAMBDA_MAX:
                break

    k, phi = float(p[0]), float(p[1])
    if not (math.isfinite(k) and math.isfinite(phi)) or abs(k) < 1e-6:
        raise ValueError('o ajuste não convergiu')
    if abs(math.degrees(phi - centro)) > MAX_PHI_OFFSET_DEG:
        raise ValueError('o ajuste não convergiu (direção do alvo sem sentido físico)')

    return AjusteEixo(
        f=abs(k),
        k=k,
        phi_deg=math.degrees(phi),
        rms=math.sqrt(custo / n),
        n=n,
        residuos=tuple(float(x) for x in r),
    )


def k_sondagem(theta_a_deg: float, erro_a: float, theta_b_deg: float, erro_b: float,
               theta_ref_deg: float) -> float:
    """
    Estimativa grosseira de k com dois pontos, supondo o alvo na direção
    theta_ref (o operador o centralizou ali). Levanta ValueError se os ângulos
    coincidirem.
    """
    ta = math.tan(math.radians(theta_a_deg - theta_ref_deg))
    tb = math.tan(math.radians(theta_b_deg - theta_ref_deg))
    if abs(tb - ta) < 1e-9:
        raise ValueError('os dois pontos da sondagem têm o mesmo ângulo')
    return (erro_b - erro_a) / (tb - ta)


def grade_eixo(f_px: float, meia_dim_px: float, n_points: int, max_angle_deg: float,
               fracao: float) -> list:
    """
    Deslocamentos angulares (graus, em relação ao home) de um eixo, em ordem
    crescente para que todos os pontos sejam alcançados no mesmo sentido.

    O maior ângulo é o que leva o alvo a fracao da meia-dimensão do quadro,
    limitado a max_angle_deg.
    """
    if f_px <= 0.0:
        raise ValueError(f'f deve ser positivo, recebido {f_px}')
    amplitude = min(max_angle_deg, math.degrees(math.atan(fracao * meia_dim_px / f_px)))
    return [float(x) for x in np.linspace(-amplitude, amplitude, n_points)]


def alvo_centralizado(erro_x: float, erro_y: float, largura: int, altura: int,
                      fracao: float) -> bool:
    """True se o alvo estiver dentro de fracao da meia-dimensão em cada eixo."""
    return abs(erro_x) <= fracao * largura / 2.0 and abs(erro_y) <= fracao * altura / 2.0


def fov_deg(f_px: float, dim_px: float) -> float:
    """Campo de visão implícito por f numa dimensão do quadro."""
    return math.degrees(2.0 * math.atan(dim_px / (2.0 * f_px)))


class DetectorParada:
    """
    Detecta a parada de uma junta no alvo de posição.

    A junta está parada no alvo quando |posição − alvo| < tol_deg e
    |velocidade| < vel_max_deg_s, continuamente por hold_s.
    """

    def __init__(self, tol_deg: float, vel_max_deg_s: float, hold_s: float):
        self.tol = tol_deg
        self.vel_max = vel_max_deg_s
        self.hold = hold_s
        self._no_alvo_desde = None
        self._parado_desde = None
        self.posicao = None
        self.velocidade = None

    def update(self, now: float, posicao: float, velocidade: float, alvo: float):
        """Retorna o instante em que a parada foi confirmada, ou None."""
        self.posicao = posicao
        self.velocidade = velocidade
        parado = abs(velocidade) < self.vel_max
        if not parado:
            self._parado_desde = None
        elif self._parado_desde is None:
            self._parado_desde = now
        if parado and abs(posicao - alvo) < self.tol:
            if self._no_alvo_desde is None:
                self._no_alvo_desde = now
            if now - self._no_alvo_desde >= self.hold:
                return now
        else:
            self._no_alvo_desde = None
        return None

    def parado_fora_do_alvo(self, now: float) -> bool:
        """True se a junta está parada há hold_s, mas fora da tolerância do alvo."""
        return (self._parado_desde is not None and now - self._parado_desde >= self.hold
                and self._no_alvo_desde is None)


def validar_parametros(probe_deg: float, max_angle_deg: float, n_points: int,
                       samples_per_point: int, settle_tol_deg: float, settle_extra_s: float,
                       min_points: int):
    """Levanta ValueError com o motivo se algum parâmetro do calibration_node for inválido."""
    if probe_deg <= 0.0:
        raise ValueError(f'probe_deg deve ser positivo, recebido {probe_deg}')
    if not 0.0 < max_angle_deg <= MAX_GRID_ANGLE_DEG:
        raise ValueError(
            f'max_angle_deg deve estar em (0, {MAX_GRID_ANGLE_DEG:.0f}], recebido {max_angle_deg}')
    if probe_deg > max_angle_deg:
        raise ValueError(f'probe_deg ({probe_deg}) não pode passar de max_angle_deg ({max_angle_deg})')
    if min_points < MIN_FIT_POINTS:
        raise ValueError(f'min_points deve ser pelo menos {MIN_FIT_POINTS}, recebido {min_points}')
    if n_points < min_points:
        raise ValueError(f'n_points ({n_points}) não pode ser menor que min_points ({min_points})')
    if samples_per_point < 1:
        raise ValueError(f'samples_per_point deve ser pelo menos 1, recebido {samples_per_point}')
    if settle_tol_deg <= 0.0:
        raise ValueError(f'settle_tol_deg deve ser positivo, recebido {settle_tol_deg}')
    if settle_extra_s < 0.0:
        raise ValueError(f'settle_extra_s não pode ser negativo, recebido {settle_extra_s}')
