"""
Padrão de varredura em zigue-zague executado pelo scan_node.

Lógica pura (sem ROS) para poder ser testada sem hardware. Posições em graus,
velocidades em graus/s e o instante `now` em segundos de um relógio
monotônico. O scan_node converte de e para rad e rad/s nos tópicos.

Padrão (docs/architecture.md, seção 4.4):
  - começa pela ponta do pan mais próxima da posição atual e percorre o pan de
    uma ponta à outra em cada faixa de tilt, alternando o sentido;
  - cada trecho move um eixo por vez: pan até a ponta inicial, tilt até a
    primeira faixa, pan até a outra ponta, tilt até a faixa seguinte, e assim
    por diante;
  - o alvo de um eixo conta como atingido dentro de TOLERANCE_DEG ou quando é
    ultrapassado no sentido do movimento, para o eixo não inverter perto dele;
  - perto do alvo a velocidade cai com a distância (até MIN_SPEED_DEG_S), o que
    reduz a ultrapassagem causada pelo atraso da telemetria;
  - se, com velocidade comandada, nenhum eixo se mover por mais de
    STALL_TIMEOUT_S, a varredura falha. Cobre eixo travado e sentido de giro
    invertido, que leva o eixo ao limite do bridge, onde ele para. O jog do
    operador move o eixo e não dispara a falha.
"""

from dataclasses import dataclass

# Distância ao alvo considerada atingida
TOLERANCE_DEG = 0.5

# Perto do alvo, velocidade = distância / SLOWDOWN_S (limitada a speed)
SLOWDOWN_S = 0.5

# Velocidade mínima na aproximação, para não se arrastar até a tolerância.
# Com SLOWDOWN_S = 0,5 s, vale no último 1° antes do alvo.
MIN_SPEED_DEG_S = 2.0

# Tempo máximo sem movimento com velocidade comandada
STALL_TIMEOUT_S = 3.0

# Deslocamento mínimo, em qualquer eixo, para considerar que houve movimento
MOVE_EPS_DEG = 0.2


@dataclass(frozen=True)
class Waypoint:
    pan: float
    tilt: float
    pass_index: int     # faixa de tilt a que o trecho pertence


@dataclass(frozen=True)
class ScanStep:
    pan_vel: float = 0.0
    tilt_vel: float = 0.0
    pass_index: int = 0
    done: bool = False
    error: str = ''     # não vazio = falha; o nó deve parar e abortar


def _sign(x: float) -> int:
    return (x > 0) - (x < 0)


class ScanPattern:
    def __init__(self, pan_min: float, pan_max: float, tilt_levels, speed: float):
        tilt_levels = [float(t) for t in tilt_levels]
        if pan_min >= pan_max:
            raise ValueError(f'pan_min ({pan_min}) deve ser menor que pan_max ({pan_max})')
        if not tilt_levels:
            raise ValueError('tilt_levels precisa de pelo menos uma faixa')
        if speed <= 0.0:
            raise ValueError(f'a velocidade deve ser positiva, recebido {speed}')
        self.pan_min = float(pan_min)
        self.pan_max = float(pan_max)
        self.tilt_levels = tilt_levels
        self.speed = float(speed)

        self._waypoints = []
        self._index = 0
        self._dirs = (0, 0)         # sentido de cada eixo no trecho atual
        self._ref = (0.0, 0.0)      # posição da última detecção de movimento
        self._last_move = 0.0

    @property
    def waypoints(self):
        return list(self._waypoints)

    def start(self, now: float, pan: float, tilt: float):
        """Monta o padrão a partir da posição atual."""
        near_min = abs(pan - self.pan_min) <= abs(pan - self.pan_max)
        end = self.pan_min if near_min else self.pan_max
        # Primeiro trecho: só o pan, até a ponta inicial, mantendo o tilt atual
        waypoints = [Waypoint(end, tilt, 0)]
        for i, level in enumerate(self.tilt_levels):
            waypoints.append(Waypoint(end, level, i))
            end = self.pan_max if end == self.pan_min else self.pan_min
            waypoints.append(Waypoint(end, level, i))

        self._waypoints = waypoints
        self._index = 0
        self._ref = (pan, tilt)
        self._last_move = now
        self._begin_segment(pan, tilt)

    def update(self, now: float, pan: float, tilt: float) -> ScanStep:
        """Velocidades para a posição atual. Chamar a cada ciclo do laço."""
        if not self._waypoints:
            raise RuntimeError('start() precisa ser chamado antes de update()')

        if abs(pan - self._ref[0]) > MOVE_EPS_DEG or abs(tilt - self._ref[1]) > MOVE_EPS_DEG:
            self._ref = (pan, tilt)
            self._last_move = now

        # Trechos já atingidos (inclusive os de comprimento zero) são pulados no mesmo ciclo
        while self._index < len(self._waypoints):
            wp = self._waypoints[self._index]
            err_pan, err_tilt = wp.pan - pan, wp.tilt - tilt
            if self._reached(err_pan, self._dirs[0]) and self._reached(err_tilt, self._dirs[1]):
                self._index += 1
                self._begin_segment(pan, tilt)
                continue

            if now - self._last_move > STALL_TIMEOUT_S:
                return ScanStep(pass_index=wp.pass_index, error=(
                    f'Nenhum eixo se moveu em {STALL_TIMEOUT_S:.0f} s indo para '
                    f'pan {wp.pan:.1f}°, tilt {wp.tilt:.1f}° (eixo travado, no limite '
                    'ou com sentido de giro invertido)'))

            return ScanStep(
                pan_vel=self._axis_velocity(err_pan, self._dirs[0]),
                tilt_vel=self._axis_velocity(err_tilt, self._dirs[1]),
                pass_index=wp.pass_index,
            )

        return ScanStep(pass_index=self._waypoints[-1].pass_index, done=True)

    # ---------------- Internos ----------------
    def _begin_segment(self, pan: float, tilt: float):
        if self._index < len(self._waypoints):
            wp = self._waypoints[self._index]
            self._dirs = (_sign(wp.pan - pan), _sign(wp.tilt - tilt))

    @staticmethod
    def _reached(err: float, direction: int) -> bool:
        if abs(err) <= TOLERANCE_DEG:
            return True
        # Ultrapassou o alvo no sentido do movimento
        return direction != 0 and err * direction < 0

    def _axis_velocity(self, err: float, direction: int) -> float:
        if self._reached(err, direction):
            return 0.0
        floor = min(MIN_SPEED_DEG_S, self.speed)
        magnitude = max(min(self.speed, abs(err) / SLOWDOWN_S), floor)
        return _sign(err) * magnitude
