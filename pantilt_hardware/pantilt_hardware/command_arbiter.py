"""
Arbitragem de comandos aplicada pelo command_mux.

Lógica pura (sem ROS) para poder ser testada sem hardware. Todos os métodos
recebem o instante `now` em segundos de um relógio monotônico.

Regras (docs/architecture.md, seção 4.6):
  - prioridade: um comando da web é sempre aceito e, enquanto a web tiver
    publicado nos últimos operator_hold_s, os comandos automáticos são descartados;
  - watchdog: se a fonte ativa ficar em silêncio por mais de cmd_timeout_s
    depois de uma velocidade não nula, pede o envio de velocidade zero uma vez.
    Comandos de posição e velocidades nulas não armam o watchdog: um zero
    depois de um comando de posição interromperia o movimento no firmware.
"""

SOURCE_WEB = 'web'
SOURCE_AUTO = 'auto'
SOURCE_NONE = 'none'


class CommandArbiter:
    def __init__(self, operator_hold_s: float, cmd_timeout_s: float):
        if operator_hold_s <= 0.0:
            raise ValueError(f'operator_hold_s deve ser positivo, recebido {operator_hold_s}')
        if cmd_timeout_s <= 0.0:
            raise ValueError(f'cmd_timeout_s deve ser positivo, recebido {cmd_timeout_s}')
        self.operator_hold_s = operator_hold_s
        self.cmd_timeout_s = cmd_timeout_s

        self._last_web = None       # instante do último comando da web
        self._last_auto = None      # instante do último comando automático aceito
        self._last_forward = None   # instante do último comando repassado
        self._armed_source = None   # fonte da última velocidade não nula; None = desarmado

    @property
    def armed(self) -> bool:
        """True se a última velocidade repassada não foi nula e ainda não houve zero."""
        return self._armed_source is not None

    def operator_holding(self, now: float) -> bool:
        """True enquanto a janela de prioridade do operador estiver aberta."""
        return self._last_web is not None and (now - self._last_web) < self.operator_hold_s

    def on_web(self, now: float, is_velocity: bool, is_zero: bool = False):
        """Registra um comando da web. Ele é sempre repassado."""
        self._last_web = now
        self._forwarded(now, SOURCE_WEB, arm=is_velocity and not is_zero)

    def on_auto(self, now: float, is_zero: bool) -> bool:
        """Registra uma velocidade automática. Retorna False se ela deve ser descartada."""
        if self.operator_holding(now):
            return False
        self._last_auto = now
        self._forwarded(now, SOURCE_AUTO, arm=not is_zero)
        return True

    def check_watchdog(self, now: float):
        """
        Retorna a fonte que ficou em silêncio quando é preciso enviar velocidade
        zero, ou None. Dispara uma única vez por velocidade não nula.
        """
        if self._armed_source is None:
            return None
        if (now - self._last_forward) <= self.cmd_timeout_s:
            return None
        source = self._armed_source
        self._armed_source = None
        return source

    def source(self, now: float) -> str:
        """Fonte ativa: web (janela do operador), auto (sem silêncio) ou none."""
        if self.operator_holding(now):
            return SOURCE_WEB
        if self._last_auto is not None and (now - self._last_auto) < self.cmd_timeout_s:
            return SOURCE_AUTO
        return SOURCE_NONE

    def _forwarded(self, now: float, source: str, arm: bool):
        self._last_forward = now
        self._armed_source = source if arm else None
