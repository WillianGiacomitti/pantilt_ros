"""
Lógica da captura sem dependência do ROS nem do OpenCV (testável isoladamente):
interpretação do parâmetro source, validação dos parâmetros e cadência de
publicação.
"""

import re

KIND_DEVICE = 'device'
KIND_URL = 'url'
KIND_FILE = 'file'

_DEVICE_PATH = re.compile(r'^/dev/video\d+$')


def parse_source(source: str):
    """
    Interpreta o parâmetro source do camera_node.

    Retorna (tipo, valor):
      "0", "2"            -> ('device', 0), ('device', 2)    índice V4L2
      "/dev/video0"       -> ('device', '/dev/video0')       caminho do dispositivo
      "http://..." etc.   -> ('url', source)                 stream (ex.: MJPEG do Windows)
      qualquer outro      -> ('file', source)                arquivo de vídeo
    """
    text = str(source).strip()
    if not text:
        raise ValueError('source vazio: use um índice ("0"), uma URL ou um caminho de vídeo')
    if text.isdigit():
        return KIND_DEVICE, int(text)
    if _DEVICE_PATH.match(text):
        return KIND_DEVICE, text
    if '://' in text:
        return KIND_URL, text
    return KIND_FILE, text


def validate_params(width: int, height: int, fps: float):
    """Levanta ValueError se a resolução ou a taxa não forem positivas."""
    if width <= 0 or height <= 0:
        raise ValueError(f'resolução {width}x{height} inválida: width e height devem ser positivos')
    if fps <= 0.0:
        raise ValueError(f'fps={fps} inválido: deve ser positivo')


def fourcc_to_str(code: float) -> str:
    """Converte o CAP_PROP_FOURCC do OpenCV (ex.: 1196444237.0) em texto ('MJPG')."""
    value = int(code)
    text = ''.join(chr((value >> (8 * i)) & 0xFF) for i in range(4))
    return text if text.isprintable() and text.strip() else '?'


class FrameThrottle:
    """
    Limita a publicação a fps quadros por segundo, com base em prazos.

    Um limite ingênuo ("publica se passou 1/fps desde a última") perde metade
    dos quadros quando a fonte tem a mesma taxa do limite: com jitter, um
    quadro chega a 32 ms quando o período é 33,3 ms e é descartado. Aqui cada
    publicação agenda o próximo prazo (prazo anterior + período) e aceita
    quadros que cheguem até slack_ratio x período antes dele.
    """

    def __init__(self, fps: float, slack_ratio: float = 0.1):
        if fps <= 0.0:
            raise ValueError(f'fps={fps} inválido: deve ser positivo')
        self.period = 1.0 / fps
        self.slack = slack_ratio * self.period
        self.next_t = None

    def ready(self, now: float) -> bool:
        """True se o quadro recebido em now (s, relógio monotônico) deve ser publicado."""
        if self.next_t is None or now - self.next_t > self.period:
            # Primeiro quadro ou fonte atrasada (reconexão, travamento): realinha
            # para não publicar uma rajada tentando recuperar o atraso
            self.next_t = now
        elif now < self.next_t - self.slack:
            return False
        self.next_t += self.period
        return True

    def wait_time(self, now: float) -> float:
        """Tempo até o próximo prazo (s). Usado para cadenciar a leitura de arquivos."""
        if self.next_t is None:
            return 0.0
        return max(0.0, self.next_t - now)

    def reset(self):
        self.next_t = None
