"""
Protocolo serial binário entre o host e a ESP32.

Contrato definido em pantilt_firmware/include/Serialprotocol.h. Qualquer
mudança em tipos, códigos ou payloads exige alterar o firmware também.

Formato do frame: [0xA5][0x5A][TYPE][LEN][PAYLOAD...][CRC8]
CRC-8 com polinômio 0x07, calculado sobre TYPE, LEN e PAYLOAD.
Floats em little-endian.
"""

SYNC1, SYNC2 = 0xA5, 0x5A
MSG_TELEMETRY = 0x01
MSG_CMD_POS = 0x02
MSG_CMD_VEL = 0x03
MSG_SET_ZERO_REQ = 0x04
MSG_SET_ZERO_ACK = 0x05
MSG_HEARTBEAT = 0x06
MSG_ERROR = 0x07
MSG_BOOT_INFO = 0x08

# Espelho de FAILSAFE_TIMEOUT_MS do firmware: motores param após esse tempo sem frames
FAILSAFE_TIMEOUT_S = 0.5
# Taxa de envio da telemetria pelo firmware
TELEMETRY_RATE_HZ = 20.0

# Precisa ficar em sincronia com os #define ERR_* em Serialprotocol.h
ERROR_CODES = {
    1: 'Timeout de leitura I2C no encoder PAN',
    2: 'Timeout de leitura I2C no encoder TILT',
    3: 'Barramento I2C estava travado - recuperação automática acionada',
    4: 'Fail-safe acionado: motores parados por perda de comunicação',
    5: 'Comando /ptu/cmd_pos recebido com payload inválido',
    6: 'Comando /ptu/cmd_vel recebido com payload inválido',
    7: 'Heap livre baixo na ESP32 (possível vazamento de memória)',
}

# Precisa ficar em sincronia com enum esp_reset_reason_t do ESP-IDF
RESET_REASONS = {
    0: 'ESP_RST_UNKNOWN (desconhecido)',
    1: 'ESP_RST_POWERON (ligou/energizou normalmente)',
    2: 'ESP_RST_EXT (reset externo via pino)',
    3: 'ESP_RST_SW (reset via software, ex: esp_restart())',
    4: 'ESP_RST_PANIC (crash/exceção)',
    5: 'ESP_RST_INT_WDT (watchdog de interrupção)',
    6: 'ESP_RST_TASK_WDT (watchdog de task - nosso watchdog pegou uma trava)',
    7: 'ESP_RST_WDT (outro watchdog)',
    8: 'ESP_RST_DEEPSLEEP',
    9: 'ESP_RST_BROWNOUT (queda de tensão! checar fonte de alimentação)',
    10: 'ESP_RST_SDIO',
    12: 'ESP_RST_USB',
    14: 'ESP_RST_JTAG',
}


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def build_frame(msg_type: int, payload: bytes) -> bytes:
    header = bytes([msg_type, len(payload)])
    crc = crc8(header + payload)
    return bytes([SYNC1, SYNC2]) + header + payload + bytes([crc])


class FrameParser:
    """State machine não bloqueante, alimentada byte a byte."""

    (WAIT_SYNC1, WAIT_SYNC2, WAIT_TYPE,
     WAIT_LEN, WAIT_PAYLOAD, WAIT_CRC) = range(6)

    def __init__(self, on_frame):
        self.state = self.WAIT_SYNC1
        self.on_frame = on_frame
        self.type = 0
        self.length = 0
        self.payload = bytearray()

    def feed(self, byte: int):
        if self.state == self.WAIT_SYNC1:
            if byte == SYNC1:
                self.state = self.WAIT_SYNC2

        elif self.state == self.WAIT_SYNC2:
            self.state = self.WAIT_TYPE if byte == SYNC2 else self.WAIT_SYNC1

        elif self.state == self.WAIT_TYPE:
            self.type = byte
            self.state = self.WAIT_LEN

        elif self.state == self.WAIT_LEN:
            self.length = byte
            self.payload = bytearray()
            self.state = self.WAIT_CRC if self.length == 0 else self.WAIT_PAYLOAD

        elif self.state == self.WAIT_PAYLOAD:
            self.payload.append(byte)
            if len(self.payload) >= self.length:
                self.state = self.WAIT_CRC

        elif self.state == self.WAIT_CRC:
            calc = crc8(bytes([self.type, self.length]) + bytes(self.payload))
            if calc == byte:
                self.on_frame(self.type, bytes(self.payload))
            # CRC errado -> descarta silenciosamente e resincroniza
            self.state = self.WAIT_SYNC1
