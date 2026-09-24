"""Testes do protocolo serial (sem hardware)."""

import struct

from pantilt_hardware.serial_protocol import (
    MSG_CMD_VEL, MSG_HEARTBEAT, MSG_TELEMETRY, SYNC1, SYNC2, FrameParser, build_frame, crc8,
)


def parse_all(data: bytes):
    frames = []
    parser = FrameParser(lambda t, p: frames.append((t, p)))
    for b in data:
        parser.feed(b)
    return frames


def test_crc8_valor_de_referencia():
    # CRC-8 (polinômio 0x07, init 0x00): valor de verificação padrão para "123456789"
    assert crc8(b'123456789') == 0xF4


def test_formato_do_frame():
    payload = struct.pack('<ff', 1.5, -2.0)
    frame = build_frame(MSG_CMD_VEL, payload)
    assert frame[:4] == bytes([SYNC1, SYNC2, MSG_CMD_VEL, len(payload)])
    assert frame[4:-1] == payload
    assert frame[-1] == crc8(bytes([MSG_CMD_VEL, len(payload)]) + payload)


def test_ida_e_volta():
    payload = struct.pack('<ffff', 0.1, -0.2, 0.3, -0.4)
    assert parse_all(build_frame(MSG_TELEMETRY, payload)) == [(MSG_TELEMETRY, payload)]


def test_payload_vazio():
    assert parse_all(build_frame(MSG_HEARTBEAT, b'')) == [(MSG_HEARTBEAT, b'')]


def test_lixo_antes_do_frame():
    frame = build_frame(MSG_HEARTBEAT, b'')
    assert parse_all(b'\x00\xA5\x13\xFF' + frame) == [(MSG_HEARTBEAT, b'')]


def test_crc_errado_descarta_e_resincroniza():
    bom = build_frame(MSG_CMD_VEL, struct.pack('<ff', 1.0, 2.0))
    ruim = bytearray(bom)
    ruim[-1] ^= 0xFF
    frames = parse_all(bytes(ruim) + bom)
    assert frames == [(MSG_CMD_VEL, struct.pack('<ff', 1.0, 2.0))]
