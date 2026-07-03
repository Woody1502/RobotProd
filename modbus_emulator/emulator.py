#!/usr/bin/env python3
"""
emulator.py — stand-in for the real WiFi-RS485 adapter (192.168.5.42:81).

rs485_bridge.py talks Modbus RTU over a virtual serial port that socat
tunnels to a TCP host:port. Point that host:port at this process instead of
the real adapter (see docker-compose.yml's modbus_emulator service / the
VIM_HOST, VIM_PORT env vars) to test the bridge's protocol without hardware.

This is NOT a generic Modbus simulator — it only understands the exact frame
shapes rs485_bridge.py actually sends (FC03/04/05/06, single coil/register,
always 8-byte RTU request frames), and it keeps a small state model per BLDC
address (6, 8-11: enabled / direction / speed) to flag command sequences that
don't make physical sense, e.g. reversing direction while the board still
has a nonzero held speed.
"""

import os
import signal
import socket
import struct
import sys
import threading
from datetime import datetime

EMU_HOST = '0.0.0.0'
EMU_PORT = int(os.environ.get('EMU_PORT', '81'))
EMU_LOG_FILE = os.environ.get('EMU_LOG_FILE', '/app/logs/modbus_emulator.log')

# below this the real board (per cpptest/rs485_bridge findings) just clicks
# without actually spinning the wheel
MIN_POWER = 200

_DEVICE_NAMES = {
    1: 'manipulator', 2: 'bucket', 3: 'frame', 4: 'bunker', 5: 'flaps',
    6: 'separator(BLDC)', 7: 'steering',
    8: 'motor-FL', 9: 'motor-FR', 10: 'motor-RL', 11: 'motor-RR',
}
_BLDC_ADDRS = (6, 8, 9, 10, 11)


def _ts() -> str:
    return datetime.now().strftime('%H:%M:%S.%f')[:-3]


def _name(addr: int) -> str:
    return _DEVICE_NAMES.get(addr, f'addr{addr}')


def _log(addr: int, line: str):
    print(f'[{_ts()}] addr={addr:<2} ({_name(addr):<16}) {line}', flush=True)


def _err(addr: int, line: str):
    print(f'[{_ts()}] addr={addr:<2} ({_name(addr):<16}) !!! ОШИБКА: {line}', flush=True)


def _warn(addr: int, line: str):
    print(f'[{_ts()}] addr={addr:<2} ({_name(addr):<16}) !! ПРЕДУПРЕЖДЕНИЕ: {line}', flush=True)


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _with_crc(frame: bytes) -> bytes:
    crc = _crc16(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class DeviceState:
    """Per-BLDC-address model: what the board would actually be doing."""

    def __init__(self):
        self.enabled = False
        self.direction = None   # None = unknown, False = forward, True = reverse
        self.speed = 0          # last value written to reg0 (FC06)

    def on_enable(self, addr: int, value: bool):
        self.enabled = value
        _log(addr, f'ENABLE      coil0={int(value)}')

    def on_direction(self, addr: int, value: bool):
        word = 'REVERSE' if value else 'FORWARD'
        line = f'DIRECTION   coil1={int(value)} ({word})'

        if not self.enabled:
            _err(addr, f'направление установлено ДО Enable (coil0) — двигатель не должен слушать эту команду')

        if self.direction is not None and value != self.direction and self.speed > 0:
            was = 'НАЗАД' if self.direction else 'ВПЕРЁД'
            now = 'НАЗАД' if value else 'ВПЕРЁД'
            _err(addr,
                 f'был послан сигнал {now}, когда колёса всё ещё едут {was} '
                 f'(текущая скорость={self.speed}, направление сменили без остановки speed=0)')

        _log(addr, line)
        self.direction = value

    def on_speed(self, addr: int, value: int):
        _log(addr, f'SPEED       reg0={value}')

        if value > 0 and not self.enabled:
            _err(addr, f'скорость задана ДО Enable (val={value})')

        if 0 < value < MIN_POWER:
            _warn(addr,
                  f'скорость {value} ниже порога раскрутки ({MIN_POWER}) — '
                  f'мотор будет щёлкать без вращения')

        self.speed = value


class Emulator:
    def __init__(self):
        self.states = {addr: DeviceState() for addr in _BLDC_ADDRS}

    # ── Modbus request handling ────────────────────────────────────────────

    def handle_frame(self, frame: bytes) -> bytes | None:
        if len(frame) != 8:
            print(f'[{_ts()}] !? необычная длина кадра ({len(frame)} байт), пропускаю: {frame.hex()}', flush=True)
            return None

        if _crc16(frame[:-2]) != (frame[-2] | (frame[-1] << 8)):
            print(f'[{_ts()}] !? CRC не совпадает, пропускаю кадр: {frame.hex()}', flush=True)
            return None

        addr, fc, b2, b3, b4, b5 = frame[0], frame[1], frame[2], frame[3], frame[4], frame[5]

        if fc == 0x05:   # write single coil
            coil = (b2 << 8) | b3
            value = (b4 << 8) | b5
            on = (value == 0xFF00)
            self._on_write_bit(addr, coil, on)
            return frame   # Modbus spec: write-single-coil response echoes the request

        if fc == 0x06:   # write single register
            reg = (b2 << 8) | b3
            value = (b4 << 8) | b5
            self._on_write_register(addr, reg, value)
            return frame   # write-single-register response also echoes the request

        if fc in (0x03, 0x04):   # read holding / input register
            reg = (b2 << 8) | b3
            count = (b4 << 8) | b5
            value = self._on_read_register(addr, reg, fc)
            body = struct.pack('>BBB', addr, fc, 2 * max(count, 1)) + struct.pack('>H', value)
            return _with_crc(body)

        _log(addr, f'неизвестный FC=0x{fc:02X}, эхо-ответ как есть')
        return frame

    def _on_write_bit(self, addr: int, coil: int, value: bool):
        st = self.states.get(addr)
        if st is None:
            _log(addr, f'WRITE_BIT   coil={coil} val={int(value)}  (не BLDC-адрес, без модели состояния)')
            return
        if coil == 0:
            st.on_enable(addr, value)
        elif coil == 1:
            st.on_direction(addr, value)
        else:
            _log(addr, f'WRITE_BIT   coil={coil} val={int(value)}  (неизвестный coil для BLDC)')

    def _on_write_register(self, addr: int, reg: int, value: int):
        st = self.states.get(addr)
        if st is None or reg != 0:
            _log(addr, f'WRITE_REG   reg={reg} val={value}')
            return
        st.on_speed(addr, value)

    def _on_read_register(self, addr: int, reg: int, fc: int) -> int:
        """Reads (FC03 probe, FC04 Hall poll) are telemetry, not commands —
        intentionally not logged, only writes are."""
        st = self.states.get(addr)
        if fc == 0x04 and st is not None and reg == 0:
            # simulated Hall sensor feedback: pretend rotation tracks commanded speed
            return st.speed if (st.enabled and st.speed > 0) else 0
        return 0

    # ── TCP server ──────────────────────────────────────────────────────────

    def serve(self, host: str, port: int):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(1)
        print(f'[{_ts()}] Modbus emulator listening on {host}:{port}', flush=True)

        while True:
            conn, peer = srv.accept()
            print(f'[{_ts()}] подключение от {peer}', flush=True)
            threading.Thread(target=self._serve_conn, args=(conn,), daemon=True).start()

    def _serve_conn(self, conn: socket.socket):
        with conn:
            buf = b''
            while True:
                try:
                    chunk = conn.recv(64)
                except OSError:
                    break
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= 8:
                    frame, buf = buf[:8], buf[8:]
                    resp = self.handle_frame(frame)
                    if resp is not None:
                        try:
                            conn.sendall(resp)
                        except OSError:
                            return
        print(f'[{_ts()}] соединение закрыто', flush=True)


class _Tee:
    """Mirrors writes to several streams — lets every existing print(...)
    call reach both stdout (docker logs) and the log file with no other
    code changes."""

    def __init__(self, *streams):
        self._streams = streams

    def write(self, data):
        for s in self._streams:
            s.write(data)

    def flush(self):
        for s in self._streams:
            s.flush()


def _setup_file_logging():
    """Mirror stdout into EMU_LOG_FILE so the log survives after the
    container stops (mount the containing dir as a volume in compose)."""
    os.makedirs(os.path.dirname(EMU_LOG_FILE), exist_ok=True)
    logfile = open(EMU_LOG_FILE, 'w', buffering=1)
    sys.stdout = _Tee(sys.stdout, logfile)
    print(f'[{_ts()}] === modbus_emulator session started, writing log to {EMU_LOG_FILE} ===', flush=True)
    return logfile


def _shutdown(reason: str):
    print(f'[{_ts()}] === modbus_emulator stopped ({reason}) ===', flush=True)
    sys.stdout.flush()
    sys.exit(0)


def main():
    _setup_file_logging()
    signal.signal(signal.SIGTERM, lambda *_: _shutdown('SIGTERM'))
    signal.signal(signal.SIGINT, lambda *_: _shutdown('SIGINT'))
    Emulator().serve(EMU_HOST, EMU_PORT)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        _shutdown('KeyboardInterrupt')
