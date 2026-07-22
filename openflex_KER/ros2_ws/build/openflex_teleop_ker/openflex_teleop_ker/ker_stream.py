# Copyright 2026 Enactic, Inc.
# Copyright 2026 Chengdu Changshu Robot Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""OpenArm KER self-describing USB, serial, and WiFi stream protocol.

Protocol behavior follows the Apache-2.0 openarm_ker implementation from Enactic.
"""

from queue import Empty, Queue
import socket
import struct
import threading
import time
from typing import Any


HEADER_STREAM = b'\xa5\x5a'
HEADER_PING = b'\xa5\x50'
CMD_PING = b'\x00'
CMD_STANDBY = b'\x01'
CMD_STREAM = b'\x02'

TYPE_MAP = {
    0: 'I', 1: 'H', 2: 'B', 3: 'i', 4: 'h', 5: 'f', 6: '?',
}


class KERStream:
    def __init__(self, transport: str = 'usb', port: str = '/dev/ttyACM0',
                 baud: int = 2000000, vid: int = 0x303A, pid: int = 0x4002,
                 wifi_host: str = 'openarm-ker.local', wifi_port: int = 19090,
                 connect_timeout: float = 3.0, socket_timeout: float = 0.02):
        self.transport = transport
        self.port = port
        self.baud = baud
        self.vid = vid
        self.pid = pid
        self.wifi_host = wifi_host
        self.wifi_port = wifi_port
        self.connect_timeout = connect_timeout
        self.socket_timeout = socket_timeout
        self.metadata: dict[str, str] = {}
        self._device = None
        self._ep_in = None
        self._ep_out = None
        self._serial = None
        self._socket: socket.socket | None = None
        self._buffer = bytearray()
        self._fields: list[tuple[str, int]] = []
        self._format = ''
        self._packet_size = 0
        self._queue: Queue[dict[str, Any]] = Queue(maxsize=2)
        self._running = False
        self._thread = None

    @property
    def is_connected(self) -> bool:
        return self._running

    @property
    def endpoint(self) -> str:
        if self.transport == 'usb':
            return f'USB {self.vid:#06x}:{self.pid:#06x}'
        if self.transport == 'serial':
            return f'{self.port} @ {self.baud}'
        return f'{self.wifi_host}:{self.wifi_port}'

    def connect(self) -> None:
        if self.transport == 'usb':
            self._connect_usb()
        elif self.transport == 'serial':
            self._connect_serial()
        elif self.transport == 'wifi':
            self._connect_wifi()
        else:
            raise ValueError(f'unsupported transport: {self.transport}')
        self._fetch_schema()
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _connect_usb(self) -> None:
        import usb.core
        import usb.util

        device = usb.core.find(idVendor=self.vid, idProduct=self.pid)
        if device is None:
            raise RuntimeError(f'USB device {self.vid:#06x}:{self.pid:#06x} not found')
        if device.is_kernel_driver_active(0):
            device.detach_kernel_driver(0)
        device.set_configuration()
        interface = device.get_active_configuration()[(0, 0)]
        self._ep_in = usb.util.find_descriptor(
            interface, custom_match=lambda endpoint: usb.util.endpoint_direction(
                endpoint.bEndpointAddress) == usb.util.ENDPOINT_IN)
        self._ep_out = usb.util.find_descriptor(
            interface, custom_match=lambda endpoint: usb.util.endpoint_direction(
                endpoint.bEndpointAddress) == usb.util.ENDPOINT_OUT)
        if self._ep_in is None or self._ep_out is None:
            raise RuntimeError('KER USB endpoints not found')
        self._device = device
        self.send_command(CMD_STANDBY)

    def _connect_serial(self) -> None:
        import serial

        self._serial = serial.Serial(self.port, self.baud, timeout=0.01)
        self.send_command(CMD_STANDBY)
        time.sleep(0.05)
        self._serial.reset_input_buffer()

    def _connect_wifi(self) -> None:
        self._socket = socket.create_connection(
            (self.wifi_host, self.wifi_port), timeout=self.connect_timeout)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._socket.settimeout(self.socket_timeout)
        self.send_command(CMD_STANDBY)

    def _fetch_schema(self) -> None:
        self._buffer.clear()
        deadline = time.monotonic() + 3.0
        next_ping = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_ping:
                self.send_command(CMD_PING)
                next_ping = now + 0.5
            self._buffer.extend(self._read_raw(512))
            index = self._buffer.find(HEADER_PING)
            if index >= 0:
                del self._buffer[:index]
                if self._parse_schema():
                    return
        raise TimeoutError('timed out waiting for KER schema')

    def _parse_schema(self) -> bool:
        if len(self._buffer) < 47:
            return False
        position = 2
        values = []
        for size in (16, 16, 12):
            encoded = self._buffer[position:position + size]
            values.append(encoded.decode('utf-8', 'ignore').rstrip('\0'))
            position += size
        field_count = self._buffer[position]
        position += 1
        if len(self._buffer) < position + field_count * 18:
            return False
        fields = []
        format_string = '<'
        for _ in range(field_count):
            key = self._buffer[position:position + 16].decode('utf-8', 'ignore').rstrip('\0')
            type_id = self._buffer[position + 16]
            count = self._buffer[position + 17]
            position += 18
            if type_id not in TYPE_MAP:
                raise RuntimeError(f'unsupported KER schema type {type_id}')
            fields.append((key, count))
            format_string += (str(count) if count > 1 else '') + TYPE_MAP[type_id]
        self.metadata = {'fw': values[0], 'hw': values[1], 'updated': values[2]}
        self._fields = fields
        self._format = format_string
        self._packet_size = 2 + struct.calcsize(format_string) + 1
        self._buffer.clear()
        return True

    def _read_loop(self) -> None:
        try:
            while self._running:
                packets = self._read_packets()
                for packet in packets:
                    if self._queue.full():
                        try:
                            self._queue.get_nowait()
                        except Empty:
                            pass
                    self._queue.put_nowait(packet)
                if not packets:
                    time.sleep(0.001)
        except Exception:
            self._running = False

    def _read_packets(self) -> list[dict[str, Any]]:
        self._buffer.extend(self._read_raw(4096))
        result = []
        while len(self._buffer) >= self._packet_size:
            index = self._buffer.find(HEADER_STREAM)
            if index < 0:
                self._buffer.clear()
                break
            del self._buffer[:index]
            if len(self._buffer) < self._packet_size:
                break
            packet = bytes(self._buffer[:self._packet_size])
            del self._buffer[:self._packet_size]
            checksum = 0
            for byte in packet[2:-1]:
                checksum ^= byte
            if checksum != packet[-1]:
                continue
            unpacked = struct.unpack(self._format, packet[2:-1])
            data = {}
            value_index = 0
            for key, count in self._fields:
                data[key] = (unpacked[value_index] if count == 1
                             else list(unpacked[value_index:value_index + count]))
                value_index += count
            result.append(data)
        return result

    def _read_raw(self, size: int) -> bytes:
        if self.transport == 'usb':
            import usb.core
            try:
                return bytes(self._device.read(self._ep_in.bEndpointAddress, size, timeout=20))
            except usb.core.USBError as error:
                if error.errno in (110, 116) or 'timeout' in str(error).lower():
                    return b''
                raise
        if self.transport == 'serial':
            waiting = self._serial.in_waiting
            return self._serial.read(min(waiting, size)) if waiting else b''
        if self._socket is None:
            return b''
        try:
            data = self._socket.recv(size)
            if not data:
                raise ConnectionError('KER WiFi peer closed the connection')
            return data
        except socket.timeout:
            return b''

    def recv(self) -> dict[str, Any] | None:
        try:
            return self._queue.get_nowait()
        except Empty:
            return None

    def send_command(self, command: bytes) -> None:
        if self.transport == 'usb':
            self._device.write(self._ep_out.bEndpointAddress, command)
        elif self.transport == 'serial':
            self._serial.write(command)
        else:
            if self._socket is None:
                raise RuntimeError('WiFi socket not connected')
            self._socket.sendall(self.encode_wifi_command(command))

    @staticmethod
    def encode_wifi_command(command: bytes) -> bytes:
        if not command or len(command) > 3:
            raise ValueError('KER WiFi command payload must contain 1 to 3 bytes')
        payload = command + bytes(3 - len(command))
        checksum = payload[0] ^ payload[1] ^ payload[2]
        return b'\xa5\x43' + payload + bytes([checksum])

    def close(self) -> None:
        if self._running:
            try:
                self.send_command(CMD_STANDBY)
            except Exception:
                pass
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._serial:
            self._serial.close()
            self._serial = None
        if self._socket:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._socket.close()
            self._socket = None
        if self._device:
            import usb.util
            usb.util.dispose_resources(self._device)
        self._device = None
