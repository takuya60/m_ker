#!/usr/bin/env python3
"""Connect to the KER M5 WiFi transport and validate streamed packets."""

import argparse
from dataclasses import dataclass
import socket
import struct
import sys
import time


HEADER_COMMAND = b'\xA5\x43'
HEADER_PING = b'\xA5\x50'
HEADER_STREAM = b'\xA5\x5A'

CMD_PING = 0x00
CMD_STANDBY = 0x01
CMD_STREAM = 0x02

TYPE_MAP = {
    0: 'I',
    1: 'H',
    2: 'B',
    3: 'i',
    4: 'h',
    5: 'f',
    6: '?',
}


@dataclass(frozen=True)
class Field:
    name: str
    count: int


@dataclass(frozen=True)
class Schema:
    firmware: str
    hardware: str
    updated: str
    fields: tuple[Field, ...]
    payload_struct: struct.Struct

    @property
    def packet_size(self) -> int:
        return 2 + self.payload_struct.size + 1


def encode_command(command: int, argument_high: int = 0,
                   argument_low: int = 0) -> bytes:
    payload = bytes((command & 0xFF, argument_high & 0xFF, argument_low & 0xFF))
    checksum = payload[0] ^ payload[1] ^ payload[2]
    return HEADER_COMMAND + payload + bytes((checksum,))


def decode_text(value: bytes) -> str:
    return value.decode('utf-8', 'replace').rstrip('\0')


def receive_schema(sock: socket.socket, timeout: float) -> tuple[Schema, bytearray]:
    deadline = time.monotonic() + timeout
    next_ping = 0.0
    buffer = bytearray()

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_ping:
            sock.sendall(encode_command(CMD_PING))
            next_ping = now + 0.5

        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            raise ConnectionError('M5 closed the connection while waiting for schema')
        buffer.extend(chunk)

        header_index = buffer.find(HEADER_PING)
        if header_index < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            continue
        if header_index:
            del buffer[:header_index]
        if len(buffer) < 47:
            continue

        position = 2
        firmware = decode_text(buffer[position:position + 16])
        position += 16
        hardware = decode_text(buffer[position:position + 16])
        position += 16
        updated = decode_text(buffer[position:position + 12])
        position += 12
        field_count = buffer[position]
        position += 1

        if field_count > 32:
            raise ValueError(f'invalid schema field count: {field_count}')
        schema_size = position + field_count * 18
        if len(buffer) < schema_size:
            continue

        fields = []
        format_string = '<'
        for _ in range(field_count):
            name = decode_text(buffer[position:position + 16])
            type_id = buffer[position + 16]
            count = buffer[position + 17]
            position += 18
            if not name:
                raise ValueError('schema contains an empty field name')
            if type_id not in TYPE_MAP:
                raise ValueError(f'unsupported schema type {type_id} for {name}')
            if count == 0:
                raise ValueError(f'schema field {name} has zero elements')
            format_string += (str(count) if count > 1 else '') + TYPE_MAP[type_id]
            fields.append(Field(name=name, count=count))

        schema = Schema(
            firmware=firmware,
            hardware=hardware,
            updated=updated,
            fields=tuple(fields),
            payload_struct=struct.Struct(format_string),
        )
        return schema, buffer[schema_size:]

    raise TimeoutError('timed out waiting for KER WiFi schema')


def decode_payload(schema: Schema, payload: bytes) -> dict[str, object]:
    values = schema.payload_struct.unpack(payload)
    result = {}
    value_index = 0
    for field in schema.fields:
        if field.count == 1:
            result[field.name] = values[value_index]
        else:
            result[field.name] = list(
                values[value_index:value_index + field.count])
        value_index += field.count
    return result


def extract_packets(buffer: bytearray, schema: Schema) -> tuple[list[dict[str, object]], int]:
    packets = []
    checksum_errors = 0
    packet_size = schema.packet_size

    while len(buffer) >= packet_size:
        header_index = buffer.find(HEADER_STREAM)
        if header_index < 0:
            if len(buffer) > 1:
                del buffer[:-1]
            break
        if header_index:
            del buffer[:header_index]
        if len(buffer) < packet_size:
            break

        packet = bytes(buffer[:packet_size])
        del buffer[:packet_size]
        checksum = 0
        for value in packet[2:-1]:
            checksum ^= value
        if checksum != packet[-1]:
            checksum_errors += 1
            continue
        packets.append(decode_payload(schema, packet[2:-1]))

    return packets, checksum_errors


def format_fields(fields: tuple[Field, ...]) -> str:
    return ', '.join(f'{field.name}[{field.count}]' for field in fields)


def error_mask(errors: object) -> int:
    if not isinstance(errors, list):
        return 0
    return sum((1 << index) for index, error in enumerate(errors) if bool(error))


def format_angles(angles: object) -> str:
    if not isinstance(angles, list):
        return 'unavailable'
    return '[' + ', '.join(f'{float(value):+.2f}' for value in angles) + ']'


def run(args: argparse.Namespace) -> int:
    print(f'[INFO] Connecting to {args.host}:{args.port} ...')
    sock = socket.create_connection(
        (args.host, args.port), timeout=args.connect_timeout)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.settimeout(args.socket_timeout)

    valid_packets = 0
    checksum_errors = 0
    connected_at = time.monotonic()

    try:
        print(f'[OK] Connected to {args.host}:{args.port}')
        sock.sendall(encode_command(CMD_STANDBY))
        schema, buffer = receive_schema(sock, args.schema_timeout)
        print(
            f'[OK] FW={schema.firmware} HW={schema.hardware} '
            f'updated={schema.updated}')
        print(f'[OK] Fields: {format_fields(schema.fields)}')
        print(f'[OK] Stream packet size: {schema.packet_size} bytes')

        if args.expect_fw_prefix and not schema.firmware.startswith(args.expect_fw_prefix):
            raise RuntimeError(
                f'firmware {schema.firmware!r} does not start with '
                f'{args.expect_fw_prefix!r}')

        sock.sendall(encode_command(CMD_STREAM))
        stream_started = time.monotonic()
        last_packet_time = stream_started
        rate_started = stream_started
        rate_packets = 0
        next_print = stream_started

        while args.duration <= 0 or time.monotonic() - stream_started < args.duration:
            received_data = False
            try:
                chunk = sock.recv(4096)
                received_data = True
            except socket.timeout:
                chunk = b''
            if received_data and not chunk:
                raise ConnectionError('M5 closed the stream connection')
            if chunk:
                buffer.extend(chunk)

            packets, new_checksum_errors = extract_packets(buffer, schema)
            checksum_errors += new_checksum_errors
            now = time.monotonic()
            for packet in packets:
                valid_packets += 1
                rate_packets += 1
                last_packet_time = now
                if now >= next_print:
                    rate_elapsed = max(now - rate_started, 1e-6)
                    print(
                        f'[DATA] hz={rate_packets / rate_elapsed:.1f} '
                        f'error_mask=0x{error_mask(packet.get("errors")):04X} '
                        f'angles_deg={format_angles(packet.get("angles"))}')
                    next_print = now + 1.0 / args.print_rate

            if now - last_packet_time > args.data_timeout:
                raise TimeoutError(
                    f'no valid stream packet received for {args.data_timeout:.1f} seconds')
            if now - rate_started >= 1.0:
                rate_started = now
                rate_packets = 0

        elapsed = max(time.monotonic() - stream_started, 1e-6)
        print(
            f'[OK] Completed: packets={valid_packets}, '
            f'average_hz={valid_packets / elapsed:.1f}, '
            f'checksum_errors={checksum_errors}')
        return 0 if valid_packets > 0 and checksum_errors == 0 else 2
    finally:
        try:
            sock.sendall(encode_command(CMD_STANDBY))
        except OSError:
            pass
        sock.close()
        print(f'[INFO] Connection closed after {time.monotonic() - connected_at:.1f}s')


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Validate KER M5 WiFi PING/schema and streamed encoder data.')
    parser.add_argument('host', nargs='?', default='openarm-ker.local')
    parser.add_argument('--port', type=int, default=19090)
    parser.add_argument('--duration', type=float, default=0.0,
                        help='test duration in seconds; 0 runs until Ctrl+C')
    parser.add_argument('--print-rate', type=float, default=2.0,
                        help='maximum data print frequency in Hz')
    parser.add_argument('--connect-timeout', type=float, default=3.0)
    parser.add_argument('--socket-timeout', type=float, default=0.2)
    parser.add_argument('--schema-timeout', type=float, default=5.0)
    parser.add_argument('--data-timeout', type=float, default=3.0)
    parser.add_argument('--expect-fw-prefix', default='')
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    if args.duration < 0:
        parser.error('--duration cannot be negative')
    if args.print_rate <= 0:
        parser.error('--print-rate must be greater than zero')
    for name in ('connect_timeout', 'socket_timeout', 'schema_timeout', 'data_timeout'):
        if getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")} must be greater than zero')
    return args


def main() -> int:
    try:
        return run(parse_args())
    except KeyboardInterrupt:
        print('\n[INFO] Interrupted by user')
        return 130
    except (ConnectionError, OSError, RuntimeError, TimeoutError, ValueError) as error:
        print(f'[ERROR] {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
