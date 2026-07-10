#!/usr/bin/env python3
"""Capture read-only AresPlot monitor data to timestamped CSV."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

SOP = 0xA5
EOP = 0x5A
CMD_START_MONITOR = 0x01
CMD_SET_SAMPLE_RATE = 0x03
CMD_MONITOR_DATA = 0x81
CMD_ACK = 0x82
MAX_PROTOCOL_FRAME = 512

TYPE_CODES = {
    "int8": 0x00,
    "uint8": 0x01,
    "int16": 0x02,
    "uint16": 0x03,
    "int32": 0x04,
    "uint32": 0x05,
    "float32": 0x06,
    "float64": 0x07,
    "bool": 0x08,
}
TYPE_SIZES = {
    "int8": 1,
    "uint8": 1,
    "int16": 2,
    "uint16": 2,
    "int32": 4,
    "uint32": 4,
    "float32": 4,
    "float64": 8,
    "bool": 1,
}
ACK_STATUS = {
    0x00: "OK",
    0x01: "ERROR_CHECKSUM",
    0x02: "ERROR_UNKNOWN_CMD",
    0x03: "ERROR_INVALID_PAYLOAD",
    0x04: "ERROR_ADDR_INVALID",
    0x05: "ERROR_TYPE_UNSUPPORTED",
    0x06: "ERROR_RATE_UNACHIEVABLE",
    0x07: "ERROR_MCU_BUSY_OR_LIMIT",
    0xFF: "ERROR_GENERAL_FAIL",
}
DM_MC02_SAFE_RANGES = (
    (0x20000000, 0x2001FFFF),
    (0x24000000, 0x2404FFFF),
    (0x30000000, 0x30007FFF),
    (0x38000000, 0x38003FFF),
)


class CaptureError(RuntimeError):
    """Report an actionable capture setup or protocol error."""


@dataclass(frozen=True)
class Variable:
    name: str
    address: int
    type_name: str


@dataclass(frozen=True)
class CaptureConfig:
    serial_port: str
    baud_rate: int
    sample_rate_hz: int
    duration_seconds: float
    output_csv: Path
    variables: tuple[Variable, ...]
    ack_timeout_seconds: float
    max_frame_bytes: int


@dataclass(frozen=True)
class Frame:
    command: int
    payload: bytes


class FrameParser:
    """Incrementally recover valid AresPlot frames from a noisy byte stream."""

    def __init__(self, max_frame_bytes: int = 256) -> None:
        if not 6 <= max_frame_bytes <= MAX_PROTOCOL_FRAME:
            raise ValueError("max_frame_bytes must be within 6..512")
        self.max_frame_bytes = max_frame_bytes
        self.buffer = bytearray()
        self.bad_checksum = 0
        self.bad_eop = 0
        self.bad_length = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        frames: list[Frame] = []
        while True:
            start = self.buffer.find(bytes((SOP,)))
            if start < 0:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break
            if start:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < 4:
                break

            command = self.buffer[1]
            payload_len = self.buffer[2] | (self.buffer[3] << 8)
            frame_len = 6 + payload_len
            if frame_len > self.max_frame_bytes:
                self.bad_length += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < frame_len:
                break

            payload_end = 4 + payload_len
            payload = bytes(self.buffer[4:payload_end])
            received_checksum = self.buffer[payload_end]
            received_eop = self.buffer[payload_end + 1]
            expected_checksum = checksum(command, payload_len, payload)
            if received_eop != EOP:
                self.bad_eop += 1
                del self.buffer[0]
                continue
            if received_checksum != expected_checksum:
                self.bad_checksum += 1
                del self.buffer[0]
                continue
            frames.append(Frame(command, payload))
            del self.buffer[:frame_len]
        return frames


def checksum(command: int, payload_len: int, payload: bytes) -> int:
    value = command ^ (payload_len & 0xFF) ^ ((payload_len >> 8) & 0xFF)
    for byte in payload:
        value ^= byte
    return value


def make_frame(command: int, payload: bytes) -> bytes:
    if len(payload) > 0xFFFF:
        raise CaptureError("AresPlot payload exceeds the uint16 protocol length")
    length = len(payload)
    return bytes((SOP, command)) + struct.pack("<H", length) + payload + bytes(
        (checksum(command, length, payload), EOP)
    )


def _parse_int(value: Any, field: str) -> int:
    if isinstance(value, bool):
        raise CaptureError(f"{field} must be an integer, not a boolean")
    if isinstance(value, int):
        return value
    if not isinstance(value, str):
        raise CaptureError(f"{field} must be an integer or 0x-prefixed integer string")
    try:
        parsed = int(value, 16 if value.lower().startswith("0x") else 10)
    except ValueError as exc:
        raise CaptureError(f"{field} must be an integer or 0x-prefixed integer string") from exc
    return parsed


def _inside_ranges(address: int, size: int, ranges: Iterable[tuple[int, int]]) -> bool:
    end = address + size - 1
    return any(start <= address and end <= stop for start, stop in ranges)


def _parse_narrow_ranges(value: Any) -> tuple[tuple[int, int], ...]:
    if value is None:
        return DM_MC02_SAFE_RANGES
    if not isinstance(value, list) or not value:
        raise CaptureError("address_ranges must be a non-empty list when provided")
    parsed: list[tuple[int, int]] = []
    for index, item in enumerate(value):
        if not isinstance(item, dict):
            raise CaptureError(f"address_ranges[{index}] must be an object")
        start = _parse_int(item.get("start"), f"address_ranges[{index}].start")
        stop = _parse_int(item.get("end"), f"address_ranges[{index}].end")
        if start > stop or not _inside_ranges(start, stop - start + 1, DM_MC02_SAFE_RANGES):
            raise CaptureError(
                f"address_ranges[{index}] must narrow a known DM-MC02 RAM range, not expand it"
            )
        parsed.append((start, stop))
    return tuple(parsed)


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise CaptureError(f"config file not found: {path}") from exc
    except UnicodeDecodeError as exc:
        raise CaptureError(f"config is not UTF-8 JSON: {path}") from exc
    except json.JSONDecodeError as exc:
        raise CaptureError(f"bad JSON in {path}: line {exc.lineno}, column {exc.colno}: {exc.msg}") from exc
    if not isinstance(value, dict):
        raise CaptureError("config root must be a JSON object")
    return value


def load_config(path: Path, args: argparse.Namespace) -> CaptureConfig:
    raw = _load_json(path)
    if args.port is not None:
        raw["serial_port"] = args.port
    if args.baud is not None:
        raw["baud_rate"] = args.baud
    if args.duration is not None:
        raw["duration_seconds"] = args.duration
    if args.output is not None:
        raw["output_csv"] = args.output
    if args.sample_rate_hz is not None:
        raw.pop("sample_period_ms", None)
        raw["sample_rate_hz"] = args.sample_rate_hz
    if args.sample_period_ms is not None:
        raw.pop("sample_rate_hz", None)
        raw["sample_period_ms"] = args.sample_period_ms

    required = ("serial_port", "baud_rate", "duration_seconds", "output_csv", "variables")
    missing = [field for field in required if field not in raw]
    if missing:
        raise CaptureError("missing required config field(s): " + ", ".join(missing))
    has_rate = "sample_rate_hz" in raw
    has_period = "sample_period_ms" in raw
    if has_rate == has_period:
        raise CaptureError("provide exactly one of sample_rate_hz or sample_period_ms")
    if has_period:
        period_ms = _parse_int(raw["sample_period_ms"], "sample_period_ms")
        if period_ms <= 0:
            raise CaptureError("sample_period_ms must be greater than zero")
        sample_rate_hz = max(1, 1000 // period_ms)
    else:
        sample_rate_hz = _parse_int(raw["sample_rate_hz"], "sample_rate_hz")
    if not 1 <= sample_rate_hz <= 1000:
        raise CaptureError("sample_rate_hz must be within 1..1000 for the current firmware timer")

    port = raw["serial_port"]
    if not isinstance(port, str) or not port.strip():
        raise CaptureError("serial_port must be a non-empty string")
    baud_rate = _parse_int(raw["baud_rate"], "baud_rate")
    if baud_rate <= 0:
        raise CaptureError("baud_rate must be greater than zero")
    try:
        duration_seconds = float(raw["duration_seconds"])
    except (TypeError, ValueError) as exc:
        raise CaptureError("duration_seconds must be a number") from exc
    if not math.isfinite(duration_seconds) or duration_seconds <= 0:
        raise CaptureError("duration_seconds must be a finite number greater than zero")
    try:
        ack_timeout = float(raw.get("ack_timeout_seconds", 2))
    except (TypeError, ValueError) as exc:
        raise CaptureError("ack_timeout_seconds must be a number") from exc
    if not math.isfinite(ack_timeout) or ack_timeout <= 0:
        raise CaptureError("ack_timeout_seconds must be a finite number greater than zero")
    max_frame_bytes = _parse_int(raw.get("max_frame_bytes", 256), "max_frame_bytes")
    if not 64 <= max_frame_bytes <= MAX_PROTOCOL_FRAME:
        raise CaptureError("max_frame_bytes must be within 64..512")
    max_variables = _parse_int(raw.get("max_variables", 10), "max_variables")
    if not 1 <= max_variables <= 50:
        raise CaptureError("max_variables must be within 1..50")
    ranges = _parse_narrow_ranges(raw.get("address_ranges"))
    output_csv = raw["output_csv"]
    if not isinstance(output_csv, str) or not output_csv.strip():
        raise CaptureError("output_csv must be a non-empty string")

    source_variables = raw["variables"]
    if not isinstance(source_variables, list) or not source_variables:
        raise CaptureError("variables must be a non-empty list")
    if len(source_variables) > max_variables:
        raise CaptureError(
            f"variables has {len(source_variables)} entries but max_variables is {max_variables}"
        )
    variables: list[Variable] = []
    names: set[str] = set()
    addresses: set[int] = set()
    for index, item in enumerate(source_variables):
        if not isinstance(item, dict):
            raise CaptureError(f"variables[{index}] must be an object")
        name = item.get("name")
        if not isinstance(name, str) or not name or name in names:
            raise CaptureError(f"variables[{index}].name must be a unique non-empty string")
        type_name = item.get("type")
        if type_name == "float64":
            raise CaptureError(
                "float64 is not capture-compatible: current firmware sends 0.0 because it has no FLOAT64 sender case"
            )
        if type_name not in TYPE_CODES:
            allowed = ", ".join(name for name in TYPE_CODES if name != "float64")
            raise CaptureError(f"variables[{index}].type must be one of: {allowed}")
        address = _parse_int(item.get("address"), f"variables[{index}].address")
        if not 0 <= address <= 0xFFFFFFFF:
            raise CaptureError(f"variables[{index}].address is outside uint32 range")
        if address in addresses:
            raise CaptureError(f"variables[{index}].address duplicates 0x{address:08x}")
        if address % TYPE_SIZES[type_name] != 0:
            raise CaptureError(
                f"variables[{index}].address 0x{address:08x} is not aligned for {type_name}"
            )
        if not _inside_ranges(address, TYPE_SIZES[type_name], ranges):
            raise CaptureError(
                f"variables[{index}].address 0x{address:08x} is unsafe or outside the permitted RAM ranges"
            )
        names.add(name)
        addresses.add(address)
        variables.append(Variable(name, address, type_name))

    start_frame_size = 7 + 5 * len(variables)
    data_frame_size = 10 + 4 * len(variables)
    if start_frame_size > max_frame_bytes or data_frame_size > max_frame_bytes:
        raise CaptureError(
            f"variables exceed max_frame_bytes={max_frame_bytes}: start={start_frame_size}, data={data_frame_size}"
        )
    return CaptureConfig(
        serial_port=port,
        baud_rate=baud_rate,
        sample_rate_hz=sample_rate_hz,
        duration_seconds=duration_seconds,
        output_csv=Path(output_csv),
        variables=tuple(variables),
        ack_timeout_seconds=ack_timeout,
        max_frame_bytes=max_frame_bytes,
    )


def start_monitor_payload(variables: Iterable[Variable]) -> bytes:
    selected = tuple(variables)
    if len(selected) > 255:
        raise CaptureError("START_MONITOR supports at most 255 variables")
    payload = bytearray((len(selected),))
    for variable in selected:
        payload.extend(struct.pack("<I", variable.address))
        payload.append(TYPE_CODES[variable.type_name])
    return bytes(payload)


def _open_serial(config: CaptureConfig):
    try:
        import serial
    except ImportError as exc:
        raise CaptureError("pyserial is missing; install it in this Python environment: python3 -m pip install pyserial") from exc
    try:
        return serial.Serial(config.serial_port, config.baud_rate, timeout=0.1, write_timeout=2)
    except PermissionError as exc:
        raise CaptureError(f"serial permission denied for {config.serial_port}; check group/udev or OS driver access") from exc
    except serial.SerialException as exc:
        message = str(exc)
        if "Permission" in message or "permission" in message:
            raise CaptureError(f"serial permission denied for {config.serial_port}; check group/udev or OS driver access") from exc
        raise CaptureError(f"cannot open serial port {config.serial_port}: {message}") from exc


def _write_frame(serial_port: Any, frame: bytes) -> None:
    try:
        written = serial_port.write(frame)
        if written != len(frame):
            raise CaptureError(f"serial short write: sent {written!r} of {len(frame)} bytes")
        serial_port.flush()
    except CaptureError:
        raise
    except Exception as exc:
        raise CaptureError(f"serial write failed: {exc}") from exc


def _wait_for_ack(serial_port: Any, parser: FrameParser, command: int, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            data = serial_port.read(256)
        except Exception as exc:
            raise CaptureError(f"serial read failed while waiting for ACK: {exc}") from exc
        for frame in parser.feed(data):
            if frame.command != CMD_ACK:
                continue
            if len(frame.payload) != 2:
                continue
            ack_command, status = frame.payload
            if ack_command != command:
                continue
            if status != 0:
                raise CaptureError(
                    f"device rejected command 0x{command:02x}: {ACK_STATUS.get(status, f'0x{status:02x}') }"
                )
            return
    raise CaptureError(f"timeout waiting {timeout:g}s for ACK to command 0x{command:02x}")


def _monitor_values(frame: Frame, variable_count: int) -> tuple[int, tuple[float, ...]] | None:
    if frame.command != CMD_MONITOR_DATA:
        return None
    expected = 4 + 4 * variable_count
    if len(frame.payload) != expected:
        return None
    unpacked = struct.unpack("<I" + "f" * variable_count, frame.payload)
    return unpacked[0], unpacked[1:]


def capture(config: CaptureConfig) -> tuple[int, int, FrameParser]:
    parser = FrameParser(config.max_frame_bytes)
    output = config.output_csv
    output.parent.mkdir(parents=True, exist_ok=True)
    samples = 0
    timestamp_gaps = 0
    previous_timestamp: int | None = None
    expected_period_ms = max(1, 1000 // config.sample_rate_hz)
    monitoring_started = False
    serial_port = _open_serial(config)
    try:
        try:
            serial_port.reset_input_buffer()
        except Exception:
            pass
        _write_frame(serial_port, make_frame(CMD_SET_SAMPLE_RATE, struct.pack("<I", config.sample_rate_hz)))
        _wait_for_ack(serial_port, parser, CMD_SET_SAMPLE_RATE, config.ack_timeout_seconds)
        _write_frame(serial_port, make_frame(CMD_START_MONITOR, start_monitor_payload(config.variables)))
        # Once a complete start frame is sent, attempt STOP even if its ACK is lost.
        monitoring_started = True
        _wait_for_ack(serial_port, parser, CMD_START_MONITOR, config.ack_timeout_seconds)
        deadline = time.monotonic() + config.duration_seconds
        with output.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["timestamp_ms", "host_time_utc", *[item.name for item in config.variables]])
            while time.monotonic() < deadline:
                try:
                    data = serial_port.read(512)
                except Exception as exc:
                    raise CaptureError(f"serial read failed during capture: {exc}") from exc
                for frame in parser.feed(data):
                    sample = _monitor_values(frame, len(config.variables))
                    if sample is None:
                        continue
                    timestamp_ms, values = sample
                    if previous_timestamp is not None:
                        delta = (timestamp_ms - previous_timestamp) & 0xFFFFFFFF
                        if delta > expected_period_ms * 2:
                            timestamp_gaps += 1
                    previous_timestamp = timestamp_ms
                    writer.writerow(
                        [
                            timestamp_ms,
                            dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds"),
                            *values,
                        ]
                    )
                    samples += 1
    finally:
        if monitoring_started:
            try:
                _write_frame(serial_port, make_frame(CMD_START_MONITOR, b"\x00"))
            except CaptureError as exc:
                print(f"warning: failed to stop monitoring cleanly: {exc}", file=sys.stderr)
        try:
            serial_port.close()
        except Exception as exc:
            print(f"warning: failed to close serial port cleanly: {exc}", file=sys.stderr)
    return samples, timestamp_gaps, parser


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path, help="capture JSON config")
    parser.add_argument("--trusted-debug-target", action="store_true", help="confirm trusted target and reviewed addresses")
    parser.add_argument("--port", help="override serial_port")
    parser.add_argument("--baud", type=int, help="override baud_rate")
    rate = parser.add_mutually_exclusive_group()
    rate.add_argument("--sample-rate-hz", type=int, help="override sample rate; encoded as protocol uint32 rate_hz")
    rate.add_argument("--sample-period-ms", type=int, help="override period; converted to whole rate_hz before transmission")
    parser.add_argument("--duration", type=float, help="override duration_seconds")
    parser.add_argument("--output", help="override output_csv")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.trusted_debug_target:
        print("error: refuse to send raw addresses without --trusted-debug-target", file=sys.stderr)
        return 2
    try:
        config = load_config(args.config, args)
        samples, timestamp_gaps, parser = capture(config)
    except CaptureError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("capture interrupted; stop command was attempted", file=sys.stderr)
        return 130
    print(
        f"captured {samples} samples to {config.output_csv}; timestamp_gaps={timestamp_gaps}, "
        f"discarded={parser.discarded_bytes}, "
        f"bad_checksum={parser.bad_checksum}, bad_eop={parser.bad_eop}, bad_length={parser.bad_length}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
