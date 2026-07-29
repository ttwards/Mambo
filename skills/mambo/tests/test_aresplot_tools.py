#!/usr/bin/env python3
"""Hardware-free tests for AresPlot stream parsing, config validation, and PNG rendering."""

from __future__ import annotations

import csv
import importlib.util
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


capture = load_module(
    "mambo_aresplot_capture", ROOT / "scripts" / "aresplot_capture.py"
)
renderer = load_module(
    "mambo_aresplot_renderer", ROOT / "scripts" / "render_aresplot_csv.py"
)


class FrameParserTests(unittest.TestCase):
    def test_fragmented_noisy_stream_recovers_monitor_frame(self) -> None:
        payload = struct.pack("<Iff", 1234, 1.5, -2.0)
        frame = capture.make_frame(capture.CMD_MONITOR_DATA, payload)
        parser = capture.FrameParser()
        received = []
        for chunk in (b"\x00junk\xa5\x81", frame[:3], frame[3:7], frame[7:] + b"tail"):
            received.extend(parser.feed(chunk))
        self.assertEqual(received, [capture.Frame(capture.CMD_MONITOR_DATA, payload)])
        self.assertGreaterEqual(parser.discarded_bytes, 4)

    def test_bad_checksum_and_eop_do_not_hide_later_valid_frame(self) -> None:
        valid = capture.make_frame(
            capture.CMD_ACK, bytes((capture.CMD_START_MONITOR, 0))
        )
        bad_checksum = bytearray(valid)
        bad_checksum[-2] ^= 0x01
        bad_eop = bytearray(valid)
        bad_eop[-1] = 0x00
        parser = capture.FrameParser()
        received = parser.feed(bytes(bad_checksum) + bytes(bad_eop) + valid)
        self.assertEqual(
            received,
            [capture.Frame(capture.CMD_ACK, bytes((capture.CMD_START_MONITOR, 0)))],
        )
        self.assertEqual(parser.bad_checksum, 1)
        self.assertEqual(parser.bad_eop, 1)

    def test_start_payload_and_rate_wire_encoding(self) -> None:
        variable = capture.Variable("x", 0x24000100, "uint32")
        self.assertEqual(
            capture.start_monitor_payload([variable]), b"\x01\x00\x01\x00\x24\x05"
        )
        frame = capture.make_frame(capture.CMD_SET_SAMPLE_RATE, struct.pack("<I", 100))
        parsed = capture.FrameParser().feed(frame)
        self.assertEqual(parsed[0].payload, b"d\x00\x00\x00")


class ConfigTests(unittest.TestCase):
    @staticmethod
    def args() -> SimpleNamespace:
        return SimpleNamespace(
            port=None,
            baud=None,
            duration=None,
            output=None,
            sample_rate_hz=None,
            sample_period_ms=None,
        )

    def write_config(self, directory: Path, variable_type: str = "float32") -> Path:
        path = directory / "capture.json"
        path.write_text(
            """{
  "serial_port": "/dev/null",
  "baud_rate": 921600,
  "sample_period_ms": 10,
  "duration_seconds": 1,
  "output_csv": "capture.csv",
  "variables": [{"name": "x", "address": "0x24000100", "type": "%s"}]
}"""
            % variable_type,
            encoding="utf-8",
        )
        return path

    def test_period_converts_to_wire_rate(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = capture.load_config(
                self.write_config(Path(directory)), self.args()
            )
        self.assertEqual(config.sample_rate_hz, 100)

    def test_float64_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(
                capture.CaptureError, "float64 is not capture-compatible"
            ):
                capture.load_config(
                    self.write_config(Path(directory), "float64"), self.args()
                )

    def test_nonfinite_duration_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_config(Path(directory))
            path.write_text(
                path.read_text(encoding="utf-8").replace(
                    '"duration_seconds": 1', '"duration_seconds": NaN'
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(capture.CaptureError, "finite number"):
                capture.load_config(path, self.args())

    def test_unaligned_typed_address_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_config(Path(directory))
            path.write_text(
                path.read_text(encoding="utf-8").replace("0x24000100", "0x24000101"),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(capture.CaptureError, "not aligned"):
                capture.load_config(path, self.args())

    def test_fractional_integer_field_is_rejected(self) -> None:
        with self.assertRaisesRegex(capture.CaptureError, "must be an integer"):
            capture._parse_int(1.5, "baud_rate")


class CaptureTests(unittest.TestCase):
    class FakeSerial:
        def __init__(self, acknowledge_start: bool = True) -> None:
            self.acknowledge_start = acknowledge_start
            self.commands = []
            self.pending = bytearray()
            self.closed = False

        def reset_input_buffer(self) -> None:
            self.pending.clear()

        def write(self, data: bytes) -> int:
            frames = capture.FrameParser().feed(data)
            if len(frames) != 1:
                raise AssertionError("capture wrote an invalid or incomplete frame")
            frame = frames[0]
            self.commands.append(frame)
            is_stop = (
                frame.command == capture.CMD_START_MONITOR and frame.payload == b"\x00"
            )
            if frame.command == capture.CMD_SET_SAMPLE_RATE or (
                frame.command == capture.CMD_START_MONITOR
                and self.acknowledge_start
                and not is_stop
            ):
                self.pending.extend(
                    capture.make_frame(capture.CMD_ACK, bytes((frame.command, 0)))
                )
            return len(data)

        def flush(self) -> None:
            pass

        def read(self, size: int) -> bytes:
            data = bytes(self.pending[:size])
            del self.pending[:size]
            return data

        def close(self) -> None:
            self.closed = True

    @staticmethod
    def config(output: Path, ack_timeout: float = 0.01) -> object:
        return capture.CaptureConfig(
            serial_port="fake",
            baud_rate=921600,
            sample_rate_hz=100,
            duration_seconds=0.001,
            output_csv=output,
            variables=(capture.Variable("x", 0x24000100, "float32"),),
            ack_timeout_seconds=ack_timeout,
            max_frame_bytes=256,
        )

    def test_rate_precedes_start_and_capture_always_stops(self) -> None:
        fake = self.FakeSerial()
        original = capture._open_serial
        capture._open_serial = lambda _config: fake
        try:
            with tempfile.TemporaryDirectory() as directory:
                capture.capture(self.config(Path(directory) / "capture.csv"))
        finally:
            capture._open_serial = original
        self.assertEqual(
            [(frame.command, frame.payload) for frame in fake.commands],
            [
                (capture.CMD_SET_SAMPLE_RATE, struct.pack("<I", 100)),
                (
                    capture.CMD_START_MONITOR,
                    capture.start_monitor_payload(
                        self.config(Path("unused")).variables
                    ),
                ),
                (capture.CMD_START_MONITOR, b"\x00"),
            ],
        )
        self.assertTrue(fake.closed)

    def test_lost_start_ack_still_sends_stop(self) -> None:
        fake = self.FakeSerial(acknowledge_start=False)
        original = capture._open_serial
        capture._open_serial = lambda _config: fake
        try:
            with tempfile.TemporaryDirectory() as directory:
                with self.assertRaisesRegex(capture.CaptureError, "timeout waiting"):
                    capture.capture(
                        self.config(Path(directory) / "capture.csv", ack_timeout=0.001)
                    )
        finally:
            capture._open_serial = original
        self.assertEqual(
            fake.commands[-1], capture.Frame(capture.CMD_START_MONITOR, b"\x00")
        )
        self.assertTrue(fake.closed)

    def test_short_serial_write_is_rejected(self) -> None:
        class ShortWriter:
            def write(self, data: bytes) -> int:
                return len(data) - 1

            def flush(self) -> None:
                raise AssertionError("flush must not run after a short write")

        with self.assertRaisesRegex(capture.CaptureError, "serial short write"):
            capture._write_frame(ShortWriter(), b"123")


class RendererTests(unittest.TestCase):
    def test_headless_renderer_handles_wrap_and_malformed_row(self) -> None:
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            self.skipTest("matplotlib is not installed")
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            source = directory_path / "capture.csv"
            output = directory_path / "plot.png"
            with source.open("w", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(["timestamp_ms", "host_time_utc", "x"])
                writer.writerow([4294967290, "2026-01-01T00:00:00Z", 1.0])
                writer.writerow(["bad", "2026-01-01T00:00:00Z", 2.0])
                writer.writerow([4, "2026-01-01T00:00:00Z", 3.0])
            count, malformed, resets = renderer.render(source, output, ["x"], 0, 1)
            self.assertEqual(count, 2)
            self.assertEqual(malformed, 1)
            self.assertEqual(resets, 1)
            self.assertTrue(output.exists())
            self.assertGreater(output.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
