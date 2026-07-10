#!/usr/bin/env python3
"""Render selected AresPlot CSV series to a labeled PNG without a display server."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path


class RenderError(RuntimeError):
    """Report a CSV or rendering error without hiding its cause."""


@dataclass(frozen=True)
class Point:
    seconds: float
    values: dict[str, float]


def _unwrapped_seconds(raw_timestamp: int, previous_raw: int | None, previous_ms: int | None) -> tuple[int, int]:
    if previous_raw is None or previous_ms is None:
        return raw_timestamp, raw_timestamp
    if raw_timestamp >= previous_raw:
        return raw_timestamp, previous_ms + (raw_timestamp - previous_raw)
    if previous_raw - raw_timestamp > 0x80000000:
        return raw_timestamp, previous_ms + ((1 << 32) - previous_raw + raw_timestamp)
    # A target reboot/reset: keep a monotonic plot but record only the smallest plausible gap.
    return raw_timestamp, previous_ms + 1


def read_csv(path: Path, requested: list[str]) -> tuple[list[Point], list[str], int, int]:
    try:
        source = path.open(newline="", encoding="utf-8")
    except FileNotFoundError as exc:
        raise RenderError(f"CSV not found: {path}") from exc
    with source:
        reader = csv.DictReader(source)
        if not reader.fieldnames or "timestamp_ms" not in reader.fieldnames:
            raise RenderError("CSV must contain a timestamp_ms column")
        default_series = [name for name in reader.fieldnames if name not in ("timestamp_ms", "host_time_utc")]
        series = requested or default_series
        missing = [name for name in series if name not in reader.fieldnames]
        if missing:
            raise RenderError("CSV is missing requested series: " + ", ".join(missing))
        if not series:
            raise RenderError("CSV has no value columns; select at least one series")

        points: list[Point] = []
        malformed = 0
        resets_or_wraps = 0
        previous_raw: int | None = None
        previous_unwrapped: int | None = None
        first_ms: int | None = None
        for line_number, row in enumerate(reader, start=2):
            try:
                raw = int(row["timestamp_ms"], 0)
                if not 0 <= raw <= 0xFFFFFFFF:
                    raise ValueError("timestamp outside uint32")
                values = {name: float(row[name]) for name in series}
                if any(not math.isfinite(value) for value in values.values()):
                    raise ValueError("non-finite series value")
            except (KeyError, TypeError, ValueError):
                malformed += 1
                continue
            if previous_raw is not None and raw < previous_raw:
                resets_or_wraps += 1
            previous_raw, unwrapped = _unwrapped_seconds(raw, previous_raw, previous_unwrapped)
            previous_unwrapped = unwrapped
            if first_ms is None:
                first_ms = unwrapped
            points.append(Point((unwrapped - first_ms) / 1000.0, values))
    return points, series, malformed, resets_or_wraps


def parse_series(items: list[str]) -> list[str]:
    values: list[str] = []
    for item in items:
        values.extend(value.strip() for value in item.split(",") if value.strip())
    return list(dict.fromkeys(values))


def render(input_path: Path, output_path: Path, series_items: list[str], start: float | None, end: float | None) -> tuple[int, int, int]:
    if start is not None and end is not None and start > end:
        raise RenderError("--start must not be greater than --end")
    points, series, malformed, resets = read_csv(input_path, parse_series(series_items))
    selected = [point for point in points if (start is None or point.seconds >= start) and (end is None or point.seconds <= end)]
    if not selected:
        raise RenderError("selected time interval contains no valid samples")
    try:
        import matplotlib

        matplotlib.use("Agg")
        from matplotlib import pyplot as plt
    except ImportError as exc:
        raise RenderError("matplotlib is missing; install it with: python3 -m pip install matplotlib") from exc

    figure, axis = plt.subplots(figsize=(11, 5.5), constrained_layout=True)
    for name in series:
        axis.plot([point.seconds for point in selected], [point.values[name] for point in selected], label=name, linewidth=1.2)
    axis.set_title(f"AresPlot capture: {input_path.name}")
    axis.set_xlabel("MCU time since first sample (s)")
    axis.set_ylabel("Value (AresPlot float32 payload)")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return len(selected), malformed, resets


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="CSV produced by aresplot_capture.py")
    parser.add_argument("--series", action="append", default=[], help="series name; repeat or use comma-separated names")
    parser.add_argument("--start", type=float, help="inclusive start, seconds in unwrapped MCU timeline")
    parser.add_argument("--end", type=float, help="inclusive end, seconds in unwrapped MCU timeline")
    parser.add_argument("--output", required=True, type=Path, help="output PNG path")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        count, malformed, resets = render(args.csv, args.output, args.series, args.start, args.end)
    except RenderError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    print(f"rendered {count} samples to {args.output}; malformed_rows={malformed}, timestamp_wraps_or_resets={resets}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
