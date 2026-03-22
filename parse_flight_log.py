#!/usr/bin/env python3
"""Parse CanSat logger output and generate plots.

Usage:
  python parse_flight_log.py --input path/to/log.txt --output-dir out

Log format expected:
- Telemetry lines:
  ax,ay,az$gx,gy,gz$mx,my,mz$yaw,pitch,roll$temp$count$refresh
- Marker lines:
  MARKER,<millis>,<name>
Other lines are ignored.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import List

import matplotlib.pyplot as plt


@dataclass
class Sample:
    t_s: float
    accel: List[float]
    gyro: List[float]
    mag: List[float]
    ypr: List[float]
    temp_c: float
    refresh_hz: float


@dataclass
class Marker:
    t_s: float
    name: str


def parse_telemetry_line(line: str) -> Sample | None:
    parts = line.split("$")
    if len(parts) != 7:
        return None

    try:
        accel = [float(v) for v in parts[0].split(",")]
        gyro = [float(v) for v in parts[1].split(",")]
        mag = [float(v) for v in parts[2].split(",")]
        ypr = [float(v) for v in parts[3].split(",")]
        temp_c = float(parts[4])
        count_ms = float(parts[5])
        refresh_hz = float(parts[6])
    except ValueError:
        return None

    if not (len(accel) == len(gyro) == len(mag) == len(ypr) == 3):
        return None

    return Sample(
        t_s=count_ms / 1000.0,
        accel=accel,
        gyro=gyro,
        mag=mag,
        ypr=ypr,
        temp_c=temp_c,
        refresh_hz=refresh_hz,
    )


def parse_marker_line(line: str) -> Marker | None:
    if not line.startswith("MARKER,"):
        return None

    parts = line.split(",", 2)
    if len(parts) != 3:
        return None

    try:
        ms = float(parts[1])
    except ValueError:
        return None

    return Marker(t_s=ms / 1000.0, name=parts[2].strip())


def add_markers(ax, markers: List[Marker]) -> None:
    for m in markers:
        ax.axvline(m.t_s, color="black", alpha=0.2, linewidth=1)
        ax.text(m.t_s, 0.98, m.name, rotation=90, transform=ax.get_xaxis_transform(),
                va="top", ha="right", fontsize=8)


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse CanSat log file and generate graphs")
    parser.add_argument("--input", required=True, help="Path to logger output file")
    parser.add_argument("--output-dir", default="plots", help="Directory for output images")
    args = parser.parse_args()

    input_path = Path(args.input)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    samples: List[Sample] = []
    markers: List[Marker] = []

    with input_path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue

            marker = parse_marker_line(line)
            if marker is not None:
                markers.append(marker)
                continue

            sample = parse_telemetry_line(line)
            if sample is not None:
                samples.append(sample)

    if not samples:
        raise SystemExit("No telemetry samples found in input log file")

    t = [s.t_s for s in samples]

    fig, axes = plt.subplots(2, 2, figsize=(16, 10), sharex=True)

    ax = axes[0, 0]
    ax.plot(t, [s.accel[0] for s in samples], label="Accel X")
    ax.plot(t, [s.accel[1] for s in samples], label="Accel Y")
    ax.plot(t, [s.accel[2] for s in samples], label="Accel Z")
    ax.set_ylabel("mG")
    ax.set_title("Accelerometer")
    ax.grid(True, alpha=0.3)
    add_markers(ax, markers)
    ax.legend()

    ax = axes[0, 1]
    ax.plot(t, [s.gyro[0] for s in samples], label="Gyro X")
    ax.plot(t, [s.gyro[1] for s in samples], label="Gyro Y")
    ax.plot(t, [s.gyro[2] for s in samples], label="Gyro Z")
    ax.set_ylabel("deg/s")
    ax.set_title("Gyroscope")
    ax.grid(True, alpha=0.3)
    add_markers(ax, markers)
    ax.legend()

    ax = axes[1, 0]
    ax.plot(t, [s.ypr[0] for s in samples], label="Yaw")
    ax.plot(t, [s.ypr[1] for s in samples], label="Pitch")
    ax.plot(t, [s.ypr[2] for s in samples], label="Roll")
    ax.set_ylabel("deg")
    ax.set_xlabel("Time (s)")
    ax.set_title("Orientation")
    ax.grid(True, alpha=0.3)
    add_markers(ax, markers)
    ax.legend()

    ax = axes[1, 1]
    ax.plot(t, [s.temp_c for s in samples], label="Temperature")
    ax.plot(t, [s.refresh_hz for s in samples], label="Refresh Rate")
    ax.set_ylabel("Value")
    ax.set_xlabel("Time (s)")
    ax.set_title("Temperature and Refresh")
    ax.grid(True, alpha=0.3)
    add_markers(ax, markers)
    ax.legend()

    fig.tight_layout()
    fig.savefig(output_dir / "overview.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(16, 5), sharex=True)

    ax = axes[0]
    ax.plot(t, [s.mag[0] for s in samples], label="Mag X")
    ax.plot(t, [s.mag[1] for s in samples], label="Mag Y")
    ax.plot(t, [s.mag[2] for s in samples], label="Mag Z")
    ax.set_title("Magnetometer")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Raw")
    ax.grid(True, alpha=0.3)
    add_markers(ax, markers)
    ax.legend()

    ax = axes[1]
    ax.scatter([m.t_s for m in markers], list(range(len(markers))), s=40)
    for i, m in enumerate(markers):
        ax.text(m.t_s, i, m.name, fontsize=8, va="bottom", ha="left")
    ax.set_title("Mission Markers")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Marker Index")
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_dir / "markers_and_mag.png", dpi=150)
    plt.close(fig)

    print(f"Parsed {len(samples)} telemetry samples and {len(markers)} markers")
    print(f"Saved plots to: {output_dir}")


if __name__ == "__main__":
    main()
