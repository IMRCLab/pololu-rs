#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


EXPECTED_COLUMNS = {
    "stamp_ns",
    "name",
    "x",
    "y",
    "z",
}


@dataclass
class MocapSeries:
    name: str
    time_s: np.ndarray
    x: np.ndarray
    y: np.ndarray
    theta: np.ndarray


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def load_mocap_csv(path: str | Path, max_time: float | None = 5.0) -> list[MocapSeries]:
    path = Path(path)
    by_name: dict[str, list[tuple[int, float, float, float]]] = {}

    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no CSV header")
        columns = {name.strip() for name in reader.fieldnames}
        missing = EXPECTED_COLUMNS - columns
        if missing:
            raise ValueError(f"{path} is missing columns: {', '.join(sorted(missing))}")
        has_theta = "theta" in columns
        if not has_theta:
            quaternion_columns = {"qx", "qy", "qz", "qw"}
            missing_quaternion = quaternion_columns - columns
            if missing_quaternion:
                raise ValueError(
                    f"{path} needs either theta or quaternion columns; missing: "
                    f"{', '.join(sorted(missing_quaternion))}"
                )

        for row in reader:
            try:
                stamp_ns = int(row["stamp_ns"])
                name = row["name"].strip()
                x = float(row["x"])
                y = float(row["y"])
                theta = (
                    float(row["theta"])
                    if has_theta
                    else yaw_from_quaternion(
                        float(row["qx"]),
                        float(row["qy"]),
                        float(row["qz"]),
                        float(row["qw"]),
                    )
                )
            except (KeyError, TypeError, ValueError):
                continue
            if not name:
                name = "unnamed"
            by_name.setdefault(name, []).append((stamp_ns, x, y, theta))

    if not by_name:
        raise ValueError(f"No valid mocap rows found in {path}")

    global_start_ns = min(rows[0][0] for rows in by_name.values() if rows)
    series = []
    for name, rows in sorted(by_name.items()):
        rows = sorted(rows, key=lambda item: item[0])
        data = np.asarray(rows, dtype=float)
        time_s = (data[:, 0] - global_start_ns) * 1e-9
        keep = np.isfinite(time_s) & np.all(np.isfinite(data[:, 1:]), axis=1)
        if max_time is not None:
            if max_time < 0:
                raise ValueError("--max-time must be non-negative")
            keep &= time_s <= max_time
        time_s = time_s[keep]
        if len(time_s) == 0:
            continue
        theta = np.unwrap(data[keep, 3])
        series.append(
            MocapSeries(
                name=name,
                time_s=time_s,
                x=data[keep, 1],
                y=data[keep, 2],
                theta=theta,
            )
        )

    if not series:
        raise ValueError(f"No rows remain after clipping {path}")
    return series


def finite_difference_velocity(series: MocapSeries) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if len(series.time_s) < 2:
        return np.empty(0), np.empty(0), np.empty(0)

    dt = np.diff(series.time_s)
    valid = dt > 0.0
    if not np.any(valid):
        return np.empty(0), np.empty(0), np.empty(0)

    dx = np.diff(series.x)
    dy = np.diff(series.y)
    dtheta = np.diff(series.theta)
    heading = series.theta[1:]

    v = (dx * np.cos(heading) + dy * np.sin(heading)) / dt
    omega = dtheta / dt
    return series.time_s[1:][valid], v[valid], omega[valid]


def plot_mocap_csv(
    csv_path: str | Path,
    output_path: str | Path,
    max_time: float | None = 5.0,
) -> None:
    csv_path = Path(csv_path)
    series_list = load_mocap_csv(csv_path, max_time=max_time)

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(f"Mocap CSV Summary ({csv_path.stem})", fontsize=16)

    ax = axes[0, 0]
    for series in series_list:
        ax.plot(series.x, series.y, linewidth=1.0, label=series.name)
        ax.plot(series.x[0], series.y[0], marker="o", markersize=4)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajectory")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend()

    ax_v = axes[0, 1]
    ax_w = ax_v.twinx()
    velocity_lines = []
    for series in series_list:
        vel_time, v, omega = finite_difference_velocity(series)
        if len(vel_time) == 0:
            continue
        velocity_lines.append(ax_v.plot(vel_time, v, linewidth=0.75, label=f"{series.name} v")[0])
        velocity_lines.append(
            ax_w.plot(vel_time, omega, linestyle="--", linewidth=0.75, label=f"{series.name} omega")[0]
        )
    ax_v.set_xlabel("time [s]")
    ax_v.set_ylabel("linear velocity [m/s]")
    ax_w.set_ylabel("angular velocity [rad/s]")
    ax_v.set_title("Finite-Difference Velocity")
    ax_v.grid(True)
    if velocity_lines:
        ax_v.legend(handles=velocity_lines, loc="best")

    axes[0, 2].axis("off")

    state_axes = axes[1, :]
    for state_ax, attr, ylabel, title in zip(
        state_axes,
        ("x", "y", "theta"),
        ("x [m]", "y [m]", "theta [rad]"),
        ("x State", "y State", "theta State"),
    ):
        for series in series_list:
            state_ax.plot(series.time_s, getattr(series, attr), linewidth=0.85, label=series.name)
        state_ax.set_xlabel("time [s]")
        state_ax.set_ylabel(ylabel)
        state_ax.set_title(title)
        state_ax.grid(True)
        state_ax.legend()

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(output_path, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def mocap_csv_files(directory: Path) -> list[Path]:
    paths = sorted(path for path in directory.iterdir() if path.is_file() and path.name.endswith("_mocap.csv"))
    if not paths:
        raise ValueError(f"No *_mocap.csv files found in {directory}")
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot ROS-side mocap CSV logs without ROS dependencies.")
    parser.add_argument("--csv", required=True, help="Path to one *_mocap.csv file or a directory containing them.")
    parser.add_argument("--output", default=None, help="Output PDF path, or output directory when --csv is a directory.")
    parser.add_argument("--max-time", type=float, default=5.0, help="Clip plot to this many seconds from first timestamp.")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    if csv_path.is_dir():
        output_dir = Path(args.output) if args.output is not None else csv_path / "plots"
        output_dir.mkdir(parents=True, exist_ok=True)
        for path in mocap_csv_files(csv_path):
            output_path = output_dir / f"{path.stem}.pdf"
            plot_mocap_csv(path, output_path, max_time=args.max_time)
            print(output_path)
        return

    output_path = Path(args.output) if args.output is not None else csv_path.with_suffix(".pdf")
    plot_mocap_csv(csv_path, output_path, max_time=args.max_time)
    print(output_path)


if __name__ == "__main__":
    main()
