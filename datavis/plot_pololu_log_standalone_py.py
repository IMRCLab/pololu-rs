from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


EXPECTED_COLUMNS = (
    "ts",
    "x",
    "y",
    "yaw",
    "x_des",
    "y_des",
    "yaw_des",
    "v_ff",
    "w_ff",
    "v_actual",
    "w_actual",
    "omega_l_cmd",
    "omega_r_cmd",
    "omega_l_meas",
    "omega_r_meas",
    "duty_l",
    "duty_r",
    "x_err",
    "y_err",
    "yaw_err",
    "x_raw",
    "y_raw",
    "yaw_raw",
    "acc_x",
    "acc_y",
    "acc_z",
    "gyro_x",
    "gyro_y",
    "gyro_z",
)


def looks_like_log(path: Path) -> bool:
    try:
        first_line = path.read_text(encoding="utf-8").splitlines()[0]
    except (OSError, UnicodeDecodeError, IndexError):
        return False
    return tuple(name.strip() for name in first_line.split(",")) == EXPECTED_COLUMNS


def log_files(directory: Path) -> list[Path]:
    paths = sorted(path for path in directory.iterdir() if path.is_file() and looks_like_log(path))
    if not paths:
        raise ValueError(f"No Pololu log files found in {directory}")
    return paths


def load_columns(path: str | Path) -> tuple[list[str], np.ndarray]:
    data = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    columns = list(data.dtype.names or ())
    values = np.column_stack([data[name] for name in columns])
    if values.ndim == 1:
        values = values[None, :]
    ts_index = columns.index("ts")
    values = values[np.isfinite(values[:, ts_index])]
    values = values[np.argsort(values[:, ts_index])]
    values = values.copy()
    values[:, ts_index] = (values[:, ts_index] - values[0, ts_index]) / 1000.0
    return columns, values


def rows_with(columns: list[str], data: np.ndarray, names: tuple[str, ...]) -> np.ndarray:
    indices = [columns.index(name) for name in names]
    return data[np.all(np.isfinite(data[:, indices]), axis=1)]


def col(columns: list[str], data: np.ndarray, name: str) -> np.ndarray:
    return data[:, columns.index(name)]


def mocap_vel_omega(time_s: np.ndarray, pose: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if len(time_s) < 2:
        return time_s, np.zeros((len(time_s), 2))
    dt = np.diff(time_s)
    dxy = np.diff(pose[:, :2], axis=0)
    theta = np.unwrap(pose[:, 2])
    heading = theta[1:]
    v = (dxy[:, 0] * np.cos(heading) + dxy[:, 1] * np.sin(heading)) / dt
    omega = np.diff(theta) / dt
    return time_s[1:], np.column_stack([v, omega])


def stair_series(time: np.ndarray, values: np.ndarray, end_time: float | None) -> tuple[np.ndarray, np.ndarray]:
    if end_time is None or len(time) == 0 or end_time <= time[-1]:
        return time, values
    return np.concatenate([time, [end_time]]), np.concatenate([values, values[-1:]])


def clip_after_first_trajectory(columns: list[str], data: np.ndarray, min_zero_rows: int = 3) -> np.ndarray:
    reference_rows = rows_with(columns, data, ("x_des", "y_des", "yaw_des", "v_ff", "w_ff"))
    if len(reference_rows) < min_zero_rows:
        return data
    ts_index = columns.index("ts")
    zero = (np.abs(col(columns, reference_rows, "v_ff")) <= 1e-6) & (
        np.abs(col(columns, reference_rows, "w_ff")) <= 1e-6
    )
    motion_seen = False
    for index in range(len(reference_rows) - min_zero_rows + 1):
        if not zero[index]:
            motion_seen = True
            continue
        if motion_seen and np.all(zero[index : index + min_zero_rows]):
            return data[data[:, ts_index] < reference_rows[index, ts_index]]
    return data


def plot_log(log_path: str | Path, output_path: str | Path, clip: bool = False):
    columns, data = load_columns(log_path)
    if clip:
        data = clip_after_first_trajectory(columns, data)

    reference_rows = rows_with(columns, data, ("x_des", "y_des", "yaw_des", "v_ff", "w_ff"))
    pose_rows = rows_with(columns, data, ("x_raw", "y_raw", "yaw_raw"))
    wheel_rows = rows_with(columns, data, ("omega_r_meas", "omega_l_meas"))
    command_rows = rows_with(columns, data, ("omega_r_cmd", "omega_l_cmd"))

    ref_time = col(columns, reference_rows, "ts")
    reference = np.column_stack(
        [
            col(columns, reference_rows, "x_des"),
            col(columns, reference_rows, "y_des"),
            col(columns, reference_rows, "yaw_des"),
            col(columns, reference_rows, "v_ff"),
            col(columns, reference_rows, "w_ff"),
        ]
    )
    pose_time = col(columns, pose_rows, "ts")
    measured_pose = np.column_stack(
        [col(columns, pose_rows, "x_raw"), col(columns, pose_rows, "y_raw"), col(columns, pose_rows, "yaw_raw")]
    )
    wheel_time = col(columns, wheel_rows, "ts")
    wheel_speeds = np.column_stack(
        [col(columns, wheel_rows, "omega_r_meas"), col(columns, wheel_rows, "omega_l_meas")]
    )
    command_time = col(columns, command_rows, "ts")
    wheel_cmd = np.column_stack(
        [col(columns, command_rows, "omega_r_cmd"), col(columns, command_rows, "omega_l_cmd")]
    )
    mocap_time, mocap_vel = mocap_vel_omega(pose_time, measured_pose)

    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle(f"Pololu Log Summary ({Path(log_path).stem})", fontsize=16)

    ax = axes[0, 0]
    ax.plot(reference[:, 0], reference[:, 1], color="tab:red", linestyle="--", linewidth=1.0, label="Reference")
    ax.plot(measured_pose[:, 0], measured_pose[:, 1], color="tab:blue", linewidth=0.95, label="Measured")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajectory")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend()

    ax_v = axes[0, 1]
    ax_w = ax_v.twinx()
    line_ref_v = ax_v.step(ref_time, np.abs(reference[:, 3]), where="post", color="lightskyblue", linestyle="--", linewidth=0.9, label="ref v")[0]
    line_mocap_v = ax_v.plot(mocap_time, mocap_vel[:, 0], color="tab:blue", linewidth=0.65, label="mocap v")[0]
    line_ref_w = ax_w.step(ref_time, reference[:, 4], where="post", color="khaki", linestyle="--", linewidth=0.9, label="ref omega")[0]
    line_mocap_w = ax_w.plot(mocap_time, mocap_vel[:, 1], color="goldenrod", linewidth=0.65, label="mocap omega")[0]
    ax_v.set_xlabel("time [s]")
    ax_v.set_ylabel("linear velocity [m/s]")
    ax_w.set_ylabel("angular velocity [rad/s]")
    ax_v.set_title("Velocity")
    ax_v.grid(True)
    ax_v.legend(handles=[line_ref_v, line_mocap_v, line_ref_w, line_mocap_w], loc="best")

    ax = axes[0, 2]
    cmd_time, cmd_right = stair_series(command_time, wheel_cmd[:, 0], wheel_time[-1] if len(wheel_time) else None)
    _, cmd_left = stair_series(command_time, wheel_cmd[:, 1], wheel_time[-1] if len(wheel_time) else None)
    lines = [
        ax.step(cmd_time, cmd_right, where="post", color="tab:green", linestyle="--", linewidth=0.75, label="cmd right")[0],
        ax.plot(wheel_time, wheel_speeds[:, 0], color="tab:green", linewidth=0.8, label="meas right")[0],
        ax.step(cmd_time, cmd_left, where="post", color="tab:orange", linestyle="--", linewidth=0.75, label="cmd left")[0],
        ax.plot(wheel_time, wheel_speeds[:, 1], color="tab:orange", linewidth=0.8, label="meas left")[0],
    ]
    ax.set_xlabel("time [s]")
    ax.set_ylabel("wheel speed [rad/s]")
    ax.set_title("Wheel Speeds")
    ax.grid(True)
    ax.legend(handles=lines)

    for index, ax in enumerate(axes[1, :]):
        ax.step(ref_time, reference[:, index], where="post", color="tab:red", linestyle="--", linewidth=0.9, label="Reference")
        ax.plot(pose_time, measured_pose[:, index], color="tab:blue", linewidth=0.95, label="Measured")
        ax.set_xlabel("time [s]")
        ax.set_ylabel(("x [m]", "y [m]", "theta [rad]")[index])
        ax.set_title(("x State", "y State", "theta State")[index])
        ax.grid(True)
        ax.legend()

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(output_path, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Standalone Pololu log summary plotter.")
    parser.add_argument("log")
    parser.add_argument("--output", default=None)
    parser.add_argument("--clip-after-first-trajectory", default=True)
    args = parser.parse_args()

    log_path = Path(args.log)
    if log_path.is_dir():
        output_dir = Path(args.output) if args.output is not None else log_path
        output_dir = output_dir / log_path.name
        output_dir.mkdir(parents=True, exist_ok=True)
        for path in log_files(log_path):
            output_path = output_dir / f"{path.stem}.pdf"
            plot_log(path, output_path, clip=args.clip_after_first_trajectory)
            print(output_path)
        return

    output_path = Path(args.output) if args.output is not None else log_path.with_suffix(".pdf")
    plot_log(log_path, output_path, clip=args.clip_after_first_trajectory)
    print(output_path)


if __name__ == "__main__":
    main()
