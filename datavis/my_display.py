#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Make a single dashboard figure comparing target vs actual for a robotics log.
- Reads a CSV with columns like: ts,target_x,target_y,target_theta,actual_x,actual_y,actual_theta,...
- Saves one big PNG/PDF (no GUI).

Usage:
  python3 dashboard_plot.py your_log.csv
"""

import sys
import os
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

def deg(x):
    return np.degrees(x)

def safe_col(df, name):
    return name in df.columns and not df[name].isna().all()

def plot_pair(ax, t, df, tcol, acol, label, ylabel, to_deg=False):
    """Plot target vs actual on the same axes if columns exist."""
    has = False
    if safe_col(df, tcol):
        ax.plot(t, deg(df[tcol]) if to_deg else df[tcol], linestyle="--", label=f"target {label}")
        has = True
    if safe_col(df, acol):
        ax.plot(t, deg(df[acol]) if to_deg else df[acol], linestyle="-", label=f"actual {label}")
        has = True
    if has:
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    return has

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 dashboard_plot.py your_log.csv")
        sys.exit(1)

    csv_path = Path(sys.argv[1])
    if not csv_path.exists():
        print(f"File not found: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)
    if "ts" not in df.columns:
        print("CSV must contain 'ts' column (milliseconds).")
        sys.exit(1)

    t = df["ts"].to_numpy() / 1000.0

    outdir = csv_path.parent / "plots"
    outdir.mkdir(exist_ok=True)
    base = csv_path.stem
    png_path = outdir / f"{base}_dashboard.png"
    pdf_path = outdir / f"{base}_dashboard.pdf"


    fig = plt.figure(figsize=(18, 16), constrained_layout=True)
    gs = GridSpec(6, 3, figure=fig, height_ratios=[1.2, 1, 1, 1.2, 1, 1])

    ax_traj = fig.add_subplot(gs[0, 0:2])
    has_tx = safe_col(df, "target_x")
    has_ty = safe_col(df, "target_y")
    has_ax = safe_col(df, "actual_x")
    has_ay = safe_col(df, "actual_y")
    if has_tx and has_ty:
        ax_traj.plot(df["target_x"], df["target_y"], "--", label="target traj")
    if has_ax and has_ay:
        ax_traj.plot(df["actual_x"], df["actual_y"], "-", label="actual traj")
    ax_traj.set_title("XY Trajectory (target vs actual)")
    ax_traj.set_xlabel("X (m)")
    ax_traj.set_ylabel("Y (m)")
    ax_traj.axis("equal")
    ax_traj.grid(True, alpha=0.3)
    ax_traj.legend(loc="best")


    ax_px = fig.add_subplot(gs[1, 0])
    plot_pair(ax_px, t, df, "target_x", "actual_x", "x", "X (m)")

    ax_py = fig.add_subplot(gs[1, 1])
    plot_pair(ax_py, t, df, "target_y", "actual_y", "y", "Y (m)")

    ax_th = fig.add_subplot(gs[1, 2])
    target_theta_col = "target_theta" if "target_theta" in df.columns else "target_yaw"
    actual_theta_col = "actual_theta" if "actual_theta" in df.columns else "actual_yaw"
    _ = plot_pair(ax_th, t, df, target_theta_col, actual_theta_col, "theta", "Theta (deg)", to_deg=True)


    ax_px2 = fig.add_subplot(gs[2, 0])
    plot_pair(ax_px2, t, df, "target_x", "actual_x", "x", "X (m)")

    ax_py2 = fig.add_subplot(gs[2, 1])
    plot_pair(ax_py2, t, df, "target_y", "actual_y", "y", "Y (m)")

    ax_th_err = fig.add_subplot(gs[2, 2])
    if safe_col(df, "thetaerror"):
        ax_th_err.plot(t, deg(df["thetaerror"]), "-", label="theta error (deg)")
        ax_th_err.axhline(0, linewidth=1, alpha=0.5)
        ax_th_err.set_ylabel("deg")
        ax_th_err.grid(True, alpha=0.3)
        ax_th_err.legend(loc="best")

    ax_vx = fig.add_subplot(gs[3, 0])
    plot_pair(ax_vx, t, df, "target_vx", "actual_vx", "vx", "vx (m/s)")

    ax_vy = fig.add_subplot(gs[3, 1])
    plot_pair(ax_vy, t, df, "target_vy", "actual_vy", "vy", "vy (m/s)")

    ax_vz = fig.add_subplot(gs[3, 2])
    plot_pair(ax_vz, t, df, "target_vz", "actual_vz", "vz", "vz (m/s)")

    ax_qw = fig.add_subplot(gs[4, 0])
    plot_pair(ax_qw, t, df, "target_qw", "actual_qw", "qw", "qw")

    ax_qx = fig.add_subplot(gs[4, 1])
    plot_pair(ax_qx, t, df, "target_qx", "actual_qx", "qx", "qx")

    ax_qy = fig.add_subplot(gs[4, 2])
    plot_pair(ax_qy, t, df, "target_qy", "actual_qy", "qy", "qy")

    ax_qz = fig.add_subplot(gs[5, 0])
    plot_pair(ax_qz, t, df, "target_qz", "actual_qz", "qz", "qz")

    ax_err = fig.add_subplot(gs[5, 1])
    has_any_err = False
    if safe_col(df, "xerror"):
        ax_err.plot(t, df["xerror"], "-", label="x error (m)")
        has_any_err = True
    if safe_col(df, "yerror"):
        ax_err.plot(t, df["yerror"], "-", label="y error (m)")
        has_any_err = True
    if has_any_err:
        ax_err.axhline(0, linewidth=1, alpha=0.5)
        ax_err.set_ylabel("Position Error (m)")
        ax_err.grid(True, alpha=0.3)
        ax_err.legend(loc="best", fontsize=8)

    ax_act = fig.add_subplot(gs[5, 2])
    plotted = False
    if safe_col(df, "ul"):
        ax_act.plot(t, df["ul"], "-", label="ul")
        plotted = True
    if safe_col(df, "ur"):
        ax_act.plot(t, df["ur"], "-", label="ur")
        plotted = True
    if safe_col(df, "dutyl"):
        ax_act.plot(t, df["dutyl"], "--", label="dutyl")
        plotted = True
    if safe_col(df, "dutyr"):
        ax_act.plot(t, df["dutyr"], "--", label="dutyr")
        plotted = True
    if plotted:
        ax_act.set_ylabel("actuation")
        ax_act.grid(True, alpha=0.3)
        ax_act.legend(loc="best", fontsize=8)

    fig.suptitle(f"Robotics Log Dashboard — {base}", fontsize=16, fontweight="bold")

    fig.savefig(png_path, dpi=200)
    fig.savefig(pdf_path, dpi=200)
    print(f"Saved:\n  {png_path}\n  {pdf_path}")

if __name__ == "__main__":
    main()
