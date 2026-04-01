#!/usr/bin/env python3
"""Plot pose log CSV from server.py tracking."""
import sys
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import glob

# Find latest CSV if none provided
if len(sys.argv) > 1:
    csv_path = sys.argv[1]
else:
    files = sorted(glob.glob("logs/poses_*.csv"))
    if not files:
        print("No CSV files found in logs/")
        sys.exit(1)
    csv_path = files[-1]
    print(f"Using latest: {csv_path}")

df = pd.read_csv(csv_path)
df["t_rel"] = df["timestamp"] - df["timestamp"].iloc[0]  # seconds from start

robot_ids = sorted(df["robot_id"].unique())
n_robots = len(robot_ids)

fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
fig.suptitle(f"Pose Log: {Path(csv_path).name}  ({len(df)} rows, {n_robots} robots)", fontsize=13)

colors = plt.cm.tab10(np.linspace(0, 1, max(n_robots, 1)))

for i, rid in enumerate(robot_ids):
    rd = df[df["robot_id"] == rid]
    det = rd[rd["detected"] == 1]
    miss = rd[rd["detected"] == 0]

    c = colors[i]
    label = f"Robot {rid}"

    # X position
    axes[0].plot(det["t_rel"], det["x"], ".-", color=c, markersize=2, linewidth=0.8, label=label)
    axes[0].scatter(miss["t_rel"], [None]*len(miss), marker="|", color=c, s=50, alpha=0.5)

    # Y position
    axes[1].plot(det["t_rel"], det["y"], ".-", color=c, markersize=2, linewidth=0.8, label=label)

    # Angle
    axes[2].plot(det["t_rel"], det["angle"], ".-", color=c, markersize=2, linewidth=0.8, label=label)

# Mark missing detections as red bands on all axes
for rid in robot_ids:
    rd = df[df["robot_id"] == rid]
    miss = rd[rd["detected"] == 0]
    for _, row in miss.iterrows():
        for ax in axes:
            ax.axvline(row["t_rel"], color="red", alpha=0.08, linewidth=1)

axes[0].set_ylabel("X (m)")
axes[1].set_ylabel("Y (m)")
axes[2].set_ylabel("Angle (deg)")
axes[2].set_xlabel("Time (s)")

for ax in axes:
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

plt.tight_layout()

# --- 2D trajectory plot ---
fig2, ax2 = plt.subplots(figsize=(10, 6))
fig2.suptitle(f"2D Trajectory: {Path(csv_path).name}", fontsize=13)

for i, rid in enumerate(robot_ids):
    det = df[(df["robot_id"] == rid) & (df["detected"] == 1)]
    if det.empty:
        continue
    c = colors[i]
    ax2.plot(det["x"], det["y"], ".-", color=c, markersize=3, linewidth=0.8, label=f"Robot {rid}")
    # Start marker
    ax2.scatter(det["x"].iloc[0], det["y"].iloc[0], color=c, marker="o", s=80, zorder=5, edgecolors="black")
    # End marker
    ax2.scatter(det["x"].iloc[-1], det["y"].iloc[-1], color=c, marker="s", s=80, zorder=5, edgecolors="black")

ax2.set_xlabel("X (m)")
ax2.set_ylabel("Y (m)")
ax2.set_aspect("equal")
ax2.legend(fontsize=9)
ax2.grid(True, alpha=0.3)

plt.tight_layout()

# --- Detection rate summary ---
print("\n--- Detection Summary ---")
for rid in robot_ids:
    rd = df[df["robot_id"] == rid]
    n_total = len(rd)
    n_det = (rd["detected"] == 1).sum()
    n_miss = n_total - n_det
    pct = 100 * n_det / n_total if n_total > 0 else 0
    print(f"  Robot {rid}: {n_det}/{n_total} detected ({pct:.1f}%), {n_miss} missed")

duration = df["t_rel"].iloc[-1]
print(f"  Duration: {duration:.1f}s, Avg frame rate: {len(df)/n_robots/duration:.1f} Hz")

plt.show()
