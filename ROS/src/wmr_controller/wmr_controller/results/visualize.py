from ast import arg
from sys import argv
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

csv_path = argv[1] if len(argv) > 1 else "traj_bug_trap1.csv"
df = pd.read_csv("traj_bug_trap1.csv")
df.columns = df.columns.str.strip()

fig, ax = plt.subplots(figsize=(6, 9))
ax.plot(df["x"], df["y"], label="Trajectory")

# Show heading arrows at approximately 30 points; theta is in radians.
step = max(1, len(df) // 30)
points = df.iloc[::step]
ax.quiver(
    points["x"], points["y"],
    np.cos(points["theta"]), np.sin(points["theta"]),
    angles="xy", scale_units="xy", scale=8, color="red",
)

ax.set(
    xlim=(-1.5, 1.5), ylim=(-2.5, 2.5),
    xlabel="x (m)", ylabel="y (m)", title="Robot trajectory",
)
ax.set_aspect("equal")
ax.grid(True)
ax.legend()
plt.tight_layout()
plt.show()