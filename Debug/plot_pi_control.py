
#!/usr/bin/env python3
"""
Parse PI control debug logs and plot diagnostics (errors, u_k, RPM target/raw/filtered).

Usage:
  python plot_pi_control.py --input PIcontroldebug.csv \
                            --out-csv PIcontrol_parsed.csv \
                            --out-png control_subplots.png
"""

import re
import argparse
from pathlib import Path

import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use("Agg")  # non gui backend



PAIR_RE = re.compile(r"([A-Za-z][A-Za-z _\-\/]*?):\s*(-?\d+(?:\.\d+)?)")
UPDATED_CONTROL_RE = re.compile(r"Updated Control:\s*(-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?)", re.I)
RAW_CMDS_RE = re.compile(r"Raw commands:\s*v=(-?\d+(?:\.\d+)?),\s*omega=(-?\d+(?:\.\d+)?)", re.I)
GEAR_RATIO_RE = re.compile(r"Actual gear ratio\s*=\s*(-?\d+(?:\.\d+)?)", re.I)


def norm_key(s: str) -> str:
    return (
        s.strip()
        .lower()
        .replace(" ", "_")
        .replace("-", "_")
        .replace("__", "_")
    )


def parse_message_header(line: str):
    """
    Extract the message text and any Left/Right prefix.
    Example line:
      "[INFO ] Left RPM - Raw: 0.0, Filtered: 0.0, Target: 0.0, Error: 0.0 (file.rs:13)"
    """
    m = re.search(r"\]\s*(.*?)\s*\(", line)
    if not m:
        m = re.search(r"\]\s*(.*)", line)
    text = m.group(1).strip() if m else line.strip()
    side = None
    msg = text
    tl = text.lower()
    if tl.startswith("left "):
        side = "left"
        msg = text[5:].strip()
    elif tl.startswith("right "):
        side = "right"
        msg = text[6:].strip()
    return msg, side


def parse_log(path: Path) -> pd.DataFrame:
    lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()

    state = {}
    rows = []
    t = 0

    for line in lines:
        if "[INFO" not in line and "[WARN" not in line and "[ERROR" not in line:
            continue

        msg, side = parse_message_header(line)

        local_updates = {}

        # Specific patterns
        uc = UPDATED_CONTROL_RE.search(line)
        if uc:
            local_updates["updated_control_0"] = float(uc.group(1))
            local_updates["updated_control_1"] = float(uc.group(2))

        rc = RAW_CMDS_RE.search(line)
        if rc:
            local_updates["cmd_v"] = float(rc.group(1))
            local_updates["cmd_omega"] = float(rc.group(2))

        gr = GEAR_RATIO_RE.search(line)
        if gr:
            local_updates["actual_gear_ratio"] = float(gr.group(1))

        # Generic key:value pairs
        for k, v in PAIR_RE.findall(line):
            k_norm = norm_key(k)
            # Add side prefix for canonical PI and RPM fields
            if side and k_norm in {"raw", "filtered", "target", "error", "p_inc", "i_total", "i_inc", "u_k"}:
                k_norm = f"{side}_{k_norm}"
            local_updates[k_norm] = float(v)

        if local_updates:
            state.update(local_updates)
            state["t"] = t
            rows.append(state.copy())
            t += 1

    if not rows:
        print("Debug: No rows found. Checking first few lines of log file:")
        for i, line in enumerate(lines[:10]):
            print(f"  Line {i}: {line}")
        raise ValueError("No numeric data found in the log. Check if the log file contains proper debug messages with [INFO], [WARN], or [ERROR] tags and numeric values.")

    print(f"Successfully parsed {len(rows)} data points from log file.")
    df = pd.DataFrame(rows).sort_values("t").reset_index(drop=True).ffill()
    return df


def plot_subplots(df: pd.DataFrame, out_png: Path | None):
    """Two subplots (left/right) with Error, u_k, Target/Raw/Filtered RPM. No command v or omega lines."""
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Left wheel subplot
    ax = axes[0]
    if "left_error" in df:       ax.plot(df["t"], df["left_error"], label="Error")
    if "left_u_k" in df:         ax.plot(df["t"], df["left_u_k"], label="u_k")
    if "left_target" in df:      ax.plot(df["t"], df["left_target"], label="Target RPM")
    if "left_raw" in df:         ax.plot(df["t"], df["left_raw"], label="Raw RPM")
    if "left_filtered" in df:    ax.plot(df["t"], df["left_filtered"], label="Filtered RPM")
    ax.set_title("Left Wheel")
    ax.set_ylabel("Value")
    ax.grid(True)
    ax.legend()

    # Right wheel subplot
    ax = axes[1]
    if "right_error" in df:      ax.plot(df["t"], df["right_error"], label="Error")
    if "right_u_k" in df:        ax.plot(df["t"], df["right_u_k"], label="u_k")
    if "right_target" in df:     ax.plot(df["t"], df["right_target"], label="Target RPM")
    if "right_raw" in df:        ax.plot(df["t"], df["right_raw"], label="Raw RPM")
    if "right_filtered" in df:   ax.plot(df["t"], df["right_filtered"], label="Filtered RPM")
    ax.set_title("Right Wheel")
    ax.set_xlabel("t (log step)")
    ax.set_ylabel("Value")
    ax.grid(True)
    ax.legend()

    plt.tight_layout()
    
    # Always save the figure
    if out_png is None:
        out_png = Path("control_subplots.png")
    plt.savefig(out_png, dpi=150)
    print(f"Figure saved to: {out_png.resolve()}")
    
    # Try to show, but don't fail if no display
    try:
        plt.show()
    except Exception as e:
        print(f"Could not display figure (no GUI available): {e}")
        print("Figure has been saved to file instead.")
    
    plt.close(fig)  # Free memory


def main():
    import argparse
    ap = argparse.ArgumentParser(description="Parse PI control debug logs and plot diagnostics.")
    ap.add_argument("--input", required=True, type=Path, help="Path to the mixed debug log.")
    ap.add_argument("--out-csv", type=Path, default=Path("PIcontrol_parsed.csv"), help="Where to write the parsed CSV.")
    ap.add_argument("--out-png", type=Path, default=Path("control_subplots.png"), help="Where to save the subplot figure PNG.")
    args = ap.parse_args()

    df = parse_log(args.input)
    df.to_csv(args.out_csv, index=False)
    print(f"Saved parsed CSV to: {args.out_csv.resolve()}")

    plot_subplots(df, args.out_png)
    if args.out_png:
        print(f"Saved figure to: {args.out_png.resolve()}")


if __name__ == "__main__":
    main()
