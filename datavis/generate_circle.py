#!/usr/bin/env python
import argparse
import json
import math
import sys
from pathlib import Path

def main():
    # Pre-process sys.argv to convert '-c -value' or '--center -value' to '--center=-value'.
    # This bypasses argparse's limitation where it treats negative values as options.
    new_args = []
    i = 0
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg in ("-c", "--center") and i + 1 < len(sys.argv):
            next_arg = sys.argv[i+1]
            if next_arg.startswith("-"):
                new_args.append(f"--center={next_arg}")
                i += 2
                continue
        new_args.append(arg)
        i += 1
    sys.argv = new_args

    parser = argparse.ArgumentParser(
        description="Generate a circular trajectory JSN file for the Pololu robot."
    )
    parser.add_argument(
        "--center", "-c",
        required=True,
        type=str,
        help="Center of the circle as 'x,y' (e.g., '-0.5,1.4' or '0,0')"
    )
    parser.add_argument(
        "--diameter", "-d",
        required=True,
        type=float,
        help="Diameter of the circle in meters"
    )
    parser.add_argument(
        "--time", "-t",
        required=True,
        type=float,
        help="Time to complete the circle in seconds"
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.05,
        help="Time step in seconds (default: 0.05)"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=".",
        help="Directory to save the generated file (default: current directory)"
    )

    args = parser.parse_args()

    # Parse center coordinates
    try:
        parts = args.center.split(',')
        if len(parts) != 2:
            raise ValueError()
        x_c = float(parts[0])
        y_c = float(parts[1])
    except ValueError:
        print("Error: Center must be in the format 'x,y' where x and y are numbers.")
        sys.exit(1)

    # Compute radius and validate parameters
    r = args.diameter / 2.0
    T = args.time
    dt = args.dt

    if T <= 0:
        print("Error: Circle time must be positive.")
        sys.exit(1)
    if args.diameter <= 0:
        print("Error: Diameter must be positive.")
        sys.exit(1)
    if dt <= 0:
        print("Error: Time step (dt) must be positive.")
        sys.exit(1)

    # Calculate number of steps (N actions, N+1 states)
    N = int(round(T / dt))
    if N < 1:
        print("Error: The combination of time and dt yields no trajectory steps.")
        sys.exit(1)

    # Pololu firmware MAX_POINTS check
    MAX_POINTS = 350
    num_states = N + 1
    num_actions = N
    if num_states > MAX_POINTS:
        print(f"Warning: The trajectory has {num_states} states, which exceeds the firmware's "
              f"limit of {MAX_POINTS} points (MAX_POINTS).")
        print(f"To fix this, consider increasing --dt or decreasing the circle time.")

    # Calculate velocities
    # For a circle:
    # omega = 2 * pi / T
    # v = r * omega = 2 * pi * r / T
    omega = (2.0 * math.pi) / T
    v = r * omega

    states = []
    actions = []

    # Generate states
    # Start heading: theta_0 = 0 (facing +x)
    # Start position: (x_c, y_c - r)
    # The circle curves counter-clockwise (+y direction)
    # state = [x, y, theta]
    for i in range(num_states):
        t_i = i * dt
        theta_i = omega * t_i
        
        # Position on the circle
        x_i = x_c + r * math.sin(theta_i)
        y_i = y_c - r * math.cos(theta_i)
        
        # Round values to 6 decimal places for cleanliness
        states.append([
            round(x_i, 6),
            round(y_i, 6),
            round(theta_i, 6)
        ])

    # Generate actions (v, omega) for each step
    for i in range(num_actions):
        actions.append([
            round(v, 6),
            round(omega, 6)
        ])

    # Build the JSN structure in the style of 50ms_fast_bridge.JSN
    traj_data = {
        "result": [
            {
                "time_stamp": 0.0,
                "cost": 100.0,
                "num_states": num_states,
                "states": states,
                "num_actions": num_actions,
                "actions": actions,
                "dt": round(dt, 6),
                "start": states[0],
                "goal": states[-1]
            }
        ]
    }

    # Naming convention: circle_<x,y>_<r>_<t>.JSN
    filename = f"circle_{args.center}_{r}_{T}.JSN"
    output_path = Path(args.output_dir) / filename

    try:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(traj_data, f, indent=2)
        print(f"Successfully generated trajectory: {output_path}")
    except Exception as e:
        print(f"Error saving trajectory file: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
