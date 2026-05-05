import json
import sys
import matplotlib.pyplot as plt

def plot_jsn(filepath):
    try:
        with open(filepath, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"Error loading {filepath}: {e}")
        return

    # Extract all XY points
    states = data.get('result', [{}])[0].get('states', [])
    if not states:
        print("No valid states array found in this JSN file.")
        return

    x = [s[0] for s in states]
    y = [s[1] for s in states]
    
    # Setup Plot
    plt.figure(figsize=(8, 8))
    plt.plot(x, y, 'b-', linewidth=2, label='Path')
    plt.plot(x[0], y[0], 'go', markersize=10, label='Start')
    plt.plot(x[-1], y[-1], 'rs', markersize=10, label='End')
    
    # Highlight the first few points to show the start direction
    plt.plot(x[:10], y[:10], 'g-', linewidth=4, alpha=0.5, label='Initial Heading')

    plt.title(f"Trajectory Preview: {filepath}")
    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 visualize_jsn.py path/to/TRAJ.JSN")
        sys.exit(1)
        
    plot_jsn(sys.argv[1])
