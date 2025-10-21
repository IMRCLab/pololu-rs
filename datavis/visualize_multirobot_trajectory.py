#!/usr/bin/env python3
"""
Multi-Robot Trajectory Visualizer
Simple 2D trajectory visualization for up to 4 robots
Usage: python3 visualize_multirobot_trajectory.py TR01 TR02 TR03
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import argparse
from pathlib import Path
import sys

# Optional YAML support for obstacles
try:
    import yaml
    YAML_AVAILABLE = True
except ImportError:
    YAML_AVAILABLE = False

def load_trajectory_data(filepath):
    """Load trajectory data from CSV file"""
    try:
        # First try standard pandas read
        df = pd.read_csv(filepath)
        if len(df) > 1:
            df = df.dropna()
            return df
    except:
        pass
    
    # Handle malformed CSV - all data on one line
    try:
        with open(filepath, 'r') as f:
            content = f.read().strip()
        
        lines = content.split('\n')
        if len(lines) <= 2:  # Malformed CSV
            parts = content.split(',')
            
            # Find header end
            header_end = 0
            for i, part in enumerate(parts):
                try:
                    float(part)
                    header_end = i
                    break
                except ValueError:
                    continue
            
            if header_end == 0:
                return None
            
            header = parts[:header_end]
            data_parts = parts[header_end:]
            
            # Clean header
            header = [col.strip() for col in header]
            
            # Reconstruct rows
            data_rows = []
            num_cols = len(header)
            
            for i in range(0, len(data_parts), num_cols):
                if i + num_cols <= len(data_parts):
                    row = data_parts[i:i+num_cols]
                    data_rows.append(row)
            
            df = pd.DataFrame(data_rows, columns=header)
            
            # Convert to numeric
            for col in df.columns:
                df[col] = pd.to_numeric(df[col], errors='coerce')
            
            df = df.dropna()
            return df
        else:
            # Normal multi-line CSV
            header = lines[0].strip().split(',')
            data_rows = []
            
            for line in lines[1:]:
                values = line.strip().split(',')
                if len(values) >= len(header):
                    data_rows.append(values[:len(header)])
            
            df = pd.DataFrame(data_rows, columns=header)
            
            for col in df.columns:
                df[col] = pd.to_numeric(df[col], errors='coerce')
            
            df = df.dropna()
            return df
            
    except Exception as e:
        print(f"Error loading {filepath}: {e}")
        return None

def parse_obstacles(obstacles_config):
    """Parse obstacles configuration from YAML-like format"""
    if not obstacles_config:
        return []
    
    if not YAML_AVAILABLE:
        print("Warning: PyYAML not installed. Install with 'pip install pyyaml' to use obstacles feature.")
        return []
    
    obstacles = []
    
    # Handle both file path and direct YAML string
    if isinstance(obstacles_config, str):
        if Path(obstacles_config).exists():
            # It's a file path
            with open(obstacles_config, 'r') as f:
                config = yaml.safe_load(f)
        else:
            # It's a YAML string
            config = yaml.safe_load(obstacles_config)
    else:
        config = obstacles_config
    
    # Handle both top-level obstacles and environment.obstacles
    obstacle_list = []
    if 'obstacles' in config:
        obstacle_list = config['obstacles']
    elif 'environment' in config and 'obstacles' in config['environment']:
        obstacle_list = config['environment']['obstacles']
    
    for obstacle in obstacle_list:
        center = obstacle.get('center', [0, 0])
        size = obstacle.get('size', [0.25, 0.25])
        obstacle_type = obstacle.get('type', 'box')
        color = obstacle.get('color', 'red')
        alpha = obstacle.get('alpha', 0.7)
        
        obstacles.append({
            'center': center,
            'size': size,
            'type': obstacle_type,
            'color': color,
            'alpha': alpha
        })
    
    return obstacles

def plot_obstacles(obstacles, ax):
    """Plot obstacles on the given axes"""
    for obstacle in obstacles:
        center = obstacle['center']
        size = obstacle['size']
        obstacle_type = obstacle['type']
        color = obstacle['color']
        alpha = obstacle['alpha']
        
        if obstacle_type == 'box':
            # Create rectangle patch
            # Rectangle uses bottom-left corner, so we need to offset by half the size
            x = center[0] - size[0] / 2
            y = center[1] - size[1] / 2
            
            rect = patches.Rectangle((x, y), size[0], size[1], 
                                   linewidth=1, edgecolor='black', 
                                   facecolor=color, alpha=alpha)
            ax.add_patch(rect)
        
        # Could add other shapes like circles in the future
        # elif obstacle_type == 'circle':
        #     circle = patches.Circle(center, size[0]/2, 
        #                           linewidth=1, edgecolor='black',
        #                           facecolor=color, alpha=alpha)
        #     ax.add_patch(circle)

def plot_multi_robot_trajectories(robot_data, title="Multi-Robot Trajectories", obstacles=None):
    """Plot trajectories for multiple robots"""
    
    # Colors and styles for up to 4 robots
    colors = ['blue', 'red', 'green', 'purple']
    line_styles = ['-', '--', '-.', ':']
    markers = ['o', 's', '^', 'D']
    robot_names = ['Robot 1', 'Robot 2', 'Robot 3', 'Robot 4']
    
    plt.figure(figsize=(12, 10))
    
    # Track bounds for equal aspect ratio
    all_x = []
    all_y = []
    
    for i, (trajectory_name, df) in enumerate(robot_data.items()):
        if i >= 4:  # Limit to 4 robots
            print(f"Warning: Only plotting first 4 robots, skipping {trajectory_name}")
            break
            
        color = colors[i]
        line_style = line_styles[i]
        marker = markers[i]
        robot_name = robot_names[i]
        
        # Check which columns are available
        has_target = 'target_x' in df.columns and 'target_y' in df.columns
        has_actual = 'actual_x' in df.columns and 'actual_y' in df.columns
        has_target_theta = 'target_theta' in df.columns
        has_actual_theta = 'actual_theta' in df.columns
        
        if has_target:
            # Plot target trajectory
            plt.plot(df['target_x'], df['target_y'], 
                    color=color, linestyle=line_style, linewidth=2.5,
                    label=f'{robot_name} (Target)', alpha=0.8)
            
            # Add start and end markers for target
            plt.plot(df['target_x'].iloc[0], df['target_y'].iloc[0], 
                    marker=marker, color=color, markersize=10, 
                    markerfacecolor='white', markeredgewidth=2)
            plt.plot(df['target_x'].iloc[-1], df['target_y'].iloc[-1], 
                    marker=marker, color=color, markersize=10, 
                    markerfacecolor=color, markeredgewidth=2)
            
            # Add orientation arrows for target trajectory
            if has_target_theta:
                # Plot orientation arrows for every 10th data point
                for j in range(0, len(df), 10):
                    x = df['target_x'].iloc[j]
                    y = df['target_y'].iloc[j]
                    theta = df['target_theta'].iloc[j]
                    
                    # Larger, more visible arrows
                    arrow_length = 0.08  # 8cm arrows (increased from 5cm)
                    dx = arrow_length * np.cos(theta)
                    dy = arrow_length * np.sin(theta)
                    
                    # Use darker, more contrasting colors
                    arrow_color = 'darkblue' if color == 'blue' else \
                                 'darkred' if color == 'red' else \
                                 'darkgreen' if color == 'green' else 'indigo'
                    
                    plt.arrow(x, y, dx, dy, head_width=0.035, head_length=0.02,
                             fc=arrow_color, ec=arrow_color, alpha=0.9, linewidth=1.5)
            
            all_x.extend(df['target_x'])
            all_y.extend(df['target_y'])
        
        if has_actual:
            # Plot actual trajectory with dotted style
            plt.plot(df['actual_x'], df['actual_y'], 
                    color=color, linestyle=':', linewidth=2,
                    label=f'{robot_name} (Actual)', alpha=0.6)
            
            # Add orientation arrows for actual trajectory
            if has_actual_theta:
                # Plot orientation arrows for every 10th data point
                for j in range(0, len(df), 10):
                    x = df['actual_x'].iloc[j]
                    y = df['actual_y'].iloc[j]
                    theta = df['actual_theta'].iloc[j]
                    
                    # Larger arrows for actual trajectory too
                    arrow_length = 0.07  # 7cm arrows (slightly smaller than target)
                    dx = arrow_length * np.cos(theta)
                    dy = arrow_length * np.sin(theta)
                    
                    # Use lighter but still visible colors for actual
                    arrow_color = 'navy' if color == 'blue' else \
                                 'crimson' if color == 'red' else \
                                 'forestgreen' if color == 'green' else 'mediumpurple'
                    
                    plt.arrow(x, y, dx, dy, head_width=0.03, head_length=0.015,
                             fc=arrow_color, ec=arrow_color, alpha=0.8, linewidth=1.2)
            
            all_x.extend(df['actual_x'])
            all_y.extend(df['actual_y'])
        
        if not has_target and not has_actual:
            print(f"Warning: No trajectory data found for {trajectory_name}")
    
    # Set labels and title
    plt.xlabel('X Position (m)', fontsize=12)
    plt.ylabel('Y Position (m)', fontsize=12)
    plt.title(title, fontsize=14, fontweight='bold')
    
    # Set equal aspect ratio and grid
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    
    # Plot obstacles if provided
    if obstacles:
        plot_obstacles(obstacles, plt.gca())
    
    # Add some padding to the plot
    if all_x and all_y:
        x_range = max(all_x) - min(all_x)
        y_range = max(all_y) - min(all_y)
        padding = max(x_range, y_range) * 0.1
        
        plt.xlim(min(all_x) - padding, max(all_x) + padding)
        plt.ylim(min(all_y) - padding, max(all_y) + padding)
    
    # Create custom legend entries
    legend_elements = []
    
    # Add robot trajectory legends
    for i, (trajectory_name, df) in enumerate(robot_data.items()):
        if i >= 4:
            break
        color = colors[i]
        line_style = line_styles[i]
        robot_name = robot_names[i]
        
        has_target = 'target_x' in df.columns and 'target_y' in df.columns
        has_actual = 'actual_x' in df.columns and 'actual_y' in df.columns
        
        if has_target:
            legend_elements.append(plt.Line2D([0], [0], color=color, linestyle=line_style, 
                                            linewidth=2.5, label=f'{robot_name} (Target)'))
        if has_actual:
            legend_elements.append(plt.Line2D([0], [0], color=color, linestyle=':', 
                                            linewidth=2, alpha=0.6, label=f'{robot_name} (Actual)'))
    
    # Add general legend items
    legend_elements.extend([
        plt.Line2D([0], [0], marker='o', color='gray', markersize=8, 
                  markerfacecolor='white', markeredgewidth=2, linestyle='None',
                  label='Start Position'),
        plt.Line2D([0], [0], marker='o', color='gray', markersize=8, 
                  markerfacecolor='gray', markeredgewidth=2, linestyle='None',
                  label='End Position'),
        plt.Line2D([0], [0], color='gray', linewidth=0, 
                  label='→ Robot Orientation')
    ])
    
    # Add legend with better positioning
    plt.legend(handles=legend_elements, bbox_to_anchor=(1.05, 1), 
              loc='upper left', fontsize=10)
    plt.tight_layout()
    
    return plt.gcf()

def find_trajectory_file(trajectory_name):
    """Find trajectory file with given name (with or without .csv extension)"""
    current_dir = Path.cwd()
    
    # Try different extensions and formats
    possible_names = [
        f"{trajectory_name}",
        f"{trajectory_name}.csv",
    ]
    
    for name in possible_names:
        file_path = current_dir / name
        if file_path.exists():
            return file_path
    
    return None

def main():
    parser = argparse.ArgumentParser(description='Visualize trajectories for up to 4 robots')
    
    parser.add_argument('trajectories', nargs='+',
                        help='Trajectory names (e.g., TR01 TR02 TR03). Will look for files with these names in current directory')
    
    parser.add_argument('-o', '--output', type=str,
                        help='Output filename for saved plot (e.g., multi_robot_plot.png)')
    
    parser.add_argument('--title', type=str, default='Multi-Robot Trajectories',
                        help='Plot title')
    
    parser.add_argument('--no-show', action='store_true',
                        help='Do not display plot (only save)')
    
    parser.add_argument('--obstacles', type=str,
                        help='Obstacles configuration (YAML file path or YAML string)')
    
    args = parser.parse_args()
    
    if len(args.trajectories) > 4:
        print(f"Warning: Only plotting first 4 trajectories out of {len(args.trajectories)} provided")
        args.trajectories = args.trajectories[:4]
    
    print(f"Looking for trajectory files: {', '.join(args.trajectories)}")
    
    # Find and load trajectory files
    robot_data = {}
    for trajectory_name in args.trajectories:
        file_path = find_trajectory_file(trajectory_name)
        
        if file_path is None:
            print(f"Error: Could not find file for trajectory '{trajectory_name}'")
            print(f"  Looked for: {trajectory_name}, {trajectory_name}.csv")
            continue
        
        print(f"Loading {trajectory_name} from {file_path}...")
        df = load_trajectory_data(file_path)
        
        if df is not None:
            # Convert timestamps if needed
            if 'ts' in df.columns and df['ts'].max() > 1000:
                df['ts'] = df['ts'] / 1000.0
            
            robot_data[trajectory_name] = df
            print(f"  Loaded {len(df)} points")
            
            # Show available columns for debugging
            has_target = 'target_x' in df.columns and 'target_y' in df.columns
            has_actual = 'actual_x' in df.columns and 'actual_y' in df.columns
            print(f"  Has target trajectory: {has_target}")
            print(f"  Has actual trajectory: {has_actual}")
        else:
            print(f"  Failed to load {trajectory_name}")
    
    if not robot_data:
        print("Error: No valid trajectory data loaded!")
        sys.exit(1)
    
    print(f"\nSuccessfully loaded {len(robot_data)} trajectories")
    
    # Parse obstacles if provided
    obstacles = []
    if args.obstacles:
        try:
            obstacles = parse_obstacles(args.obstacles)
            print(f"Loaded {len(obstacles)} obstacles")
        except Exception as e:
            print(f"Warning: Failed to parse obstacles: {e}")
    
    # Create plot
    fig = plot_multi_robot_trajectories(robot_data, args.title, obstacles=obstacles)
    
    # Save plot if requested
    if args.output:
        output_path = Path(args.output)
        fig.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Saved plot to {output_path}")
    
    # Show plot unless --no-show
    if not args.no_show:
        plt.show()

if __name__ == "__main__":
    main()
