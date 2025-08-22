#!/usr/bin/env python3
"""
Script to display and visualize data from robotics sensor log files.
This script reads CSV data from robotics sensor logs and creates visualizations.

Usage: python3 display_data.py --TR00_final.csv
       python3 display_data.py --TR02_final.csv
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from pathlib import Path

def load_data(filename):
    """Load CSV data from the specified file."""
    try:
        data = pd.read_csv(filename)
        print(f"Successfully loaded {filename}")
        print(f"Data shape: {data.shape}")
        print(f"Columns: {list(data.columns)}")
        return data
    except Exception as e:
        print(f"Error loading {filename}: {e}")
        return None

def display_data_summary(data, filename):
    """Display summary statistics of the data."""
    print(f"\n{'='*50}")
    print(f"DATA SUMMARY FOR {filename}")
    print(f"{'='*50}")
    
    print(f"Number of records: {len(data)}")
    print(f"Time range: {data['ts'].min()} to {data['ts'].max()} ms")
    print(f"Duration: {(data['ts'].max() - data['ts'].min()) / 1000:.2f} seconds")
    
    print(f"\nTarget position range:")
    print(f"  x: {data['target_vx'].min():.3f} to {data['target_vx'].max():.3f}")
    print(f"  y: {data['target_vy'].min():.3f} to {data['target_vy'].max():.3f}")
    
    print(f"\nActual position range:")
    print(f"  x: {data['actual_vx'].min():.3f} to {data['actual_vx'].max():.3f}")
    print(f"  y: {data['actual_vy'].min():.3f} to {data['actual_vy'].max():.3f}")
    
    print(f"\nAngle range (degrees):")
    print(f"  roll: {np.degrees(data['roll']).min():.1f} to {np.degrees(data['roll']).max():.1f}")
    print(f"  pitch: {np.degrees(data['pitch']).min():.1f} to {np.degrees(data['pitch']).max():.1f}")
    print(f"  yaw (actual): {np.degrees(data['yaw']).min():.1f} to {np.degrees(data['yaw']).max():.1f}")
    
    # Add desired angle information if available
    if 'target_qw' in data.columns and not data['target_qw'].isna().all():
        print(f"  yaw (desired): {np.degrees(data['target_qw']).min():.1f} to {np.degrees(data['target_qw']).max():.1f}")
    
    # Calculate and display velocity information
    if 'target_vx' in data.columns and 'target_vy' in data.columns:
        target_speed = np.sqrt(data['target_vx']**2 + data['target_vy']**2)
        actual_speed = np.sqrt(data['actual_vx']**2 + data['actual_vy']**2)
        print(f"\nSpeed range (m/s):")
        print(f"  target speed: {target_speed.min():.3f} to {target_speed.max():.3f}")
        print(f"  actual speed: {actual_speed.min():.3f} to {actual_speed.max():.3f}")

def plot_single_trajectory(data, name, color='blue'):
    """Create comprehensive trajectory and position comparison plots for a single dataset."""
    time_s = data['ts'] / 1000.0  # Convert to seconds
    
    # Create figure with 6 subplots (2x3 grid)
    fig, axes = plt.subplots(2, 3, figsize=(24, 12))
    fig.suptitle(f'{name} Robotics Data Analysis - Target vs Actual Comparison', fontsize=16, fontweight='bold')
    
    # Plot 1: XY Trajectory Comparison with Velocity Arrows
    ax1 = axes[0, 0]
    ax1.plot(data['target_vx'], data['target_vy'], 'r--', label='Target Trajectory', linewidth=2, marker='o', markersize=3)
    ax1.plot(data['actual_vx'], data['actual_vy'], 'b-', label='Actual Trajectory', linewidth=2, marker='s', markersize=3)
    
    # Add quiver arrows showing velocity magnitude and direction
    # Calculate velocity components (assuming these are position derivatives)
    # Skip some points for clarity (every 5th point)
    skip = max(1, len(data) // 20)  # Show about 20 arrows
    
    # Calculate velocity from position differences
    target_vx_calc = np.gradient(data['target_vx'])
    target_vy_calc = np.gradient(data['target_vy'])
    actual_vx_calc = np.gradient(data['actual_vx'])
    actual_vy_calc = np.gradient(data['actual_vy'])
    
    # Calculate speeds for scaling
    target_speed = np.sqrt(target_vx_calc**2 + target_vy_calc**2)
    actual_speed = np.sqrt(actual_vx_calc**2 + actual_vy_calc**2)
    
    # Normalize and scale arrows
    scale_factor = 0.05  # Adjust this to make arrows visible but not overwhelming
    
    # Target trajectory arrows (red)
    ax1.quiver(data['target_vx'][::skip], data['target_vy'][::skip], 
               target_vx_calc[::skip], target_vy_calc[::skip],
               target_speed[::skip], scale=1/scale_factor, scale_units='xy', angles='xy',
               cmap='Reds', alpha=0.7, width=0.003, label='Target Velocity')
    
    # Actual trajectory arrows (blue)
    ax1.quiver(data['actual_vx'][::skip], data['actual_vy'][::skip], 
               actual_vx_calc[::skip], actual_vy_calc[::skip],
               actual_speed[::skip], scale=1/scale_factor, scale_units='xy', angles='xy',
               cmap='Blues', alpha=0.7, width=0.003, label='Actual Velocity')
    
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('XY Position: Target vs Actual')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.grid(True, which='minor', alpha=0.1)
    ax1.minorticks_on()
    # Set grid spacing to 0.1 meters
    from matplotlib.ticker import MultipleLocator
    ax1.xaxis.set_major_locator(MultipleLocator(0.1))
    ax1.yaxis.set_major_locator(MultipleLocator(0.1))
    ax1.xaxis.set_minor_locator(MultipleLocator(0.02))
    ax1.yaxis.set_minor_locator(MultipleLocator(0.02))
    ax1.axis('equal')
    
    # Plot 2: Position X Comparison over Time
    ax2 = axes[0, 1]
    ax2.plot(time_s, data['target_vx'], 'r--', label='Target X', linewidth=2)
    ax2.plot(time_s, data['actual_vx'], 'b-', label='Actual X', linewidth=2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Position X (m)')
    ax2.set_title('Position X over Time')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Position Y Comparison over Time
    ax3 = axes[1, 0]
    ax3.plot(time_s, data['target_vy'], 'r--', label='Target Y', linewidth=2)
    ax3.plot(time_s, data['actual_vy'], 'g-', label='Actual Y', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Position Y (m)')
    ax3.set_title('Position Y over Time')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Orientation (Yaw) over Time - Both Desired and Actual
    ax4 = axes[1, 1]
    ax4.plot(time_s, np.degrees(data['yaw']), 'b-', label='Actual Yaw', linewidth=2, marker='o', markersize=2)
    
    # Add desired orientation if available
    if 'target_qw' in data.columns and not data['target_qw'].isna().all():
        ax4.plot(time_s, np.degrees(data['target_qw']), 'r--', label='Desired Yaw', linewidth=2)
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Yaw Angle (degrees)')
    ax4.set_title('Robot Orientation over Time')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    # Plot 5: Desired vs Actual Orientation Comparison
    ax5 = axes[0, 2]
    # Check if we have target orientation data
    if 'target_qw' in data.columns and not data['target_qw'].isna().all():
        # Target orientation is stored in target_qw
        ax5.plot(time_s, np.degrees(data['target_qw']), 'r--', label='Desired Orientation', linewidth=2)
        target_orientation = data['target_qw']
    else:
        # Generate expected circle trajectory orientation for comparison
        # For a circle trajectory, orientation should follow the tangent
        angular_velocity = 2*np.pi / (time_s[-1] - time_s[0])  # Estimate from total time
        expected_orientation = angular_velocity * time_s
        ax5.plot(time_s, np.degrees(expected_orientation), 'r--', label='Expected Orientation (Circle)', linewidth=2)
        target_orientation = expected_orientation
    
    ax5.plot(time_s, np.degrees(data['yaw']), 'b-', label='Actual Orientation', linewidth=2)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Orientation (degrees)')
    ax5.set_title('Desired vs Actual Orientation')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    
    # Plot 6: Orientation Error over Time
    ax6 = axes[1, 2]
    # Calculate orientation error
    orientation_error = target_orientation - data['yaw']
    
    # Wrap angle error to [-pi, pi]
    orientation_error = np.arctan2(np.sin(orientation_error), np.cos(orientation_error))
    
    ax6.plot(time_s, np.degrees(orientation_error), 'red', label='Orientation Error', linewidth=2)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Orientation Error (degrees)')
    ax6.set_title('Orientation Tracking Error')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    ax6.axhline(y=0, color='black', linestyle='-', alpha=0.5)
    
    # Add RMS error annotation
    rms_error = np.sqrt(np.mean(orientation_error**2))
    ax6.text(0.02, 0.98, f'RMS Error: {np.degrees(rms_error):.2f}°', 
             transform=ax6.transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    return fig

def main():
    """Main function to load and display data for a single file."""
    # Parse command line arguments
    if len(sys.argv) != 2 or not sys.argv[1].startswith('--'):
        print("Usage: python3 display_data.py --<filename>")
        print("Example: python3 display_data.py --TR00_final.csv")
        print("         python3 display_data.py --TR02_final.csv")
        sys.exit(1)
    
    # Extract filename from command line argument
    filename = sys.argv[1][2:]  # Remove the '--' prefix
    script_dir = Path(__file__).parent
    file_path = script_dir / filename
    
    # Check if file exists
    if not file_path.exists():
        print(f"Error: File '{filename}' not found in {script_dir}")
        sys.exit(1)
    
    print(f"Loading robotics sensor data from {filename}...")
    print("=" * 50)
    
    # Load data
    data = load_data(file_path)
    
    if data is None:
        print(f"Failed to load data from {filename}!")
        return
    
    # Extract dataset name from filename (e.g., TR00_final.csv -> TR00)
    dataset_name = filename.replace('_final.csv', '').replace('.csv', '')
    
    # Display summary
    display_data_summary(data, dataset_name)
    
    # Create visualizations
    print(f"\n{'='*50}")
    print("CREATING VISUALIZATIONS")
    print(f"{'='*50}")
    
    print(f"Creating {dataset_name} trajectory plots...")
    fig = plot_single_trajectory(data, dataset_name)
    
    # Save plot
    print("Saving plot...")
    output_filename = f"{dataset_name}_trajectory.png"
    fig.savefig(script_dir / output_filename, dpi=300, bbox_inches='tight')
    
    print(f"Plot saved as {output_filename} in the current directory.")
    print("\nDisplaying plot...")
    plt.show()

if __name__ == "__main__":
    main()
