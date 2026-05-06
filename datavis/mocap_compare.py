#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
from pathlib import Path

# Import the robust loading function from the existing visualization script
from visualize_trajectory_v2 import load_trajectory_data_v2

def load_mocap_data(filepath):
    """Load mocap data and synthesize timestamps based on 50Hz logging rate."""
    try:
        # Mocap data format: x,y,z,theta
        df = pd.read_csv(filepath)
        
        # Strip whitespace from column names just in case
        df.columns = [col.strip() for col in df.columns]
        
        # 50Hz = 0.02s per row
        df['ts'] = np.arange(len(df)) * 0.02
        return df
    except Exception as e:
        print(f"Error loading mocap data from {filepath}: {e}")
        return None

def plot_comparison(sd_df, mocap_df):
    """Plot the SD card data and Mocap data together."""
    fig, ax_main = plt.subplots(figsize=(12, 10))
    fig.suptitle('Trajectory Comparison: SD Card (EKF) vs Mocap (Ground Truth)', fontsize=18)
    
    available_cols = set(sd_df.columns)
    
    # Plot SD Target Path if available
    if 'target_x' in available_cols and 'target_y' in available_cols:
        ax_main.plot(sd_df['target_x'], sd_df['target_y'], 'b--', alpha=0.5, linewidth=2, label='SD Target Path')
        
    # Plot SD Actual (EKF)
    if 'actual_x' in available_cols and 'actual_y' in available_cols:
        ax_main.plot(sd_df['actual_x'], sd_df['actual_y'], 'y-', linewidth=2.5, label='SD Card (EKF Estimate)')
        
    # Plot Mocap (Ground Truth)
    ax_main.plot(mocap_df['x'], mocap_df['y'], 'g-', linewidth=2.5, label='Mocap (Ground Truth)')
    
    # Mark starting points
    if 'actual_x' in available_cols:
        ax_main.plot(sd_df['actual_x'].iloc[0], sd_df['actual_y'].iloc[0], 'yo', markersize=8, label='SD Start')
    ax_main.plot(mocap_df['x'].iloc[0], mocap_df['y'].iloc[0], 'go', markersize=8, label='Mocap Start')
    
    ax_main.set_xlabel('X Position (m)', fontsize=12)
    ax_main.set_ylabel('Y Position (m)', fontsize=12)
    ax_main.set_title('2D Path Comparison', fontsize=14)
    ax_main.grid(True)
    ax_main.legend(fontsize=12)
    ax_main.axis('equal')  # Crucial for 2D physical paths!

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Compare SD Card EKF logs with Mocap Ground Truth.')
    parser.add_argument('--sd', type=str, required=True, help='Path to the SD card CSV log file')
    parser.add_argument('--mocap', type=str, required=True, help='Path to the Mocap CSV log file')
    
    args = parser.parse_args()
    
    print(f"Loading SD log: {args.sd}")
    sd_df = load_trajectory_data_v2(args.sd)
    
    if sd_df is not None:
        # Force numeric conversion for all columns
        for col in sd_df.columns:
            sd_df[col] = pd.to_numeric(sd_df[col], errors='coerce')
        
        # Drop rows that couldn't be converted
        sd_df = sd_df.dropna(subset=[c for c in ['actual_x', 'actual_y'] if c in sd_df.columns])
        
        if not sd_df.empty:
            # Check if timestamps are in ms (like TrajControlLog struct) and convert to seconds
            # Use 'timestamp_ms' if 'ts' is missing
            time_col = 'ts' if 'ts' in sd_df.columns else ('timestamp_ms' if 'timestamp_ms' in sd_df.columns else None)
            
            if time_col:
                ts_max = sd_df[time_col].max()
                if not pd.isna(ts_max) and ts_max > 1000:
                    sd_df[time_col] = sd_df[time_col] / 1000.0
                    print(f"  Converted SD timestamps ({time_col}) from ms to seconds.")
                
                # Ensure we have a standard 'ts' column for plotting
                if time_col != 'ts':
                    sd_df['ts'] = sd_df[time_col]
        else:
            print("  Warning: SD dataframe is empty after numeric conversion.")
            sd_df = None
            
    print(f"Loading Mocap log: {args.mocap}")
    mocap_df = load_mocap_data(args.mocap)
    
    if sd_df is not None and mocap_df is not None:
        print("Both datasets loaded successfully. Generating plots...")
        plot_comparison(sd_df, mocap_df)
    else:
        print("\nERROR: Failed to load one or both datasets. Please check the file paths.")
