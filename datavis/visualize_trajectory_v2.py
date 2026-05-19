import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import os
import argparse
from pathlib import Path

def load_trajectory_data_v2(filepath):
    """Load and parse t    
    # Plot 4: Theta vs Time with error bars
    ax4 = axes[1,0]
    if 'ts' in available_cols:
        if 'target_theta' in available_cols:
            ax4.plot(df['ts'], np.degrees(df['target_theta']), 'b-', label='Target θ', linewidth=2)
        if 'actual_theta' in available_cols:
            ax4.plot(df['ts'], np.degrees(df['actual_theta']), 'r--', label='Actual θ', linewidth=2)
        
        # Add theta error as shaded region
        if 'target_theta' in available_cols and 'theta_error' in available_cols:
            # Show error bounds around target (convert to degrees)
            theta_target_deg = np.degrees(df['target_theta'])
            theta_error_deg = np.degrees(np.abs(df['theta_error']))
            theta_upper = theta_target_deg + theta_error_deg
            theta_lower = theta_target_deg - theta_error_deg
            ax4.fill_between(df['ts'], theta_lower, theta_upper, alpha=0.3, color='purple', label='θ Error Band')
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Orientation (degrees)')
    ax4.set_title('Orientation vs Time')
    ax4.legend()
    ax4.grid(True)ata from CSV (new format with positions)"""
    try:
        # First try standard pandas read
        try:
            df = pd.read_csv(filepath)
            if len(df) > 1:  # If we got multiple rows, we're good
                df = df.dropna()
                return df
        except:
            pass
        
        # Handle malformed CSV - all data on one line
        with open(filepath, 'r') as f:
            content = f.read().strip()
        
        # Check if it's all on one line (malformed)
        lines = content.split('\n')
        if len(lines) <= 2:  # Header + one data line or just one line
            print(f"Detected malformed CSV (all on one line), attempting to fix...")
            
            # Split by comma and try to reconstruct proper CSV structure
            parts = content.split(',')
            
            # Find header (first part should be "ts")
            if parts[0] != 'ts':
                print(f"Warning: Expected 'ts' as first column, got '{parts[0]}'")
                return None
            
            # Count columns in header by finding the first numeric value (timestamp)
            header_end = 0
            for i, part in enumerate(parts):
                try:
                    float(part)
                    header_end = i
                    break
                except ValueError:
                    continue
            
            if header_end == 0:
                print("Could not find start of data")
                return None
            
            header = parts[:header_end]
            data_parts = parts[header_end:]
            
            # Clean up the header - remove any trailing newlines or extra characters
            header = [col.strip() for col in header]
            
            # Fix the last column name if it has extra characters
            if len(header) > 0:
                last_col = header[-1]
                if '\n' in last_col or any(c.isdigit() for c in last_col):
                    # Extract just the column name part
                    clean_name = ''.join(c for c in last_col if c.isalpha() or c == '_')
                    if clean_name:
                        header[-1] = clean_name
            
            print(f"Found {len(header)} columns: {header}")
            
            # Reconstruct rows
            data_rows = []
            num_cols = len(header)
            
            for i in range(0, len(data_parts), num_cols):
                if i + num_cols <= len(data_parts):
                    row = data_parts[i:i+num_cols]
                    data_rows.append(row)
            
            print(f"Reconstructed {len(data_rows)} data rows")
            
            # Create DataFrame
            df = pd.DataFrame(data_rows, columns=header)
            
            # Convert to numeric
            for col in df.columns:
                df[col] = pd.to_numeric(df[col], errors='coerce')
            
            # Drop rows with NaN values
            df = df.dropna()
            
            return df
        
        else:
            # Normal multi-line CSV processing
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

def plot_trajectory_comparison_v2(df, title="Trajectory Following"):
    """Plot target vs actual trajectory (new format)"""
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle(title, fontsize=16)
    
    # Check which columns are available
    available_cols = set(df.columns)
    print(f"Available columns: {list(available_cols)}")
    
    # Calculate position error magnitude for color mapping
    pos_error = None
    if 'xerror' in available_cols and 'yerror' in available_cols:
        pos_error = np.sqrt(df['xerror']**2 + df['yerror']**2)
    
    # Plot 1: 2D Trajectory (without error coloring)
    if 'target_x' in available_cols and 'target_y' in available_cols:
        axes[0,0].plot(df['target_x'], df['target_y'], 'b-', label='Target', linewidth=2)
    if 'actual_x' in available_cols and 'actual_y' in available_cols:
        axes[0,0].plot(df['actual_x'], df['actual_y'], 'r--', label='Actual', linewidth=2)
    axes[0,0].set_xlabel('X Position (m)')
    axes[0,0].set_ylabel('Y Position (m)')
    axes[0,0].set_title('2D Trajectory')
    axes[0,0].legend()
    axes[0,0].grid(True)
    axes[0,0].axis('equal')
    
    # Add start and end markers
    if 'target_x' in available_cols and 'target_y' in available_cols:
        axes[0,0].plot(df['target_x'].iloc[0], df['target_y'].iloc[0], 'bo', markersize=8, label='Start')
        axes[0,0].plot(df['target_x'].iloc[-1], df['target_y'].iloc[-1], 'bs', markersize=8, label='End')
    if 'actual_x' in available_cols and 'actual_y' in available_cols:
        axes[0,0].plot(df['actual_x'].iloc[0], df['actual_y'].iloc[0], 'ro', markersize=8)
        axes[0,0].plot(df['actual_x'].iloc[-1], df['actual_y'].iloc[-1], 'rs', markersize=8)
    
    # Plot 2: X Position vs Time with error line
    ax2 = axes[0,1]
    if 'ts' in available_cols:
        if 'target_x' in available_cols:
            ax2.plot(df['ts'], df['target_x'], 'b-', label='Target X', linewidth=2)
        if 'actual_x' in available_cols:
            ax2.plot(df['ts'], df['actual_x'], 'r--', label='Actual X', linewidth=2)
        
        # Add X error as a separate line
        if 'xerror' in available_cols:
            ax2_twin = ax2.twinx()
            ax2_twin.plot(df['ts'], df['xerror'], 'orange', label='X Error', linewidth=1, alpha=0.8)
            ax2_twin.set_ylabel('X Error (m)', color='orange')
            ax2_twin.tick_params(axis='y', labelcolor='orange')
    
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('X Position (m)')
    ax2.set_title('X Position vs Time')
    ax2.legend(loc='upper left')
    ax2.grid(True)
    
    # Plot 3: Y Position vs Time with error line
    ax3 = axes[0,2]
    if 'ts' in available_cols:
        if 'target_y' in available_cols:
            ax3.plot(df['ts'], df['target_y'], 'b-', label='Target Y', linewidth=2)
        if 'actual_y' in available_cols:
            ax3.plot(df['ts'], df['actual_y'], 'r--', label='Actual Y', linewidth=2)
        
        # Add Y error as a separate line
        if 'yerror' in available_cols:
            ax3_twin = ax3.twinx()
            ax3_twin.plot(df['ts'], df['yerror'], 'orange', label='Y Error', linewidth=1, alpha=0.8)
            ax3_twin.set_ylabel('Y Error (m)', color='orange')
            ax3_twin.tick_params(axis='y', labelcolor='orange')
    
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Y Position (m)')
    ax3.set_title('Y Position vs Time')
    ax3.legend(loc='upper left')
    ax3.grid(True)
    
    # Plot 4: Theta vs Time with error line
    ax4 = axes[1,0]
    if 'ts' in available_cols:
        if 'target_theta' in available_cols:
            ax4.plot(df['ts'], np.rad2deg(df['target_theta']), 'b-', label='Target θ', linewidth=2)
        if 'actual_theta' in available_cols:
            ax4.plot(df['ts'], np.rad2deg(df['actual_theta']), 'r--', label='Actual θ', linewidth=2)
        
        # Add theta error as a separate line
        if 'thetaerror' in available_cols:
            ax4_twin = ax4.twinx()
            ax4_twin.plot(df['ts'], np.rad2deg(df['thetaerror']), 'purple', label='θ Error', linewidth=1, alpha=0.8)
            ax4_twin.set_ylabel('θ Error (deg)', color='purple')
            ax4_twin.tick_params(axis='y', labelcolor='purple')
    
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Orientation (degrees)')
    ax4.set_title('Orientation vs Time')
    ax4.legend(loc='upper left')
    ax4.grid(True)
    
    # Plot 5: Tracking Errors
    if 'ts' in available_cols:
        if 'xerror' in available_cols:
            axes[1,1].plot(df['ts'], df['xerror'], 'r-', label='X Error', linewidth=2)
        if 'yerror' in available_cols:
            axes[1,1].plot(df['ts'], df['yerror'], 'g-', label='Y Error', linewidth=2)
        if 'thetaerror' in available_cols:
            axes[1,1].plot(df['ts'], df['thetaerror'], 'b-', label='θ Error', linewidth=2)
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('Error')
    axes[1,1].set_title('Tracking Errors vs Time')
    axes[1,1].legend()
    axes[1,1].grid(True)
    
    # Plot 6: Motor Commands
    ax6 = axes[1,2]
    has_motor_commands = False
    has_duty_commands = False
    
    if 'ts' in available_cols:
        # Plot motor commands (rad/s)
        if 'ul' in available_cols:
            ax6.plot(df['ts'], df['ul'], 'g-', label='Left Motor (ul)', linewidth=2)
            has_motor_commands = True
        if 'ur' in available_cols:
            ax6.plot(df['ts'], df['ur'], 'orange', label='Right Motor (ur)', linewidth=2)
            has_motor_commands = True
        else:
            # Check for alternative column names
            ur_candidates = [col for col in available_cols if 'ur' in col.lower()]
            if ur_candidates:
                print(f"Using '{ur_candidates[0]}' for right motor instead of 'ur'")
                ax6.plot(df['ts'], df[ur_candidates[0]], 'orange', label=f'Right Motor ({ur_candidates[0]})', linewidth=2)
                has_motor_commands = True
        
        # Plot duty cycle commands (normalized) on secondary y-axis if available
        if 'dutyl' in available_cols or 'dutyr' in available_cols:
            ax6_twin = ax6.twinx()
            if 'dutyl' in available_cols:
                ax6_twin.plot(df['ts'], df['dutyl'], 'c--', label='Left Duty', linewidth=2, alpha=0.8)
                has_duty_commands = True
            if 'dutyr' in available_cols:
                ax6_twin.plot(df['ts'], df['dutyr'], 'm--', label='Right Duty', linewidth=2, alpha=0.8)
                has_duty_commands = True
            
            ax6_twin.set_ylabel('Duty Cycle (normalized)', color='cyan')
            ax6_twin.tick_params(axis='y', labelcolor='cyan')
            ax6_twin.set_ylim([-1.1, 1.1])  # Duty cycle typically ranges from -1 to 1
    
    ax6.set_xlabel('Time (s)')
    
    if has_motor_commands and has_duty_commands:
        ax6.set_ylabel('Motor Command (rad/s)')
        ax6.set_title('Motor Commands vs Time')
        # Combine legends from both axes
        lines1, labels1 = ax6.get_legend_handles_labels()
        lines2, labels2 = ax6_twin.get_legend_handles_labels()
        ax6.legend(lines1 + lines2, labels1 + labels2, loc='upper left')
    elif has_motor_commands:
        ax6.set_ylabel('Motor Command (rad/s)')
        ax6.set_title('Motor Commands vs Time')
        ax6.legend(loc='upper left')
    elif has_duty_commands:
        ax6.set_ylabel('Duty Cycle (normalized)')
        ax6.set_title('Motor Duty Cycles vs Time')
        ax6.legend(loc='upper left')
    else:
        ax6.set_ylabel('Motor Command')
        ax6.set_title('Motor Commands vs Time')
        ax6.text(0.5, 0.5, 'No motor command data available', 
                transform=ax6.transAxes, ha='center', va='center')
    
    ax6.grid(True)
    
    plt.tight_layout()
    return fig

def analyze_trajectory_performance_v2(df, is_circle=False, is_bezier=False):
    """Analyze trajectory following performance (new format)"""
    # Calculate position errors (already in the data)
    pos_error = np.sqrt(df['xerror']**2 + df['yerror']**2)
    
    # Calculate velocity errors
    vel_error_x = df['target_vx'] - df['actual_vx'] 
    vel_error_y = df['target_vy'] - df['actual_vy']
    vel_error_mag = np.sqrt(vel_error_x**2 + vel_error_y**2)
    
    # Calculate orientation error
    theta_error = np.abs(df['thetaerror'])
    
    # Calculate basic statistics
    stats = {
        'mean_pos_error': np.mean(pos_error),
        'max_pos_error': np.max(pos_error),
        'rms_pos_error': np.sqrt(np.mean(pos_error**2)),
        'mean_vel_error': np.mean(vel_error_mag),
        'max_vel_error': np.max(vel_error_mag),
        'rms_vel_error': np.sqrt(np.mean(vel_error_mag**2)),
        'mean_theta_error': np.mean(theta_error),
        'max_theta_error': np.max(theta_error),
        'rms_theta_error': np.sqrt(np.mean(theta_error**2))
    }
    
    # Add circle-specific analysis if requested
    if is_circle and 'target_x' in df.columns and 'target_y' in df.columns:
        # Calculate target circle parameters
        target_center_x = np.mean(df['target_x'])
        target_center_y = np.mean(df['target_y'])
        target_radius = np.sqrt((df['target_x'] - target_center_x)**2 + (df['target_y'] - target_center_y)**2)
        desired_radius = np.mean(target_radius)
        
        # Calculate actual circle parameters
        actual_center_x = np.mean(df['actual_x'])
        actual_center_y = np.mean(df['actual_y'])
        actual_radius = np.sqrt((df['actual_x'] - actual_center_x)**2 + (df['actual_y'] - actual_center_y)**2)
        
        # Calculate angular velocity (target)
        target_angle = np.arctan2(df['target_y'] - target_center_y, df['target_x'] - target_center_x)
        target_angle = np.unwrap(target_angle)
        
        # Calculate angular velocity from angle change
        dt = np.diff(df['ts'])
        dtheta = np.diff(target_angle)
        angular_velocity = dtheta / dt
        mean_angular_velocity = np.mean(angular_velocity)
        
        # Calculate linear speed from radius and angular velocity
        estimated_linear_speed = desired_radius * np.abs(mean_angular_velocity)
        
        # Add circle statistics
        stats.update({
            'circle_center_x': target_center_x,
            'circle_center_y': target_center_y,
            'desired_radius': desired_radius,
            'actual_radius_mean': np.mean(actual_radius),
            'radius_error_mean': np.mean(np.abs(actual_radius - target_radius)),
            'radius_error_rms': np.sqrt(np.mean((actual_radius - target_radius)**2)),
            'angular_velocity': mean_angular_velocity,
            'estimated_linear_speed': estimated_linear_speed,
            'circle_completion_time': df['ts'].iloc[-1] - df['ts'].iloc[0],
            'total_angle_traversed': target_angle[-1] - target_angle[0]
        })
    
    # Add Bezier-specific analysis if requested
    if is_bezier and 'target_x' in df.columns and 'target_y' in df.columns:
        # Calculate curve length
        dx = np.diff(df['target_x'])
        dy = np.diff(df['target_y'])
        segment_lengths = np.sqrt(dx**2 + dy**2)
        curve_length = np.sum(segment_lengths)
        
        # Calculate curvature statistics
        if len(df) > 2:
            # Approximate curvature using finite differences
            x = df['target_x'].values
            y = df['target_y'].values
            dx_dt = np.gradient(x)
            dy_dt = np.gradient(y)
            d2x_dt2 = np.gradient(dx_dt)
            d2y_dt2 = np.gradient(dy_dt)
            
            # Curvature formula: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
            numerator = np.abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2)
            denominator = (dx_dt**2 + dy_dt**2)**(3/2)
            curvature = np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator!=0)
            
            max_curvature = np.max(curvature[~np.isnan(curvature)])
            mean_curvature = np.mean(curvature[~np.isnan(curvature)])
        else:
            max_curvature = 0.0
            mean_curvature = 0.0
        
        # Calculate trajectory bounds
        x_range = np.max(df['target_x']) - np.min(df['target_x'])
        y_range = np.max(df['target_y']) - np.min(df['target_y'])
        
        # Add Bezier statistics
        stats.update({
            'curve_length': curve_length,
            'max_curvature': max_curvature,
            'mean_curvature': mean_curvature,
            'x_range': x_range,
            'y_range': y_range,
            'bezier_completion_time': df['ts'].iloc[-1] - df['ts'].iloc[0],
            'start_point': (df['target_x'].iloc[0], df['target_y'].iloc[0]),
            'end_point': (df['target_x'].iloc[-1], df['target_y'].iloc[-1])
        })
    
    return stats, pos_error, vel_error_mag, theta_error

def estimate_bezier_control_points(df):
    """Estimate Bezier control points from trajectory data"""
    if len(df) < 4:
        return None
    
    # Extract start and end points
    p0 = (df['target_x'].iloc[0], df['target_y'].iloc[0])
    p3 = (df['target_x'].iloc[-1], df['target_y'].iloc[-1])
    
    # Use derivatives at start and end to estimate control points
    # Simple approximation: use 1/3 of the way along initial and final tangent directions
    
    # Estimate initial tangent
    if len(df) > 1:
        dx_start = df['target_x'].iloc[1] - df['target_x'].iloc[0]
        dy_start = df['target_y'].iloc[1] - df['target_y'].iloc[0]
        tangent_length_start = np.sqrt(dx_start**2 + dy_start**2) * 3
        
        # Estimate final tangent
        dx_end = df['target_x'].iloc[-1] - df['target_x'].iloc[-2]
        dy_end = df['target_y'].iloc[-1] - df['target_y'].iloc[-2]
        tangent_length_end = np.sqrt(dx_end**2 + dy_end**2) * 3
        
        # Control points
        p1 = (p0[0] + dx_start/np.sqrt(dx_start**2 + dy_start**2) * tangent_length_start,
              p0[1] + dy_start/np.sqrt(dx_start**2 + dy_start**2) * tangent_length_start)
        p2 = (p3[0] - dx_end/np.sqrt(dx_end**2 + dy_end**2) * tangent_length_end,
              p3[1] - dy_end/np.sqrt(dx_end**2 + dy_end**2) * tangent_length_end)
    else:
        # Fallback: simple linear interpolation
        p1 = (p0[0] + (p3[0] - p0[0])/3, p0[1] + (p3[1] - p0[1])/3)
        p2 = (p0[0] + 2*(p3[0] - p0[0])/3, p0[1] + 2*(p3[1] - p0[1])/3)
    
    return {'p0': p0, 'p1': p1, 'p2': p2, 'p3': p3}

def plot_bezier_analysis(df, title="Bezier Trajectory Analysis"):
    """Special analysis for Bezier trajectories"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(title, fontsize=16)
    
    # Estimate control points
    control_points = estimate_bezier_control_points(df)
    
    # Plot 1: 2D trajectory with control points
    axes[0,0].plot(df['target_x'], df['target_y'], 'b-', label='Target Trajectory', linewidth=2)
    axes[0,0].plot(df['actual_x'], df['actual_y'], 'r--', label='Actual Trajectory', linewidth=2)
    
    if control_points:
        # Plot control points and control polygon
        cp_x = [control_points['p0'][0], control_points['p1'][0], control_points['p2'][0], control_points['p3'][0]]
        cp_y = [control_points['p0'][1], control_points['p1'][1], control_points['p2'][1], control_points['p3'][1]]
        
        axes[0,0].plot(cp_x, cp_y, 'go--', label='Control Polygon', alpha=0.7, markersize=8)
        
        # Label control points
        for i, (x, y) in enumerate([(control_points['p0'][0], control_points['p0'][1]),
                                    (control_points['p1'][0], control_points['p1'][1]),
                                    (control_points['p2'][0], control_points['p2'][1]),
                                    (control_points['p3'][0], control_points['p3'][1])]):
            axes[0,0].annotate(f'P{i}', (x, y), xytext=(5, 5), textcoords='offset points', fontsize=10)
    
    axes[0,0].set_xlabel('X Position (m)')
    axes[0,0].set_ylabel('Y Position (m)')
    axes[0,0].set_title('Bezier Curve with Control Points')
    axes[0,0].legend()
    axes[0,0].grid(True)
    axes[0,0].axis('equal')
    
    # Plot 2: Position Error Magnitude
    pos_error = np.sqrt(df['xerror']**2 + df['yerror']**2)
    axes[0,1].plot(df['ts'], pos_error, 'r-', linewidth=2)
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('Position Error (m)')
    axes[0,1].set_title(f'Position Error (mean: {np.mean(pos_error):.4f}m)')
    axes[0,1].grid(True)
    
    # Plot 3: Velocity profile
    target_vel = np.sqrt(df['target_vx']**2 + df['target_vy']**2)
    actual_vel = np.sqrt(df['actual_vx']**2 + df['actual_vy']**2)
    
    axes[1,0].plot(df['ts'], target_vel, 'b-', label='Target Velocity', linewidth=2)
    axes[1,0].plot(df['ts'], actual_vel, 'r--', label='Actual Velocity', linewidth=2)
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('Velocity (m/s)')
    axes[1,0].set_title('Velocity Profile')
    axes[1,0].legend()
    axes[1,0].grid(True)
    
    # Plot 4: Curvature analysis (approximate)
    if len(df) > 2:
        x = df['target_x'].values
        y = df['target_y'].values
        dx_dt = np.gradient(x)
        dy_dt = np.gradient(y)
        d2x_dt2 = np.gradient(dx_dt)
        d2y_dt2 = np.gradient(dy_dt)
        
        # Curvature formula
        numerator = np.abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2)
        denominator = (dx_dt**2 + dy_dt**2)**(3/2)
        curvature = np.divide(numerator, denominator, out=np.zeros_like(numerator), where=denominator!=0)
        
        # Filter out extreme values for better visualization
        curvature_filtered = curvature[~np.isnan(curvature)]
        if len(curvature_filtered) > 0:
            max_reasonable = np.percentile(curvature_filtered, 95)
            curvature = np.clip(curvature, 0, max_reasonable)
        
        axes[1,1].plot(df['ts'], curvature, 'purple', linewidth=2)
        axes[1,1].set_xlabel('Time (s)')
        axes[1,1].set_ylabel('Curvature (1/m)')
        axes[1,1].set_title('Trajectory Curvature')
        axes[1,1].grid(True)
    else:
        axes[1,1].text(0.5, 0.5, 'Not enough points\nfor curvature calculation', 
                      transform=axes[1,1].transAxes, ha='center', va='center')
        axes[1,1].set_title('Trajectory Curvature')
    
    plt.tight_layout()
    return fig

def plot_circle_analysis(df, title="Circle Trajectory Analysis"):
    """Special analysis for circle trajectories"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(title, fontsize=16)
    
    # Calculate radius from center for target trajectory
    target_center_x = np.mean(df['target_x'])
    target_center_y = np.mean(df['target_y'])
    target_radius = np.sqrt((df['target_x'] - target_center_x)**2 + (df['target_y'] - target_center_y)**2)
    
    # Calculate radius from center for actual trajectory
    actual_center_x = np.mean(df['actual_x'])
    actual_center_y = np.mean(df['actual_y'])
    actual_radius = np.sqrt((df['actual_x'] - actual_center_x)**2 + (df['actual_y'] - actual_center_y)**2)
    
    # Plot 1: Radius vs Time
    axes[0,0].plot(df['ts'], target_radius, 'b-', label=f'Target (mean: {np.mean(target_radius):.3f}m)', linewidth=2)
    axes[0,0].plot(df['ts'], actual_radius, 'r--', label=f'Actual (mean: {np.mean(actual_radius):.3f}m)', linewidth=2)
    axes[0,0].set_xlabel('Time (s)')
    axes[0,0].set_ylabel('Radius (m)')
    axes[0,0].set_title('Circle Radius vs Time')
    axes[0,0].legend()
    axes[0,0].grid(True)
    
    # Plot 2: Position Error Magnitude
    pos_error = np.sqrt(df['xerror']**2 + df['yerror']**2)
    axes[0,1].plot(df['ts'], pos_error, 'r-', linewidth=2)
    axes[0,1].set_xlabel('Time (s)')
    axes[0,1].set_ylabel('Position Error (m)')
    axes[0,1].set_title(f'Position Error (mean: {np.mean(pos_error):.4f}m)')
    axes[0,1].grid(True)
    
    # Plot 3: Angular velocity analysis
    # Calculate angular position
    target_angle = np.arctan2(df['target_y'] - target_center_y, df['target_x'] - target_center_x)
    actual_angle = np.arctan2(df['actual_y'] - actual_center_y, df['actual_x'] - actual_center_x)
    
    # Unwrap angles to handle 2π discontinuities
    target_angle = np.unwrap(target_angle)
    actual_angle = np.unwrap(actual_angle)
    
    axes[1,0].plot(df['ts'], target_angle, 'b-', label='Target', linewidth=2)
    axes[1,0].plot(df['ts'], actual_angle, 'r--', label='Actual', linewidth=2)
    axes[1,0].set_xlabel('Time (s)')
    axes[1,0].set_ylabel('Angular Position (rad)')
    axes[1,0].set_title('Angular Position vs Time')
    axes[1,0].legend()
    axes[1,0].grid(True)
    
    # Plot 4: Motor command analysis
    axes[1,1].plot(df['ts'], df['ul'], 'g-', label='Left Motor', linewidth=2)
    axes[1,1].plot(df['ts'], df['ur'], 'orange', label='Right Motor', linewidth=2)
    motor_diff = df['ur'] - df['ul']
    axes[1,1].plot(df['ts'], motor_diff, 'purple', label='Difference (ur-ul)', linewidth=1)
    axes[1,1].set_xlabel('Time (s)')
    axes[1,1].set_ylabel('Motor Command (rad/s)')
    axes[1,1].set_title('Motor Commands Analysis')
    axes[1,1].legend()
    axes[1,1].grid(True)
    
    plt.tight_layout()
    return fig

def get_available_trajectories(directory_path):
    """Get list of available trajectory files"""
    directory = Path(directory_path)
    if not directory.exists():
        return []
    
    tr_files = sorted(directory.glob("TR*"))
    return [f.stem for f in tr_files]

def parse_trajectory_selection(selection, available_trajectories):
    """Parse trajectory selection string and return list of trajectory names"""
    if not selection:
        return available_trajectories
    
    selected = []
    parts = selection.split(',')
    
    for part in parts:
        part = part.strip()
        
        # Handle ranges like TR25-TR30
        if '-' in part and part.count('-') == 1:
            try:
                start_str, end_str = part.split('-')
                start_num = int(start_str.replace('TR', ''))
                end_num = int(end_str.replace('TR', ''))
                
                for i in range(start_num, end_num + 1):
                    tr_name = f"TR{i:02d}" if i < 100 else f"TR{i}"
                    if tr_name in available_trajectories:
                        selected.append(tr_name)
            except ValueError:
                print(f"Warning: Invalid range format: {part}")
        
        # Handle single trajectories like TR25 or 25
        else:
            if part.startswith('TR'):
                tr_name = part
            else:
                try:
                    num = int(part)
                    tr_name = f"TR{num:02d}" if num < 100 else f"TR{num}"
                except ValueError:
                    print(f"Warning: Invalid trajectory number: {part}")
                    continue
            
            if tr_name in available_trajectories:
                selected.append(tr_name)
            else:
                print(f"Warning: Trajectory {tr_name} not found")
    
    return sorted(list(set(selected)))  # Remove duplicates and sort

def visualize_trajectory_directory_v2(directory_path, selected_trajectories=None, save_plots=True, show_plots=True, circle_analysis=True, is_circle=False, is_bezier=False):
    """Visualize selected trajectory files in a directory (new format)"""
    directory = Path(directory_path)
    
    if not directory.exists():
        print(f"Directory {directory_path} does not exist!")
        return
    
    # Get available trajectories
    available_trajectories = get_available_trajectories(directory_path)
    
    if not available_trajectories:
        print(f"No trajectory files found in {directory_path}")
        return
    
    # Parse selection
    if selected_trajectories:
        trajectory_list = parse_trajectory_selection(selected_trajectories, available_trajectories)
    else:
        trajectory_list = available_trajectories
    
    if not trajectory_list:
        print("No valid trajectories selected")
        return
    
    print(f"Available trajectories: {', '.join(available_trajectories)}")
    print(f"Selected trajectories: {', '.join(trajectory_list)}")
    print(f"Processing {len(trajectory_list)} trajectory files...")
    
    for tr_name in trajectory_list:
        tr_file = directory / tr_name
        print(f"\nProcessing {tr_name}...")
        
        # Load data
        df = load_trajectory_data_v2(tr_file)
        if df is None:
            continue
        
        print(f"Loaded {len(df)} data points")
        print(f"Columns: {list(df.columns)}")
        
        # Create main visualization
        fig1 = plot_trajectory_comparison_v2(df, f"Trajectory: {tr_name}")
        
        if save_plots:
            # Save plot
            output_path1 = directory / f"{tr_name}_trajectory.png"
            fig1.savefig(output_path1, dpi=300, bbox_inches='tight')
            print(f"Saved trajectory plot to {output_path1}")
        
        # Create circle analysis (if requested and it looks like a circle trajectory)
        if circle_analysis and 'target_x' in df.columns and 'target_y' in df.columns:
            if is_bezier:
                fig2 = plot_bezier_analysis(df, f"Bezier Analysis: {tr_name}")
                analysis_suffix = "bezier_analysis"
            else:
                fig2 = plot_circle_analysis(df, f"Circle Analysis: {tr_name}")
                analysis_suffix = "circle_analysis"
            
            if save_plots:
                output_path2 = directory / f"{tr_name}_{analysis_suffix}.png"
                fig2.savefig(output_path2, dpi=300, bbox_inches='tight')
                print(f"Saved {analysis_suffix} to {output_path2}")
        
        # Analyze performance
        stats, pos_error, vel_error, theta_error = analyze_trajectory_performance_v2(df, is_circle=is_circle, is_bezier=is_bezier)
        
        # Print statistics
        print(f"Performance Statistics for {tr_name}:")
        print(f"  Mean Position Error: {stats['mean_pos_error']:.4f} m")
        print(f"  Max Position Error: {stats['max_pos_error']:.4f} m")
        print(f"  RMS Position Error: {stats['rms_pos_error']:.4f} m")
        print(f"  Mean Velocity Error: {stats['mean_vel_error']:.4f} m/s")
        print(f"  Max Velocity Error: {stats['max_vel_error']:.4f} m/s")
        print(f"  RMS Velocity Error: {stats['rms_vel_error']:.4f} m/s")
        print(f"  Mean Theta Error: {stats['mean_theta_error']:.4f} rad ({np.rad2deg(stats['mean_theta_error']):.2f} deg)")
        print(f"  Max Theta Error: {stats['max_theta_error']:.4f} rad ({np.rad2deg(stats['max_theta_error']):.2f} deg)")
        print(f"  RMS Theta Error: {stats['rms_theta_error']:.4f} rad ({np.rad2deg(stats['rms_theta_error']):.2f} deg)")
        
        # Print circle-specific statistics if available
        if is_circle and 'circle_center_x' in stats:
            print(f"Circle-Specific Statistics:")
            print(f"  Circle Center: ({stats['circle_center_x']:.4f}, {stats['circle_center_y']:.4f}) m")
            print(f"  Desired Radius: {stats['desired_radius']:.4f} m")
            print(f"  Actual Mean Radius: {stats['actual_radius_mean']:.4f} m")
            print(f"  Radius Error (Mean): {stats['radius_error_mean']:.4f} m")
            print(f"  Radius Error (RMS): {stats['radius_error_rms']:.4f} m")
            print(f"  Angular Velocity: {stats['angular_velocity']:.4f} rad/s ({np.rad2deg(stats['angular_velocity']):.2f} deg/s)")
            print(f"  Estimated Linear Speed: {stats['estimated_linear_speed']:.4f} m/s")
            print(f"  Circle Completion Time: {stats['circle_completion_time']:.2f} s")
            print(f"  Total Angle Traversed: {stats['total_angle_traversed']:.2f} rad ({np.rad2deg(stats['total_angle_traversed']):.1f} deg)")
        
        # Print Bezier-specific statistics if available
        if is_bezier and 'curve_length' in stats:
            print(f"Bezier-Specific Statistics:")
            print(f"  Curve Length: {stats['curve_length']:.4f} m")
            print(f"  Max Curvature: {stats['max_curvature']:.4f} 1/m")
            print(f"  Mean Curvature: {stats['mean_curvature']:.4f} 1/m")
            print(f"  X Range: {stats['x_range']:.4f} m")
            print(f"  Y Range: {stats['y_range']:.4f} m")
            print(f"  Bezier Completion Time: {stats['bezier_completion_time']:.2f} s")
            print(f"  Start Point: ({stats['start_point'][0]:.4f}, {stats['start_point'][1]:.4f}) m")
            print(f"  End Point: ({stats['end_point'][0]:.4f}, {stats['end_point'][1]:.4f}) m")
            
            # Print estimated control points
            control_points = estimate_bezier_control_points(df)
            if control_points:
                print(f"  Estimated Control Points:")
                print(f"    P0: ({control_points['p0'][0]:.4f}, {control_points['p0'][1]:.4f}) m")
                print(f"    P1: ({control_points['p1'][0]:.4f}, {control_points['p1'][1]:.4f}) m")
                print(f"    P2: ({control_points['p2'][0]:.4f}, {control_points['p2'][1]:.4f}) m")
                print(f"    P3: ({control_points['p3'][0]:.4f}, {control_points['p3'][1]:.4f}) m")
        
        print("-" * 70)
        
        if show_plots:
            plt.show()

def visualize_single_file(filepath, save_plots=True, show_plots=True, circle_analysis=True, is_circle=False, is_bezier=False):
    """Visualize a single trajectory file"""
    from pathlib import Path
    
    filepath = Path(filepath)
    if not filepath.exists():
        print(f"File {filepath} does not exist!")
        return
    
    print(f"Processing single file: {filepath.name}")
    
    # Load data
    df = load_trajectory_data_v2(filepath)
    if df is None:
        print("Failed to load trajectory data")
        return
    
    # Fix timestamp format if needed (convert from milliseconds to seconds)
    if df['ts'].max() > 1000:  # Likely in milliseconds
        df['ts'] = df['ts'] / 1000.0
        print(f"Converted timestamps from milliseconds to seconds")
    
    print(f"Loaded {len(df)} data points")
    print(f"Columns: {list(df.columns)}")
    print(f"Time range: {df['ts'].min():.2f}s to {df['ts'].max():.2f}s")
    
    # Create main visualization
    fig1 = plot_trajectory_comparison_v2(df, f"Trajectory: {filepath.name}")
    
    if save_plots:
        # Save plot in same directory as source file
        output_path1 = filepath.parent / f"{filepath.stem}_trajectory.png"
        fig1.savefig(output_path1, dpi=300, bbox_inches='tight')
        print(f"Saved trajectory plot to {output_path1}")
    
    # Create circle analysis (if requested and it looks like a circle trajectory)
    if circle_analysis and 'target_x' in df.columns and 'target_y' in df.columns:
        if is_bezier:
            fig2 = plot_bezier_analysis(df, f"Bezier Analysis: {filepath.name}")
            analysis_suffix = "bezier_analysis"
        else:
            fig2 = plot_circle_analysis(df, f"Circle Analysis: {filepath.name}")
            analysis_suffix = "circle_analysis"
        
        if save_plots:
            output_path2 = filepath.parent / f"{filepath.stem}_{analysis_suffix}.png"
            fig2.savefig(output_path2, dpi=300, bbox_inches='tight')
            print(f"Saved {analysis_suffix} to {output_path2}")
    
    # Analyze performance
    stats, pos_error, vel_error, theta_error = analyze_trajectory_performance_v2(df, is_circle=is_circle, is_bezier=is_bezier)
    
    # Print statistics
    print(f"Performance Statistics for {filepath.name}:")
    print(f"  Mean Position Error: {stats['mean_pos_error']:.4f} m")
    print(f"  Max Position Error: {stats['max_pos_error']:.4f} m")
    print(f"  RMS Position Error: {stats['rms_pos_error']:.4f} m")
    print(f"  Mean Velocity Error: {stats['mean_vel_error']:.4f} m/s")
    print(f"  Max Velocity Error: {stats['max_vel_error']:.4f} m/s")
    print(f"  RMS Velocity Error: {stats['rms_vel_error']:.4f} m/s")
    print(f"  Mean Theta Error: {stats['mean_theta_error']:.4f} rad ({np.rad2deg(stats['mean_theta_error']):.2f} deg)")
    print(f"  Max Theta Error: {stats['max_theta_error']:.4f} rad ({np.rad2deg(stats['max_theta_error']):.2f} deg)")
    print(f"  RMS Theta Error: {stats['rms_theta_error']:.4f} rad ({np.rad2deg(stats['rms_theta_error']):.2f} deg)")
    
    # Print circle-specific statistics if available
    if is_circle and 'circle_center_x' in stats:
        print(f"Circle-Specific Statistics:")
        print(f"  Circle Center: ({stats['circle_center_x']:.4f}, {stats['circle_center_y']:.4f}) m")
        print(f"  Desired Radius: {stats['desired_radius']:.4f} m")
        print(f"  Actual Mean Radius: {stats['actual_radius_mean']:.4f} m")
        print(f"  Radius Error (Mean): {stats['radius_error_mean']:.4f} m")
        print(f"  Radius Error (RMS): {stats['radius_error_rms']:.4f} m")
        print(f"  Angular Velocity: {stats['angular_velocity']:.4f} rad/s ({np.rad2deg(stats['angular_velocity']):.2f} deg/s)")
        print(f"  Estimated Linear Speed: {stats['estimated_linear_speed']:.4f} m/s")
        print(f"  Circle Completion Time: {stats['circle_completion_time']:.2f} s")
        print(f"  Total Angle Traversed: {stats['total_angle_traversed']:.2f} rad ({np.rad2deg(stats['total_angle_traversed']):.1f} deg)")
    
    # Print Bezier-specific statistics if available
    if is_bezier and 'curve_length' in stats:
        print(f"Bezier-Specific Statistics:")
        print(f"  Curve Length: {stats['curve_length']:.4f} m")
        print(f"  Max Curvature: {stats['max_curvature']:.4f} 1/m")
        print(f"  Mean Curvature: {stats['mean_curvature']:.4f} 1/m")
        print(f"  X Range: {stats['x_range']:.4f} m")
        print(f"  Y Range: {stats['y_range']:.4f} m")
        print(f"  Bezier Completion Time: {stats['bezier_completion_time']:.2f} s")
        print(f"  Start Point: ({stats['start_point'][0]:.4f}, {stats['start_point'][1]:.4f}) m")
        print(f"  End Point: ({stats['end_point'][0]:.4f}, {stats['end_point'][1]:.4f}) m")
        
        # Print estimated control points
        control_points = estimate_bezier_control_points(df)
        if control_points:
            print(f"  Estimated Control Points:")
            print(f"    P0: ({control_points['p0'][0]:.4f}, {control_points['p0'][1]:.4f}) m")
            print(f"    P1: ({control_points['p1'][0]:.4f}, {control_points['p1'][1]:.4f}) m")
            print(f"    P2: ({control_points['p2'][0]:.4f}, {control_points['p2'][1]:.4f}) m")
            print(f"    P3: ({control_points['p3'][0]:.4f}, {control_points['p3'][1]:.4f}) m")
    
    print("-" * 70)
    
    if show_plots:
        plt.show()

def main():
    parser = argparse.ArgumentParser(description='Visualize robot trajectory following data')
    
    parser.add_argument('input', nargs='?', default='27-08-trajfollowing-diffdrive',
                        help='Directory containing trajectory files or single trajectory file (default: 27-08-trajfollowing-diffdrive)')
    
    parser.add_argument('-t', '--trajectories', type=str, 
                        help='Comma-separated list of trajectories to plot (e.g., "TR25,TR26" or "25,26" or "25-30")')
    
    parser.add_argument('--list', '-l', action='store_true',
                        help='List available trajectories and exit')
    
    parser.add_argument('--no-save', action='store_true',
                        help='Do not save plots to files')
    
    parser.add_argument('--no-show', action='store_true',
                        help='Do not show plots on screen')
    
    parser.add_argument('--no-circle', action='store_true',
                        help='Skip circle analysis plots')
    
    parser.add_argument('--circle', action='store_true',
                        help='Enable circle-specific performance statistics (radius, center, angular velocity, linear speed)')
    
    parser.add_argument('--bezier', action='store_true',
                        help='Enable Bezier curve analysis with control points, curvature, and curve-specific statistics')
    
    args = parser.parse_args()
    
    # Check if input is a file or directory
    from pathlib import Path
    input_path = Path(args.input)
    
    if input_path.is_file():
        # Handle single file
        visualize_single_file(
            filepath=input_path,
            save_plots=not args.no_save,
            show_plots=not args.no_show,
            circle_analysis=not args.no_circle,
            is_circle=args.circle,
            is_bezier=args.bezier
        )
    else:
        # Handle directory (existing functionality)
        if args.list:
            available = get_available_trajectories(args.input)
            if available:
                print(f"Available trajectories in {args.input}:")
                for tr in available:
                    print(f"  {tr}")
            else:
                print(f"No trajectories found in {args.input}")
            return
        
        # Visualize trajectories
        visualize_trajectory_directory_v2(
            directory_path=args.input,
            selected_trajectories=args.trajectories,
            save_plots=not args.no_save,
            show_plots=not args.no_show,
            circle_analysis=not args.no_circle,
            is_circle=args.circle,
            is_bezier=args.bezier
        )

if __name__ == "__main__":
    main()
