"""
Visualization utilities for robot trajectory analysis and testing.

This module provides visualization functions for analyzing robot trajectories,
tracking performance, and controller b    # Velocity over time (second state variable is velocity)
    axes[0, 1].plot(times, states[:, 1], 'g-', linewidth=2, label='Velocity')
    if setpoint_states is not None:
        min_len = min(len(times), len(setpoint_states))
        axes[0, 1].plot(times[:min_len], setpoint_states[:min_len, 1], 'r--', linewidth=2, alpha=0.7, label='Target')ior across different robot types.
"""

import os
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# Configure matplotlib backend for non-interactive use
matplotlib.use('Agg')
plt.ioff()


def plot_trajectory_2d(states, title, filename, robot_name, output_dir=".", setpoint_states=None, actions=None):
    """Plot 2D trajectory for robots with x,y positions."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(f'{robot_name}: {title}', fontsize=14)
    
    states = np.array(states)
    setpoint_states = np.array(setpoint_states) if setpoint_states is not None else None
    
    # Position trajectory
    axes[0, 0].plot(states[:, 0], states[:, 1], 'b-', linewidth=2, label='Actual path')
    axes[0, 0].scatter(states[0, 0], states[0, 1], color='green', s=100, label='Start', zorder=5)
    axes[0, 0].scatter(states[-1, 0], states[-1, 1], color='red', s=100, label='End', zorder=5)
    
    if setpoint_states is not None:
        setpoint_states = np.array(setpoint_states)
        axes[0, 0].plot(setpoint_states[:, 0], setpoint_states[:, 1], 'r--', linewidth=2, alpha=0.7, label='Target path')
        axes[0, 0].scatter(setpoint_states[-1, 0], setpoint_states[-1, 1], color='orange', s=100, label='Target', zorder=5)
    
    axes[0, 0].set_xlabel('X Position')
    axes[0, 0].set_ylabel('Y Position')
    axes[0, 0].set_title('Trajectory')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # X position over time
    axes[0, 1].plot(states[:, -1], states[:, 0], 'b-', linewidth=2, label='X position')
    if setpoint_states is not None:
        axes[0, 1].plot(setpoint_states[:, -1], setpoint_states[:, 0], 'r--', linewidth=2, alpha=0.7, label='Target X')
    axes[0, 1].set_xlabel('Time')
    axes[0, 1].set_ylabel('X Position')
    axes[0, 1].set_title('X Position vs Time')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Y position over time
    axes[1, 0].plot(states[:, -1], states[:, 1], 'b-', linewidth=2, label='Y position')
    if setpoint_states is not None:
        axes[1, 0].plot(setpoint_states[:, -1], setpoint_states[:, 1], 'r--', linewidth=2, alpha=0.7, label='Target Y')
    axes[1, 0].set_xlabel('Time')
    axes[1, 0].set_ylabel('Y Position')
    axes[1, 0].set_title('Y Position vs Time')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Actions over time (if provided)
    if actions is not None:
        actions = np.array(actions)
        # Use states time for action plotting (actions are one step less than states)
        action_times = states[:-1, -1] if len(actions) == len(states) - 1 else states[:len(actions), -1]
        if actions.shape[1] >= 2:  # 2D actions
            axes[1, 1].plot(action_times, actions[:, 0], 'g-', linewidth=2, label='Action X')
            axes[1, 1].plot(action_times, actions[:, 1], 'r-', linewidth=2, label='Action Y')
        else:  # 1D actions
            axes[1, 1].plot(action_times, actions[:, 0], 'g-', linewidth=2, label='Action')
        axes[1, 1].set_xlabel('Time')
        axes[1, 1].set_ylabel('Action')
        axes[1, 1].set_title('Control Actions vs Time')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
    else:
        axes[1, 1].text(0.5, 0.5, 'No action data', ha='center', va='center', transform=axes[1, 1].transAxes)
        axes[1, 1].set_title('Control Actions')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename), dpi=150, bbox_inches='tight')
    plt.close()


def plot_unicycle_trajectory(states, title, filename, robot_name, output_dir=".", setpoint_states=None, actions=None):
    """Plot trajectory for unicycle robots with orientation."""
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle(f'{robot_name}: {title}', fontsize=14)
    
    states = np.array(states)
    setpoint_states = np.array(setpoint_states) if setpoint_states is not None else None    
    # Position trajectory with orientation arrows
    axes[0, 0].plot(states[:, 0], states[:, 1], 'b-', linewidth=2, label='Actual path')
    axes[0, 0].scatter(states[0, 0], states[0, 1], color='green', s=100, label='Start', zorder=5)
    axes[0, 0].scatter(states[-1, 0], states[-1, 1], color='red', s=100, label='End', zorder=5)
    
    # Add orientation arrows
    for i in range(0, len(states), max(1, len(states)//10)):  # Show ~10 arrows
        x, y, theta = states[i, 0], states[i, 1], states[i, 2]
        dx, dy = 0.3 * np.cos(theta), 0.3 * np.sin(theta)
        axes[0, 0].arrow(x, y, dx, dy, head_width=0.1, head_length=0.1, fc='blue', ec='blue', alpha=0.7)
    
    if setpoint_states is not None:
        setpoint_states = np.array(setpoint_states)
        axes[0, 0].plot(setpoint_states[:, 0], setpoint_states[:, 1], 'r--', linewidth=2, alpha=0.7, label='Target path')
        axes[0, 0].scatter(setpoint_states[-1, 0], setpoint_states[-1, 1], color='orange', s=100, label='Target', zorder=5)
    
    axes[0, 0].set_xlabel('X Position')
    axes[0, 0].set_ylabel('Y Position')
    axes[0, 0].set_title('Trajectory with Orientation')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axis('equal')
    
    # X position over time
    axes[0, 1].plot(states[:, -1], states[:, 0], 'b-', linewidth=2, label='X position')
    if setpoint_states is not None:
        axes[0, 1].plot(setpoint_states[:, -1], setpoint_states[:, 0], 'r--', linewidth=2, alpha=0.7, label='Target X')
    axes[0, 1].set_xlabel('Time')
    axes[0, 1].set_ylabel('X Position')
    axes[0, 1].set_title('X Position vs Time')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Y position over time
    axes[0, 2].plot(states[:, -1], states[:, 1], 'b-', linewidth=2, label='Y position')
    if setpoint_states is not None:
        axes[0, 2].plot(setpoint_states[:, -1], setpoint_states[:, 1], 'r--', linewidth=2, alpha=0.7, label='Target Y')
    axes[0, 2].set_xlabel('Time')
    axes[0, 2].set_ylabel('Y Position')
    axes[0, 2].set_title('Y Position vs Time')
    axes[0, 2].legend()
    axes[0, 2].grid(True, alpha=0.3)
    
    # Orientation over time
    axes[1, 0].plot(states[:, -1], states[:, 2], 'b-', linewidth=2, label='Orientation')
    if setpoint_states is not None:
        axes[1, 0].plot(setpoint_states[:, -1], setpoint_states[:, 2], 'r--', linewidth=2, alpha=0.7, label='Target θ')
    axes[1, 0].set_xlabel('Time')
    axes[1, 0].set_ylabel('Orientation (rad)')
    axes[1, 0].set_title('Orientation vs Time')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Actions over time (if provided)
    if actions is not None:
        actions = np.array(actions)
        # Use states time for action plotting (actions are one step less than states)
        action_times = states[:-1, -1] if len(actions) == len(states) - 1 else states[:len(actions), -1]
        axes[1, 1].plot(action_times, actions[:, 0], 'g-', linewidth=2, label='Linear velocity')
        axes[1, 1].set_xlabel('Time')
        axes[1, 1].set_ylabel('Linear Velocity')
        axes[1, 1].set_title('Linear Velocity vs Time')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)

        axes[1, 2].plot(action_times, actions[:, 1], 'r-', linewidth=2, label='Angular velocity')
        axes[1, 2].set_xlabel('Time')
        axes[1, 2].set_ylabel('Angular Velocity')
        axes[1, 2].set_title('Angular Velocity vs Time')
        axes[1, 2].legend()
        axes[1, 2].grid(True, alpha=0.3)
    else:
        axes[1, 1].text(0.5, 0.5, 'No action data', ha='center', va='center', transform=axes[1, 1].transAxes)
        axes[1, 2].text(0.5, 0.5, 'No action data', ha='center', va='center', transform=axes[1, 2].transAxes)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename), dpi=150, bbox_inches='tight')
    plt.close()


def plot_trajectory_1d(states, title, filename, robot_name, output_dir=".", setpoint_states=None, actions=None):
    """Plot 1D trajectory for robots with only one spatial dimension (like baseline double integrator)."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(f'{robot_name}: {title}', fontsize=14)
    
    states = np.array(states)
    setpoint_states = np.array(setpoint_states) if setpoint_states is not None else None
    times = states[:, -1]  # Last column is time
    
    # Position over time (first state variable is position)
    axes[0, 0].plot(states[:, -1], states[:, 0], 'b-', linewidth=2, label='Position')
    if setpoint_states is not None:
        setpoint_states = np.array(setpoint_states)
        axes[0, 0].plot(setpoint_states[:, -1], setpoint_states[:, 0], 'r--', linewidth=2, alpha=0.7, label='Target')
    axes[0, 0].set_xlabel('Time')
    axes[0, 0].set_ylabel('Position')
    axes[0, 0].set_title('Position vs Time')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Velocity over time (second state variable is velocity)
    axes[0, 1].plot(states[:, -1], states[:, 1], 'g-', linewidth=2, label='Velocity')
    if setpoint_states is not None:
        axes[0, 1].plot(setpoint_states[:, -1], setpoint_states[:, 1], 'r--', linewidth=2, alpha=0.7, label='Target velocity')
    axes[0, 1].set_xlabel('Time')
    axes[0, 1].set_ylabel('Velocity')
    axes[0, 1].set_title('Velocity vs Time')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Phase plot (position vs velocity)
    axes[1, 0].plot(states[:, 0], states[:, 1], 'b-', linewidth=2, label='Actual')
    axes[1, 0].scatter(states[0, 0], states[0, 1], color='green', s=100, label='Start', zorder=5)
    axes[1, 0].scatter(states[-1, 0], states[-1, 1], color='red', s=100, label='End', zorder=5)
    if setpoint_states is not None:
        axes[1, 0].plot(setpoint_states[:, 0], setpoint_states[:, 1], 'r--', linewidth=2, alpha=0.7, label='Target')
        axes[1, 0].scatter(setpoint_states[-1, 0], setpoint_states[-1, 1], color='orange', s=100, label='Target end', zorder=5)
    axes[1, 0].set_xlabel('Position')
    axes[1, 0].set_ylabel('Velocity')
    axes[1, 0].set_title('Phase Plot (Position vs Velocity)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Actions over time (if provided)
    if actions is not None:
        actions = np.array(actions)
        # Use states time for action plotting (actions are one step less than states)
        action_times = times[:-1] if len(actions) == len(times) - 1 else times[:len(actions)]
        axes[1, 1].plot(action_times, actions.flatten(), 'r-', linewidth=2, label='Control action')
        axes[1, 1].set_xlabel('Time')
        axes[1, 1].set_ylabel('Action')
        axes[1, 1].set_title('Control Actions vs Time')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
    else:
        axes[1, 1].text(0.5, 0.5, 'No action data', ha='center', va='center', transform=axes[1, 1].transAxes)
        axes[1, 1].set_title('Control Actions')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename), dpi=150, bbox_inches='tight')
    plt.close()


def plot_tracking_error(actual_states, target_states, title, filename, robot_name, output_dir="."):
    """Plot tracking error over time with interpolation for different trajectory lengths."""
    if len(actual_states) == 0 or len(target_states) == 0:
        return
        
    actual_states = np.array(actual_states)
    target_states = np.array(target_states)
    
    # Extract time series (assuming last column is time)
    actual_times = actual_states[:, -1]
    target_times = target_states[:, -1]
    
    # Create common time grid covering both trajectories
    min_time = max(actual_times[0], target_times[0])
    max_time = min(actual_times[-1], target_times[-1])
    
    if min_time >= max_time:
        print(f"Warning: No time overlap between trajectories. Actual: [{actual_times[0]:.3f}, {actual_times[-1]:.3f}], Target: [{target_times[0]:.3f}, {target_times[-1]:.3f}]")
        return
    
    # Use 100 points for smooth interpolation over the common time range
    common_times = np.linspace(min_time, max_time, 100)
    
    # Interpolate both trajectories onto the common time grid
    actual_interp = np.zeros((len(common_times), actual_states.shape[1]))
    target_interp = np.zeros((len(common_times), target_states.shape[1]))
    
    for i in range(actual_states.shape[1] - 1):  # Don't interpolate time column
        actual_interp[:, i] = np.interp(common_times, actual_times, actual_states[:, i])
        target_interp[:, i] = np.interp(common_times, target_times, target_states[:, i])
    
    # Set time column
    actual_interp[:, -1] = common_times
    target_interp[:, -1] = common_times
    
    # Calculate position error
    position_error = np.linalg.norm(actual_interp[:, :2] - target_interp[:, :2], axis=1)
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f'{robot_name}: {title}', fontsize=14)
    
    # Position error over time
    axes[0, 0].plot(common_times, position_error, 'r-', linewidth=2, label='Position error')
    axes[0, 0].set_xlabel('Time')
    axes[0, 0].set_ylabel('Position Error')
    axes[0, 0].set_title('Position Tracking Error')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # X error over time
    x_error = actual_interp[:, 0] - target_interp[:, 0]
    axes[0, 1].plot(common_times, x_error, 'b-', linewidth=2, label='X error')
    axes[0, 1].set_xlabel('Time')
    axes[0, 1].set_ylabel('X Error')
    axes[0, 1].set_title('X Position Error')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Y error over time
    y_error = actual_interp[:, 1] - target_interp[:, 1]
    axes[1, 0].plot(common_times, y_error, 'g-', linewidth=2, label='Y error')
    axes[1, 0].set_xlabel('Time')
    axes[1, 0].set_ylabel('Y Error')
    axes[1, 0].set_title('Y Position Error')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Error statistics
    stats_text = f"""Error Statistics:
Mean Position Error: {np.mean(position_error):.4f}
Max Position Error: {np.max(position_error):.4f}
Final Position Error: {position_error[-1]:.4f}
RMS Position Error: {np.sqrt(np.mean(position_error**2)):.4f}"""
    
    axes[1, 1].text(0.1, 0.5, stats_text, transform=axes[1, 1].transAxes, 
                   verticalalignment='center', fontfamily='monospace', fontsize=10)
    axes[1, 1].set_title('Error Statistics')
    axes[1, 1].axis('off')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename), dpi=150, bbox_inches='tight')
    plt.close()


def plot_tracking_error_1d(actual_states, target_states, title, filename, robot_name, output_dir="."):
    """Plot tracking error over time for 1D robots with interpolation for different trajectory lengths."""
    if len(actual_states) == 0 or len(target_states) == 0:
        return
        
    actual_states = np.array(actual_states)
    target_states = np.array(target_states)
    
    # Extract time series (assuming last column is time)
    actual_times = actual_states[:, -1]
    target_times = target_states[:, -1]
    
    # Create common time grid covering both trajectories
    min_time = max(actual_times[0], target_times[0])
    max_time = min(actual_times[-1], target_times[-1])
    
    if min_time >= max_time:
        print(f"Warning: No time overlap between trajectories. Actual: [{actual_times[0]:.3f}, {actual_times[-1]:.3f}], Target: [{target_times[0]:.3f}, {target_times[-1]:.3f}]")
        return
    
    # Use 100 points for smooth interpolation over the common time range
    common_times = np.linspace(min_time, max_time, 100)
    
    # Interpolate both trajectories onto the common time grid
    actual_interp = np.zeros((len(common_times), actual_states.shape[1]))
    target_interp = np.zeros((len(common_times), target_states.shape[1]))
    
    for i in range(actual_states.shape[1] - 1):  # Don't interpolate time column
        actual_interp[:, i] = np.interp(common_times, actual_times, actual_states[:, i])
        target_interp[:, i] = np.interp(common_times, target_times, target_states[:, i])
    
    # Set time column
    actual_interp[:, -1] = common_times
    target_interp[:, -1] = common_times
    
    # Calculate position and velocity errors
    position_error = np.abs(actual_interp[:, 0] - target_interp[:, 0])
    velocity_error = np.abs(actual_interp[:, 1] - target_interp[:, 1])
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle(f'{robot_name}: {title}', fontsize=14)
    
    # Position error over time
    axes[0, 0].plot(common_times, position_error, 'r-', linewidth=2, label='Position error')
    axes[0, 0].set_xlabel('Time')
    axes[0, 0].set_ylabel('Position Error')
    axes[0, 0].set_title('Position Tracking Error')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Velocity error over time
    axes[0, 1].plot(common_times, velocity_error, 'b-', linewidth=2, label='Velocity error')
    axes[0, 1].set_xlabel('Time')
    axes[0, 1].set_ylabel('Velocity Error')
    axes[0, 1].set_title('Velocity Tracking Error')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Combined error over time
    combined_error = np.sqrt(position_error**2 + velocity_error**2)
    axes[1, 0].plot(common_times, combined_error, 'g-', linewidth=2, label='Combined error')
    axes[1, 0].set_xlabel('Time')
    axes[1, 0].set_ylabel('Combined Error')
    axes[1, 0].set_title('Combined Position+Velocity Error')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Error statistics
    stats_text = f"""Error Statistics:
Mean Position Error: {np.mean(position_error):.4f}
Max Position Error: {np.max(position_error):.4f}
Final Position Error: {position_error[-1]:.4f}
RMS Position Error: {np.sqrt(np.mean(position_error**2)):.4f}

Mean Velocity Error: {np.mean(velocity_error):.4f}
Max Velocity Error: {np.max(velocity_error):.4f}
Final Velocity Error: {velocity_error[-1]:.4f}
RMS Velocity Error: {np.sqrt(np.mean(velocity_error**2)):.4f}"""
    
    axes[1, 1].text(0.1, 0.5, stats_text, transform=axes[1, 1].transAxes, 
                   verticalalignment='center', fontfamily='monospace', fontsize=9)
    axes[1, 1].set_title('Error Statistics')
    axes[1, 1].axis('off')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, filename), dpi=150, bbox_inches='tight')
    plt.close()


def create_trajectory_visualization(robot, robot_name, initial_state, controller_actions, controller_dts, 
                                   tracking_actions, tracking_dts, output_dir="."):
    """
    Helper function to generate appropriate trajectory visualizations based on robot type.
    
    Args:
        robot: Robot instance
        robot_name: Name of the robot for plot titles
        initial_state: Initial state of the robot
        controller_actions: Actions from controller
        controller_dts: Time steps from controller
        tracking_actions: Target tracking actions
        tracking_dts: Target tracking time steps
        output_dir: Directory to save plots
    """
    # Generate actual trajectory
    trajectory_states = [initial_state]
    current_state = initial_state.copy()
    for action, dt in zip(controller_actions, controller_dts):
        current_state = robot.transition_model(current_state, action, dt)
        trajectory_states.append(current_state.copy())
    
    # Generate target trajectory for comparison
    target_trajectory = [initial_state]
    current_target = initial_state.copy()
    for action, dt in zip(tracking_actions, tracking_dts):
        current_target = robot.transition_model(current_target, action, dt)
        target_trajectory.append(current_target.copy())
    
    # Choose appropriate visualization method based on robot type
    if 'unicycle' in robot_name.lower():
        plot_unicycle_trajectory(trajectory_states, "Controller Trajectory Tracking", 
                                f"{robot_name.lower()}_trajectory.png", robot_name, 
                                output_dir=output_dir, setpoint_states=target_trajectory, actions=controller_actions)
        plot_tracking_error(trajectory_states, target_trajectory, "Controller Tracking Error",
                           f"{robot_name.lower()}_tracking_error.png", robot_name, output_dir=output_dir)
    elif len(initial_state) == 3 and 'baseline' in robot_name.lower():  # 1D robot like baseline double integrator
        plot_trajectory_1d(trajectory_states, "Controller Trajectory Tracking", 
                          f"{robot_name.lower()}_trajectory.png", robot_name, 
                          output_dir=output_dir, setpoint_states=target_trajectory, actions=controller_actions)
        plot_tracking_error_1d(trajectory_states, target_trajectory, "Controller Tracking Error",
                              f"{robot_name.lower()}_tracking_error.png", robot_name, output_dir=output_dir)
    else:  # 2D robot (single integrator, etc.)
        plot_trajectory_2d(trajectory_states, "Controller Trajectory Tracking", 
                          f"{robot_name.lower()}_trajectory.png", robot_name, 
                          output_dir=output_dir, setpoint_states=target_trajectory, actions=controller_actions)
        plot_tracking_error(trajectory_states, target_trajectory, "Controller Tracking Error",
                           f"{robot_name.lower()}_tracking_error.png", robot_name, output_dir=output_dir)