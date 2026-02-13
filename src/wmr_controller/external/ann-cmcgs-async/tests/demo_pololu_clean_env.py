"""
Demo: Pololu Controller in Clean Navigation Environment

This module demonstrates the Pololu controller operating in a navigation 
environment with no obstacles, suitable for controller testing and validation.
"""

import numpy as np
import time
from tools.envs.navigation_envs.navigation_env_pololu import NavigationEnvPololu


def create_clean_env(render=True):
    """
    Create a navigation environment optimized for controller testing.
    
    Parameters:
    -----------
    render : bool
        Whether to enable rendering (human mode)
    
    Returns:
    --------
    env : NavigationEnvPololu
        Clean environment with no obstacles
    """
    render_mode = "human" if render else None
    env = NavigationEnvPololu(
        dt=0.1,
        sparse_reward=False,
        obstacle_mode="none",
        render_mode=render_mode
    )
    
    # Reset and configure
    obs, info = env.reset(seed=0)
    
    # Ensure no obstacles
    if hasattr(env, "obstacles"):
        env.obstacles = []
    
    # Disable truncation to focus on controller tracking
    env.is_truncated = lambda state: False
    
    return env


def run_controller_test(goal_pos=(1.0, 0.0), target_theta=0.0, render=True, pause_time=10):
    """
    Run a simple controller test to a goal position.
    
    Parameters:
    -----------
    goal_pos : tuple
        Target position (x, y)
    target_theta : float
        Target heading angle (radians)
    render : bool
        Whether to render the trajectory
    pause_time : float
        Time in seconds to keep visualization window open
    
    Returns:
    --------
    env : NavigationEnvPololu
        Environment with trajectory completed
    success : bool
        Whether the goal was reached
    """
    env = create_clean_env(render=render)
    
    # Set initial state at origin
    env.agent.state = np.array([0.0, 0.0, 0.0, env.start_t], dtype=np.float32)
    env.goal_pos = np.array(goal_pos, dtype=np.float32)
    
    # Create setpoint state
    setpoint_state = np.array([
        goal_pos[0],
        goal_pos[1],
        target_theta,
        env.agent.state[3] + 5.0,
    ], dtype=np.float32)
    
    print(f"Starting position: {env.agent.state[:3]}")
    print(f"Goal: {setpoint_state[:3]}")
    
    # Run controller
    action_list, dt_list = env.agent.controller_pololu(
        env.agent.state, setpoint_state, N=60
    )
    
    print(f"Generated {len(action_list)} actions")
    
    # Execute actions step-by-step
    total_reward = 0.0
    for i, (action, dt) in enumerate(zip(action_list, dt_list)):
        obs, reward, terminated, truncated, info = env.step(action)
        if render:
            env.render()
        total_reward += reward
        
        if terminated or truncated:
            print(f"Episode ended at step {i}")
            break
    
    # Check success
    success = env.agent.is_finished(
        env.agent.state, setpoint_state, 
        atol=env.atol, rtol=env.rtol
    )
    
    print(f"\nFinal position: {env.agent.state[:3]}")
    print(f"Total reward: {total_reward:.4f}")
    print(f"Goal reached: {success}")
    
    if render:
        print(f"Visualization window open for {pause_time} seconds...")
        time.sleep(pause_time)
        env.close()
    
    return env, success


if __name__ == "__main__":
    # Run demo test
    print("=" * 60)
    print("Pololu Controller Demo: Simple Goal Reaching")
    print("=" * 60)
    print()
    
    env, success = run_controller_test(
        goal_pos=(1.0, 0.0),
        target_theta=0.0,
        render=True,
        pause_time=10
    )
    
    if success:
        print("\n✓ Test PASSED: Controller reached goal!")
    else:
        print("\n✗ Test FAILED: Controller did not reach goal")
