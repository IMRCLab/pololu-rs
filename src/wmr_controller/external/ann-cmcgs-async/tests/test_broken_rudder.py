#!/usr/bin/env python3
"""Test script for broken rudder environment and robot."""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import gymnasium as gym

# Import our modules
import tools.robots as robots
import tools.envs as envs

def test_broken_rudder_robot():
    """Test the broken rudder robot directly."""
    print("Testing SingleIntegratorUnicycleBrokenRudder robot...")
    
    try:
        robot = robots.SingleIntegratorUnicycleBrokenRudder(
            id=0,
            name="test_broken_rudder",
            dt=0.1,
            max_velocity=1.0,
            max_angular=1.0
        )
        
        print(f"✓ Robot created successfully")
        print(f"  State space: {robot.state_space}")
        print(f"  Action space: {robot.action_space}")
        
        # Test state and goal
        start_state = np.array([0.0, 0.0, 0.0, 0.0])
        goal_state = np.array([1.0, 1.0, np.pi/2, 10.0])
        
        robot.state = start_state
        robot.state_goal = goal_state
        
        print(f"  Initial state: {robot.state}")
        print(f"  Goal state: {robot.state_goal}")
        
        # Test transition model
        action = np.array([0.5, 0.3])  # positive angular velocity only
        new_state = robot.transition_model(robot.state, action, dt=0.1)
        print(f"  After action {action}: {new_state}")
        
        # Test controller
        actions, dts = robot.controller(start_state, goal_state)
        print(f"  Controller returned {len(actions)} actions and {len(dts)} durations")
        
        return True
        
    except Exception as e:
        print(f"✗ Error testing robot: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_broken_rudder_environment():
    """Test the broken rudder navigation environment."""
    print("\nTesting NavigationEnvBrokenRudder environment...")
    
    try:
        # Import the environment to register it
        from tools.envs.navigation_envs.navigation_env_single_integrator_broken_rudder import NavigationEnvSingleIntegratorBrokenRudder
        
        # Create environment
        env = gym.make('NavigationEnvBrokenRudder-v0')
        print(f"✓ Environment created successfully")
        
        # Reset environment
        obs, info = env.reset(seed=42)
        print(f"  Observation space: {env.observation_space}")
        print(f"  Action space: {env.action_space}")
        print(f"  Initial observation shape: {obs.shape}")
        print(f"  Initial observation: {obs}")
        
        # Test a few steps
        for i in range(3):
            # Sample action (note: angular velocity must be >= 0)
            action = env.action_space.sample()
            action[1] = abs(action[1])  # Ensure angular velocity is positive
            
            obs, reward, terminated, truncated, info = env.step(action)
            print(f"  Step {i+1}: action={action}, reward={reward:.3f}, done={terminated or truncated}")
            
            if terminated or truncated:
                break
        
        env.close()
        return True
        
    except Exception as e:
        print(f"✗ Error testing environment: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("Testing Broken Rudder Robot and Environment")
    print("=" * 60)
    
    success = True
    success &= test_broken_rudder_robot()
    success &= test_broken_rudder_environment()
    
    print("\n" + "=" * 60)
    if success:
        print("✓ All tests passed!")
    else:
        print("✗ Some tests failed!")
    print("=" * 60)