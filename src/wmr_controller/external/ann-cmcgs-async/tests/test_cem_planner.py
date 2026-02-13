#!/usr/bin/env python3
"""
Test script for the CEM Planner
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
import yaml
from tools.planners.cem_planner import CEMPlanner
from tools.envs.navigation_envs.navigation_env_baseline import NavigationEnvBaseline


def test_cem_planner():
    """Test the CEM planner with a simple navigation environment."""
    
    print("Testing CEM Planner...")
    
    # Load a config file
    config_path = "configs/navigation_env_baseline_config.yaml"
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    else:
        # Default config if file doesn't exist
        config = {
            'env_config': {
                'name': 'NavigationEnvBaseline-v0',
                'dt': 1.0,
                'atol': 0.2,
                'rtol': 0.0,
                'multi_step_count': 5,
                'obstacle_mode': 'circles',
                'render_sleep': 0.0
            }
        }
    
    # Create environment
    print("Creating environment...")
    env = NavigationEnvBaseline(config=config['env_config'])
    
    # Create CEM planner
    print("Creating CEM planner...")
    planner = CEMPlanner(
        env=env,
        computational_budget_max=500,  # Small budget for quick test
        population_size=20,
        elite_fraction=0.2,
        planning_horizon=5,
        num_iterations=3,
        noise_scale=0.5,
    )
    
    # Reset environment and planner
    print("Resetting environment and planner...")
    env.reset(seed=42)
    planner.reset(seed=42)
    
    # Test planning
    print("Testing planning...")
    planner.plan_while_in_budget(computational=True, time=False)
    
    # Get best action
    best_action = planner.get_best_action()
    print(f"Best action: {best_action}")
    
    # Get planning info
    info = planner.get_planning_info()
    print(f"Planning info: {info}")
    
    # Test plan and step
    print("Testing plan and step...")
    obs, reward, terminated, truncated, info = planner.plan_and_step(computational=True, time=False)
    print(f"Observation: {obs}")
    print(f"Reward: {reward}")
    print(f"Terminated: {terminated}")
    print(f"Truncated: {truncated}")
    
    print("CEM Planner test completed successfully!")
    return True


if __name__ == "__main__":
    try:
        test_cem_planner()
        print("✅ All tests passed!")
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)