#!/usr/bin/env python3
"""
Simplified test script for the CEM Planner that avoids circular imports
"""

import sys
import os
import numpy as np

# Add the project root to the path
sys.path.insert(0, os.path.abspath('.'))

def test_cem_planner_standalone():
    """Test the CEM planner logic without full environment integration."""
    
    print("Testing CEM Planner logic...")
    
    # Import only what we need directly
    from tools.planners.cem_planner import CEMPlanner
    from tools.robots.single_integrator import SingleIntegrator
    from tools.envs.gym_robot_plan_env import GymRobotPlanEnv
    from gymnasium import spaces
    
    # Create a simple robot
    robot = SingleIntegrator(
        id=0, 
        name="test_robot",
        dt=0.1,
        state=np.array([0.0, 0.0])
    )
    
    # Create a minimal environment
    env = GymRobotPlanEnv(
        agent=robot,
        render_mode=None,
        multi_step_count=1
    )
    
    # Set goal for testing
    robot.state_goal = np.array([5.0, 5.0])
    
    print("Creating CEM planner...")
    planner = CEMPlanner(
        env=env,
        computational_budget_max=100,  # Small budget for quick test
        population_size=10,
        elite_fraction=0.3,
        planning_horizon=3,
        num_iterations=2,
        noise_scale=1.0,
    )
    
    print("Testing planner initialization...")
    assert planner.population_size == 10
    assert planner.elite_size == 3  # 30% of 10
    assert planner.planning_horizon == 3
    assert planner.action_dim == 2  # Single integrator has 2D action space
    
    print("Testing action sequence sampling...")
    planner.reset()
    sequences = planner._sample_action_sequences()
    print(f"Sampled sequences shape: {sequences.shape}")
    assert sequences.shape == (10, 3, 2)  # (pop_size, horizon, action_dim)
    
    print("Testing single sequence evaluation...")
    test_sequence = np.random.uniform(-1, 1, (3, 2))
    reward = planner._evaluate_sequence(test_sequence)
    print(f"Test sequence reward: {reward}")
    
    print("Testing planning...")
    planner.reset()
    planner.plan()
    
    print("Testing get_best_action...")
    action = planner.get_best_action()
    print(f"Best action: {action}")
    assert action.shape == (2,)
    
    print("Testing planning info...")
    info = planner.get_planning_info()
    print(f"Planning info: {info}")
    
    print("✅ CEM Planner standalone test completed successfully!")
    return True


if __name__ == "__main__":
    try:
        test_cem_planner_standalone()
        print("✅ All tests passed!")
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)