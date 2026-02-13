#!/usr/bin/env python3
"""
Test script to verify all navigation environments are available.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import gymnasium as gym
import yaml
from experiments.run_experiment import Runner

def test_environments():
    """Test that all navigation environments can be created."""
    environments = [
        'NavigationEnvBaseline-v0',
        'NavigationEnvSingleIntegrator-v0',
        'NavigationEnvSingleIntegratorUnicycle-v0', 
        'NavigationEnvDoubleIntegrator-v0',
        'NavigationEnvSingleIntegratorUnicycleBrokenRudder-v0'
    ]
    
    print("Testing environment availability...")
    print("=" * 50)
    
    # Load baseline config as dummy config for testing
    try:
        with open('configs/navigation_env_baseline_config.yaml', 'r') as f:
            baseline_config = yaml.safe_load(f)
        
        # Extract and merge config sections for the dummy runner
        dummy_config = baseline_config['planners']['MCGS']['config'].copy()
        dummy_config.update(baseline_config['environment'])
        dummy_config.update(baseline_config['experiment'])
        
        dummy_runner = Runner('NavigationEnvBaseline-v0', 'MCGS', config=dummy_config, render_mode=None)
        print("✓ Environment registration successful")
    except Exception as e:
        print(f"✗ Failed to create dummy runner: {e}")
        return False
    
    available_envs = []
    failed_envs = []
    
    for env_name in environments:
        try:
            print(f"Testing {env_name}...", end=" ")
            env = gym.make(env_name, render_mode=None)
            obs, info = env.reset(seed=42)
            env.close()
            print("✓")
            available_envs.append(env_name)
        except Exception as e:
            print(f"✗ - {e}")
            failed_envs.append((env_name, str(e)))
    
    print("\n" + "=" * 50)
    print("RESULTS:")
    print(f"✅ Available environments ({len(available_envs)}):")
    for env in available_envs:
        print(f"   - {env}")
    
    if failed_envs:
        print(f"\n❌ Failed environments ({len(failed_envs)}):")
        for env, error in failed_envs:
            print(f"   - {env}: {error}")
    
    return len(failed_envs) == 0

if __name__ == "__main__":
    success = test_environments()
    if success:
        print("\n🎉 All environments are ready for optimization!")
    else:
        print("\n⚠️  Some environments have issues. Please fix before running optimization.")