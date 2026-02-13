#!/usr/bin/env python3
"""
Test script to verify figure suppression during optimization.
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Suppress matplotlib GUI to prevent figures from showing during optimization
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend

from experiments.bayesian_optimization import BayesianHyperparameterOptimizer

def test_figure_suppression():
    """Test that optimization runs without showing any GUI figures."""
    print("Testing figure suppression during hyperparameter optimization...")
    
    try:
        # Create a minimal optimizer
        optimizer = BayesianHyperparameterOptimizer(
            env_name='NavigationEnvBaseline-v0',
            planner_name='MCGS',
            n_episodes_per_trial=1,  # Very minimal for testing
            max_steps_per_episode=100
        )
        
        print("✓ Optimizer created successfully")
        
        # Run a few trials
        results = optimizer.optimize(n_trials=2)
        print(f"✓ Optimization completed - Best value: {results['best_value']:.4f}")
        
        # Try to create plots (should save to file, not show GUI)
        optimizer.plot_optimization_history()
        print("✓ Plot saved to file without showing GUI")
        
        return True
        
    except Exception as e:
        print(f"✗ Error during optimization: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("=" * 50)
    print("Testing Figure Suppression")
    print("=" * 50)
    
    success = test_figure_suppression()
    
    print("\n" + "=" * 50)
    if success:
        print("✓ Figure suppression test passed!")
        print("No GUI windows should have appeared.")
    else:
        print("✗ Figure suppression test failed!")
    print("=" * 50)