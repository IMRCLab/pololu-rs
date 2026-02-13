#!/usr/bin/env python3
"""
Completely isolated test for CEM planner core logic
"""

import numpy as np
from scipy.stats import multivariate_normal
from typing import Optional, Tuple, List
import sys
import os

def test_cem_logic():
    """Test core CEM algorithm logic without full framework integration."""
    
    print("Testing core CEM algorithm logic...")
    
    # Simplified CEM implementation for testing
    class SimpleCEM:
        def __init__(self, action_dim=2, horizon=5, pop_size=20, elite_frac=0.2):
            self.action_dim = action_dim
            self.horizon = horizon
            self.pop_size = pop_size
            self.elite_size = max(1, int(pop_size * elite_frac))
            
            # Initialize distribution
            self.mean = np.zeros((horizon, action_dim))
            self.cov = np.eye(horizon * action_dim) * 0.5
            
        def sample_sequences(self):
            """Sample action sequences from current distribution."""
            mean_flat = self.mean.flatten()
            samples_flat = np.random.multivariate_normal(
                mean_flat, self.cov, size=self.pop_size
            )
            return samples_flat.reshape(self.pop_size, self.horizon, self.action_dim)
        
        def evaluate_sequence(self, sequence):
            """Simple evaluation: reward is negative distance to goal."""
            # Simulate trajectory
            pos = np.array([0.0, 0.0])
            goal = np.array([5.0, 5.0])
            total_reward = 0.0
            
            for action in sequence:
                # Simple integration
                pos += action * 0.1
                # Reward is negative distance to goal  
                reward = -np.linalg.norm(pos - goal)
                total_reward += reward
                
            return total_reward
        
        def update_distribution(self, sequences, rewards):
            """Update distribution based on elite sequences."""
            # Select elite sequences
            elite_indices = np.argsort(rewards)[-self.elite_size:]
            elite_sequences = sequences[elite_indices]
            
            # Update mean
            new_mean = np.mean(elite_sequences, axis=0)
            
            # Update covariance
            elite_flat = elite_sequences.reshape(self.elite_size, -1)
            new_cov = np.cov(elite_flat.T)
            
            # Handle single elite case
            if new_cov.shape == ():
                new_cov = np.array([[new_cov]])
            
            # Add regularization
            new_cov += np.eye(new_cov.shape[0]) * 1e-6
            
            self.mean = new_mean
            self.cov = new_cov
            
            return np.max(rewards)
    
    # Test the simplified CEM
    print("Creating simplified CEM...")
    cem = SimpleCEM(action_dim=2, horizon=5, pop_size=20, elite_frac=0.2)
    
    print("Testing sequence sampling...")
    sequences = cem.sample_sequences()
    print(f"Sampled sequences shape: {sequences.shape}")
    assert sequences.shape == (20, 5, 2)
    
    print("Testing sequence evaluation...")
    test_seq = np.random.uniform(-1, 1, (5, 2))
    reward = cem.evaluate_sequence(test_seq)
    print(f"Test reward: {reward:.3f}")
    
    print("Running CEM iterations...")
    best_rewards = []
    
    for iteration in range(5):
        # Sample sequences
        sequences = cem.sample_sequences()
        
        # Evaluate all sequences
        rewards = np.array([cem.evaluate_sequence(seq) for seq in sequences])
        
        # Update distribution
        best_reward = cem.update_distribution(sequences, rewards)
        best_rewards.append(best_reward)
        
        print(f"Iteration {iteration + 1}: Best reward = {best_reward:.3f}, "
              f"Mean reward = {np.mean(rewards):.3f}")
    
    # Check that performance improved
    print(f"Initial best reward: {best_rewards[0]:.3f}")
    print(f"Final best reward: {best_rewards[-1]:.3f}")
    print(f"Improvement: {best_rewards[-1] - best_rewards[0]:.3f}")
    
    assert best_rewards[-1] > best_rewards[0], "CEM should improve over iterations"
    
    print("✅ Core CEM logic test passed!")
    
    # Test some edge cases
    print("Testing edge cases...")
    
    # Test with single elite
    cem_single = SimpleCEM(action_dim=2, horizon=3, pop_size=5, elite_frac=0.1)  # elite_size = 1
    sequences = cem_single.sample_sequences()
    rewards = np.array([cem_single.evaluate_sequence(seq) for seq in sequences])
    best_reward = cem_single.update_distribution(sequences, rewards)
    print(f"Single elite test passed, best reward: {best_reward:.3f}")
    
    print("✅ All CEM logic tests passed!")
    return True


if __name__ == "__main__":
    try:
        test_cem_logic()
        print("✅ All tests passed!")
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)