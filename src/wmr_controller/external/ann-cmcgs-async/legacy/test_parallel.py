#!/usr/bin/env python3
"""
Quick test script to demonstrate parallel episode execution
"""
import time
from experiments.run_experiment import ExperimentManager

def test_parallel_vs_sequential():
    """Compare parallel vs sequential execution times"""
    config_path = 'configs/epsilon_greedy/navigation_env_single_integrator_config.yaml'
    
    print("="*60)
    print("PARALLEL vs SEQUENTIAL EXECUTION TEST")
    print("="*60)
    
    # Test with few episodes for quick comparison
    num_episodes = 4
    
    # Test sequential execution
    print(f"\n🔄 Testing SEQUENTIAL execution ({num_episodes} episodes)...")
    start_time = time.time()
    
    manager_seq = ExperimentManager(config_path, 'MCGS', n_workers=1)
    # Temporarily reduce episodes for testing
    manager_seq.num_episodes = num_episodes
    results_seq = manager_seq.run_all_planners()
    
    sequential_time = time.time() - start_time
    
    # Test parallel execution
    print(f"\n⚡ Testing PARALLEL execution ({num_episodes} episodes, 2 workers)...")
    start_time = time.time()
    
    manager_par = ExperimentManager(config_path, 'MCGS', n_workers=2)
    # Temporarily reduce episodes for testing
    manager_par.num_episodes = num_episodes
    results_par = manager_par.run_all_planners()
    
    parallel_time = time.time() - start_time
    
    # Compare results
    print(f"\n📊 PERFORMANCE COMPARISON:")
    print(f"Sequential time: {sequential_time:.2f} seconds")
    print(f"Parallel time:   {parallel_time:.2f} seconds")
    print(f"Speedup:         {sequential_time/parallel_time:.2f}x")
    print(f"Efficiency:      {(sequential_time/parallel_time)/2*100:.1f}%")
    
    print(f"\n✅ Both executions completed successfully!")
    print("="*60)

if __name__ == "__main__":
    test_parallel_vs_sequential()