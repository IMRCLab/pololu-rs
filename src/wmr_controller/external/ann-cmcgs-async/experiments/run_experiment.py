import gymnasium as gym
import numpy as np
from typing import Optional, Tuple, Any
import sys
from types import SimpleNamespace
import yaml
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing

import tools.robots as robots
# import tools.envs.navigation_envs as navigation_envs
import tools.envs as envs
import tools.planners as planners

from time import sleep, time
from datetime import datetime
import logging
import matplotlib.pyplot as plt
import json
import os
import pandas as pd

from gymnasium.utils.env_checker import check_env

NOTICE_LEVEL_NUM = 25
logging.addLevelName(NOTICE_LEVEL_NUM, "NOTICE")

def notice(self, message, *args, **kws):
    if self.isEnabledFor(NOTICE_LEVEL_NUM):
        self._log(NOTICE_LEVEL_NUM, message, args, **kws)

logging.Logger.notice = notice
logging.basicConfig(level=logging.WARNING)
logger = logging.getLogger(__name__)


def _register_envs_for_parallel():
    """Register environments for parallel processes"""
    import gymnasium as gym
    
    envs = ['NavigationEnv', 'NavigationEnvSingleIntegrator', 'NavigationEnvSingleIntegratorUnicycle', 
            'NavigationEnvSingleIntegratorBrokenRudder', 'NavigationEnvDoubleIntegrator', 'NavigationEnvBaseline']
    entry_points = ['tools.envs.navigation_envs.navigation_env:NavigationEnv',
                    'tools.envs.navigation_envs.navigation_env_single_integrator:NavigationEnvSingleIntegrator',
                    'tools.envs.navigation_envs.navigation_env_single_integrator_unicycle:NavigationEnvSingleIntegratorUnicycle',
                    'tools.envs.navigation_envs.navigation_env_single_integrator_broken_rudder:NavigationEnvSingleIntegratorBrokenRudder',
                    'tools.envs.navigation_envs.navigation_env_double_integrator:NavigationEnvDoubleIntegrator',
                    'tools.envs.navigation_envs.navigation_env_baseline:NavigationEnvBaseline']
    
    for i, env in enumerate(envs):
        if env not in gym.envs.registry:
            gym.register(
                id=f'{env}-v0',
                entry_point=entry_points[i],
            )


def run_single_episode_parallel(args):
    """Run a single episode in a separate process"""
    try:
        (env_name, env_config, planner_name, planner_config, 
         computational_budget_max, time_budget_max, max_steps_per_episode, 
         episode_id, render_mode) = args
        
        # Import here to avoid pickling issues with multiprocessing
        import gymnasium as gym
        import tools.envs as envs
        import tools.planners as planners
        import numpy as np
        
        # Register environments (needed in each process)
        _register_envs_for_parallel()
        
        # Create environment (remove 'name' from config as it's not expected by the environment)
        env_config_clean = {k: v for k, v in env_config.items() if k != 'name'}
        env = gym.make(env_name, render_mode=render_mode, **env_config_clean)
        
        # Create planner
        if planner_name == 'MCGS':
            planner = planners.MCGSPlanner(
                env=env.unwrapped,
                computational_budget_max=computational_budget_max,
                time_budget_max=time_budget_max,
                **planner_config
            )
        elif planner_name == 'CMCGS':
            planner = planners.CMCGSWrapperPlanner(
                env=env.unwrapped,
                computational_budget_max=computational_budget_max,
                time_budget_max=time_budget_max,
                cmcgs_config=planner_config
            )
        else:
            raise ValueError(f"Unknown planner: {planner_name}. Available planners: MCGS, CMCGS")
        
        # Reset environment and planner
        observation, info = env.reset()
        planner.reset()
        
        # Run episode
        total_reward = 0
        step_count = 0
        terminated = False
        truncated = False
        
        while not (terminated or truncated) and step_count < max_steps_per_episode:
            observation, reward, terminated, truncated, info = planner.plan_and_step()
            step_count += 1
            total_reward += reward
        
        # Clean up
        env.close()
        
        return episode_id, float(total_reward), int(step_count)
        
    except Exception as e:
        try:
            print(f"Error in episode {episode_id}: {e}")
            return episode_id, 0.0, 0
        except:
            print(f"Error in episode (unknown ID): {e}")
            return 0, 0.0, 0


class ExperimentManager():
    def __init__(self, experiment_yaml: str, planner_name: Optional[str] = None, n_workers: Optional[int] = None):
        # Load config directly as dict
        with open(experiment_yaml, 'r') as f:
            self.config = yaml.safe_load(f)

        # Get environment and planner info
        self.env_name = self.config['environment']['name']
        self.planner_name = planner_name or self.config['default_planner']
        
        # Get experiment settings
        self.render_mode = self.config['experiment']['render_mode']
        self.computational_budget_max = self.config['experiment']['computational_budget_max']
        self.time_budget_max = self.config['experiment']['time_budget_max']
        self.save_replay = self.config['experiment'].get('save_replay', False)
        self.sleep_time = self.config['experiment']['sleep_time']
        self.max_steps = self.config['experiment']['max_steps']
        self.num_episodes = self.config['experiment']['num_episodes']
        self.n_workers = n_workers or 1  # Default to sequential execution
        
        # Provide helpful info about parallel execution
        if self.n_workers > 1:
            max_workers = multiprocessing.cpu_count()
            if self.n_workers > max_workers:
                print(f"Warning: Requested {self.n_workers} workers but only {max_workers} CPU cores available.")
            print(f"Using parallel execution with {self.n_workers} workers (CPU cores: {max_workers})")

        self.planner_config = self.config['planners'][self.planner_name]['config']
        self.env_config = self.config['environment']

        print(f"Planner config: {self.planner_config}")

        self.runner = Runner(env=self.env_name,
                             planner=self.planner_name,
                             computational_budget_max=self.computational_budget_max,
                             time_budget_max=self.time_budget_max,
                             env_config=self.env_config,
                             planner_config=self.planner_config,
                             render_mode=self.render_mode,
                             sleep_time=self.sleep_time,
                             save_replay=self.save_replay)
        print(f"Initialized ExperimentManager with environment {self.env_name} and planner {self.planner_name}")


    def run_all_planners(self):
        """Run experiments with all available planners in the config."""
        available_planners = list(self.config['planners'].keys())
        if len(available_planners) == 1:
            print("Only one planner configured. Running single planner.")
            return self.runner.run_multi_episode(
                num_episodes=self.num_episodes, 
                max_steps_per_episode=self.max_steps,
                n_workers=self.n_workers
            )
        
        print(f"\n{'='*80}")
        print(f"MULTI-PLANNER COMPARISON EXPERIMENT")
        print(f"{'='*80}")
        print(f"Environment: {self.env_name}")
        print(f"Planners: {', '.join(available_planners)}")
        print(f"Episodes per planner: {self.num_episodes}")
        print(f"Max steps per episode: {self.max_steps}")
        print(f"Computational budget: {self.computational_budget_max}")
        print(f"Time budget: {self.time_budget_max}")
        print(f"Workers: {self.n_workers} ({'parallel' if self.n_workers > 1 else 'sequential'})")
        print(f"{'='*80}\n")
        
        results = {}
        for planner_name in available_planners:
            print(f"\n=== Running experiment with {planner_name} ===")
            
            # Get the complete merged configuration for this planner
            planner_config = self.config['planners'][planner_name]['config']

            runner = Runner(env=self.env_name,
                            planner=planner_name,
                            computational_budget_max=self.computational_budget_max,
                            time_budget_max=self.time_budget_max,
                            env_config=self.env_config,
                            planner_config=planner_config,
                            render_mode=self.render_mode,
                            sleep_time=self.sleep_time,
                            save_replay=self.save_replay)

            rewards, steps = runner.run_multi_episode(
                num_episodes=self.num_episodes,
                max_steps_per_episode=self.max_steps,
                n_workers=self.n_workers
            )
            
            results[planner_name] = {
                'rewards': rewards,
                'steps': steps,
                'avg_reward': float(np.mean(rewards)),
                'avg_steps': float(np.mean(steps)),
                'std_reward': float(np.std(rewards)),
                'std_steps': float(np.std(steps)),
                'min_reward': float(np.min(rewards)),
                'max_reward': float(np.max(rewards)),
                'min_steps': float(np.min(steps)),
                'max_steps': float(np.max(steps)),
                'success_rate': float(np.mean([r > 0 for r in rewards]))  # Assuming positive reward means success
            }
            
            print(f"{planner_name} Results:")
            print(f"  Avg Reward: {results[planner_name]['avg_reward']:.3f} ± {results[planner_name]['std_reward']:.3f}")
            print(f"  Avg Steps: {results[planner_name]['avg_steps']:.1f} ± {results[planner_name]['std_steps']:.1f}")
            print(f"  Success Rate: {results[planner_name]['success_rate']*100:.1f}%")
        
        # Create comprehensive comparison log
        self._log_planner_comparison(results, available_planners)
        
        return results
    
    def _log_planner_comparison(self, results, planner_names):
        """Create a comprehensive comparison log and save to file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = "logs/planner_comparison"
        os.makedirs(log_dir, exist_ok=True)
        
        # Create comparison summary
        print(f"\n{'='*80}")
        print(f"PLANNER COMPARISON SUMMARY")
        print(f"{'='*80}")
        
        # Print comparison table
        print(f"{'Planner':<15} {'Avg Reward':<12} {'Std Reward':<12} {'Avg Steps':<10} {'Success %':<10}")
        print("-" * 70)
        
        best_avg_reward = max(results[p]['avg_reward'] for p in planner_names)
        best_success_rate = max(results[p]['success_rate'] for p in planner_names)
        lowest_avg_steps = min(results[p]['avg_steps'] for p in planner_names)
        
        for planner in planner_names:
            r = results[planner]
            reward_marker = " 🏆" if r['avg_reward'] == best_avg_reward else ""
            success_marker = " 🎯" if r['success_rate'] == best_success_rate else ""
            steps_marker = " ⚡" if r['avg_steps'] == lowest_avg_steps else ""
            
            print(f"{planner:<15} {r['avg_reward']:<12.3f} {r['std_reward']:<12.3f} "
                  f"{r['avg_steps']:<10.1f}{steps_marker} {r['success_rate']*100:<10.1f}{success_marker}{reward_marker}")
        
        print(f"\n🏆 Best Average Reward  🎯 Best Success Rate  ⚡ Fewest Steps")
        
        # Statistical comparison
        print(f"\nSTATISTICAL ANALYSIS:")
        print("-" * 40)
        for i, planner1 in enumerate(planner_names):
            for planner2 in planner_names[i+1:]:
                r1_rewards = np.array(results[planner1]['rewards'])
                r2_rewards = np.array(results[planner2]['rewards'])
                
                # Simple t-test approximation
                diff_mean = np.mean(r1_rewards - r2_rewards)
                diff_std = np.std(r1_rewards - r2_rewards)
                
                if abs(diff_mean) > 2 * diff_std / np.sqrt(len(r1_rewards)):
                    significance = "**"
                elif abs(diff_mean) > diff_std / np.sqrt(len(r1_rewards)):
                    significance = "*"
                else:
                    significance = ""
                
                winner = planner1 if diff_mean > 0 else planner2
                print(f"{planner1} vs {planner2}: {winner} wins by {abs(diff_mean):.3f} {significance}")
        
        # Save detailed results to JSON
        json_filename = f"{log_dir}/comparison_{timestamp}.json"
        comparison_data = {
            'timestamp': timestamp,
            'environment': self.env_name,
            'experiment_config': {
                'num_episodes': self.num_episodes,
                'max_steps': self.max_steps,
                'computational_budget_max': self.computational_budget_max,
                'time_budget_max': self.time_budget_max
            },
            'results': results,
            'summary': {
                'best_avg_reward': {'planner': max(planner_names, key=lambda p: results[p]['avg_reward']), 'value': best_avg_reward},
                'best_success_rate': {'planner': max(planner_names, key=lambda p: results[p]['success_rate']), 'value': best_success_rate},
                'lowest_avg_steps': {'planner': min(planner_names, key=lambda p: results[p]['avg_steps']), 'value': lowest_avg_steps}
            }
        }
        
        with open(json_filename, 'w') as f:
            json.dump(comparison_data, f, indent=2)
        
        # Save CSV for easy analysis
        csv_filename = f"{log_dir}/comparison_{timestamp}.csv"
        df_data = []
        for planner in planner_names:
            for episode, (reward, steps) in enumerate(zip(results[planner]['rewards'], results[planner]['steps'])):
                df_data.append({
                    'planner': planner,
                    'episode': episode + 1,
                    'reward': reward,
                    'steps': steps
                })
        
        df = pd.DataFrame(df_data)
        df.to_csv(csv_filename, index=False)
        
        print(f"\nDETAILED LOGS SAVED:")
        print(f"JSON: {json_filename}")
        print(f"CSV:  {csv_filename}")
        print(f"{'='*80}\n")


class Runner():
    def __init__(self, env: str, planner: str, computational_budget_max: int, time_budget_max: float, env_config: dict, planner_config: dict, render_mode: Optional[str] = None, sleep_time: float = 0.1, save_replay: bool = False):
        self._register_envs()
        if render_mode:
            assert render_mode in ['human', 'rgb_array', 'ansi'], "Render mode must be in ['human', 'rgb_array', 'ansi', 'matplotlib']."
        self.render_mode = render_mode
        self.planner_name = planner
        self.env_name = env 

        self.computational_budget_max = computational_budget_max
        self.time_budget_max = time_budget_max
        self.sleep_time = sleep_time
        self.save_replay = save_replay

        self.env_config = env_config
        self.planner_config = planner_config

        print(f"Creating environment: {env} with config: {self.env_config}")
        print(f"Using planner: {planner} with config: {self.planner_config}")

        self.env = gym.make(env, 
                            render_mode=render_mode,
                            dt=self.env_config['dt'],
                            atol=self.env_config['atol'],
                            rtol=self.env_config['rtol'],
                            multi_step_count=self.env_config['multi_step_count'],
                            obstacle_mode=self.env_config.get('obstacle_mode', 'circles'),
                            render_sleep=self.env_config.get('render_sleep', 0.0),
                            plan_in_space_time=self.env_config.get('plan_in_space_time', True),
                            default_step_divisor=self.env_config.get('default_step_divisor', 10),
                            sparse_reward=self.env_config.get('sparse_reward', True))
        assert isinstance(self.env.unwrapped, envs.GymRobotPlanEnv), "Environment must be a GymRobotPlanEnv."
        check_env(self.env.unwrapped, warn=True)
        self.env.reset()

        assert planner in ['MCGS', 'CMCGS', 'MCTS_Bootstrap'], "Planner must be 'MCGS', or 'CMCGS'."
        if planner == 'MCGS':

            self.planner = planners.MCGSPlanner(env=self.env.unwrapped,
                                               computational_budget_max=self.computational_budget_max,
                                               time_budget_max=self.time_budget_max,
                                               expand_n_times=self.planner_config['expand_n_times'],
                                               expand_n_times_min=self.planner_config['expand_n_times_min'],
                                               expand_n_times_max=self.planner_config['expand_n_times_max'],
                                               sample_best_X_actions=self.planner_config.get('sample_best_X_actions', 10),
                                               kappa=self.planner_config['kappa'],
                                               alpha=self.planner_config['alpha'],
                                               gamma=self.planner_config.get('gamma', 1.0),
                                               k=self.planner_config['k'],
                                               radius_threshold=self.planner_config['radius_threshold'],
                                               progressive_widening_method=self.planner_config['progressive_widening_method'],
                                               epsilon=self.planner_config.get('epsilon', 0.1),
                                               abstraction_refinement_exponent=self.planner_config['abstraction_refinement_exponent'],
                                               c_uct=self.planner_config['c_uct'],
                                               plan_in_space_time=self.planner_config['plan_in_space_time'],
                                               use_controller=self.planner_config['use_controller'],
                                               tracking_tolerance=self.planner_config['tracking_tolerance'],
                                               yield_mode=self.planner_config['yield_mode'],
                                               random_rollout_n_times=self.planner_config['random_rollout_n_times'],
                                               random_rollout_length=self.planner_config['random_rollout_length'],
                                               force_exact_update=self.planner_config.get('force_exact_update', True),
                                               save_replay=self.save_replay)
        elif planner == 'CMCGS':
            self.planner = planners.CMCGSWrapperPlanner(env=self.env.unwrapped,
                                                        computational_budget_max=self.computational_budget_max,
                                                        time_budget_max=self.time_budget_max,
                                                        cmcgs_config=self.planner_config,
                                                        save_replay=self.save_replay)
        elif planner == 'MCTS_Bootstrap':
            self.planner = planners.MCGSPlanner(env=self.env.unwrapped,
                                               computational_budget_max=self.computational_budget_max,
                                               time_budget_max=self.time_budget_max,
                                               expand_n_times=self.planner_config['expand_n_times'],
                                               expand_n_times_min=self.planner_config['expand_n_times_min'],
                                               expand_n_times_max=self.planner_config['expand_n_times_max'],
                                               sample_best_X_actions=self.planner_config.get('sample_best_X_actions', 10),
                                               kappa=self.planner_config['kappa'],
                                               alpha=self.planner_config['alpha'],
                                               gamma=self.planner_config.get('gamma', 1.0),
                                               k=self.planner_config['k'],
                                               radius_threshold=self.planner_config['radius_threshold'],
                                               progressive_widening_method=self.planner_config['progressive_widening_method'],
                                               epsilon=self.planner_config.get('epsilon', 0.1),
                                               abstraction_refinement_exponent=self.planner_config['abstraction_refinement_exponent'],
                                               c_uct=self.planner_config['c_uct'],
                                               plan_in_space_time=self.planner_config['plan_in_space_time'],
                                               use_controller=self.planner_config['use_controller'],
                                               tracking_tolerance=self.planner_config['tracking_tolerance'],
                                               yield_mode=self.planner_config['yield_mode'],
                                               random_rollout_n_times=self.planner_config['random_rollout_n_times'],
                                               random_rollout_length=self.planner_config['random_rollout_length'],
                                               force_exact_update=self.planner_config.get('force_exact_update', True),
                                               save_replay=self.save_replay)
        


    def _register_envs(self):
        envs = ['NavigationEnv', 'NavigationEnvSingleIntegrator', 'NavigationEnvSingleIntegratorUnicycle', 'NavigationEnvSingleIntegratorBrokenRudder', 'NavigationEnvDoubleIntegrator', 'NavigationEnvBaseline']
        entry_points = ['tools.envs.navigation_envs.navigation_env:NavigationEnv',
                        'tools.envs.navigation_envs.navigation_env_single_integrator:NavigationEnvSingleIntegrator',
                        'tools.envs.navigation_envs.navigation_env_single_integrator_unicycle:NavigationEnvSingleIntegratorUnicycle',
                        'tools.envs.navigation_envs.navigation_env_single_integrator_broken_rudder:NavigationEnvSingleIntegratorBrokenRudder',
                        'tools.envs.navigation_envs.navigation_env_double_integrator:NavigationEnvDoubleIntegrator',
                        'tools.envs.navigation_envs.navigation_env_baseline:NavigationEnvBaseline']
        for i, env in enumerate(envs):
            if env not in gym.envs.registry:
                print(f"Registering environment: {env}")
                gym.register(
                    id=f'{env}-v0',
                    entry_point=entry_points[i],)
            else:
                pass


    def run(self, max_steps: int = 10) -> Tuple[float, int]:
        self.planner.reset()
        episode_over = False
        step_count = 0
        total_reward = 0

        while not episode_over and step_count < max_steps:
            observation, reward, terminated, truncated, info = self.planner.plan_and_step()
            # print(observation, reward, terminated, truncated, info)
            episode_over = terminated or truncated

            if self.render_mode:
                self.env.render()
                sleep(self.sleep_time)

            step_count += 1
            total_reward += reward

        logger.notice(f"Episode finished with total reward of {total_reward} after {step_count} steps.")
        return total_reward, step_count
    

    def run_multi_episode(self, num_episodes: int = 10, max_steps_per_episode: int = 10, n_workers: Optional[int] = None):
        """
        Run multiple episodes either sequentially or in parallel.
        
        Args:
            num_episodes: Number of episodes to run
            max_steps_per_episode: Maximum steps per episode
            n_workers: Number of parallel workers. If None or 1, runs sequentially.
                      If > 1, runs in parallel using multiprocessing.
        """
        if n_workers is None or n_workers == 1:
            # Sequential execution (original behavior)
            return self._run_multi_episode_sequential(num_episodes, max_steps_per_episode)
        else:
            # Parallel execution
            return self._run_multi_episode_parallel(num_episodes, max_steps_per_episode, n_workers)
    
    def _run_multi_episode_sequential(self, num_episodes: int, max_steps_per_episode: int):
        """Original sequential implementation"""
        rewards = []
        steps = []
        for episode in range(num_episodes):
            logger.notice(f"Starting episode {episode + 1}/{num_episodes}")
            total_reward, step_count = self.run(max_steps=max_steps_per_episode)
            rewards.append(float(total_reward))
            steps.append(int(step_count))
            self.planner.reset()
        avg_reward = np.mean(rewards)
        avg_steps = np.mean(steps)
        return self._finalize_multi_episode_results(rewards, steps, num_episodes, avg_reward, avg_steps)
    
    def _run_multi_episode_parallel(self, num_episodes: int, max_steps_per_episode: int, n_workers: int):
        """Parallel implementation with fallback to sequential for circular import issues"""
        logger.notice(f"Starting {num_episodes} episodes in parallel with {n_workers} workers")
        
        try:
            # Try parallel execution first
            return self._try_parallel_execution(num_episodes, max_steps_per_episode, n_workers)
        except Exception as e:
            logger.error(f"Parallel execution failed due to: {e}")
            logger.notice("Falling back to sequential execution")
            return self._run_multi_episode_sequential(num_episodes, max_steps_per_episode)
    
    def _try_parallel_execution(self, num_episodes: int, max_steps_per_episode: int, n_workers: int):
        """Attempt parallel execution"""
        # Prepare arguments for parallel execution
        episode_args = []
        for episode_id in range(num_episodes):
            args = (
                self.env_name,
                self.env_config,
                self.planner_name,
                self.planner_config,
                self.computational_budget_max,
                self.time_budget_max,
                max_steps_per_episode,
                episode_id,
                'ansi'  # Force ansi mode for parallel to avoid rendering issues
            )
            episode_args.append(args)
        
        rewards = [0.0] * num_episodes
        steps = [0] * num_episodes
        
        # Run episodes in parallel
        with ProcessPoolExecutor(max_workers=n_workers) as executor:
            # Submit all episodes
            future_to_episode = {
                executor.submit(run_single_episode_parallel, args): args[7]  # episode_id is at index 7
                for args in episode_args
            }
            
            # Collect results as they complete
            completed = 0
            for future in as_completed(future_to_episode):
                episode_id = future_to_episode[future]
                try:
                    result_episode_id, reward, step_count = future.result()
                    rewards[result_episode_id] = reward
                    steps[result_episode_id] = step_count
                    completed += 1
                    logger.notice(f"Completed episode {result_episode_id + 1}/{num_episodes} (Total: {completed}/{num_episodes})")
                except Exception as exc:
                    logger.error(f"Episode {episode_id} generated an exception: {exc}")
                    rewards[episode_id] = 0.0
                    steps[episode_id] = 0
        
        avg_reward = np.mean(rewards)
        avg_steps = np.mean(steps)
        return self._finalize_multi_episode_results(rewards, steps, num_episodes, avg_reward, avg_steps)
    
    def _finalize_multi_episode_results(self, rewards, steps, num_episodes, avg_reward, avg_steps):
        logger.notice(f"Average reward over {num_episodes} episodes: {avg_reward}")
        logger.notice(f"Average steps over {num_episodes} episodes: {avg_steps}")

        logs = {
            'avg_reward': float(avg_reward),
            'avg_steps': float(avg_steps),
            'computational_budget_total': int(self.computational_budget_max),  # Use config value for parallel
            'time_budget_total': float(self.time_budget_max),  # Use config value for parallel
            'env_config': self.env_config,
            'planner_config': self.planner_config,
            'rewards': rewards,
            'steps': steps,
        }

        identifier = f"{self.env_name}_{self.planner_name}"
        datetime_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.save_logs(f'logs/{self.env_name}/{self.planner_name}', f'{datetime_id}_experiment_logs.json', logs)
        return rewards, steps

    def save_logs(self, directory: str, filename: str, data: dict):
        import json
        # mkdir if not exists
        import os
        if not os.path.exists(directory):
            os.makedirs(directory)
        with open(f"{directory}/{filename}", 'w') as f:
            json.dump(data, f)
        logger.notice(f"Logs saved to {directory}/{filename}")


if __name__ == "__main__":
    import argparse

    # parser = argparse.ArgumentParser(description="Run a planning experiment in a gym environment.")
    # parser.add_argument('--env', type=str, required=True, help='The gym environment to use.')
    # parser.add_argument('--planner', type=str, required=True, choices=['MCGS', 'CMCGS'], help='The planner to use.')
    # parser.add_argument('--render_mode', type=str, choices=['human', 'rgb_array'], default=None, help='Render mode for the environment.')
    # parser.add_argument('--computational_budget_max', type=int, default=100, help='Max computational budget for the planner per step.')
    # parser.add_argument('--time_budget_max', type=float, default=20.0, help='Max time budget for the planner per step.')
    # parser.add_argument('--sleep_time', type=float, default=0.1, help='Sleep time between steps when rendering.')
    # parser.add_argument('--max_steps', type=int, default=1000, help='Maximum number of steps per episode.')
    # parser.add_argument('--num_episodes', type=int, default=1, help='Number of episodes to run.')
    # parser.add_argument('--config', type=lambda s: eval(s), default=None, help='Optional configuration dictionary for the planner.')

    # args = parser.parse_args()

    # runner = Runner(env=args.env,
    #                 planner=args.planner,
    #                 render_mode=args.render_mode,
    #                 computational_budget_max=args.computational_budget_max,
    #                 time_budget_max=args.time_budget_max,
    #                 sleep_time=args.sleep_time,
    #                 config=args.config)

    # runner.run_multi_episode(num_episodes=args.num_episodes, max_steps_per_episode=args.max_steps)

    # Example usage with ExperimentManager
    parser = argparse.ArgumentParser(description="Run planning experiments from a YAML config.")
    parser.add_argument('--config', type=str, required=True, help='Path to the experiment YAML configuration file.')
    parser.add_argument('--planner', type=str, default=None, help='Optional planner name to override the config default.')
    parser.add_argument('--workers', type=int, default=1, help='Number of parallel workers for episode execution (default: 1 for sequential).')

    args = parser.parse_args()
    manager = ExperimentManager(experiment_yaml=args.config, planner_name=args.planner, n_workers=args.workers)
    results = manager.run_all_planners()
    
    if results and len(results) > 1:
        print("\nEXPERIMENT COMPLETED SUCCESSFULLY!")
        print("Check the logs/planner_comparison/ directory for detailed results.")
    elif results:
        print(f"\nSingle planner experiment completed with {args.planner or 'default planner'}.")