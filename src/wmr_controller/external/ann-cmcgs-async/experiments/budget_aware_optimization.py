import optuna
import numpy as np
import yaml
import json
import os
import pandas as pd
from datetime import datetime
from typing import Dict, List, Optional, Any
import logging
from types import SimpleNamespace
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading

# Suppress matplotlib GUI to prevent figures from showing during optimization
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
plt.ioff()  # Turn off interactive mode to prevent figure display

from experiments.run_experiment import Runner


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class BudgetAwareHyperparameterOptimizer:
    def __init__(self, 
                 experiment_yaml: str,
                 planner_name: str,
                 budget_range: List[int],
                 time_budget_fixed: float = 20.0,
                 n_episodes_per_trial: int = 5,
                 max_steps_per_episode: int = 40,
                 study_name_prefix: Optional[str] = None,
                 storage: Optional[str] = None,
                 parallel_episodes: bool = True,
                 max_workers: Optional[int] = None):
        """
        Budget-aware hyperparameter optimizer that optimizes parameters for each computational budget level.
        
        Args:
            experiment_yaml: Path to the experiment YAML configuration file
            planner_name: 'MCGS' or 'CMCGS'
            budget_range: List of computational budgets to optimize for [50, 100, 150, 200, ...]
            time_budget_fixed: Fixed time budget (not optimized)
            n_episodes_per_trial: Number of episodes to run per trial
            parallel_episodes: Whether to run episodes in parallel (default: True)
            max_workers: Maximum number of parallel workers (default: None = auto)
            max_steps_per_episode: Maximum steps per episode
            study_name_prefix: Prefix for study names
            storage: Optuna storage backend
        """
        # Load YAML config directly
        with open(experiment_yaml, 'r') as f:
            self.config = yaml.safe_load(f)

        # Get environment and planner info
        self.env_name = self.config['environment']['name']
        self.planner_name = planner_name or self.config.get('default_planner')
        
        self.budget_range = sorted(budget_range)
        self.time_budget_fixed = time_budget_fixed
        self.n_episodes_per_trial = n_episodes_per_trial
        self.max_steps_per_episode = max_steps_per_episode
        self.parallel_episodes = parallel_episodes
        self.max_workers = max_workers
        
        # Get base planner configuration
        self.base_planner_config = dict(self.config['planners'][self.planner_name]['config'])
        
        # Create studies for each budget
        self.studies = {}
        prefix = study_name_prefix or f"{planner_name}_{self.env_name}"
        
        for budget in self.budget_range:
            study_name = f"{prefix}_budget_{budget}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            self.studies[budget] = optuna.create_study(
                study_name=study_name,
                direction='maximize',
                storage=storage,
                load_if_exists=True
            )
        
        # Results storage
        self.results_dir = f'logs/hyperopt/{self.planner_name}/budget_aware'
        os.makedirs(self.results_dir, exist_ok=True)
        
        # CSV tracking initialization
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_dir = f'logs/csv_tracking/{self.env_name}/{self.planner_name}'
        os.makedirs(self.csv_dir, exist_ok=True)
        
        # Initialize comprehensive CSV files
        self._init_csv_tracking()
    
    @classmethod
    def from_config_file(cls, config_file: str, budget_range: List[int], **kwargs):
        """Create optimizer from a config file (like those used in run_experiment.py)."""
        with open(config_file, 'r') as f:
            config_data = yaml.safe_load(f)
        # Use default planner or specified planner
        default_planner = kwargs.get('planner_name', config_data.get('default_planner'))

        return cls(
            experiment_yaml=config_file,
            planner_name=default_planner,
            budget_range=budget_range,
            **{k: v for k, v in kwargs.items() if k != 'planner_name'}
        )
    
    def _init_csv_tracking(self):
        """Initialize CSV files for comprehensive tracking."""
        # Master results CSV - tracks all runs across all budgets
        self.master_csv_path = os.path.join(self.csv_dir, f"{self.timestamp}_all_runs.csv")
        master_columns = [
            'timestamp', 'trial_number', 'budget', 'mean_reward', 'std_reward', 
            'mean_steps', 'std_steps', 'success_rate'
        ]
        
        # Universal parameter columns that can handle all planner types
        # MCGS parameters
        mcgs_params = [
            'expand_n_times', 'expand_n_times_min', 'expand_n_times_max', 'sample_best_X_actions',
            'kappa', 'alpha', 'k', 'radius_threshold', 'progressive_widening_method', 'epsilon',
            'c_uct', 'tracking_tolerance', 'abstraction_refinement_exponent', 'yield_mode',
            'random_rollout_length'
        ]
        
        # CMCGS parameters  
        cmcgs_params = [
            'min_graph_length', 'rollout_length', 'elite_ratio', 'optimal_prob',
            'optimal_n_top', 'optimal_range', 'min_update_divisor', 'cmcgs_alpha', 'beta'
        ]
        
        # CEM parameters (for future compatibility)
        cem_params = [
            'population_size', 'elite_fraction', 'planning_horizon', 'num_iterations',
            'noise_scale', 'noise_decay', 'smoothing_factor'
        ]
        
        # Combine all parameter columns for universal compatibility
        all_param_columns = mcgs_params + cmcgs_params + cem_params
        master_columns.extend(all_param_columns)
        
        # Individual episode results CSV - tracks every single episode
        self.episodes_csv_path = os.path.join(self.csv_dir, f"{self.timestamp}_episodes.csv")
        episode_columns = [
            'timestamp', 'trial_number', 'budget', 'episode_number', 
            'reward', 'steps', 'success'
        ]
        episode_columns.extend(all_param_columns)
        
        # Best runs summary CSV - tracks the best performing configuration for each budget
        self.best_runs_csv_path = os.path.join(self.csv_dir, f"{self.timestamp}_best_runs.csv")
        best_columns = [
            'budget', 'best_trial_number', 'best_mean_reward', 'best_std_reward',
            'best_mean_steps', 'best_success_rate', 'optimization_timestamp'
        ]
        best_columns.extend(all_param_columns)
        
        # Initialize CSV files with headers
        pd.DataFrame(columns=master_columns).to_csv(self.master_csv_path, index=False)
        pd.DataFrame(columns=episode_columns).to_csv(self.episodes_csv_path, index=False)
        pd.DataFrame(columns=best_columns).to_csv(self.best_runs_csv_path, index=False)
        
        logger.info(f"CSV tracking initialized:")
        logger.info(f"  Master results: {self.master_csv_path}")
        logger.info(f"  Episode details: {self.episodes_csv_path}")
        logger.info(f"  Best runs: {self.best_runs_csv_path}")
    
    def _log_trial_to_csv(self, trial_number: int, budget: int, rewards: List[float], 
                          steps: List[int], params: Dict[str, Any]):
        """Log a trial's results to CSV files."""
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Calculate metrics
        mean_reward = float(np.mean(rewards))
        std_reward = float(np.std(rewards))
        mean_steps = float(np.mean(steps))
        std_steps = float(np.std(steps))
        success_rate = float(np.mean([r > 0 for r in rewards]))  # Assuming positive reward = success
        
        # Log to master results CSV
        master_row = {
            'timestamp': current_time,
            'trial_number': trial_number,
            'budget': budget,
            'mean_reward': mean_reward,
            'std_reward': std_reward,
            'mean_steps': mean_steps,
            'std_steps': std_steps,
            'success_rate': success_rate,
            **params
        }
        
        master_df = pd.DataFrame([master_row])
        master_df.to_csv(self.master_csv_path, mode='a', header=False, index=False)
        
        # Log individual episodes
        for i, (reward, step) in enumerate(zip(rewards, steps)):
            episode_row = {
                'timestamp': current_time,
                'trial_number': trial_number,
                'budget': budget,
                'episode_number': i + 1,
                'reward': reward,
                'steps': step,
                'success': 1 if reward > 0 else 0,
                **params
            }
            
            episode_df = pd.DataFrame([episode_row])
            episode_df.to_csv(self.episodes_csv_path, mode='a', header=False, index=False)
    
    def _update_best_runs_csv(self, budget: int, best_trial, best_params: Dict[str, Any]):
        """Update the best runs CSV with the best configuration for a budget."""
        # Read existing best runs
        try:
            best_df = pd.read_csv(self.best_runs_csv_path)
        except (FileNotFoundError, pd.errors.EmptyDataError):
            best_df = pd.DataFrame()
        
        # Remove any existing entry for this budget
        best_df = best_df[best_df['budget'] != budget]
        
        # Add new best entry
        best_row = {
            'budget': budget,
            'best_trial_number': best_trial.number,
            'best_mean_reward': best_trial.value,
            'best_std_reward': best_trial.user_attrs.get('std_reward', 0),
            'best_mean_steps': best_trial.user_attrs.get('mean_steps', 0),
            'best_success_rate': best_trial.user_attrs.get('success_rate', 0),
            'optimization_timestamp': datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            **best_params
        }
        
        best_df = pd.concat([best_df, pd.DataFrame([best_row])], ignore_index=True)
        best_df = best_df.sort_values('budget')
        best_df.to_csv(self.best_runs_csv_path, index=False)
        
        logger.info(f"Updated best runs CSV for budget {budget}")
    
    def suggest_mcgs_params(self, trial: optuna.Trial, budget: int) -> Dict[str, Any]:
        """Suggest MCGS hyperparameters for a specific budget."""
        return {
            # Fixed budget parameters
            'computational_budget_max': budget,
            'time_budget_max': self.time_budget_fixed,
            
            # Optimized parameters
            'expand_n_times_min': trial.suggest_int('expand_n_times_min', 1, 5),
            'expand_n_times_max': trial.suggest_int('expand_n_times_max', 5, 7),
            'sample_best_X_actions': trial.suggest_int('sample_best_X_actions', 5, 20),
            'kappa': trial.suggest_float('kappa', 0.1, 2.0),
            'alpha': trial.suggest_float('alpha', 0.1, 1.0),
            'k': trial.suggest_int('k', 5, min(30, budget // 5)),  # Scale k with budget
            'radius_threshold': trial.suggest_float('radius_threshold', 0.1, 5.0),
            'c_uct': trial.suggest_float('c_uct', 0.1, 0.8),
            'tracking_tolerance': trial.suggest_float('tracking_tolerance', 0.1, 0.5),
            'abstraction_refinement_exponent': trial.suggest_float('abstraction_refinement_exponent', -1.0, 0.0),
            'yield_mode': trial.suggest_categorical('yield_mode', ['N', 'Q']),
            # 'random_rollout_n_times': trial.suggest_int('random_rollout_n_times', 5, min(50, budget // 2)),
            'random_rollout_length': trial.suggest_int('random_rollout_length', 1, 10),
            
            # Fixed parameters
            'progressive_widening_method': trial.suggest_categorical('progressive_widening_method', ['default', 'expansion_attempts', 'epsilon_greedy']),
            'epsilon': trial.suggest_float('epsilon', 0.01, 0.5),
            'expand_n_times': 1,
            'plan_in_space_time': True,
            'use_controller': True,
            # 'yield_mode': 'N',
            'random_rollout_n_times': 1,

        }
    
    def suggest_cmcgs_params(self, trial: optuna.Trial, budget: int) -> Dict[str, Any]:
        """Suggest CMCGS hyperparameters for a specific budget."""
        return {
            # Fixed budget parameters
            'simulation_budget': budget,
            'time_budget_max': self.time_budget_fixed,
            
            # Optimized parameters
            'min_graph_length': trial.suggest_int('min_graph_length', 2, 10),
            'rollout_length': trial.suggest_int('rollout_length', 3, 15),
            'elite_ratio': trial.suggest_float('elite_ratio', 0.05, 0.3),
            'optimal_prob': trial.suggest_float('optimal_prob', 0.05, 0.3),
            'optimal_n_top': trial.suggest_int('optimal_n_top', 3, 15),
            'optimal_range': trial.suggest_float('optimal_range', 0.1, 1.0),
            'min_update_divisor': trial.suggest_int('min_update_divisor', 1, 5),
            'alpha': trial.suggest_int('alpha', 1, 10),
            'beta': trial.suggest_int('beta', 1, 10),
            
            # Fixed parameters
            'clustering_alg': 'agglomerative',
            'max_n_clusters': 2147483646,
        }
    
    def _run_single_episode(self, runner, episode_num: int, max_steps: int):
        """Helper method to run a single episode (for parallel execution)."""
        try:
            # Create a fresh copy of the runner to avoid threading issues
            # Each thread needs its own environment and planner instances
            thread_runner = Runner(
                env=runner.env_name,
                planner=runner.planner_name,
                computational_budget_max=runner.planner.computational_budget_max,
                time_budget_max=runner.planner.time_budget_max,
                env_config=runner.env_config,
                planner_config=runner.planner_config,
                render_mode=None,
                sleep_time=0.0,
                save_replay=False
            )
            
            # Run single episode
            total_reward, step_count = thread_runner.run(max_steps=max_steps)
            return float(total_reward), int(step_count)
        except Exception as e:
            logging.error(f"Episode {episode_num} failed: {e}")
            return -float('inf'), max_steps  # Return worst case scenario
    
    def _run_episodes_parallel(self, runner, num_episodes: int, max_steps: int):
        """Run episodes in parallel using ThreadPoolExecutor."""
        rewards = []
        steps = []
        
        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            # Submit all episodes
            future_to_episode = {
                executor.submit(self._run_single_episode, runner, i, max_steps): i 
                for i in range(num_episodes)
            }
            
            # Collect results as they complete
            for future in as_completed(future_to_episode):
                episode_num = future_to_episode[future]
                try:
                    reward, step_count = future.result()
                    rewards.append(reward)
                    steps.append(step_count)
                except Exception as e:
                    logging.error(f"Episode {episode_num} failed with exception: {e}")
                    rewards.append(-float('inf'))
                    steps.append(max_steps)
        
        return rewards, steps
    
    def create_objective_for_budget(self, budget: int):
        """Create objective function for a specific budget."""
        def objective(trial: optuna.Trial) -> float:
            try:
                # Suggest hyperparameters based on planner type and budget
                if self.planner_name == 'MCGS':
                    suggested_params = self.suggest_mcgs_params(trial, budget)
                elif self.planner_name == 'CMCGS':
                    suggested_params = self.suggest_cmcgs_params(trial, budget)
                else:
                    raise ValueError(f"Unknown planner: {self.planner_name}")
                
                # Start with base planner config, then override with suggested params
                planner_cfg = {**self.base_planner_config, **suggested_params}
                env_cfg = self.config['environment']

                # Create runner and run experiment
                runner = Runner(
                    env=self.env_name,
                    planner=self.planner_name,
                    computational_budget_max=budget,
                    time_budget_max=self.time_budget_fixed,
                    env_config=env_cfg,
                    planner_config=planner_cfg,
                    render_mode=None,  # No rendering during optimization
                    sleep_time=0.0,
                    save_replay=False
                )
                
                # Run episodes (parallel or sequential)
                if self.parallel_episodes and self.n_episodes_per_trial > 1:
                    rewards, steps = self._run_episodes_parallel(
                        runner, self.n_episodes_per_trial, self.max_steps_per_episode
                    )
                else:
                    # Fallback to sequential execution
                    rewards, steps = runner.run_multi_episode(
                        num_episodes=self.n_episodes_per_trial,
                        max_steps_per_episode=self.max_steps_per_episode
                    )
                
                # Calculate objective (mean reward)
                mean_reward = float(np.mean(rewards))
                mean_steps = float(np.mean(steps))
                std_reward = float(np.std(rewards))
                success_rate = float(np.mean([r > 0 for r in rewards]))
                
                # Log additional metrics
                trial.set_user_attr('mean_steps', mean_steps)
                trial.set_user_attr('std_reward', std_reward)
                trial.set_user_attr('success_rate', success_rate)
                trial.set_user_attr('budget', budget)
                
                # Extract only the optimized parameters for CSV logging
                optimized_params = {}
                for key, value in suggested_params.items():
                    if key not in ['computational_budget_max', 'time_budget_max', 'simulation_budget']:
                        optimized_params[key] = value
                
                # Log to CSV files
                self._log_trial_to_csv(
                    trial_number=trial.number,
                    budget=budget,
                    rewards=rewards,
                    steps=steps,
                    params=optimized_params
                )
                
                logger.info(f"Budget {budget}, Trial {trial.number}: Mean reward = {mean_reward:.3f}, "
                           f"Mean steps = {mean_steps:.1f}, Success rate = {success_rate:.2f}")
                
                return mean_reward
                
            except Exception as e:
                logger.error(f"Budget {budget}, Trial {trial.number} failed: {str(e)}")
                return -1000.0
        
        return objective
    
    def optimize_for_budget(self, budget: int, n_trials: int = 50) -> Dict[str, Any]:
        """Optimize hyperparameters for a specific budget."""
        logger.info(f"Optimizing {self.planner_name} for budget {budget} with {n_trials} trials")
        
        study = self.studies[budget]
        objective = self.create_objective_for_budget(budget)
        
        # Run optimization
        study.optimize(objective, n_trials=n_trials)
        
        logger.info(f"Budget {budget} optimization completed!")
        logger.info(f"Best trial: {study.best_trial.number}")
        logger.info(f"Best value: {study.best_value:.3f}")
        
        # Update best runs CSV
        if study.best_trial:
            # Get the best parameters (excluding budget params)
            best_params = {}
            for key, value in study.best_params.items():
                if key not in ['computational_budget_max', 'time_budget_max', 'simulation_budget']:
                    best_params[key] = value
            
            self._update_best_runs_csv(budget, study.best_trial, best_params)
        
        return {
            'budget': budget,
            'best_value': study.best_value,
            'best_params': study.best_params,
            'n_trials': len(study.trials)
        }
    
    def optimize_all_budgets(self, n_trials_per_budget: int = 50) -> Dict[str, Any]:
        """Optimize hyperparameters for all budget levels."""
        logger.info(f"Starting budget-aware optimization for {self.planner_name} on {self.env_name}")
        logger.info(f"Budget range: {self.budget_range}")
        logger.info(f"Trials per budget: {n_trials_per_budget}")
        
        all_results = {}
        
        for budget in self.budget_range:
            logger.info(f"\n=== Optimizing for Budget {budget} ===")
            result = self.optimize_for_budget(budget, n_trials_per_budget)
            all_results[budget] = result
            
            # Save intermediate results
            self.save_results(all_results)
        
        # Create budget performance curve
        self.create_budget_performance_curve(all_results)
        
        logger.info(f"\n=== Budget-Aware Optimization Complete ===")
        self.print_budget_analysis(all_results)
        
        return all_results
    
    def save_results(self, results: Dict[str, Any]) -> str:
        """Save optimization results."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Prepare detailed results
        detailed_results = {
            'planner_name': self.planner_name,
            'env_name': self.env_name,
            'budget_range': self.budget_range,
            'time_budget_fixed': self.time_budget_fixed,
            'base_planner_config': self.base_planner_config,
            'budget_results': results,
            'timestamp': timestamp
        }
        
        # Add trial histories for each budget
        for budget in self.budget_range:
            if budget in self.studies:
                study = self.studies[budget]
                detailed_results[f'budget_{budget}_trials'] = []
                for trial in study.trials:
                    trial_data = {
                        'number': trial.number,
                        'value': trial.value,
                        'params': trial.params,
                        'state': trial.state.name,
                        'user_attrs': trial.user_attrs
                    }
                    detailed_results[f'budget_{budget}_trials'].append(trial_data)
        
        # Save to file
        filename = f"{timestamp}_budget_aware_results.json"
        filepath = os.path.join(self.results_dir, filename)
        
        with open(filepath, 'w') as f:
            json.dump(detailed_results, f, indent=2)
        
        logger.info(f"Results saved to {filepath}")
        
        # Create best configs for each budget
        self.save_best_configs(results, timestamp)
        
        # Generate comprehensive CSV summary
        self._generate_final_summary()
        
        return filepath
    
    def _generate_final_summary(self):
        """Generate a comprehensive summary of all optimization results."""
        try:
            # Read the master results CSV
            master_df = pd.read_csv(self.master_csv_path)
            
            if master_df.empty:
                logger.warning("No results found in master CSV for summary generation")
                return
            
            # Create summary statistics CSV
            summary_path = os.path.join(self.csv_dir, f"{self.timestamp}_optimization_summary.csv")
            
            # Group by budget and calculate statistics
            summary_stats = master_df.groupby('budget').agg({
                'mean_reward': ['count', 'mean', 'std', 'min', 'max'],
                'mean_steps': ['mean', 'std', 'min', 'max'],
                'success_rate': ['mean', 'std', 'min', 'max']
            }).round(4)
            
            # Flatten column names
            summary_stats.columns = ['_'.join(col).strip() for col in summary_stats.columns.values]
            summary_stats = summary_stats.rename(columns={'mean_reward_count': 'n_trials'})
            
            # Add best trial info for each budget
            best_trials = master_df.loc[master_df.groupby('budget')['mean_reward'].idxmax()]
            best_info = best_trials[['budget', 'trial_number', 'mean_reward']].set_index('budget')
            best_info.columns = ['best_trial_number', 'best_reward']
            
            # Combine statistics
            final_summary = summary_stats.join(best_info)
            final_summary.to_csv(summary_path)
            
            # Create performance ranking CSV
            ranking_path = os.path.join(self.csv_dir, f"{self.timestamp}_budget_ranking.csv")
            ranking_df = master_df.loc[master_df.groupby('budget')['mean_reward'].idxmax()].copy()
            ranking_df = ranking_df.sort_values('mean_reward', ascending=False)
            ranking_df['performance_rank'] = range(1, len(ranking_df) + 1)
            ranking_df['efficiency'] = ranking_df['mean_reward'] / ranking_df['budget']
            ranking_df = ranking_df.sort_values('efficiency', ascending=False)
            ranking_df['efficiency_rank'] = range(1, len(ranking_df) + 1)
            ranking_df.to_csv(ranking_path, index=False)
            
            logger.info(f"Generated optimization summary: {summary_path}")
            logger.info(f"Generated budget ranking: {ranking_path}")
            
            # Generate simplified single-environment report
            self._generate_single_env_simplified_report(ranking_df)
            
        except Exception as e:
            logger.error(f"Failed to generate final summary: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def _generate_single_env_simplified_report(self, ranking_df: pd.DataFrame):
        """Generate simplified report for single environment optimization."""
        try:
            # Find the best performing budget
            best_row = ranking_df.loc[ranking_df['mean_reward'].idxmax()]
            best_budget = int(best_row['budget'])
            best_reward = best_row['mean_reward']
            
            # Find the config file path
            config_filename = f"{self.timestamp}_best_config_budget_{best_budget}.yaml"
            config_path = os.path.join(self.results_dir, config_filename)
            
            # Make path relative to project root for cleaner display
            try:
                project_root = "/home/christoph/Dokumente/research/projects/dec-mcgs"
                if config_path.startswith(project_root):
                    config_path = config_path[len(project_root) + 1:]  # +1 to remove leading slash
            except:
                pass  # Keep absolute path if relative conversion fails
            
            # Create simplified report
            simplified_result = {
                'Environment': self.env_name,
                'Planner': self.planner_name,
                'Max_Avg_Reward': round(best_reward, 4),
                'Best_Budget': best_budget,
                'Config_Path': config_path
            }
            
            # Save to CSV
            simplified_df = pd.DataFrame([simplified_result])
            simplified_csv_path = os.path.join(self.csv_dir, f"{self.timestamp}_simplified_summary.csv")
            simplified_df.to_csv(simplified_csv_path, index=False)
            
            # Print simplified report
            print(f"\n{'='*80}")
            print(f"SIMPLIFIED OPTIMIZATION SUMMARY")
            print(f"{'='*80}")
            print(f"Environment:    {self.env_name}")
            print(f"Planner:        {self.planner_name}")
            print(f"Max Avg Reward: {best_reward:.4f}")
            print(f"Best Budget:    {best_budget}")
            print(f"Config Path:    {config_path}")
            print(f"\n� Simplified report: {simplified_csv_path}")
            print(f"{'='*80}\n")
            
            logger.info(f"📋 Single-env simplified report saved: {simplified_csv_path}")
            
        except Exception as e:
            logger.error(f"Failed to generate single-env simplified report: {str(e)}")
            import traceback
            traceback.print_exc()
            
        except Exception as e:
            logger.error(f"Failed to generate final summary: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def save_best_configs(self, results: Dict[str, Any], timestamp: str):
        """Save best configuration for each budget as separate YAML files."""
        for budget, result in results.items():
            if 'best_params' in result:
                # Create environment config with proper structure
                env_in = self.config['environment']
                environment_config = {
                    'name': self.env_name,
                    'dt': env_in['dt'],
                    'atol': env_in['atol'],
                    'rtol': env_in['rtol'],
                    'multi_step_count': env_in['multi_step_count']
                }
                
                # Create experiment config with proper structure  
                exp_in = self.config['experiment']
                experiment_config = {
                    'render_mode': None,
                    'computational_budget_max': budget,
                    'time_budget_max': self.time_budget_fixed,
                    'sleep_time': exp_in.get('sleep_time', 0.0),
                    'save_replay': exp_in.get('save_replay', False),
                    'max_steps': exp_in['max_steps'],
                    'num_episodes': exp_in['num_episodes']
                }
                
                config_yaml = {
                    'environment': environment_config,
                    'planners': {
                        self.planner_name: {
                            'config': {
                                **self.base_planner_config, 
                                **result['best_params'],
                                'computational_budget_max': budget,
                                'time_budget_max': self.time_budget_fixed
                            }
                        }
                    },
                    'experiment': experiment_config,
                    'default_planner': self.planner_name,
                    'optimization_info': {
                        'best_reward': result['best_value'],
                        'n_trials': result['n_trials'],
                        'timestamp': timestamp
                    }
                }
                
                yaml_filename = f"{timestamp}_best_config_budget_{budget}.yaml"
                yaml_filepath = os.path.join(self.results_dir, yaml_filename)
                
                with open(yaml_filepath, 'w') as f:
                    yaml.dump(config_yaml, f, default_flow_style=False)
                
                logger.info(f"Best config for budget {budget} saved to {yaml_filepath}")
    
    def create_budget_performance_curve(self, results: Dict[str, Any]):
        """Create performance curve showing best reward vs budget."""
        try:
            import matplotlib.pyplot as plt
            
            budgets = []
            best_rewards = []
            
            for budget, result in results.items():
                if 'best_value' in result:
                    budgets.append(budget)
                    best_rewards.append(result['best_value'])
            
            if len(budgets) < 2:
                logger.warning("Not enough budget points to create performance curve")
                return
            
            plt.figure(figsize=(10, 6))
            plt.plot(budgets, best_rewards, 'b-o', linewidth=2, markersize=8)
            plt.xlabel('Computational Budget')
            plt.ylabel('Best Mean Reward')
            plt.title(f'Budget vs Performance - {self.planner_name} on {self.env_name}')
            plt.grid(True, alpha=0.3)
            
            # Add annotations
            for budget, reward in zip(budgets, best_rewards):
                plt.annotate(f'{reward:.3f}', (budget, reward), 
                           textcoords="offset points", xytext=(0,10), ha='center')
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = os.path.join(self.results_dir, f"{timestamp}_budget_performance_curve.png")
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            logger.info(f"Performance curve saved to {save_path}")
            
            plt.show()
            
        except ImportError:
            logger.warning("Matplotlib not available. Cannot create performance curve.")
        except Exception as e:
            logger.error(f"Error creating performance curve: {str(e)}")
    
    def print_budget_analysis(self, results: Dict[str, Any]):
        """Print analysis of budget vs performance."""
        print("\n" + "="*60)
        print("BUDGET-AWARE OPTIMIZATION ANALYSIS")
        print("="*60)
        
        budgets = []
        rewards = []
        
        for budget, result in results.items():
            if 'best_value' in result:
                budgets.append(budget)
                rewards.append(result['best_value'])
                print(f"Budget {budget:3d}: Best Reward = {result['best_value']:.4f}")
        
        if len(budgets) > 1:
            # Calculate efficiency metrics
            print(f"\nEFFICIENCY ANALYSIS:")
            for i in range(1, len(budgets)):
                budget_increase = budgets[i] - budgets[i-1]
                reward_increase = rewards[i] - rewards[i-1]
                efficiency = reward_increase / budget_increase if budget_increase > 0 else 0
                print(f"Budget {budgets[i-1]} → {budgets[i]}: "
                      f"Reward gain = {reward_increase:+.4f}, "
                      f"Efficiency = {efficiency:.6f} reward/budget")


class MultiEnvironmentPlannerOptimizer:
    """Optimizer that can run across multiple environments and planners."""
    
    def __init__(self, 
                 config_files: List[str],
                 planner_names: List[str],
                 budget_range: List[int],
                 time_budget_fixed: float = 20.0,
                 n_episodes_per_trial: int = 5,
                 max_steps_per_episode: int = 40,
                 parallel_episodes: bool = True,
                 max_workers: Optional[int] = None):
        """
        Initialize multi-environment/planner optimizer.
        
        Args:
            config_files: List of config file paths for different environments
            planner_names: List of planner names to test ['MCGS', 'CMCGS', 'CEM']
            budget_range: List of computational budgets to optimize for
            time_budget_fixed: Fixed time budget for all experiments
            n_episodes_per_trial: Number of episodes per trial
            max_steps_per_episode: Maximum steps per episode
            parallel_episodes: Whether to run episodes in parallel within each trial
            max_workers: Maximum number of parallel workers for episodes
        """
        self.config_files = config_files
        self.planner_names = planner_names
        self.budget_range = budget_range
        self.time_budget_fixed = time_budget_fixed
        self.n_episodes_per_trial = n_episodes_per_trial
        self.max_steps_per_episode = max_steps_per_episode
        self.parallel_episodes = parallel_episodes
        self.max_workers = max_workers
        
        # Create master results directory
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.master_results_dir = f'logs/multi_env_budget_optimization/{self.timestamp}'
        os.makedirs(self.master_results_dir, exist_ok=True)
        
        # Store all optimizers
        self.optimizers = {}
        self.all_results = {}
        
        logger.info(f"Multi-environment/planner optimizer initialized")
        logger.info(f"Environments: {len(config_files)} configs")
        logger.info(f"Planners: {planner_names}")
        logger.info(f"Budget range: {budget_range}")
        logger.info(f"Results will be saved to: {self.master_results_dir}")
    
    def optimize_all_combinations(self, n_trials_per_budget: int = 30) -> Dict[str, Any]:
        """Run optimization for all environment-planner combinations."""
        logger.info(f"\n{'='*100}")
        logger.info(f"STARTING MULTI-ENVIRONMENT/PLANNER OPTIMIZATION")
        logger.info(f"{'='*100}")
        logger.info(f"Total combinations: {len(self.config_files)} envs × {len(self.planner_names)} planners = {len(self.config_files) * len(self.planner_names)}")
        
        combination_count = 0
        total_combinations = len(self.config_files) * len(self.planner_names)
        
        for config_file in self.config_files:
            # Extract environment name from config
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
            env_name = config_data['environment']['name']
            
            for planner_name in self.planner_names:
                combination_count += 1
                logger.info(f"\n{'='*80}")
                logger.info(f"COMBINATION {combination_count}/{total_combinations}: {env_name} + {planner_name}")
                logger.info(f"{'='*80}")
                
                try:
                    # Create optimizer for this combination
                    optimizer = BudgetAwareHyperparameterOptimizer.from_config_file(
                        config_file=config_file,
                        budget_range=self.budget_range,
                        planner_name=planner_name,
                        time_budget_fixed=self.time_budget_fixed,
                        n_episodes_per_trial=self.n_episodes_per_trial,
                        max_steps_per_episode=self.max_steps_per_episode,
                        parallel_episodes=self.parallel_episodes,
                        max_workers=self.max_workers
                    )
                    
                    # Store optimizer
                    key = f"{env_name}_{planner_name}"
                    self.optimizers[key] = optimizer
                    
                    # Run optimization
                    results = optimizer.optimize_all_budgets(n_trials_per_budget=n_trials_per_budget)
                    self.all_results[key] = {
                        'env_name': env_name,
                        'planner_name': planner_name,
                        'config_file': config_file,
                        'results': results,
                        'optimizer': optimizer
                    }
                    
                    logger.info(f"✅ Completed {env_name} + {planner_name}")
                    
                except Exception as e:
                    logger.error(f"❌ Failed {env_name} + {planner_name}: {str(e)}")
                    self.all_results[f"{env_name}_{planner_name}"] = {
                        'env_name': env_name,
                        'planner_name': planner_name,
                        'config_file': config_file,
                        'error': str(e),
                        'results': None
                    }
        
        # Generate comprehensive comparison
        self._generate_master_comparison()
        
        logger.info(f"\n{'='*100}")
        logger.info(f"MULTI-ENVIRONMENT/PLANNER OPTIMIZATION COMPLETE")
        logger.info(f"Results saved to: {self.master_results_dir}")
        logger.info(f"{'='*100}")
        
        return self.all_results
    
    def _generate_master_comparison(self):
        """Generate master comparison across all environments and planners."""
        try:
            # Collect all CSV data
            all_master_data = []
            all_episode_data = []
            all_best_runs = []
            
            for key, data in self.all_results.items():
                if data['results'] is None:
                    continue
                    
                optimizer = data['optimizer']
                env_name = data['env_name']
                planner_name = data['planner_name']
                
                # Read CSV files if they exist
                if hasattr(optimizer, 'master_csv_path') and os.path.exists(optimizer.master_csv_path):
                    master_df = pd.read_csv(optimizer.master_csv_path)
                    master_df['env_name'] = env_name
                    master_df['planner_name'] = planner_name
                    master_df['combination'] = key
                    all_master_data.append(master_df)
                
                if hasattr(optimizer, 'episodes_csv_path') and os.path.exists(optimizer.episodes_csv_path):
                    episodes_df = pd.read_csv(optimizer.episodes_csv_path)
                    episodes_df['env_name'] = env_name
                    episodes_df['planner_name'] = planner_name
                    episodes_df['combination'] = key
                    all_episode_data.append(episodes_df)
                
                if hasattr(optimizer, 'best_runs_csv_path') and os.path.exists(optimizer.best_runs_csv_path):
                    best_df = pd.read_csv(optimizer.best_runs_csv_path)
                    best_df['env_name'] = env_name
                    best_df['planner_name'] = planner_name
                    best_df['combination'] = key
                    all_best_runs.append(best_df)
            
            # Combine all data
            if all_master_data:
                combined_master = pd.concat(all_master_data, ignore_index=True)
                master_path = os.path.join(self.master_results_dir, f"combined_master_results.csv")
                combined_master.to_csv(master_path, index=False)
                logger.info(f"📊 Combined master results: {master_path}")
            
            if all_episode_data:
                combined_episodes = pd.concat(all_episode_data, ignore_index=True)
                episodes_path = os.path.join(self.master_results_dir, f"combined_episode_results.csv")
                combined_episodes.to_csv(episodes_path, index=False)
                logger.info(f"📝 Combined episode results: {episodes_path}")
            
            if all_best_runs:
                combined_best = pd.concat(all_best_runs, ignore_index=True)
                best_path = os.path.join(self.master_results_dir, f"combined_best_runs.csv")
                combined_best.to_csv(best_path, index=False)
                logger.info(f"🏆 Combined best runs: {best_path}")
                
                # Generate performance ranking across all combinations
                self._generate_global_ranking(combined_best)
            
            # Save summary JSON
            self._save_master_summary()
            
        except Exception as e:
            logger.error(f"Failed to generate master comparison: {str(e)}")
            import traceback
            traceback.print_exc()
    
    def _generate_simplified_report(self):
        """Generate a simplified report with just planner, environment, max reward, and config path."""
        try:
            # Find the best result for each environment-planner combination
            simplified_results = []
            
            for key, data in self.all_results.items():
                if data['results'] is None:
                    continue
                    
                env_name = data['env_name']
                planner_name = data['planner_name']
                
                # Find the budget with the best reward
                best_budget = None
                best_reward = float('-inf')
                
                for budget, result in data['results'].items():
                    if result['best_value'] > best_reward:
                        best_reward = result['best_value']
                        best_budget = budget
                
                # Find the config file path
                optimizer = data.get('optimizer')
                if optimizer and hasattr(optimizer, 'results_dir'):
                    # Look for the best config file for this budget
                    config_filename = f"{optimizer.timestamp}_best_config_budget_{best_budget}.yaml"
                    config_path = os.path.join(optimizer.results_dir, config_filename)
                    
                    # Make path relative to project root for cleaner display
                    try:
                        project_root = "/home/christoph/Dokumente/research/projects/dec-mcgs"
                        if config_path.startswith(project_root):
                            config_path = config_path[len(project_root) + 1:]  # +1 to remove leading slash
                    except:
                        pass  # Keep absolute path if relative conversion fails
                else:
                    config_path = "Config path not available"
                
                simplified_results.append({
                    'Environment': env_name,
                    'Planner': planner_name,
                    'Max_Avg_Reward': round(best_reward, 4),
                    'Best_Budget': best_budget,
                    'Config_Path': config_path
                })
            
            # Sort by reward (descending)
            simplified_results.sort(key=lambda x: x['Max_Avg_Reward'], reverse=True)
            
            # Create simplified CSV report
            simplified_df = pd.DataFrame(simplified_results)
            simplified_csv_path = os.path.join(self.master_results_dir, "simplified_results_summary.csv")
            simplified_df.to_csv(simplified_csv_path, index=False)
            
            # Print simplified report
            print(f"\n{'='*100}")
            print(f"SIMPLIFIED OPTIMIZATION RESULTS SUMMARY")
            print(f"{'='*100}")
            print(f"{'Rank':<4} {'Environment':<35} {'Planner':<10} {'Max Reward':<12} {'Budget':<8} {'Config Path'}")
            print("-" * 100)
            
            for i, result in enumerate(simplified_results, 1):
                env_short = result['Environment'][:32] + "..." if len(result['Environment']) > 35 else result['Environment']
                config_short = "..." + result['Config_Path'][-50:] if len(result['Config_Path']) > 53 else result['Config_Path']
                
                print(f"{i:<4} {env_short:<35} {result['Planner']:<10} {result['Max_Avg_Reward']:<12} "
                      f"{result['Best_Budget']:<8} {config_short}")
            
            print(f"\n📊 Detailed CSV report: {simplified_csv_path}")
            print(f"{'='*100}\n")
            
            logger.info(f"📋 Simplified report saved: {simplified_csv_path}")
            
        except Exception as e:
            logger.error(f"Failed to generate simplified report: {str(e)}")
            import traceback
            traceback.print_exc()

    def _generate_global_ranking(self, combined_best_df: pd.DataFrame):
        """Generate global ranking across all environment-planner combinations."""
        try:
            # Generate the simplified report first
            self._generate_simplified_report()
            
            # Overall best performance ranking
            ranking_df = combined_best_df.copy()
            ranking_df = ranking_df.sort_values('best_mean_reward', ascending=False)
            ranking_df['global_rank'] = range(1, len(ranking_df) + 1)
            
            # Calculate efficiency (reward per budget)
            ranking_df['efficiency'] = ranking_df['best_mean_reward'] / ranking_df['budget']
            ranking_df = ranking_df.sort_values('efficiency', ascending=False)
            ranking_df['efficiency_rank'] = range(1, len(ranking_df) + 1)
            
            # Save global ranking
            global_ranking_path = os.path.join(self.master_results_dir, "global_performance_ranking.csv")
            ranking_df.to_csv(global_ranking_path, index=False)
            
            logger.info(f"🥇 Global ranking saved: {global_ranking_path}")
            
        except Exception as e:
            logger.error(f"Failed to generate global ranking: {str(e)}")
    
    def _save_master_summary(self):
        """Save master summary JSON with all results."""
        summary_data = {
            'timestamp': self.timestamp,
            'config_files': self.config_files,
            'planner_names': self.planner_names,
            'budget_range': self.budget_range,
            'time_budget_fixed': self.time_budget_fixed,
            'n_episodes_per_trial': self.n_episodes_per_trial,
            'total_combinations': len(self.config_files) * len(self.planner_names),
            'successful_combinations': len([r for r in self.all_results.values() if r['results'] is not None]),
            'failed_combinations': len([r for r in self.all_results.values() if r['results'] is None]),
            'results_summary': {}
        }
        
        # Add summary for each combination
        for key, data in self.all_results.items():
            if data['results'] is not None:
                # Find best overall performance
                best_budget = max(data['results'].keys(), key=lambda b: data['results'][b]['best_value'])
                summary_data['results_summary'][key] = {
                    'env_name': data['env_name'],
                    'planner_name': data['planner_name'],
                    'best_budget': best_budget,
                    'best_reward': data['results'][best_budget]['best_value'],
                    'config_file': data['config_file']
                }
            else:
                summary_data['results_summary'][key] = {
                    'env_name': data['env_name'],
                    'planner_name': data['planner_name'],
                    'error': data.get('error', 'Unknown error'),
                    'config_file': data['config_file']
                }
        
        summary_path = os.path.join(self.master_results_dir, "master_summary.json")
        with open(summary_path, 'w') as f:
            json.dump(summary_data, f, indent=2)
        
        logger.info(f"📋 Master summary saved: {summary_path}")


def main():
    """Example usage of budget-aware optimization."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Budget-aware hyperparameter optimization")
    parser.add_argument('--config', type=str, help='Single config file to use')
    parser.add_argument('--configs', type=str, nargs='+', help='Multiple config files (space-separated)')
    parser.add_argument('--env', type=str, help='Environment name (deprecated, use config)')
    parser.add_argument('--planner', type=str, choices=['MCGS', 'CMCGS', 'CEM'], help='Single planner name')
    parser.add_argument('--planners', type=str, nargs='+', choices=['MCGS', 'CMCGS', 'CEM'], help='Multiple planners (space-separated)')
    parser.add_argument('--budget_min', type=int, default=50, help='Minimum budget')
    parser.add_argument('--budget_max', type=int, default=300, help='Maximum budget')
    parser.add_argument('--budget_step', type=int, default=50, help='Budget step size')
    parser.add_argument('--n_trials', type=int, default=30, help='Trials per budget')
    parser.add_argument('--n_episodes', type=int, default=5, help='Episodes per trial')
    parser.add_argument('--time_budget', type=float, default=20.0, help='Fixed time budget')
    parser.add_argument('--parallel', action='store_true', help='Run combinations in parallel (future feature)')
    parser.add_argument('--max_workers', type=int, default=4, help='Maximum parallel workers (future feature)')
    parser.add_argument('--parallel_episodes', action='store_true', default=True, help='Run episodes in parallel within each trial')
    parser.add_argument('--no_parallel_episodes', action='store_true', help='Disable parallel episode execution')
    parser.add_argument('--episode_workers', type=int, help='Maximum workers for parallel episodes (default: auto)')
    
    args = parser.parse_args()
    
    # Validate arguments
    if not args.config and not args.configs:
        parser.error("Either --config or --configs is required")
    
    if args.config and args.configs:
        parser.error("Cannot use both --config and --configs. Choose one.")
    
    # Create budget range
    budget_range = list(range(args.budget_min, args.budget_max + 1, args.budget_step))
    
    # Determine config files
    if args.configs:
        config_files = args.configs
    else:
        config_files = [args.config]
    
    # Determine planners
    if args.planners:
        planner_names = args.planners
    elif args.planner:
        planner_names = [args.planner]
    else:
        # Use default planner from config(s)
        planner_names = []
        for config_file in config_files:
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
            default_planner = config_data.get('default_planner', 'MCGS')
            if default_planner not in planner_names:
                planner_names.append(default_planner)
    
    # Handle parallel episodes configuration
    parallel_episodes = args.parallel_episodes and not args.no_parallel_episodes
    episode_workers = args.episode_workers
    
    print(f"Configuration:")
    print(f"  Config files: {config_files}")
    print(f"  Planners: {planner_names}")
    print(f"  Budget range: {args.budget_min}-{args.budget_max} (step {args.budget_step})")
    print(f"  Trials per budget: {args.n_trials}")
    print(f"  Episodes per trial: {args.n_episodes}")
    print(f"  Parallel episodes: {parallel_episodes}")
    if parallel_episodes and episode_workers:
        print(f"  Episode workers: {episode_workers}")
    print(f"  Total combinations: {len(config_files)} × {len(planner_names)} = {len(config_files) * len(planner_names)}")
    
    # Check if we're doing multi-environment/planner optimization
    if len(config_files) > 1 or len(planner_names) > 1:
        print(f"\n🚀 Running MULTI-ENVIRONMENT/PLANNER optimization")
        
        # Use multi-environment optimizer
        optimizer = MultiEnvironmentPlannerOptimizer(
            config_files=config_files,
            planner_names=planner_names,
            budget_range=budget_range,
            time_budget_fixed=args.time_budget,
            n_episodes_per_trial=args.n_episodes,
            parallel_episodes=parallel_episodes,
            max_workers=episode_workers
        )
        
        results = optimizer.optimize_all_combinations(n_trials_per_budget=args.n_trials)
        
    else:
        print(f"\n🎯 Running SINGLE-ENVIRONMENT/PLANNER optimization")
        
        # Use single optimizer (backward compatibility)
        optimizer = BudgetAwareHyperparameterOptimizer.from_config_file(
            config_file=config_files[0],
            budget_range=budget_range,
            planner_name=planner_names[0],
            time_budget_fixed=args.time_budget,
            n_episodes_per_trial=args.n_episodes,
            parallel_episodes=parallel_episodes,
            max_workers=episode_workers
        )
        
        results = optimizer.optimize_all_budgets(n_trials_per_budget=args.n_trials)


if __name__ == "__main__":
    main()