import optuna
import numpy as np
import yaml
import json
import os
from datetime import datetime
from typing import Dict, List, Optional, Any
import logging

# Suppress matplotlib GUI to prevent figures from showing during optimization
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
plt.ioff()  # Turn off interactive mode to prevent figure display

from experiments.run_experiment import Runner

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class BayesianHyperparameterOptimizer:
    def __init__(self, 
                 config: dict,
                 n_episodes_per_trial: int = 5,
                 max_steps_per_episode: int = 1000,
                 study_name: Optional[str] = None,
                 storage: Optional[str] = None,
                 initial_params: Optional[List[Dict[str, Any]]] = None):
        """
        Bayesian hyperparameter optimizer for MCGS/CMCGS planners.
        
        Args:
            config: Configuration object containing environment and planner settings
            n_episodes_per_trial: Number of episodes to run per trial
            max_steps_per_episode: Maximum steps per episode
            study_name: Name for the optimization study
            storage: Optuna storage backend (e.g., sqlite:///optuna.db)
            initial_params: List of initial parameter dictionaries to try first
        """
        # Store raw YAML config dict
        self.config = config
        self.env_name = config['environment']['name']
        self.planner_name = config.get('default_planner')
        self.n_episodes_per_trial = n_episodes_per_trial
        self.max_steps_per_episode = max_steps_per_episode
        self.initial_params = initial_params or []
        
        # Create study
        self.study_name = study_name or f"{self.planner_name}_{self.env_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.study = optuna.create_study(
            study_name=self.study_name,
            direction='maximize',
            storage=storage,
            load_if_exists=True
        )
        
        # Results storage
        self.results_dir = f'logs/hyperopt/{self.planner_name}'
        os.makedirs(self.results_dir, exist_ok=True)
    
    @classmethod
    def from_config_file(cls, 
                        config_file: str,
                        planner_name: Optional[str] = None,
                        n_episodes_per_trial: int = 5,
                        max_steps_per_episode: int = 1000,
                        study_name: Optional[str] = None,
                        storage: Optional[str] = None,
                        initial_params: Optional[List[Dict[str, Any]]] = None):
        """Create optimizer from a configuration file."""
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        # Set the specific planner for this optimization
        if planner_name:
            config['default_planner'] = planner_name
        
        return cls(
            config=config,
            n_episodes_per_trial=n_episodes_per_trial,
            max_steps_per_episode=max_steps_per_episode,
            study_name=study_name,
            storage=storage,
            initial_params=initial_params
        )
    
    def suggest_mcgs_params(self, trial: optuna.Trial) -> Dict[str, Any]:
        """Suggest MCGS hyperparameters."""
        return {
            'computational_budget_max': trial.suggest_int('computational_budget_max', 50, 500, step=50),
            'time_budget_max': trial.suggest_float('time_budget_max', 5.0, 30.0),
            'expand_n_times': trial.suggest_int('expand_n_times', 1, 5),
            'expand_n_times_min': trial.suggest_int('expand_n_times_min', 1, 3),
            'expand_n_times_max': trial.suggest_int('expand_n_times_max', 3, 8),
            'sample_best_X_actions': trial.suggest_int('sample_best_X_actions', 5, 20),
            'kappa': trial.suggest_float('kappa', 0.1, 2.0),
            'alpha': trial.suggest_float('alpha', 0.1, 1.0),
            'k': trial.suggest_int('k', 2, 20),
            'radius_threshold': trial.suggest_float('radius_threshold', 0.1, 5.0),
            'c_uct': trial.suggest_float('c_uct', 0.1, 2.0),
            'tracking_tolerance': trial.suggest_float('tracking_tolerance', 0.1, 1.0),
            'abstraction_refinement_exponent': trial.suggest_float('abstraction_refinement_exponent', -1.0, 1.0),
            'yield_mode': trial.suggest_categorical('yield_mode', ['N', 'Q', 'T']),
            'random_rollout_n_times': trial.suggest_int('random_rollout_n_times', 10, 50),
            'random_rollout_length': trial.suggest_int('random_rollout_length', 1, 10),
            # Progressive widening parameters
            'progressive_widening_method': trial.suggest_categorical('progressive_widening_method', ['default', 'expansion_attempts', 'epsilon_greedy']),
            'epsilon': trial.suggest_float('epsilon', 0.01, 0.5),
            'plan_in_space_time': True,
            'use_controller': True,
        }
    
    def suggest_cmcgs_params(self, trial: optuna.Trial) -> Dict[str, Any]:
        """Suggest CMCGS hyperparameters."""
        return {
            'simulation_budget': trial.suggest_int('simulation_budget', 50, 500, step=50),
            'time_budget_max': trial.suggest_float('time_budget_max', 5.0, 30.0),
            'min_graph_length': trial.suggest_int('min_graph_length', 2, 10),
            'rollout_length': trial.suggest_int('rollout_length', 3, 15),
            'elite_ratio': trial.suggest_float('elite_ratio', 0.05, 0.3),
            'optimal_prob': trial.suggest_float('optimal_prob', 0.05, 0.3),
            'optimal_n_top': trial.suggest_int('optimal_n_top', 3, 15),
            'optimal_range': trial.suggest_float('optimal_range', 0.1, 1.0),
            'min_update_divisor': trial.suggest_float('min_update_divisor', 0.5, 2.0),
            'alpha': trial.suggest_int('alpha', 1, 10),
            'beta': trial.suggest_int('beta', 1, 10),
            # Fixed parameters
            'clustering_alg': 'agglomerative',
            'max_n_clusters': 2147483646,
        }
    
    def enqueue_initial_params(self):
        """Enqueue initial parameter sets as first trials to guide optimization."""
        if not self.initial_params:
            return
            
        logger.info(f"Enqueuing {len(self.initial_params)} initial parameter sets...")
        
        for i, params in enumerate(self.initial_params):
            try:
                # Filter params to only include those that would be suggested by the optimization
                if self.planner_name == 'MCGS':
                    param_names = self.get_mcgs_param_names()
                else:
                    param_names = self.get_cmcgs_param_names()
                
                # Only include parameters that are actually optimized
                filtered_params = {k: v for k, v in params.items() if k in param_names}
                
                if filtered_params:
                    self.study.enqueue_trial(filtered_params)
                    logger.info(f"Enqueued initial params {i+1}: {filtered_params}")
                else:
                    logger.warning(f"No optimizable parameters found in initial params {i+1}")
                    
            except Exception as e:
                logger.warning(f"Failed to enqueue initial params {i+1}: {e}")
    
    def get_mcgs_param_names(self) -> set:
        """Get the names of parameters that MCGS optimization will tune."""
        return {
            'computational_budget_max', 'time_budget_max', 'expand_n_times', 'expand_n_times_min', 
            'expand_n_times_max', 'sample_best_X_actions', 'kappa', 
            'alpha', 'k', 'radius_threshold', 'c_uct', 'tracking_tolerance',
            'abstraction_refinement_exponent', 'yield_mode', 'random_rollout_n_times',
            'random_rollout_length'
        }
    
    def get_cmcgs_param_names(self) -> set:
        """Get the names of parameters that CMCGS optimization will tune."""
        return {
            'simulation_budget', 'time_budget_max', 'min_graph_length', 'rollout_length',
            'elite_ratio', 'optimal_prob', 'optimal_n_top', 'optimal_range',
            'min_update_divisor', 'alpha', 'beta'
        }
    
    def objective(self, trial: optuna.Trial) -> float:
        """Objective function for optimization."""
        try:
            # Suggest hyperparameters based on planner type
            if self.planner_name == 'MCGS':
                suggested_params = self.suggest_mcgs_params(trial)
            elif self.planner_name == 'CMCGS':
                suggested_params = self.suggest_cmcgs_params(trial)
            else:
                raise ValueError(f"Unknown planner: {self.planner_name}")
            
            # Get the base planner config and merge with suggested parameters
            base_planner_cfg = self.config['planners'][self.planner_name]['config']
            full_planner_cfg = {**base_planner_cfg, **suggested_params}
            
            # Extract environment config
            env_cfg = self.config['environment']
            
            # Determine budgets
            comp_budget = full_planner_cfg.get('computational_budget_max', self.config['experiment']['computational_budget_max'])
            time_budget = full_planner_cfg.get('time_budget_max', self.config['experiment']['time_budget_max'])
            
            # Create runner and run experiment
            runner = Runner(
                env=self.env_name,
                planner=self.planner_name,
                computational_budget_max=comp_budget,
                time_budget_max=time_budget,
                env_config=env_cfg,
                planner_config=full_planner_cfg,
                render_mode=None,  # No rendering during optimization
                sleep_time=0.0,
                save_replay=False
            )
            
            # Run episodes
            rewards, steps = runner.run_multi_episode(
                num_episodes=self.n_episodes_per_trial,
                max_steps_per_episode=self.max_steps_per_episode
            )
            
            # Calculate objective (mean reward)
            mean_reward = float(np.mean(rewards))
            mean_steps = float(np.mean(steps))
            
            # Log additional metrics
            trial.set_user_attr('mean_steps', mean_steps)
            trial.set_user_attr('std_reward', float(np.std(rewards)))
            trial.set_user_attr('min_reward', float(np.min(rewards)))
            trial.set_user_attr('max_reward', float(np.max(rewards)))
            
            logger.info(f"Trial {trial.number}: Mean reward = {mean_reward:.3f}, Mean steps = {mean_steps:.1f}")
            
            return mean_reward
            
        except Exception as e:
            logger.error(f"Trial {trial.number} failed: {str(e)}")
            # Return a very low value for failed trials
            return -1000.0
    
    def optimize(self, n_trials: int = 100, timeout: Optional[float] = None) -> Dict[str, Any]:
        """Run Bayesian optimization."""
        logger.info(f"Starting Bayesian optimization for {self.planner_name} on {self.env_name}")
        logger.info(f"Running {n_trials} trials with {self.n_episodes_per_trial} episodes each")
        
        # Enqueue initial parameter sets first
        self.enqueue_initial_params()
        
        # Add callback to save intermediate results
        def save_callback(study: optuna.Study, trial):
            if trial.number % 10 == 0:  # Save every 10 trials
                self.save_results()
        
        # Run optimization
        self.study.optimize(
            self.objective,
            n_trials=n_trials,
            timeout=timeout,
            callbacks=[save_callback]
        )
        
        # Save final results
        results = self.save_results()
        
        logger.info(f"Optimization completed!")
        logger.info(f"Best trial: {self.study.best_trial.number}")
        logger.info(f"Best value: {self.study.best_value:.3f}")
        logger.info(f"Best params: {self.study.best_params}")
        
        return results
    
    def save_results(self) -> Dict[str, Any]:
        """Save optimization results."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Prepare results
        results = {
            'study_name': self.study_name,
            'planner_name': self.planner_name,
            'env_name': self.env_name,
            'n_trials': len(self.study.trials),
            'best_value': self.study.best_value,
            'best_params': self.study.best_params,
            'best_trial_number': self.study.best_trial.number,
            'optimization_history': [],
            'base_config': self.config['planners'][self.planner_name]['config'],
            'timestamp': timestamp
        }
        
        # Add trial history
        for trial in self.study.trials:
            trial_data = {
                'number': trial.number,
                'value': trial.value,
                'params': trial.params,
                'state': trial.state.name,
                'user_attrs': trial.user_attrs
            }
            results['optimization_history'].append(trial_data)
        
        # Save to file
        filename = f"{timestamp}_optimization_results.json"
        filepath = os.path.join(self.results_dir, filename)
        
        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)
        
        logger.info(f"Results saved to {filepath}")
        
        # Also save best config in YAML format for easy use (matching repo schema)
        merged_planner_cfg = {**self.config['planners'][self.planner_name]['config'], **self.study.best_params}
        best_config_yaml = {
            'default_planner': self.planner_name,
            'experiment': {
                'render_mode': None,
                'save_replay': False,
                'computational_budget_max': merged_planner_cfg.get('computational_budget_max', self.config['experiment']['computational_budget_max']),
                'time_budget_max': merged_planner_cfg.get('time_budget_max', self.config['experiment']['time_budget_max']),
                'sleep_time': self.config['experiment'].get('sleep_time', 0.0),
                'max_steps': self.max_steps_per_episode,
                'num_episodes': self.n_episodes_per_trial
            },
            'environment': self.config['environment'],
            'planners': {
                self.planner_name: {
                    'config': merged_planner_cfg
                }
            }
        }
        
        yaml_filename = f"{timestamp}_best_config.yaml"
        yaml_filepath = os.path.join(self.results_dir, yaml_filename)
        
        with open(yaml_filepath, 'w') as f:
            yaml.dump(best_config_yaml, f, default_flow_style=False)
        
        logger.info(f"Best config saved to {yaml_filepath}")
        
        return results
    
    def plot_optimization_history(self, save_path: Optional[str] = None):
        """Plot optimization history."""
        try:
            import matplotlib.pyplot as plt
            
            # Get trial data
            trials = [t for t in self.study.trials if t.value is not None]
            trial_numbers = [t.number for t in trials]
            values = [float(t.value) for t in trials if t.value is not None]
            
            # Create plot
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
            
            # Plot 1: Optimization history
            ax1.plot(trial_numbers, values, 'b-', alpha=0.6, label='Trial values')
            
            # Running best
            best_values = []
            current_best = float('-inf')
            for v in values:
                if v is not None and v > current_best:
                    current_best = v
                best_values.append(current_best)
            
            ax1.plot(trial_numbers, best_values, 'r-', linewidth=2, label='Best value')
            ax1.set_xlabel('Trial')
            ax1.set_ylabel('Objective Value (Mean Reward)')
            ax1.set_title(f'Optimization History - {self.planner_name} on {self.env_name}')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # Plot 2: Parameter importance (if available)
            try:
                importance = optuna.importance.get_param_importances(self.study)
                params = list(importance.keys())
                importances = list(importance.values())
                
                ax2.barh(params, importances)
                ax2.set_xlabel('Importance')
                ax2.set_title('Parameter Importance')
                ax2.grid(True, alpha=0.3)
            except:
                ax2.text(0.5, 0.5, 'Parameter importance\nnot available', 
                        ha='center', va='center', transform=ax2.transAxes)
                ax2.set_title('Parameter Importance')
            
            plt.tight_layout()
            
            if save_path:
                plt.savefig(save_path, dpi=300, bbox_inches='tight')
                logger.info(f"Plot saved to {save_path}")
            else:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_path = os.path.join(self.results_dir, f"{timestamp}_optimization_plot.png")
                plt.savefig(save_path, dpi=300, bbox_inches='tight')
                logger.info(f"Plot saved to {save_path}")
            
            plt.show()
            
        except ImportError:
            logger.warning("Matplotlib not available. Cannot create plots.")
        except Exception as e:
            logger.error(f"Error creating plot: {str(e)}")


def main():
    """Example usage of the Bayesian optimizer."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Bayesian hyperparameter optimization for planners")
    parser.add_argument('--env', type=str, required=True, help='Environment name')
    parser.add_argument('--planner', type=str, required=True, choices=['MCGS', 'CMCGS'], help='Planner name')
    parser.add_argument('--n_trials', type=int, default=50, help='Number of optimization trials')
    parser.add_argument('--n_episodes', type=int, default=5, help='Episodes per trial')
    parser.add_argument('--max_steps', type=int, default=1000, help='Max steps per episode')
    parser.add_argument('--timeout', type=float, default=None, help='Optimization timeout in seconds')
    parser.add_argument('--study_name', type=str, default=None, help='Study name')
    parser.add_argument('--storage', type=str, default=None, help='Optuna storage (e.g., sqlite:///optuna.db)')
    
    args = parser.parse_args()
    
    # Manual configuration no longer supported - use from_config_file() method instead
    raise NotImplementedError(
        "Manual configuration is no longer supported. "
        "Use BayesianHyperparameterOptimizer.from_config_file() instead with a proper YAML config file."
    )
    
    # Run optimization
    results = optimizer.optimize(n_trials=args.n_trials, timeout=args.timeout)
    
    # Create plots
    optimizer.plot_optimization_history()
    
    print(f"\nOptimization completed!")
    print(f"Best parameters: {results['best_params']}")
    print(f"Best value: {results['best_value']:.3f}")


if __name__ == "__main__":
    main()