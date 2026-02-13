#!/usr/bin/env python3
"""
Run Bayesian hyperparameter optimization using clean config files and hyperopt ranges.

This script mirrors `experiments/run_experiment.py` CLI style:
- Loads a YAML config (environment + planners + experiment)
- Uses that config as the base for the chosen planner
- Loads search ranges from optimization/hyperopt_config.yaml
- Runs Bayesian optimization for the chosen planner and environment

Usage examples:
  python -m experiments.run_bayesian_optimization --config configs/navigation_env_baseline_config.yaml --planner MCGS
  python -m experiments.run_bayesian_optimization --config configs/navigation_env_single_integrator_config.yaml --planner CMCGS \
      --hyperopt-config optimization/hyperopt_config.yaml --n-trials 20 --n-episodes 3 --max-steps 500
"""

import argparse
import logging
from typing import Dict, Any

import optuna
from experiments.bayesian_optimization import BayesianHyperparameterOptimizer

import yaml
import os

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


def load_hyperopt_config(path: str) -> Dict[str, Any]:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="Run Bayesian hyperparameter optimization")
    parser.add_argument('--config', required=True, help='Path to experiment YAML config')
    parser.add_argument('--planner', required=False, choices=['MCGS', 'CMCGS'], help='Planner to optimize; defaults to config default_planner')
    parser.add_argument('--hyperopt-config', default='optimization/hyperopt_config.yaml', help='Path to hyperopt ranges YAML')
    parser.add_argument('--n-trials', type=int, help='Override number of trials from hyperopt config')
    parser.add_argument('--n-episodes', type=int, help='Override episodes per trial from hyperopt config')
    parser.add_argument('--max-steps', type=int, help='Override max steps per episode from hyperopt config')
    parser.add_argument('--study-name', type=str, help='Optuna study name (default auto)')
    parser.add_argument('--storage', type=str, help='Optuna storage, e.g. sqlite:///optuna.db')
    parser.add_argument('--timeout', type=float, help='Optuna timeout in seconds (default: None)')
    parser.add_argument('--verbosity', default='INFO', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'])

    args = parser.parse_args()
    logging.getLogger().setLevel(getattr(logging, args.verbosity))

    # Load experiment config YAML directly
    with open(args.config, 'r') as f:
        base_cfg = yaml.safe_load(f)
    env_name = base_cfg['environment']['name']

    # Choose planner: CLI -> config.default_planner
    planner_name = args.planner or base_cfg.get('default_planner')

    # Load hyperopt ranges and optimization settings
    hop = load_hyperopt_config(args.hyperopt_config)
    if planner_name == 'MCGS':
        search_space = hop['mcgs_search_space']
    else:
        search_space = hop['cmcgs_search_space']

    n_trials = args.n_trials or hop.get('optimization', {}).get('n_trials', 50)
    n_episodes = args.n_episodes or hop.get('optimization', {}).get('n_episodes_per_trial', 5)
    max_steps = args.max_steps or hop.get('optimization', {}).get('max_steps_per_episode', 1000)
    timeout = args.timeout or hop.get('optimization', {}).get('timeout', None)

    logger.info(f"Starting Bayesian optimization for {planner_name} on {env_name}")
    logger.info(f"Running {n_trials} trials with {n_episodes} episodes each (max {max_steps} steps)")

    # Create optimizer from config
    optimizer = BayesianHyperparameterOptimizer.from_config_file(
        config_file=args.config,
        planner_name=planner_name,
        n_episodes_per_trial=n_episodes,
        max_steps_per_episode=max_steps,
        study_name=args.study_name,
        storage=args.storage
    )

    # Override the suggest_* methods to honor hyperopt_config ranges
    # We implement thin wrappers that reference the config ranges.
    def suggest_from_space(trial: optuna.Trial, key: str, spec: Dict[str, Any]):
        t = spec['type']
        if t == 'int':
            low = int(spec['low']); high = int(spec['high'])
            step = spec.get('step')
            return trial.suggest_int(key, low, high, step=step) if step else trial.suggest_int(key, low, high)
        if t == 'float':
            low = float(spec['low']); high = float(spec['high'])
            return trial.suggest_float(key, low, high)
        if t == 'categorical':
            return trial.suggest_categorical(key, spec['choices'])
        raise ValueError(f"Unsupported type in hyperopt_config for {key}: {t}")

    def suggest_mcgs_params_from_cfg(trial: optuna.Trial) -> Dict[str, Any]:
        params = {}
        for key, spec in search_space.items():
            params[key] = suggest_from_space(trial, key, spec)
        # Fixed extras expected by planner
        params.setdefault('progressive_widening_method', 'default')
        params.setdefault('plan_in_space_time', True)
        params.setdefault('use_controller', True)
        return params

    def suggest_cmcgs_params_from_cfg(trial: optuna.Trial) -> Dict[str, Any]:
        params = {}
        for key, spec in search_space.items():
            params[key] = suggest_from_space(trial, key, spec)
        # Fixed extras expected by planner
        params.setdefault('clustering_alg', 'agglomerative')
        params.setdefault('max_n_clusters', 2147483646)
        return params

    # Monkey-patch the optimizer's suggestion functions to adhere to hyperopt_config ranges
    if planner_name == 'MCGS':
        optimizer.suggest_mcgs_params = suggest_mcgs_params_from_cfg  # type: ignore[attr-defined]
    else:
        optimizer.suggest_cmcgs_params = suggest_cmcgs_params_from_cfg  # type: ignore[attr-defined]

    # Optional: respect study config (sampler/pruner) in hyperopt_config
    study_cfg = hop.get('study', {})
    sampler_name = study_cfg.get('sampler', 'TPE')
    pruner_name = study_cfg.get('pruner', 'MedianPruner')

    # Recreate study if user provided a storage/name and wants specific sampler/pruner
    if args.study_name or args.storage or sampler_name or pruner_name:
        sampler = optuna.samplers.TPESampler() if sampler_name == 'TPE' else None
        pruner = optuna.pruners.MedianPruner() if pruner_name == 'MedianPruner' else None
        # Replace study on the optimizer
        optimizer.study = optuna.create_study(
            study_name=args.study_name,
            direction='maximize',
            storage=args.storage,
            load_if_exists=True,
            sampler=sampler,
            pruner=pruner
        )

    # Run optimization
    results = optimizer.optimize(n_trials=n_trials, timeout=timeout)

    # Print summary
    logger.info("Optimization completed!")
    logger.info(f"Best trial: {results['best_trial_number']}")
    logger.info(f"Best value: {results['best_value']:.3f}")
    logger.info(f"Best params: {results['best_params']}")


if __name__ == '__main__':
    main()
