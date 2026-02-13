# Navigation Environment Configuration Files

This directory contains configuration files for all navigation environments used in the DEC-MCGS project.

## Template and Dummy Configuration

### Dummy Configuration Template
- **Config File**: `dummy_config.yaml`
- **Description**: Template configuration file with all possible sections and default values
- **Usage**: Use as a reference or starting point for creating new configurations
- **Contains**: All planners (MCGS, CMCGS, MCTS_Bootstrap) with complete parameter sets

## Available Navigation Environments

### 1. NavigationEnv (Base Environment)
- **Config File**: `navigation_env_config.yaml`
- **Environment Name**: `NavigationEnv-v0`
- **Description**: Basic navigation environment with standard dynamics
- **Robot Type**: Basic robot
- **Key Parameters**: 
  - MCGS expand_n_times: 2
  - Random rollouts: 20
  - Episode steps: 200

### 2. NavigationEnvSingleIntegrator
- **Config File**: `navigation_env_single_integrator_config.yaml` 
- **Environment Name**: `NavigationEnvSingleIntegrator-v0`
- **Description**: Navigation with single integrator dynamics
- **Robot Type**: SingleIntegrator robot
- **Key Parameters**:
  - MCGS expand_n_times: 2
  - Random rollouts: 25
  - Episode steps: 200

### 3. NavigationEnvSingleIntegratorUnicycle
- **Config File**: `navigation_env_single_integrator_unicycle_config.yaml`
- **Environment Name**: `NavigationEnvSingleIntegratorUnicycle-v0`
- **Description**: Navigation with unicycle dynamics (orientation-aware)
- **Robot Type**: SingleIntegratorUnicycle robot
- **Key Parameters**:
  - MCGS expand_n_times: 3
  - Random rollouts: 30
  - Episode steps: 200
  - Enhanced tracking tolerance for orientation

### 4. NavigationEnvDoubleIntegrator
- **Config File**: `navigation_env_double_integrator_config.yaml`
- **Environment Name**: `NavigationEnvDoubleIntegrator-v0`
- **Description**: Navigation with double integrator dynamics (position + velocity)
- **Robot Type**: DoubleIntegrator robot
- **Key Parameters**:
  - MCGS expand_n_times: 3
  - Random rollouts: 30
  - Episode steps: 250
  - Longer time limit for more complex dynamics

### 5. NavigationEnvBaseline
- **Config File**: `navigation_env_baseline_config.yaml`
- **Environment Name**: `NavigationEnvBaseline-v0`
- **Description**: Baseline navigation environment for comparison
- **Robot Type**: BaselineDoubleIntegrator robot
- **Key Parameters**:
  - MCGS expand_n_times: 2
  - Random rollouts: 20
  - Episode steps: 200

### 6. NavigationEnvSingleIntegratorUnicycleBrokenRudder
- **Config File**: `navigation_env_single_integrator_unicycle_broken_rudder_config.yaml`
- **Environment Name**: `NavigationEnvSingleIntegratorUnicycleBrokenRudder-v0`
- **Description**: Unicycle navigation with broken rudder constraint
- **Robot Type**: SingleIntegratorUnicycleBrokenRudder robot
- **Key Parameters**:
  - Specialized for constrained control scenarios
  - Adapted parameters for degraded maneuverability

## Usage

To run experiments with a specific navigation environment:

```bash
python -m experiments.run_experiment --config configs/<config_file_name>
```

Examples:
```bash
# Run unicycle navigation experiment
python -m experiments.run_experiment --config configs/navigation_env_single_integrator_unicycle_config.yaml

# Run baseline experiment  
python -m experiments.run_experiment --config configs/navigation_env_baseline_config.yaml

# Run double integrator experiment
python -m experiments.run_experiment --config configs/navigation_env_double_integrator_config.yaml
```

## Configuration Structure

Each config file contains:
- **environment**: Environment name and settings
- **planners**: MCGS and CMCGS planner configurations
- **evaluation**: Testing parameters (seeds, episodes, success criteria)
- **visualization**: Output and plotting settings

## Parameter Tuning Notes

- **expand_n_times**: Higher values for more complex dynamics (unicycle, double integrator)
- **random_rollout_n_times**: Increased for environments requiring more exploration
- **tracking_tolerance**: Adjusted based on robot precision capabilities
- **max_episode_steps**: Extended for more complex navigation scenarios