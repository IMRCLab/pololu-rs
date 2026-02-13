import numpy as np
from typing import Optional, Tuple, List
import logging
from copy import deepcopy
from scipy.stats import multivariate_normal

from tools.planners.planner import Planner
from tools.envs.gym_robot_plan_env import GymRobotPlanEnv

logger = logging.getLogger(__name__)


class CEMPlanner(Planner):
    """
    Cross Entropy Method (CEM) Planner for trajectory optimization.
    
    CEM is a model-based optimization method that maintains a probability distribution
    over action sequences and iteratively updates it by:
    1. Sampling candidate action sequences from the current distribution
    2. Rolling out these sequences in the environment
    3. Selecting the top-performing sequences (elite set)
    4. Updating the distribution parameters based on the elite set
    """
    
    def __init__(self, 
                 env: GymRobotPlanEnv,
                 computational_budget_max: int = 2500,
                 time_budget_max: float = 20.0,
                 population_size: int = 100,
                 elite_fraction: float = 0.1,
                 planning_horizon: int = 10,
                 num_iterations: int = 5,
                 noise_scale: float = 1.0,
                 noise_decay: float = 0.9,
                 smoothing_factor: float = 0.1,
                 warm_start: bool = True,
                 save_replay: bool = True):
        """
        Initialize the CEM planner.
        
        Args:
            env: The environment to plan in
            computational_budget_max: Maximum computational budget
            time_budget_max: Maximum time budget  
            population_size: Number of action sequences to sample per iteration
            elite_fraction: Fraction of top sequences to use for distribution update
            planning_horizon: Length of action sequences to plan
            num_iterations: Number of CEM iterations per planning step
            noise_scale: Initial standard deviation for action sampling
            noise_decay: Factor to decay noise over iterations
            smoothing_factor: Smoothing factor for distribution updates (0 = no smoothing, 1 = no update)
            warm_start: Whether to warm start from previous solution
            save_replay: Whether to save replay data
        """
        super().__init__(env=env, computational_budget_max=computational_budget_max, 
                        time_budget_max=time_budget_max, save_replay=save_replay)
        
        self.population_size = population_size
        self.elite_fraction = elite_fraction
        self.planning_horizon = planning_horizon
        self.num_iterations = num_iterations
        self.noise_scale = noise_scale
        self.initial_noise_scale = noise_scale
        self.noise_decay = noise_decay
        self.smoothing_factor = smoothing_factor
        self.warm_start = warm_start
        
        # Get action space dimensions
        self.action_dim = env.action_space.shape[0]
        self.action_low = env.action_space.low
        self.action_high = env.action_space.high
        
        # Elite set size
        self.elite_size = max(1, int(self.population_size * self.elite_fraction))
        
        # Distribution parameters (mean and covariance)
        self.mean = None
        self.cov = None
        self.best_sequence = None
        self.best_value = float('-inf')
        
        # Initialize distribution parameters
        self._initialize_distribution()
        
    def _initialize_distribution(self):
        """Initialize the action sequence distribution."""
        # Initialize mean as zeros (or random actions within bounds)
        self.mean = np.zeros((self.planning_horizon, self.action_dim))
        
        # Initialize covariance as scaled identity matrix
        self.cov = np.eye(self.planning_horizon * self.action_dim) * (self.noise_scale ** 2)
        
    def reset(self, seed: Optional[int] = None):
        """Reset the planner and reinitialize distribution."""
        super().reset(seed=seed)
        
        if not self.warm_start:
            self._initialize_distribution()
            self.best_sequence = None
            self.best_value = float('-inf')
        
        # Reset noise scale
        self.noise_scale = self.initial_noise_scale
        
    def _sample_action_sequences(self) -> np.ndarray:
        """
        Sample action sequences from current distribution.
        
        Returns:
            Array of shape (population_size, planning_horizon, action_dim)
        """
        # Flatten mean for multivariate sampling
        mean_flat = self.mean.flatten()
        
        # Sample from multivariate normal distribution
        samples_flat = np.random.multivariate_normal(
            mean_flat, self.cov, size=self.population_size
        )
        
        # Reshape to (population_size, planning_horizon, action_dim)
        samples = samples_flat.reshape(self.population_size, self.planning_horizon, self.action_dim)
        
        # Clip actions to valid range
        samples = np.clip(samples, self.action_low, self.action_high)
        
        return samples
        
    def _evaluate_sequence(self, action_sequence: np.ndarray) -> float:
        """
        Evaluate a single action sequence by rolling it out in the environment.
        
        Args:
            action_sequence: Array of shape (planning_horizon, action_dim)
            
        Returns:
            Total reward from the rollout
        """
        # Save current environment state
        current_state = deepcopy(self.env.agent.state)
        
        total_reward = 0.0
        terminated = False
        truncated = False
        
        # Rollout the action sequence
        for step, action in enumerate(action_sequence):
            if terminated or truncated:
                break
                
            # Take step in environment
            obs, reward, terminated, truncated, info = self.env.step(action)
            total_reward += reward
            
            # Update computational budget
            self.computational_budget_used += 1
            
        # Restore environment state
        self.env.agent.state = current_state
        
        return total_reward
        
    def _evaluate_population(self, action_sequences: np.ndarray) -> np.ndarray:
        """
        Evaluate all action sequences in the population.
        
        Args:
            action_sequences: Array of shape (population_size, planning_horizon, action_dim)
            
        Returns:
            Array of rewards of shape (population_size,)
        """
        rewards = np.zeros(self.population_size)
        
        for i, sequence in enumerate(action_sequences):
            rewards[i] = self._evaluate_sequence(sequence)
            
            # Check if we've exceeded computational budget
            if self.computational_budget_used >= self.computational_budget_max:
                logger.warning(f"Computational budget exceeded during evaluation at sample {i}")
                break
                
        return rewards
        
    def _update_distribution(self, action_sequences: np.ndarray, rewards: np.ndarray):
        """
        Update distribution parameters based on elite sequences.
        
        Args:
            action_sequences: Array of shape (population_size, planning_horizon, action_dim)
            rewards: Array of rewards of shape (population_size,)
        """
        # Select elite sequences
        elite_indices = np.argsort(rewards)[-self.elite_size:]
        elite_sequences = action_sequences[elite_indices]
        elite_rewards = rewards[elite_indices]
        
        # Update best solution if improved
        best_idx = np.argmax(elite_rewards)
        if elite_rewards[best_idx] > self.best_value:
            self.best_value = elite_rewards[best_idx]
            self.best_sequence = elite_sequences[best_idx].copy()
        
        # Compute new mean
        new_mean = np.mean(elite_sequences, axis=0)
        
        # Flatten for covariance computation
        elite_flat = elite_sequences.reshape(self.elite_size, -1)
        new_mean_flat = new_mean.flatten()
        
        # Compute new covariance
        new_cov = np.cov(elite_flat.T)
        
        # Ensure covariance is positive definite
        if new_cov.shape == ():  # Single dimension case
            new_cov = np.array([[new_cov]])
        
        # Add regularization to prevent singular covariance
        reg_factor = 1e-6
        new_cov += np.eye(new_cov.shape[0]) * reg_factor
        
        # Smoothing update
        self.mean = (1 - self.smoothing_factor) * new_mean + self.smoothing_factor * self.mean
        self.cov = (1 - self.smoothing_factor) * new_cov + self.smoothing_factor * self.cov
        
        logger.debug(f"Updated distribution - Best reward: {self.best_value:.4f}, "
                    f"Elite mean reward: {np.mean(elite_rewards):.4f}")
        
    def plan(self):
        """Perform one iteration of CEM planning."""
        if self.computational_budget_used >= self.computational_budget_max:
            return
            
        # Sample action sequences
        action_sequences = self._sample_action_sequences()
        
        # Evaluate sequences
        rewards = self._evaluate_population(action_sequences)
        
        # Update distribution
        self._update_distribution(action_sequences, rewards)
        
        # Decay noise for next iteration
        self.noise_scale *= self.noise_decay
        
        # Update covariance scaling
        self.cov *= (self.noise_decay ** 2)
        
        # Store planned trajectory (best sequence found so far)
        if self.best_sequence is not None:
            self.planned_trajectory = {
                'state_list': [],  # Would need to simulate to get states
                'action_list': self.best_sequence.tolist(),
                'dt_list': [self.env.agent.dt] * len(self.best_sequence),
            }
    
    def plan_while_in_budget(self, computational: bool = True, time: bool = False):
        """
        Override parent method to perform multiple CEM iterations.
        """
        self.computational_budget_used = 0
        self.time_budget_used = 0.0
        self.timer_start = self.time_budget_used
        
        # Warm start from previous solution if available and enabled
        if self.warm_start and self.best_sequence is not None:
            # Shift previous solution and add random action at the end
            shifted_sequence = np.roll(self.best_sequence, -1, axis=0)
            shifted_sequence[-1] = self.env.action_space.sample()
            self.mean = shifted_sequence
            
        # Run CEM iterations
        for iteration in range(self.num_iterations):
            # Check budget constraints
            if computational and self.computational_budget_used >= self.computational_budget_max:
                logger.info(f"Computational budget exhausted at iteration {iteration}")
                break
                
            if time and self.time_budget_used >= self.time_budget_max:
                logger.info(f"Time budget exhausted at iteration {iteration}")
                break
                
            # Perform one CEM iteration
            self.plan()
            
            logger.debug(f"CEM iteration {iteration + 1}/{self.num_iterations} completed. "
                        f"Budget used: {self.computational_budget_used}/{self.computational_budget_max}")
        
        # Update total budgets
        self.time_budget_total += self.time_budget_used
        self.computational_budget_total += self.computational_budget_used
        
    def get_best_action(self) -> np.ndarray:
        """
        Get the best action (first action from best sequence).
        
        Returns:
            Best action to take
        """
        if self.best_sequence is None:
            logger.warning("No plan available, returning random action")
            return self.env.action_space.sample()
            
        return self.best_sequence[0]
        
    def render(self, render_mode: Optional[str] = None):
        """Render the current plan."""
        if hasattr(self.env, 'render'):
            self.env.render(mode=render_mode)
        else:
            logger.warning("Environment does not support rendering")
            
    def get_planning_info(self) -> dict:
        """
        Get additional information about the planning process.
        
        Returns:
            Dictionary with planning statistics
        """
        return {
            'best_value': self.best_value,
            'population_size': self.population_size,
            'elite_size': self.elite_size,
            'planning_horizon': self.planning_horizon,
            'current_noise_scale': self.noise_scale,
            'has_plan': self.best_sequence is not None,
            'mean_action_magnitude': np.linalg.norm(self.mean) if self.mean is not None else 0.0,
        }