import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Optional

import logging
logging.basicConfig(level=logging.INFO)

from tools.robots import SingleIntegrator, Robot, Agent

class GymAgentPlanEnv(gym.Env):
    """
    Class for a gym environment that supports planning with an agent and a planner.
    """


    def __init__(self, agent: Agent, render_mode=None, multi_step_count: int = 1, observation_space: Optional[spaces.Space] = None):
        logging.debug("-- Starting initialization of GymAgentPlanEnv with agent:", agent)

        super().__init__()
        self.agent = agent
        self.render_mode = render_mode

        self.goal_obs = None
        self.start_obs = None

        if observation_space is not None:
            self.observation_space = observation_space
        else:
            self.observation_space = agent.state_space

        self.action_space = agent.action_space

        self.multi_step_count = multi_step_count # Amounts of steps to take in multi_step

        logging.debug("-- Finished initializing GymAgentPlanEnv")

    def _get_obs(self, state: Optional[np.ndarray] = None):
        if state is None:
            state = self.agent.state
        return state

    def step(self, action, render: bool = False):
        """Take multiple steps with the same action."""
        action_list = [action] * self.multi_step_count
        obs, reward, terminated, truncated, info = self.multi_step(action_list, render=render)
        return obs, reward, terminated, truncated, info

    def multi_step(self, action_list: list, render: bool = False):
        """Take multiple steps with the given list of actions."""
        terminated = False
        truncated = False
        obs = None
        reward = 0.0
        info = {}
        states_visited = []
        assert len(action_list) > 0, "Action list must contain at least one action."
        for i, action in enumerate(action_list):
            # logging.debug(f"Step {i}: taking action {action}")
            state = self.agent.step(action)
            terminated = self.agent.is_finished(state)
            truncated = self.is_truncated(state)
            obs = self._get_obs()
            reward = self.reward_function(state)
            if terminated or truncated:
                break

            states_visited.append(state)

            if render:
                self.render()
        info['states_visited'] = states_visited
        info['steps_taken'] = i + 1  # Number of steps actually taken
        
        return obs, reward, terminated, truncated, info

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        logging.debug("GymAgentPlanEnv reset called. Super class type:", super())
        super().reset(seed=seed, options=options)
  
        # Reset agent state to start state
        self.agent.reset(rng=self.np_random, state=None, state_goal=None)

        self.start_obs = self.observation_space.sample()
        self.goal_obs = None
        return self.start_obs,  {}
    
    def reward_function(self, state):
        raise NotImplementedError("Reward function must be implemented in the subclass.")
    
    def is_truncated(self, state=None) -> bool:
        """Check if the agent has been truncated."""
        raise NotImplementedError("is_truncated must be implemented in the subclass.")
    
