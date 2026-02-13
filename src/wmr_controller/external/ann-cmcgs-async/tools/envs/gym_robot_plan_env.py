import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Optional

from tools.robots import SingleIntegrator, Robot, Agent
from tools.envs.gym_agent_plan_env import GymAgentPlanEnv


class GymRobotPlanEnv(GymAgentPlanEnv):
    """
    Class for a gym environment that supports planning with a robot and a planner.
    """


    def __init__(self, agent: Robot, render_mode=None, multi_step_count: int = 1, observations_space: Optional[spaces.Space] = None):
        super().__init__(agent=agent, render_mode=render_mode, multi_step_count=multi_step_count, observation_space=observations_space)


    def step(self, action, render: bool = False):
        """Take multiple steps with the same action."""
        action_list = [action] * self.multi_step_count
        dt_list = [self.agent.dt / self.multi_step_count] * self.multi_step_count
        obs, reward, terminated, truncated, info = self.multi_step(action_list, dt_list=dt_list, render=render)
        return obs, reward, terminated, truncated, info

    def multi_step(self, action_list: list, dt_list: list = None, render: bool = False):
        """Take multiple steps with the given list of actions."""
        terminated = False
        truncated = False
        obs = None
        reward = 0.0
        info = {}
        states_visited = []
        observations_visited = []
        assert len(action_list) > 0, "Action list must contain at least one action."
        for i, action in enumerate(action_list):
            # print(f"Step {i}: taking action {action}")
            state = self.agent.step(action, dt=dt_list[i] if dt_list else None)
            terminated = self.agent.is_finished(state)
            truncated = self.is_truncated(state)
            obs = self._get_obs()
            reward = self.reward_function(state)
            if terminated or truncated:
                break
            if render:
                self.render()
            states_visited.append(state)
            observations_visited.append(obs)

        info['states_visited'] = states_visited
        info['observations_visited'] = observations_visited
        info['steps_taken'] = i + 1  # Number of steps actually taken
        return obs, reward, terminated, truncated, info
    