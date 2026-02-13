import numpy as np
import gymnasium as gym
from typing import Optional

import tools.robots as robots
import tools.envs as envs
import tools.planners as planners



class NavigationEnvPololu(envs.NavigationEnvSingleIntegratorUnicycleSpace):
    def __init__(self, dt=1.0, render_mode=None, atol=0.0, rtol=0.1, multi_step_count: int = 1, obstacle_mode="circles", render_sleep=0.0, sparse_reward: bool = True):
        print("---- Starting Initialization of NavigationEnvSingleIntegratorUnicycleSpace")
        super().__init__(dt=dt, 
                         render_mode=render_mode, 
                         atol=atol, rtol=rtol, 
                         multi_step_count=multi_step_count, 
                         obstacle_mode=obstacle_mode, 
                         render_sleep=render_sleep,
                         sparse_reward=sparse_reward
                         )
        ##### Pololu

        self.max_velocity = 1.0  # max linear velocity
        self.max_angular = np.pi/2   # max angular velocity

        # State: [x, y, theta, time], Goal: [goal_x, goal_y]
        low_obs = np.array([self.pos_limits[0], self.pos_limits[2], -np.pi, self.start_t, self.pos_limits[0], self.pos_limits[2]], dtype=np.float32)  # [x, y, theta, time, goal_x, goal_y]
        high_obs = np.array([self.pos_limits[1], self.pos_limits[3], np.pi, np.inf, self.pos_limits[1], self.pos_limits[3]], dtype=np.float32)  # [x, y, theta, time, goal_x, goal_y]
        
        robot_state_space = gym.spaces.Box(low=low_obs[:4], high=high_obs[:4], dtype=np.float32)  # [x, y, theta, time]
        
        # Action space: [linear_velocity, angular_velocity]
        action_space = gym.spaces.Box(low=np.array([0.0, -self.max_angular]), 
                                      high=np.array([self.max_velocity, self.max_angular]), dtype=np.float32)  # [v, omega]

        self.agent = robots.Pololu(
            id=0,
            name="Pololu",
            state_space=robot_state_space,
            action_space=action_space,
            dt=self.dt,
            max_velocity=self.max_velocity,
            max_angular=self.max_angular,
            action_sampler_method='uniform_steering',
            color="#00FF00",
            atol=self.atol,
            rtol=self.rtol,
        )

        self.observation_space = gym.spaces.Box(low=low_obs, high=high_obs, dtype=np.float32) # Environment observation space

        self.__complete_init__() # After overwriting self.agent and self.observation_space in subclass, complete the rest of the init
        print("---- Finished Initialization of NavigationEnvSingleIntegratorUnicycle")


    
    # Reward function
    def reward_function(self, state):
 
        # collision
        rew_obs = -1 * self.is_truncated(state)
        rew_alive = 0 if self.is_truncated(state) else 1

        # finished
        rew_goal = 1 * self.agent.is_finished(state, atol=self.atol, rtol=self.rtol)

        # gradient to goal (position only, ignore orientation)
        if self.sparse_reward:
            rew_grad = 0
        else:
            rew_grad = -1 * np.linalg.norm(self._get_pos(state) - self.goal_pos)**2

        # Optional: Small penalty for large angular changes (encourage smooth motion)
        # rew_smooth = -0.001 * abs(self._get_orientation(state))  # Penalize large angles from 0

        reward = 0.01 * rew_alive + 0.97 * rew_goal + rew_obs + 0.00 * rew_grad

        return reward

   