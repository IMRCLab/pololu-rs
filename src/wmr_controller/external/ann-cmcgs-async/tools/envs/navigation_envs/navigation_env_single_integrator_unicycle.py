import numpy as np
import gymnasium as gym
from typing import Optional

import tools.robots as robots
import tools.envs as envs
import tools.planners as planners



class NavigationEnvSingleIntegratorUnicycle(envs.NavigationEnv):
    def __init__(self, dt=1.0, render_mode=None, atol=0.0, rtol=0.1, multi_step_count: int = 1, obstacle_mode="circles", render_sleep=0.0, default_step_divisor=1, plan_in_space_time: bool = True, sparse_reward: bool = True):
        print("---- Starting Initialization of NavigationEnvSingleIntegratorUnicycle")
        super().__init__(dt=dt, 
                         render_mode=render_mode, 
                         atol=atol, rtol=rtol, 
                         multi_step_count=multi_step_count, 
                         obstacle_mode=obstacle_mode, 
                         render_sleep=render_sleep,
                         plan_in_space_time=plan_in_space_time,
                         sparse_reward=sparse_reward
                         )
        ##### SingleIntegratorUnicycle

        self.max_velocity = 1.0  # max linear velocity
        self.max_angular = 1.0   # max angular velocity

        # State: [x, y, theta, time], Goal: [goal_x, goal_y]
        low_obs = np.array([self.pos_limits[0], self.pos_limits[2], -np.pi, self.start_t, self.pos_limits[0], self.pos_limits[2]], dtype=np.float32)  # [x, y, theta, time, goal_x, goal_y]
        high_obs = np.array([self.pos_limits[1], self.pos_limits[3], np.pi, np.inf, self.pos_limits[1], self.pos_limits[3]], dtype=np.float32)  # [x, y, theta, time, goal_x, goal_y]
        
        robot_state_space = gym.spaces.Box(low=low_obs[:4], high=high_obs[:4], dtype=np.float32)  # [x, y, theta, time]
        
        # Action space: [linear_velocity, angular_velocity]
        action_space = gym.spaces.Box(low=np.array([0.0, -self.max_angular]), 
                                      high=np.array([self.max_velocity, self.max_angular]), dtype=np.float32)  # [v, omega]

        self.agent = robots.SingleIntegratorUnicycle(
            id=0,
            name="2D Navigation Unicycle Robot",
            state_space=robot_state_space,
            action_space=action_space,
            dt=self.dt,
            max_velocity=self.max_velocity,
            max_angular=self.max_angular,
            action_sampler_method='uniform',
            color="#FFF000",  # Yellow color to distinguish from regular single integrator
            atol=self.atol,
            rtol=self.rtol,
            default_step_divisor=default_step_divisor,
            plan_in_space_time=self.plan_in_space_time
        )

        self.observation_space = gym.spaces.Box(low=low_obs, high=high_obs, dtype=np.float32) # Environment observation space

        self.__complete_init__() # After overwriting self.agent and self.observation_space in subclass, complete the rest of the init
        print("---- Finished Initialization of NavigationEnvSingleIntegratorUnicycle")

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        print("---- NavigationEnvSingleIntegratorUnicycle reset called with seed:", seed, "and options:", options)
        super().reset_rng(seed=seed, options=options)

        #### SingleIntegratorUnicycle
        # Reset robot + assign new random goal
        self.start_obs = self.observation_space.sample() # Placeholder for start observation
        
        self.start_obs[:2] = self.start_pos  # [x, y]
        self.start_obs[2] = self.agent._wrap_to_state_space_angle(0.0)  # Ensure initial angle is properly wrapped
        self.start_obs[3] = self.start_t  # time
        self.start_obs[4:] = self.goal_pos # set goal [goal_x, goal_y]
        
        gx = self.start_obs[4]
        gy = self.start_obs[5]
        theta = np.nan
        t = np.nan # allow any time
        self.goal_obs = np.array([gx, gy, theta, t, gx, gy], dtype=np.float32)  # Goal orientation doesn't matter
        print("start_obs:", self.start_obs, "goal_obs:", self.goal_obs)
        
        agent_state = self.start_obs[:4]  # [x, y, theta, t]
        agent_goal_state = self.goal_obs[[4, 5, 2, 3]]  # [gx, gy, theta, t] - ignore goal orientation
        #### SingleIntegratorUnicycle

        self.agent.reset(seed=seed, state=agent_state, state_goal=agent_goal_state)
        obs = self._get_obs()
        info = {"Sampled states": [self.agent.state, self.agent.state_goal]}
        return obs, info

    
    def _get_obs(self, state=None):
        if state is None:
            state = self.agent.state
        goal = self.goal_obs
        #### SingleIntegratorUnicycle
        # State: [x, y, theta, t], Goal: [goal_x, goal_y] (no goal orientation)
        return np.array([state[0], state[1], state[2], state[3], goal[4], goal[5]], dtype=np.float32)
    
    def _get_pos(self, state=None):
        observation = self._get_obs(state)
        # Observation is [x, y, theta, t, goal_x, goal_y]
        return observation[0:2]
    
    def _get_orientation(self, state=None):
        observation = self._get_obs(state)
        # Observation is [x, y, theta, t, goal_x, goal_y]
        return observation[2]
    
    def _get_t(self, state=None):
        observation = self._get_obs(state)
        # Observation is [x, y, theta, t, goal_x, goal_y]
        return observation[3]
    
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

        reward = 0.00 * rew_alive + 0.97 * rew_goal + rew_obs + 0.00 * rew_grad

        return reward

   