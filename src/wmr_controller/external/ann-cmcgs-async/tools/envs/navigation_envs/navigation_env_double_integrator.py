import numpy as np
import gymnasium as gym
from typing import Optional

import tools.robots as robots
import tools.envs as envs
import tools.planners as planners

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

class NavigationEnvDoubleIntegrator(envs.NavigationEnv):
    def __init__(self, dt=1.0, render_mode=None, atol=0.0, rtol=0.1, multi_step_count: int = 1, obstacle_mode="circles", render_sleep=0.0, plan_in_space_time=True, default_step_divisor: int = 1, sparse_reward: bool = True):
        logger.info("---- Starting Initialization of NavigationEnvDoubleIntegrator")
        super().__init__(dt=dt, 
                         render_mode=render_mode, 
                         atol=atol, rtol=rtol, 
                         multi_step_count=multi_step_count, 
                         obstacle_mode=obstacle_mode, 
                         render_sleep=render_sleep,
                         plan_in_space_time=plan_in_space_time,
                         sparse_reward=sparse_reward
                         )
        
        ##### DoubleIntegrator

        self.start_vel = np.array([0.0, 0.0])  # start with zero velocity

        self.max_velocity = 1.0  # max velocity in each dimension
        self.max_acceleration = 0.5  # max acceleration in each dimension

        low_obs = np.array([self.pos_limits[0], self.pos_limits[2], -self.max_velocity, -self.max_velocity, self.start_t, self.pos_limits[0], self.pos_limits[2]], dtype=np.float32)  # [x, y, vx, vy, time, goal_x, goal_y]
        high_obs = np.array([self.pos_limits[1], self.pos_limits[3], self.max_velocity, self.max_velocity, np.inf, self.pos_limits[1], self.pos_limits[3]], dtype=np.float32)  # [x, y, time, goal_x, goal_y]
        # low_obs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
        # high_obs = np.array([size, size, np.inf, size, size], dtype=np.float32)
        robot_state_space = gym.spaces.Box(low=low_obs[:5], high=high_obs[:5], dtype=np.float32)  # [x, y, dx, dy, time]

        action_space = gym.spaces.Box(low=np.array([-self.max_acceleration, -self.max_acceleration]), 
                                      high=np.array([self.max_acceleration, self.max_acceleration]), dtype=np.float32)  # [ddx, ddy]


        self.agent = robots.DoubleIntegrator(
            id=0,
            name="2D Navigation Robot",
            state_space=robot_state_space,
            action_space=action_space,
            dt=self.dt,
            max_velocity=self.max_velocity,
            max_acceleration=self.max_acceleration,
            action_sampler_method='uniform',
            color="#00FF00",
            atol=self.atol,
            rtol=self.rtol,
            default_step_divisor=default_step_divisor,
            plan_in_space_time=plan_in_space_time
        )

        self.observation_space = gym.spaces.Box(low=low_obs, high=high_obs, dtype=np.float32) # Environment observation space

        self.__complete_init__() # After overwriting self.agent and self.observation_space in subclass, complete the rest of the init
        logger.info("---- Finished Initialization of NavigationEnvDoubleIntegrator")

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        logger.info(f"---- NavigationEnvDoubleIntegrator reset called with seed: {seed} and options: {options}")
        self.reset_rng(seed=seed, options=options)
       
        #### DoubleIntegrator
        # Reset robot + assign new random goal
        self.start_obs = self.observation_space.sample()  # Initialize start_obs first
        self.start_obs[:2] = self.start_pos
        self.start_obs[2:4] = self.start_vel
        self.start_obs[4] = self.start_t
        self.start_obs[5:] = self.goal_pos # set goal
        # print("Sampled start_obs:", self.start_obs)
        gx = self.start_obs[5]
        gy = self.start_obs[6]
        vx, vy = np.nan, np.nan # allow any velocity
        t = np.nan # allow any time
        self.goal_obs = np.array([gx, gy, vx, vy, t, gx, gy], dtype=np.float32)
        logger.info(f"start_obs: {self.start_obs}, goal_obs: {self.goal_obs}")

        agent_state = self.start_obs[:5]  # [x,y,vx,vy,t]
        agent_goal_state = self.goal_obs[:5]  # [gx,gy,vx,vy,t]
        #### DoubleIntegrator

        
        self.agent.reset(seed=seed, state=agent_state, state_goal=agent_goal_state)
        # print(self.np_random.bit_generator.state['state']['state'], self.agent.np_random.bit_generator.state['state']['state'])
        # print(self.np_random_seed, self.np_random, self.np_random.uniform(0,1,1))
        obs = self._get_obs()
        info = {"Sampled states": [self.agent.state, self.agent.state_goal]}
        # print("Final obs after reset:", obs)
        return obs, info

    
    def _get_obs(self, state=None):
        if state is None:
            state = self.agent.state
        goal = self.goal_obs
        # print("Getting obs. State:", state, "Goal:", goal)
        #### DoubleIntegrator
        return np.array([state[0], state[1], state[2], state[3], state[4], goal[0], goal[1]], dtype=np.float32)

    def _get_vel(self, state=None):
        if state is None:
            state = self.agent.state
        return np.array([state[2], state[3]], dtype=np.float32)

    def _get_t(self, state=None):
        if state is None:
            state = self.agent.state
        return np.array(state[-1], dtype=np.float32)

    # Reward function
    def reward_function(self, state):
        # Example reward function: negative distance to the goal position
    
        rew_v = np.exp(-10 * np.linalg.norm(self._get_vel(state))**2)
   
        # collision
        rew_obs = -1 * self.is_truncated(state)
        rew_alive = 0 if self.is_truncated(state) else 1

        # finished
        rew_goal = 1 * self.agent.is_finished(state, atol=self.atol, rtol=self.rtol)

        # gradient to goal
        if self.sparse_reward:
            rew_grad = 0
        else:
            rew_grad = -1 * np.linalg.norm(self._get_pos(state) - self.goal_pos)**2

        reward = 0.00 * rew_alive + 0.97 * rew_goal + rew_obs + 0.00 * rew_grad # + 0.02 * rew_v

        return reward
