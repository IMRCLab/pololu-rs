import numpy as np
import gymnasium as gym
from typing import Optional
import logging

import tools.robots as robots
import tools.envs as envs
import tools.planners as planners

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


class NavigationEnvBaseline(envs.NavigationEnv):
    def __init__(self, dt=1.0, render_mode=None, atol=0.0, rtol=0.1, multi_step_count: int = 1, obstacle_mode="circles", render_sleep=0.0, default_step_divisor: int = 1, plan_in_space_time: bool=True, sparse_reward: bool = True):
        logger.info("---- Starting Initialization of NavigationEnvBaseline")
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
        self.max_acceleration = 1.0  # max acceleration in each dimension

        x_min = np.min([self.start_pos[0], self.goal_pos[0]])
        x_max = np.max([self.start_pos[0], self.goal_pos[0]])
        y_min = np.min([self.start_pos[1], self.goal_pos[1]])
        y_max = np.max([self.start_pos[1], self.goal_pos[1]])

        self.pos_limits = [x_min, 
                           x_max, 
                           self.start_pos[1] - (y_max - y_min) - 1, 
                           self.start_pos[1] + (y_max - y_min) + 1]


        low_obs = np.array([self.pos_limits[0], self.pos_limits[2], -self.max_velocity, -self.max_velocity, self.start_t, self.pos_limits[0], self.pos_limits[2]], dtype=np.float32)  # [x, y, vx, vy, time, goal_x, goal_y]
        high_obs = np.array([self.pos_limits[1], self.pos_limits[3], self.max_velocity, self.max_velocity, np.inf, self.pos_limits[1], self.pos_limits[3]], dtype=np.float32)  # [x, y, time, goal_x, goal_y]
        # low_obs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
        # high_obs = np.array([size, size, np.inf, size, size], dtype=np.float32)
        robot_state_space = gym.spaces.Box(low=np.array([-np.inf, -self.max_velocity, 0.0]), 
                                     high=np.array([np.inf, self.max_velocity, np.inf]), 
                                     dtype=np.float32)  # [y, dy, t]
        
        action_space = gym.spaces.Box(low=np.array([-self.max_acceleration]), 
                                      high=np.array([self.max_acceleration]), 
                                      dtype=np.float32)  # [ddx, ddy]


        self.agent = robots.BaselineDoubleIntegrator(
            id=0,
            name="1D Navigation Robot",
            state_space=robot_state_space,
            action_space=action_space,
            dt=self.dt,
            max_velocity=self.max_velocity,
            max_acceleration=self.max_acceleration,
            action_sampler_method='uniform',
            color="#00FF00",
            atol=self.atol,
            rtol=self.rtol,
            plan_in_space_time=self.plan_in_space_time,
            default_step_divisor=default_step_divisor
        )

        self.observation_space = gym.spaces.Box(low=low_obs, high=high_obs, dtype=np.float32) # Environment observation space



        self.__complete_init__() # After overwriting self.agent and self.observation_space in subclass, complete the rest of the init
        logger.info("---- Finished Initialization of NavigationEnvBaseline")

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        logger.info(f"---- NavigationEnvBaseline reset called with seed: {seed} and options: {options}")
        super().reset_rng(seed=seed, options=options)
       
        #### BaselineDoubleIntegrator
        # Reset robot
        self.start_obs = self.observation_space.sample()  # Initialize start_obs
        s_x = self.start_pos[0]
        s_y = self.start_pos[1]
        s_vx = self.dt  # fixed velocity in x
        s_vy = 0.0  # start with zero velocity in y
        s_t = self.start_t
        gx = self.goal_pos[0]
        gy = self.goal_pos[1]
        self.start_obs = np.array([s_x, s_y, s_vx, s_vy, s_t, gx, gy], dtype=np.float32)

        vx, vy = np.nan, np.nan # allow any velocity
        t = gx # identical to number of steps
        self.goal_obs = np.array([gx, gy, vx, vy, t, gx, gy], dtype=np.float32)
        logger.debug("start_obs:", self.start_obs, "goal_obs:", self.goal_obs)

        agent_state = self._get_state(self.start_obs)  # [y,vy,t]
        logger.debug("agent_state:", agent_state)
        agent_goal_state = np.array([gy, vy, t], dtype=np.float32)  # [gy,vy,t]
        #### BaselineDoubleIntegrator

        
        self.agent.reset(seed=seed, state=agent_state, state_goal=agent_goal_state)
        # logger.debug(self.np_random.bit_generator.state['state']['state'], self.agent.np_random.bit_generator.state['state']['state'])
        # logger.debug(self.np_random_seed, self.np_random, self.np_random.uniform(0,1,1))
        obs = self._get_obs()
        info = {"Sampled states": [self.agent.state, self.agent.state_goal]}
        # logger.debug("Final obs after reset:", obs)
        return obs, info

    
    def _get_obs(self, state=None):
        if state is None:
            state = self.agent.state
        goal = self.goal_obs
        # State is [y, vy, t]
        # Missing info for x: x=t, vx=dt
        y = state[0]
        vy = state[1]
        t = state[2]
        x = t #+ 1
        vx = self.dt
        gx = goal[0]
        gy = goal[1]
        # logger.debug("Getting obs. State:", state, "Goal:", goal)
        #### BaselineDoubleIntegrator
        obs = np.array([x, y, vx, vy, t, gx, gy], dtype=np.float32)
        # logger.debug("Obs:", obs)
        return obs
    
        
    def _get_vel(self, state=None):
        observation = self._get_obs(state)
        # Observation is [x, y, vx, vy, t, goal_x, goal_y]
        return observation[2:4]
    
    def _get_t(self, state=None):
        observation = self._get_obs(state)
        # Observation is [x, y, vx, vy, t, goal_x, goal_y]
        return observation[4]

    def _get_state(self, observation):
        # Observation is [x, y, vx, vy, t, goal_x, goal_y]
        x = observation[0]
        y = observation[1]
        vx = observation[2]
        vy = observation[3]
        t = observation[4]
        gx = observation[5]
        gy = observation[6]
        return np.array([y, vy, t], dtype=np.float32)  # state is [y, vy, t]
    

    # Reward function
    def reward_function(self, state):
    
        rew_vy = np.exp(-10 * np.power(state[1], 2))

        # collision
        rew_obs = -1 * self.is_truncated(state)
        rew_alive = 0 if self.is_truncated(state) else 1

        # finished
        rew_goal = 1 * self.agent.is_finished(state, atol=self.atol, rtol=self.rtol)

        reward = 0.01 * rew_alive + 0.97 * rew_goal + rew_obs # + 0.02 * rew_vy 

        return reward
    

    def is_truncated(self, state=None) -> bool:
        def check_collision(state, obstacle):
            [x, y, s] = obstacle
            observation = self._get_obs(state) # get full info from observation (e.g., missing x from t)
            
            # logger.debug("Checking collision with obstacle at", obstacle, "for state", state)
            if self.obstacle_mode == "circles":
                if np.linalg.norm(observation[:2] - [x, y]) < s:
                    # logger.debug("Collision with circle at", obstacle)
                    return True
            else:
                w, h = self.box_obstacle_width / 2, s / 2
                if np.abs(observation[0] - x) < w + 0.001 and np.abs(observation[1] - y) < h + 0.001:
                    return True
            return False

        # in_observation = True
        in_observation = self.observation_space.contains(self._get_obs(state))
        is_collision = bool(np.any([check_collision(state, obstacle) for obstacle in self.obstacles]))
        # logger.debug(f"Truncation check: in_observation={in_observation}, is_collision={is_collision}")
        
        return not in_observation or is_collision
    