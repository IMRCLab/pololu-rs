import gymnasium as gym
from gymnasium import spaces
import numpy as np
from typing import Optional

from .frozen_lake_continuous import FrozenLakeEnv
from tools.robots import SingleIntegrator
from tools.planners import Planner


class FrozenLakeSingleRobotEnv(gym.Env):
    """
    Gym wrapper for FrozenLake with robot dynamics.
    The agent is one Robot (e.g. SingleIntegrator, DoubleIntegrator).
    """
    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self, size=10, n_puddles=4, n_obstacles=4,
                 dt=0.1, max_velocity=1.0, slippery_everywhere=0.0, render_mode=None, atol=0.0, rtol=0.1):
        super().__init__()
        self.env = FrozenLakeEnv(size=size, n_puddles=n_puddles, n_obstacles=n_obstacles)
        self.dt = dt
        self.max_velocity = max_velocity
        self.slippery_everywhere = slippery_everywhere
        self.render_mode = render_mode

        self.atol = atol
        self.rtol = rtol

        # Gym spaces
        # obs = [x, y, t, gx, gy] for simplicity
        low_obs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
        high_obs = np.array([size, size, np.inf, size, size], dtype=np.float32)
        self.observation_space = spaces.Box(low=low_obs, high=high_obs, dtype=np.float32) # Environment observation space
        self.robot_state_space = spaces.Box(low=low_obs[:3], high=high_obs[:3], dtype=np.float32)  # [x, y, time]
        # self.robot_state_space = self.observation_space
        self.action_space = spaces.Box(low=np.array([-max_velocity, -max_velocity]), high=np.array([max_velocity, max_velocity]), dtype=np.float32)  # [dx, dy]

        # Agent
        self.robot = SingleIntegrator(
            id=0, 
            name="agent", 
            state_space=self.observation_space,
            action_space=self.action_space,
            dt=self.dt,
            max_velocity=self.max_velocity,
            action_sampler_method='uniform',
            color="#FFFF00",
            atol=self.atol,
            rtol=self.rtol
        )
        
        # self.reset()

    def _get_obs(self):
        state = self.robot.state
        goal = self.goal_obs
        # print("start_obs:", self.start_obs, "goal_obs:", self.goal_obs)
        # print("robot state:", self.robot.state, "robot goal:", self.robot.state_goal)
        return np.array([state[0], state[1], state[2], goal[0], goal[1]], dtype=np.float32)

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        if seed is None:
            seed = np.random.randint(0, 10000)
            print("WARNING: No seed provided to env.reset(). Using random seed:", seed)

        super().reset(seed=seed)
        self.observation_space.seed(seed)
        self.action_space.seed(seed)
        self.robot_state_space.seed(seed)
 
        # print(self.np_random.bit_generator.state['state']['state'], self.env.np_random.bit_generator.state['state']['state'], self.robot.np_random.bit_generator.state['state']['state'])
        # print(self.np_random_seed, self.np_random, self.np_random.uniform(0,1,1))


        # Reset robot + assign new random goal
        # self.start_obs = [0, 0, 0, 0, 0]
        self.start_obs = self.observation_space.sample()
        gx = self.start_obs[3]
        gy = self.start_obs[4]
        t = np.nan # allow any time
        self.goal_obs = np.array([gx, gy, t, gx, gy], dtype=np.float32)
        print("start_obs:", self.start_obs, "goal_obs:", self.goal_obs)
        
        self.env.reset(rng=self.np_random) # Only set seed once.

        robot_state = self.start_obs[:3]  # [x,y,t]
        robot_goal_state = self.goal_obs[[3,4,2]]  # [gx,gy,t]
        self.robot.reset(rng=self.np_random, state=robot_state, state_goal=robot_goal_state)

        obs = self._get_obs()
        info = {"Sampled states": [self.robot.state, self.robot.state_goal]}
        return obs, info

    def step(self, action):
        state = self.robot.state
        # Apply dynamics with optional puddle residual
        if self.env.n_puddles > 0:
            residual = self.env.residual_model_frozen_puddle(state, action)
        elif self.slippery_everywhere > 0:
            residual = self.slippery_everywhere * self.np_random.normal(0, 1, size=action.shape)
        else:
            residual = None
        self.robot.step(action, residual)

        # Observation
        obs = self._get_obs()

        # Reward: negative distance to goal
        reward = self.reward_function(self.robot.state)

        # Termination
        terminated = self.robot.is_finished(atol=self.atol, rtol=self.rtol)
        truncated = False  # could set time limit

        info = {"robot state": self.robot.state, "reward": reward}
        return obs, reward, terminated, truncated, info

    def render(self):
        if self.planner is None:
            planner = []
        else:
            planner = [self.planner]
        self.env.render_live(robots=[self.robot], 
                             planners=planner,
                             render_tree=True)

    def close(self):
        pass

    # Reward function
    def reward_function(self, state):
        # Example reward function: negative distance to the goal position
   
        x = state[:2]
        x_goal = self.goal_obs[:2]
        mask = ~(np.isnan(x) | np.isnan(x_goal)) # ignore NaN values (e.g., time if reaching at any time is allowed)
        c_pos = np.linalg.norm(x[mask] - x_goal[mask], ord=2)

        c_time = 1*state[2]
        return - np.sum([c_time, c_pos])

    def connect_planner(self, planner:Planner):
        self.planner = planner