import numpy as np
import gymnasium as gym
from gymnasium import spaces
from typing import Optional, TYPE_CHECKING

import tools.robots as robots
import tools.envs as envs

if TYPE_CHECKING:
    # from tools.planners.planner import Planner
    from tools.planners import Planner, MCGSPlanner

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)



class NavigationEnv(envs.GymRobotPlanEnv):
    """Base szenario environment for 2D navigation with obstacles."""
    def __init__(self, dt=1.0, render_mode=None, atol=0.0, rtol=0.1, multi_step_count: int = 1, obstacle_mode="circles", render_sleep=0.0, plan_in_space_time: bool = False, sparse_reward: bool = True):
        logger.info("---- Starting Initialization of NavigationEnv")
        self.dt = dt

        self.atol = atol
        self.rtol = rtol
        self.plan_in_space_time = plan_in_space_time
        self.sparse_reward = sparse_reward

        self.obstacle_mode = obstacle_mode
        self.box_obstacle_width = 0.15

        self._max_episode_steps = 9
        self.goal_pos = [self._max_episode_steps, 4]
        self.start_pos = [0, 3] # [1, 3]
        self.start_t = 0

        self.render_sleep = render_sleep

        x_min = np.min([self.start_pos[0], self.goal_pos[0]])
        x_max = np.max([self.start_pos[0], self.goal_pos[0]])
        y_min = np.min([self.start_pos[1], self.goal_pos[1]])
        y_max = np.max([self.start_pos[1], self.goal_pos[1]])

        self.pos_limits = [x_min, 
                           x_max, 
                           self.start_pos[1] - (y_max - y_min) - 1, 
                           self.start_pos[1] + (y_max - y_min) + 1]



        ##### Use default robot
        if not hasattr(self, 'agent'):
            self.max_velocity = 1.0  # max velocity in each dimension

            low_obs = np.array([self.pos_limits[0], self.pos_limits[2], self.start_t, self.pos_limits[0], self.pos_limits[2]], dtype=np.float32)  # [x, y, time, goal_x, goal_y]
            high_obs = np.array([self.pos_limits[1], self.pos_limits[3], np.inf, self.pos_limits[1], self.pos_limits[3]], dtype=np.float32)  # [x, y, time, goal_x, goal_y]
            # low_obs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
            # high_obs = np.array([size, size, np.inf, size, size], dtype=np.float32)
            
            
            robot_state_space = gym.spaces.Box(low=low_obs[:3], high=high_obs[:3], dtype=np.float32)  # [x, y, time]
            action_space = gym.spaces.Box(low=np.array([-self.max_velocity, -self.max_velocity]), 
                                        high=np.array([self.max_velocity, self.max_velocity]), dtype=np.float32)  # [dx, dy]


            self.agent = robots.Robot(
                id=0,
                name="2D Navigation Robot",
                state_space=gym.spaces.Box(low=np.array([self.pos_limits[0], self.pos_limits[2], -np.inf, -np.inf, self.start_t]), 
                                        high=np.array([self.pos_limits[1], self.pos_limits[3], np.inf, np.inf, np.inf]), dtype=np.float32),  # [x, y, vx, vy, time]
                action_space=gym.spaces.Box(low=np.array([-np.inf, -np.inf]), 
                                            high=np.array([np.inf, np.inf]), dtype=np.float32),  # [ddx, ddy]
                dt=self.dt,
                atol=self.atol,
                rtol=self.rtol     
            )

            self.observation_space = gym.spaces.Box(low=low_obs, high=high_obs, dtype=np.float32) # Environment observation space
        
        
        
        self.__complete_init__(agent=self.agent, 
                              render_mode=render_mode, 
                              multi_step_count=multi_step_count, 
                              observation_space=self.observation_space)
        

        self.metadata = {"render_modes": ["human", "matplotlib", "ansi"],
                         "render_fps": np.ceil(1 / self.dt)}
        


        if obstacle_mode == "circles":
            self.obstacles = [
                [2, 2.5, 1]
                , [3, 4, 0.3]
                , [5, 2.25, 0.4]
                , [7, 5, 1.75]
                , [7.5, 1.5, 0.7]
            ]
        elif obstacle_mode == "circles_small":
            self.obstacles = np.array([[2.95290874, 0.88765603, 0.10772181],
                                        [6.30594467, 3.35175003, 0.64885578],
                                        [9.06278134, 2.32004532, 0.8779015 ],
                                        [7.36353566, 4.69294498, 0.8888903 ],
                                        [3.69004885, 2.97949591, 0.77199091],
                                        [4.97897969, 4.65801671, 0.4012896 ],
                                        [2.04757936, 2.24645123, 0.47601461],
                                        [2.07593491, 4.93315506, 0.2579796 ],
                                        [6.45535538, 5.31007078, 0.52405575],
                                        [1.57847446, 0.7129848 , 0.51756384],
                                        [9.07709276, 1.14199651, 0.58505432],
                                        [1.85529041, 5.84404092, 0.41829406],
                                        [0.87963201, 4.69815558, 0.391667  ],
                                        [3.45640947, 0.97649596, 0.43951471],
                                        [4.45010895, 1.37203033, 0.18335983],
                                        [3.46798608, 5.65518546, 0.54889939],
                                        [3.25394292, 0.30534946, 0.45437182],
                                        [6.8417127, 1.37566289, 0.1557712],
                                        [5.06514258, 0.67243589, 0.50375921]])
        elif obstacle_mode == "boxes":
            self.obstacles = [
                [1, 1.5, 1]
                , [1, 3, 1]
                , [1, 4.5, 1]
                , [2, 1.6, 1.2]
                , [2, 3, 0.9]
                , [2, 4.4, 1.2]
                , [3, 1.5, 1]
                , [3, 4.5, 1]
                , [4, 3, 3.5]
                , [5, 1.25, 0.5]
                , [5, 3, 1.5]
                , [5, 4.75, 0.5]
                , [6, 2, 2]
                , [6, 4.3, 1.4]
                , [7, 1.5, 1]
                , [7, 3.25, 1]
                , [7, 4.5, 1]
                , [8, 1.5, 1]
                , [8, 4, 2]
            ]
            if True:
                self.obstacles = [
                    [1, 3, 2.25]
                    #, [2, 1.6, 1.2]
                    #, [2, 3, 0.9]
                    #, [2, 4.4, 1.2]
                    , [3, 1.5, 1]
                    , [3, 4.5, 1]
                    , [4, 1.5, 1]
                    , [4, 3, 1]
                    , [4, 4.5, 1]
                    , [5, 1.25, 0.5]
                    , [5, 3, 1.5]
                    , [5, 4.75, 0.5]
                    , [6, 2, 2]
                    , [6, 4.3, 1.4]
                    , [7, 1.5, 1]
                    , [7, 3.25, 1]
                    , [7, 4.5, 1]
                    , [8, 1.5, 1]
                    , [8, 4, 2]
                ]

        else:
            self.obstacles = []

        logger.info("---- Finished Initialization of NavigationEnv")

    def __complete_init__(self, agent: Optional[robots.Robot]=None, render_mode=None, multi_step_count=None, observation_space: Optional[spaces.Box]=None):
        super().__init__(agent=self.agent if agent is None else agent, 
                         render_mode=self.render_mode if render_mode is None else render_mode,
                         multi_step_count=self.multi_step_count if multi_step_count is None else multi_step_count,
                         observations_space=self.observation_space if observation_space is None else observation_space)

    def reset_rng(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        if seed is None:
            # seed = np.random.randint(0, 10000)
            logger.info(f"WARNING: No seed provided to env.reset(). Using random seed: {seed}")

        logger.info(f"Using provided seed: {seed}")
        super().reset(seed=seed, options=options)
        if seed is not None:
            self.observation_space.seed(seed)


        logger.debug(self.np_random.bit_generator.state['state']['state'], self.agent.np_random.bit_generator.state['state']['state'])
        logger.debug(self.np_random_seed, self.np_random, self.np_random.uniform(0,1,1))
    
    
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        logger.info(f"---- NavigationEnv reset called with seed: {seed} and options: {options}")
        self.reset_rng(seed=seed, options=options)

        #### SingleIntegrator
        # Reset robot + assign new random goal
        self.start_obs = self.observation_space.sample() # Placeholder for start observation

        self.start_obs[:2] = self.start_pos
        self.start_obs[2] = self.start_t
        self.start_obs[3:] = self.goal_pos # set goal
        # self.start_obs = self.observation_space.sample()
        # print("Sampled start_obs:", self.start_obs)
        gx = self.start_obs[3]
        gy = self.start_obs[4]
        t = np.nan # allow any time
        self.goal_obs = np.array([gx, gy, t, gx, gy], dtype=np.float32)
        logger.debug(f"start_obs: {self.start_obs}, goal_obs: {self.goal_obs}")
        
        agent_state = self.start_obs[:3]  # [x,y,t]
        agent_goal_state = self.goal_obs[[3,4,2]]  # [gx,gy,t]
        #### SingleIntegrator
        
        self.agent.reset(seed=seed, state=agent_state, state_goal=agent_goal_state)
        # print(self.np_random.bit_generator.state['state']['state'], self.agent.np_random.bit_generator.state['state']['state'])
        # print(self.np_random_seed, self.np_random, self.np_random.uniform(0,1,1))
        obs = self._get_obs()
        info = {"Sampled states": [self.agent.state, self.agent.state_goal]}
        # print("Final obs after reset:", obs)
        return obs, info

    # def step(self, action):

    #     self.agent.step(action)

    #     # Observation
    #     obs = self._get_obs()

    #     # Reward: negative distance to goal
    #     reward = self.reward_function(self.agent.state)

    #     # Termination
    #     terminated = self.agent.is_finished(atol=self.atol, rtol=self.rtol)
    #     truncated = False  # could set time limit

    #     info = {"agent state": self.agent.state, "reward": reward}
    #     return obs, reward, terminated, truncated, info
    
    def _get_obs(self, state=None):
        if state is None:
            state = self.agent.state
        goal = self.goal_obs
        # print("Getting obs. State:", state, "Goal:", goal)
        #### SingleIntegrator
        return np.array([state[0], state[1], state[2], goal[0], goal[1]], dtype=np.float32)
        #### DoubleIntegrator
        # return np.array([state[0], state[1], state[2], state[3], state[4], goal[0], goal[1]], dtype=np.float32)

    def _get_pos(self, state=None):
        observation = self._get_obs(state)
        # Observation is [x, y, t, goal_x, goal_y]
        return observation[0:2]

    def _get_t(self, state=None):
        observation = self._get_obs(state)
        # Observation is [x, y, t, goal_x, goal_y]
        return observation[2]
        return observation[4]

    # Reward function
    def reward_function(self, state):
      
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

        reward = 0.01 * rew_alive + 0.97 * rew_goal + rew_obs + 0.01 * rew_grad

        return reward



    def is_truncated(self, state=None) -> bool:
        def check_collision(state, obstacle):
            [x, y, s] = obstacle
            # print("Checking collision with obstacle at", obstacle, "for state", state)
            if self.obstacle_mode == "circles":
                if np.linalg.norm(state[:2] - [x, y]) < s:
                    # print("Collision with circle at", obstacle)
                    return True
            else:
                w, h = self.box_obstacle_width / 2, s / 2
                if np.abs(state[0] - x) < w + 0.001 and np.abs(state[1] - y) < h + 0.001:
                    return True
            return False

        # in_observation = True
        in_observation = self.observation_space.contains(self._get_obs(state))
        is_collision = bool(np.any([check_collision(state, obstacle) for obstacle in self.obstacles]))
        # print(f"Truncation check: in_observation={in_observation}, is_collision={is_collision}")
        
        return not in_observation or is_collision
    

    def render(self, render_mode=None, render_tree=True, planner: Optional["Planner"]=None, scatter_mode='Q'):
        if render_mode is None:
            render_mode = self.render_mode
        if render_mode == "human":
            render_mode = "matplotlib"
        if render_mode == "matplotlib":
            import matplotlib.pyplot as plt
            from IPython.display import display, clear_output
            
            if not hasattr(self, 'fig'):
                self.fig = plt.figure(figsize=(12, 12))
            else:
                self.fig.clf()

            self.fig = plt.figure(figsize=(6, 6))
            plt.figure(self.fig.number)   
            plt.gca().set_aspect('equal', adjustable='box')
            ax = plt.gca()
            clear_output(wait=True) 




            # Draw obstacles
            for obs in self.obstacles:
                [x, y, s] = obs
                if self.obstacle_mode == "circles":
                    circle = plt.Circle((x, y), s, color='red', alpha=0.5)
                    plt.gca().add_artist(circle)
                else:
                    w, h = self.box_obstacle_width / 2, s / 2
                    rectangle = plt.Rectangle((x - w, y - h), 2 * w, 2 * h, color='red', alpha=0.5)
                    plt.gca().add_artist(rectangle)

            # Draw planner
            if planner is not None:
                state_list = planner.planned_trajectory['state_list']
                if len(state_list) > 0:
                    plan_positions = np.empty((len(state_list), 2))
                    for i, state in enumerate(state_list):
                        plan_positions[i] = self._get_obs(state)[:2]
                    plt.gca().plot(plan_positions[:, 0], plan_positions[:, 1], color=self.agent.color, linestyle='-')
                    plt.gca().scatter(plan_positions[0, 0], plan_positions[0, 1], color=self.agent.color, s=30, alpha=0.9, zorder=90) # Root node
                    plt.gca().scatter(plan_positions[1:, 0], plan_positions[1:, 1], color=self.agent.color, s=15, alpha=0.9, zorder=90)                
                if render_tree and hasattr(planner, "graph"):
                    from tools.utils.plot_graph import plot_graph_2d
                    # print("Plot planner")
                    self.fig = plot_graph_2d(planner.graph, plot_labels=False, scatter_mode=scatter_mode, plot_values=False, bbox=self.pos_limits, fig=plt.gcf(), scatter_size=15, get_pos=self._get_pos, get_t=self._get_t, get_vel=None)
            
            # Draw agent
            radius = 0.05
            pos = self._get_pos()
            t = self._get_t()
            goal_pos = self._get_pos(state=self.agent.state_goal)
            color = self.agent.color
            name = self.agent.name
            # Draw goal as cross
            plt.gca().scatter(goal_pos[0], goal_pos[1], color=color, marker='x', s=200, label=f"{name} Goal")
            try:
                # print("Plot Circle")
                if self.agent.is_finished():
                    circle = plt.Circle(pos, radius*2, facecolor=color, edgecolor='black', alpha=1.0, zorder=100, label=f"{name} Robot")
                else:
                    circle = plt.Circle(pos, radius, facecolor=color, edgecolor='black', alpha=0.5, zorder=100, label=f"{name} Robot")

            except Exception as e:
                # print("Error plotting circle:", e)
                circle = plt.Circle(pos, radius, facecolor=color, edgecolor='black', alpha=0.5, label=f"{name} Robot")
            # Check if velocity is available and non-zero
            if hasattr(self, '_get_vel'):
                if not np.allclose(self._get_vel(state=self.agent.state), np.array([0.0, 0.0])):
                        # draw current velocity as arrow
                        vel = self._get_vel(state=self.agent.state)
                        ax.quiver(pos[0], pos[1], vel[0], vel[1], color='green', scale=20, alpha=0.5, width=0.008, headwidth=2, headlength=2, headaxislength=2)
            # Check if orientation is available and non-zero
            if hasattr(self, '_get_orientation'):
                # draw current orientation as arrow
                orientation = self._get_orientation(state=self.agent.state)
                ax.quiver(pos[0], pos[1], np.cos(orientation), np.sin(orientation), color='black', scale=25, alpha=1.0, width=0.003, headwidth=5, headlength=5, headaxislength=3)
        
            plt.gca().add_artist(circle)
            # plt.gca().add_artist(plt.Text(0.02, 0.95, f"Time: {t:.2f}s", transform=ax.transAxes))

            # plt.gca().plot(pos[0], pos[1], marker='o', color='black', markersize=5)
            plt.xlim([self.pos_limits[0]-0.2, self.pos_limits[1]+0.2])
            plt.ylim([self.pos_limits[2], self.pos_limits[3]])
            # plt.title('2D Navigation Environment')
            # plt.xlabel('X Position')
            # plt.ylabel('Y Position')
            # plt.legend(loc='lower right')
            plt.xticks([])
            plt.yticks([])
            plt.grid()
            plt.tight_layout()
            # plt.show()
            display(self.fig)
            if self.render_sleep > 0:
                plt.pause(self.render_sleep)  # pause to update the plot

        if render_mode == "ansi":
            return f"Agent state: {self.agent.state}, Goal: {self.agent.state_goal}"