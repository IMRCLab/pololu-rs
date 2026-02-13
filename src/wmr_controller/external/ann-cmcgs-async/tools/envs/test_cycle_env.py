import numpy as np
import gymnasium as gym
from typing import Optional

import tools.robots as robots
import tools.envs as envs
import tools.planners as planners



class TestCycleEnv(envs.GymRobotPlanEnv):
    def __init__(self, dt=1.0, render_mode=None, atol=0.0, rtol=0.1, multi_step_count: int = 1, render_sleep=0.0, default_step_divisor: int = 10):
        print("---- Starting Initialization of NavigationEnvSingleIntegratorUnicycleSpace")
        self.dt = dt

        self.atol = atol
        self.rtol = rtol

        self.render_sleep = render_sleep
        ##### SingleIntegratorUnicycleSpace

        self.max_velocity = 1.0  # max linear velocity
        self.max_angular = np.pi   # max angular velocity

        self._max_steps = 5

        self.start_pos = np.zeros(2)
        self.start_theta = np.pi/2  # Facing upwards
        self.start_t = 0.0
        self.goal_pos = np.array([self._max_steps, 0.0])  # Fixed goal position for testing
        # State: [x, y, theta, time], Goal: [goal_x, goal_y]
        self.pos_limits = np.array([-self._max_steps-0.25, self._max_steps+0.25, -1.5, 1.5])  # [min_x, max_x, min_y, max_y]
        low_obs = np.array([self.pos_limits[0], self.pos_limits[2], -np.pi, self.start_t], dtype=np.float32)  # [x, y, theta, time]
        high_obs = np.array([self.pos_limits[1], self.pos_limits[3], np.pi, np.inf], dtype=np.float32)  # [x, y, theta, time]

        robot_state_space = gym.spaces.Box(low=low_obs[:4], high=high_obs[:4], dtype=np.float32)  # [x, y, theta, time]
        
        # Action space: [linear_velocity, angular_velocity]
        action_space = gym.spaces.Box(low=np.array([0.0, -self.max_angular]), 
                                      high=np.array([self.max_velocity, self.max_angular]), dtype=np.float32)  # [v, omega]

        self.agent = robots.SingleIntegratorUnicycleSpace(
            id=0,
            name="2D Navigation Unicycle Robot in Space",
            state_space=robot_state_space,
            action_space=action_space,
            dt=self.dt,
            max_velocity=self.max_velocity,
            max_angular=self.max_angular,
            action_sampler_method='bang_bang',
            color="#00FF00",
            atol=self.atol,
            rtol=self.rtol,
            default_step_divisor=default_step_divisor
        )

        self.observation_space = gym.spaces.Box(low=low_obs, high=high_obs, dtype=np.float32) # Environment observation space

        super().__init__(agent=self.agent,                 
                         render_mode=render_mode, 
                         multi_step_count=multi_step_count, 
                         observations_space=self.observation_space)
        
        self.metadata = {"render_modes": ["human", "matplotlib", "ansi"],
                    "render_fps": np.ceil(1 / self.dt)}
        

        print("---- Finished Initialization of NavigationEnvSingleIntegratorUnicycle")

    def reset_rng(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        if seed is None:
            seed = np.random.randint(0, 10000)
            
        super().reset(seed=seed, options=options)
        if seed is not None:
            self.observation_space.seed(seed)

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        self.reset_rng(seed=seed, options=options)

        #### SingleIntegrator
        # Reset robot + assign new random goal
        self.start_obs = self.observation_space.sample() # Placeholder for start observation

        self.start_obs[:2] = self.start_pos
        self.start_obs[2] = self.start_theta  # [x, y, theta, t]
        self.start_obs[3] = self.start_t

        # self.start_obs = self.observation_space.sample()
        # print("Sampled start_obs:", self.start_obs)
        gx = self.goal_pos[0]
        gy = self.goal_pos[1]
        theta = np.nan
        t = np.nan # allow any time
        self.goal_obs = np.array([gx, gy, theta, t], dtype=np.float32)

        agent_state = self.start_obs[:4]  # [x,y,theta,t]
        agent_goal_state = self.goal_obs[:4]  # [gx,gy,theta,t]
        #### SingleIntegrator
        
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
        obs = np.array(state, dtype=np.float32)
        return obs
    
    def _get_pos(self, state=None):
        if state is None:
            state = self.agent.state
        return state[:2]

    def _get_orientation(self, state=None):
        if state is None:
            state = self.agent.state
        return state[2]

    def _get_t(self, state=None):
        if state is None:
            state = self.agent.state
        return state[3]

    # Reward function
    def reward_function(self, state):
 
        # collision
        rew_obs = -1 * self.is_truncated(state)
        rew_alive = 0 if self.is_truncated(state) else 1

        # finished
        rew_goal = 1 * self.agent.is_finished(state, atol=self.atol, rtol=self.rtol)

        # gradient to goal (position only, ignore orientation)
        rew_grad = -1 * np.linalg.norm(self._get_pos(state) - self.goal_pos)**2

        # Optional: Small penalty for large angular changes (encourage smooth motion)
        # rew_smooth = -0.001 * abs(self._get_orientation(state))  # Penalize large angles from 0

        reward = 0.01 * rew_alive + 0.97 * rew_goal + rew_obs + 0.00 * rew_grad

        return reward
    
    def is_truncated(self, state=None):
        if state is None:
            state = self.agent.state
        # Check if out of bounds
        obs = self._get_obs(state)
        out_of_bounds = not self.observation_space.contains(obs)
        return out_of_bounds

    def render(self, render_mode=None, render_tree=True, planner: Optional[planners.Planner] = None, scatter_mode='Q'):
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

            self.fig = plt.figure(figsize=(12, 12))
            plt.figure(self.fig.number)   
            plt.gca().set_aspect('equal', adjustable='box')
            ax = plt.gca()
            clear_output(wait=True) 

            # Draw planner
            if planner is not None:
                state_list = planner.planned_trajectory['state_list']
                if len(state_list) > 0:
                    plan_positions = np.empty((len(state_list), 2))
                    for i, state in enumerate(state_list):
                        plan_positions[i] = self._get_obs(state)[:2]
                    plt.gca().plot(plan_positions[:, 0], plan_positions[:, 1], color=self.agent.color, linestyle='-', alpha=.8, linewidth=1)
                    # plt.gca().scatter(plan_positions[:, 0], plan_positions[:, 1], color=self.agent.color, s=15, alpha=0.9, zorder=90)                
                if render_tree:
                    if isinstance(planner, planners.MCGSPlanner):
                        from tools.utils.plot_graph import plot_graph_2d
                        # print("Plot planner")
                        self.fig = plot_graph_2d(planner.graph, plot_labels=True, scatter_mode=scatter_mode, plot_values=False, bbox=self.pos_limits, fig=plt.gcf(), scatter_size=15, get_pos=self._get_pos, get_t=self._get_t, get_vel=None)
            
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
            plt.gca().add_artist(plt.Text(0.02, 0.95, f"Time: {t:.2f}s", transform=ax.transAxes))

            # plt.gca().plot(pos[0], pos[1], marker='o', color='black', markersize=5)
            plt.xlim([self.pos_limits[0]-0.2, self.pos_limits[1]+0.2])
            plt.ylim([self.pos_limits[2], self.pos_limits[3]])
            plt.title('2D Nav Environment')
            plt.xlabel('X Position')
            plt.ylabel('Y Position')
            plt.legend()
            plt.grid()
            # plt.show()
            display(self.fig)
            if self.render_sleep > 0:
                plt.pause(self.render_sleep)  # pause to update the plot

        if render_mode == "ansi":
            return f"Agent state: {self.agent.state}, Goal: {self.agent.state_goal}"