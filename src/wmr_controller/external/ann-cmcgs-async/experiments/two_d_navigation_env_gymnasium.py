import time
import numpy as np
from math import cos, sin
from matplotlib import pyplot as plt
from matplotlib import rc

import gymnasium as gym
from gymnasium import spaces
import pygame

from matplotlib.patches import Circle, Rectangle
from matplotlib.lines import Line2D
from IPython.display import clear_output


from typing import Optional


class TwoDimNavigationEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array']
        , 'video.frames_per_second': 75
    }

    def __init__(self, obstacle_mode="circles", render_mode="human"):
        self.obstacle_mode = obstacle_mode
        self.render_mode = render_mode  # "human" or "rgb_array"

        self._max_episode_steps = 10
        self.action_repeat = 50
        self.block_size = 100
        self.block_size_matplotlib = 2
        self.player_size = 0.05
        self.goal_size = 0.1
        self.box_obstacle_width = 0.15
        self.max_vel = 1.25

        if obstacle_mode == "circles":
            self.obstacles = [
                [3, 2.5, 1]
                , [4, 4, 0.3]
                , [6, 2.25, 0.4]
                , [8, 5, 1.75]
                , [8.5, 1.5, 0.7]
            ]
        else:
            self.obstacles = [
                [2, 1.5, 1]
                , [2, 3, 1]
                , [2, 4.5, 1]
                , [3, 1.6, 1.2]
                , [3, 3, 0.9]
                , [3, 4.4, 1.2]
                , [4, 1.5, 1]
                , [4, 4.5, 1]
                , [5, 3, 3.5]
                , [6, 1.25, 0.5]
                , [6, 3, 1.5]
                , [6, 4.75, 0.5]
                , [7, 2, 2]
                , [7, 4.3, 1.4]
                , [8, 1.5, 1]
                , [8, 3.25, 1]
                , [8, 4.5, 1]
                , [9, 1.5, 1]
                , [9, 4, 2]
            ]
            if True:
                self.obstacles = [
                    [2, 3, 2.25]
                    #, [3, 1.6, 1.2]
                    #, [3, 3, 0.9]
                    #, [3, 4.4, 1.2]
                    , [4, 1.5, 1]
                    , [4, 4.5, 1]
                    , [5, 1.5, 1]
                    , [5, 3, 1]
                    , [5, 4.5, 1]
                    , [6, 1.25, 0.5]
                    , [6, 3, 1.5]
                    , [6, 4.75, 0.5]
                    , [7, 2, 2]
                    , [7, 4.3, 1.4]
                    , [8, 1.5, 1]
                    , [8, 3.25, 1]
                    , [8, 4.5, 1]
                    , [9, 1.5, 1]
                    , [9, 4, 2]
                ]
        self.goal_pos = [self._max_episode_steps, 4]

        self.t = 0
        self.pos = 0  # [-1, 1]
        self.vel = 0  # [-1, 1]
        self.acc = 0  # [-1, 1]  # This is used for rendering purposes only.
        high = np.array([1, 1, 1])
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        high = np.array([1])
        self.action_space = spaces.Box(-high, high, dtype=np.float32)

        # self.spec['id'] = f'2d-navigation-{obstacle_mode}'

        self.reset()

    @property
    def player_pos(self):
        return [(self.t + 1) * self.block_size, (3 + 2 * self.pos) * self.block_size]

    @property
    def dt(self):
        return 0.2  # This can be set "almost" arbitrarily.

    def step(self, action, render=False):
        info = {'render': []}

        self.acc = action[0]
        acc = np.clip(action[0], -1, 1)

        rew_ac = np.exp(-10 * np.power(acc, 2))  # Encourage lower velocity
        rew_goal = 0
        rew_obs = 0
        rew_alive = 1

        dt = 1 / self.action_repeat
        terminated = False
        truncated = False

        for s in range(self.action_repeat):
            # Update velocity and then the position
            self.vel += acc * dt
            self.vel = np.clip(self.vel, -1, 1)
            self.pos += self.vel * self.max_vel * dt
            if self.pos >= 1 and self.vel > 0:
                self.vel = 0
            if self.pos <= -1 and self.vel < 0:
                self.vel = 0
            self.pos = np.clip(self.pos, -1, 1)
            self.t += dt
            if render:
                frame = self.render('rgb_array')
                info['render'].append(frame)
            # Check collision with obstacles
            pos = np.array(self.player_pos)
            for [x, y, s] in self.obstacles:
                obs_pos = np.array([x * self.block_size, y * self.block_size])
                if self.obstacle_mode == "circles":
                    if np.linalg.norm(pos - obs_pos) < self.block_size * (s + self.player_size):
                        rew_obs = -1
                        rew_alive = 0
                        truncated = True
                        break
                else:
                    w, h = self.box_obstacle_width * self.block_size / 2, s * self.block_size / 2
                    if np.abs(pos[0] - obs_pos[0]) < w + 0.001 and np.abs(pos[1] - obs_pos[1]) < h + 0.001:
                        rew_obs = -1
                        rew_alive = 0
                        truncated = True
                        break
            '''
            for [x, y, s] in self.obstacles:
                
                obs = rendering.make_polygon([(l, b), (l, t), (r, t), (r, b)])
            '''
            # Reached the goal?
            if not truncated and np.linalg.norm(pos - self.block_size * np.array(self.goal_pos)) < self.block_size * (self.goal_size + self.player_size):
                rew_goal = 1
                terminated = True
            if terminated:
                break
        if not terminated:
            terminated = (self.t + dt) >= (self._max_episode_steps - 1)
        reward = 0.01 * rew_alive + 0.02 * rew_ac + 0.97 * rew_goal + rew_obs
        return self._get_obs(), reward, terminated, truncated, info

    def reset(self, seed: Optional[int] = None, options: Optional[dict] = None) -> tuple[np.ndarray, dict]:        
        super().reset(seed=seed, options=options)
        self.t = 0
        self.pos = 0
        self.vel = 0
        self.acc = 0
        return self._get_obs(), {}

    def _get_obs(self):
        t = self.t / (self._max_episode_steps - 1) * 2 - 1  # Map self.t to [-1, 1]
        return np.array([t, self.pos, self.vel], dtype=np.float32)

    def get_state(self):
        return self._get_obs()

    def set_state(self, state):
        self.t = np.rint((state[0] + 1) / 2 * (self._max_episode_steps - 1))
        self.pos = state[1]
        self.vel = state[2]

    def render(self, render_mode=None):
        clear_output(wait=True)

        if render_mode is None:
            render_mode = self.render_mode

        screen_width = self.block_size * (self._max_episode_steps + 1)
        screen_height = self.block_size * 6

        # Create a figure offscreen (not stored on self)
        fig, ax = plt.subplots(figsize=(screen_width/100, screen_height/100))
        ax.set_xlim(0, screen_width)
        ax.set_ylim(0, screen_height)
        ax.set_aspect("equal")
        ax.axis("off")

        # --- Grid lines ---
        for s in range(self._max_episode_steps):
            x = (s + 1) * self.block_size
            ax.add_line(Line2D([x, x], [self.block_size, screen_height - self.block_size], color='silver'))
        for s in range(5):
            y = (s + 1) * self.block_size
            ax.add_line(Line2D([self.block_size, screen_width - self.block_size], [y, y], color='silver'))

        # --- Goal ---
        goal = Circle((self.goal_pos[0]*self.block_size, self.goal_pos[1]*self.block_size),
                    radius=self.goal_size*self.block_size, color='green')
        ax.add_patch(goal)

        # --- Obstacles ---
        for x, y, s in self.obstacles:
            if self.obstacle_mode == "circles":
                obs = Circle((x*self.block_size, y*self.block_size), radius=s*self.block_size, color='dimgray')
            else:
                w, h = self.box_obstacle_width*self.block_size, s*self.block_size
                obs = Rectangle((x*self.block_size - w/2, y*self.block_size - h/2), w, h, color='dimgray')
            ax.add_patch(obs)
        # --- Player ---
        player = Circle((self.player_pos[0], self.player_pos[1]), radius=self.player_size*self.block_size, color='blue')
        ax.add_patch(player)

        # --- Acceleration arrow ---
        unit = self.block_size * 0.5
        player_pos = np.array([self.player_pos[0], self.player_pos[1]])
        end_pos = player_pos + np.array([1, self.acc])*unit
        ax.add_line(Line2D([player_pos[0], end_pos[0]], [player_pos[1], end_pos[1]], color='black'))

        diff = 0.3 * (player_pos - end_pos)
        for theta in [30, -30]:
            rad = np.deg2rad(theta)
            rot = np.array([[np.cos(rad), -np.sin(rad)], [np.sin(rad), np.cos(rad)]])
            edge = np.dot(rot, diff)
            ax.add_line(Line2D([end_pos[0], end_pos[0]+edge[0]], [end_pos[1], end_pos[1]+edge[1]], color='black'))

        # --- Convert figure to RGB array ---
        fig.canvas.draw()
        # img = np.frombuffer(fig.canvas.print_rgb(), dtype=np.uint8)
        # img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        
        # Get RGBA buffer as numpy array
        rgba_array = np.array(fig.canvas.renderer.buffer_rgba())

        # Drop alpha channel to get RGB
        img = rgba_array[:, :, :3]
        plt.close(fig)

        if render_mode == "human":
            plt.imshow(img)
            plt.axis("off")
            plt.pause(0.001)

        return img
    
    def plot(self, fig=None, ax=None):
        n_steps = self._max_episode_steps

        if fig is None and ax is None:
            screen_width = self.block_size_matplotlib * (n_steps + 1)
            screen_height = self.block_size_matplotlib * (5 + 1)

            rc('figure', figsize=(screen_width, screen_height))
            fig, axs = plt.subplots(1, 1)
            # [axi.set_axis_off() for axi in axs.ravel()]
            ax = axs

        ax.set_xlim([-0.75, n_steps - 0.25])
        ax.set_ylim([0.25, 5.75])
        ax.set_xticks([])
        ax.set_yticks([])
        # ax.set_xticks(np.arange(n_steps))
        # ax.set_yticks(np.arange(5) + 1)

        # Vertical lines
        for s in range(n_steps):
            ax.plot([s, s], [1, 5], 'silver')

        # Horizontal lines
        for s in range(5):
            ax.plot([0, n_steps - 1], [s + 1, s + 1], 'silver')

        # Goal
        ax.add_patch(plt.Circle((self.goal_pos[0] - 1, self.goal_pos[1]), self.goal_size, color='g', zorder=2))

        # Obstacles
        if self.obstacle_mode == "circles":
            for [x, y, r] in self.obstacles:
                ax.add_patch(plt.Circle((x - 1, y), r, color='dimgray', zorder=2))
        else:
            w = self.box_obstacle_width
            for [x, y, h] in self.obstacles:
                ax.add_patch(plt.Rectangle((x - w / 2 - 1, y - h / 2), w, h, color='dimgray', zorder=2))

        # Player
        player_pos = (self.t, 3 + 2 * self.pos)
        ax.add_patch(plt.Circle(player_pos, self.player_size, color='royalblue', zorder=2))
        # v = np.array([1, self.vel])
        # v = v / np.linalg.norm(v) * 0.25
        # ax.add_patch(plt.arrow(player_pos[0], player_pos[1], v[0], v[1], width=0.025, color='royalblue', zorder=2))
        return fig, ax

    def plot_visited_states(self, name, trajectories, step=0, next_state=None, rollout_indices=None):
        fig, ax = self.plot()
        n_trajs = len(trajectories)
        obstacles = np.asarray(self.obstacles)

        for i, traj in enumerate(trajectories):
            # Drop the velocity, keep t (i.e. x-pos and y-pos)
            if rollout_indices is not None:
                states = traj[rollout_indices[i]-1:, :-1]
                Xs = (states[:, 0] + 1) * 5 * 9/10
                Ys = 3 + 2*states[:, 1]
                plt.plot(Xs, Ys, 'ro', linestyle="--")
                plt.scatter(Xs, Ys, c="red")

            states = traj[:, :-1]
            if rollout_indices is not None:
                states = states[:rollout_indices[i]]    

            Xs = (states[:, 0] + 1) * 5 * 9/10
            Ys = 3 + 2*states[:, 1]
            
            plt.scatter(Xs, Ys)
            plt.plot(Xs, Ys, 'bo', linestyle="--")
       
        if next_state is not None:
            X = [(next_state[0] + 1) * 5 * 9/10]
            Y = [3 + 2*next_state[1]]
            plt.scatter(X, Y, c='purple')
            Xs = [step, X[0]]
            Ys = [3+2*self.pos, Y[0]]
            plt.plot(Xs, Ys, 'kD', linestyle="-")

        # Obstacles
        if self.obstacle_mode == "circles":
            for [x, y, r] in self.obstacles:
                ax.add_patch(plt.Circle((x - 1, y), r+0.01, color='dimgray', zorder=2))
        else:
            w = self.box_obstacle_width
            for [x, y, h] in self.obstacles:
                ax.add_patch(plt.Rectangle((x - w / 2 - 1, y - h / 2), w, h, color='dimgray', zorder=2))
        plt.xticks([], [])
        plt.yticks([], [])
        plt.show()

    def close(self):
        if hasattr(self, "_pygame_init") and self._pygame_init:
            pygame.display.quit()
            pygame.quit()
            self._pygame_init = False


if __name__ == '__main__':
    env = TwoDimNavigationEnv()
    env.reset()
    for _ in range(10):
        action = env.action_space.sample()  # your agent here (this takes random actions)
        observation, reward, terminated, truncated, info = env.step(action, render=True)
        if terminated or truncated:
            env.reset()
    env.close()
