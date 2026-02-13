import numpy as np
from typing import Callable, Optional
from time import perf_counter

from types import SimpleNamespace

from tools.planners.planner import Planner
from tools.robots.agent import Agent
from tools.envs.gym_robot_plan_env import GymRobotPlanEnv

from baselines.cmcgs.DMC_and_2D.agents import CMCGS
from tools.compatibility_wrappers.base_wrapper import EnvWrapper
from tools.compatibility_wrappers.action_repeat import ActionRepeatWrapper
from tools.compatibility_wrappers.simulator import SimulatorWrapper



class CMCGSWrapperPlanner(Planner):
    def __init__(self, env: GymRobotPlanEnv, computational_budget_max: int, time_budget_max: float, cmcgs_config, save_replay: bool = True):
        super().__init__(env=env, computational_budget_max=computational_budget_max, time_budget_max=time_budget_max, save_replay=save_replay)
        """Initialize the planner."""
        self.env = env

        self.planned_trajectory = []

        model = SimulatorWrapper(EnvWrapper(env), compatibility_mode=True)

        # Rename computational_budget_max to simulation_budget for CMCGS
        cmcgs_config['simulation_budget'] = computational_budget_max

        cmcgs_config = SimpleNamespace(**cmcgs_config)
        
        ### Planner class
        self.base_planner = CMCGS(cmcgs_config, model)
            # N,
            #                       min_action,
            #                         max_action,
            #                         action_size,
            #                         min_graph_length,
            #                         max_graph_length,
            #                         rollout_length,
            #                         simulation_budget,
            #                         clustering_alg,
            #                         optimal_prob,
            #                         optimal_n_top,
            #                         optimal_range,
            #                         elite_ratio,
            #                         state_dim,
            #                         greedy_action=False,
            #                         planet=True,
            #                         max_n_exps=500,
            #                         fixed_init_stddev=False,
            #                         max_n_clusters=np.inf,
            #                         clustering_linkage='ward',
            #                         alpha=5,
            #                         beta=2,
            #                         )



    def reset(self, seed: Optional[int] = None):
        """Reset the planner and the environment."""
        super().reset(seed=seed)
        self.base_planner.reset()


    def plan_best_action(self, computational: bool = True, time: bool = False):
        time_start = perf_counter()
        action = self.base_planner.act(self.env._get_obs())
        self.time_budget_used += perf_counter() - time_start
        return action


    def render(self, render_mode: Optional[str] = None):
        self.env.render(render_mode=render_mode)