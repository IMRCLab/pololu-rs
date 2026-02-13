# %%
import gymnasium as gym
import numpy as np
from typing import Optional, Tuple, Any
import sys
sys.path.append("..")

import tools.planners as planners

from time import sleep

#
from gymnasium.utils.env_checker import check_env

import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# %%
gym.register(
    id='NavigationEnvSingleIntegrator-v0',
    entry_point='tools.envs.navigation_envs.navigation_env_single_integrator:NavigationEnvSingleIntegrator',)

gym.register(
    id='NavigationEnvPololu-v0',
    entry_point='tools.envs.navigation_envs.navigation_env_pololu:NavigationEnvPololu',)

# %%
# env = gym.make('NavigationEnvSingleIntegrator-v0', 
#                render_mode="human",
#                dt=1.0,
#                atol=0.3,
#                rtol=0.0,
#                multi_step_count=5,
#                plan_in_space_time=False,
#                sparse_reward=True)
# env.reset(seed=42)


# %%
env = gym.make('NavigationEnvPololu-v0', 
               render_mode="human",
               dt=1.0,
               atol=0.3,
               rtol=0.0,
               multi_step_count=10,
               obstacle_mode="circles",
               render_sleep=0.1,
               sparse_reward=True,
               )
env.reset(seed=42)


# %%
# Random movements
env.reset()
for _ in range(20):
    action = env.action_space.sample()
    observation, reward, done, truncated, info = env.step(action)
    # env.render()
    sleep(0.1)
    if done or truncated:
        observation = env.reset()

# %%
computational_budget_max = 500
time_budget_max = 0.2  # seconds

# %%
# Pololu Planner Configuration
planner = planners.MCGSPlanner(env=env.unwrapped,
                               budget_per_step=1,
                               computational_budget_max=computational_budget_max,
                               time_budget_max=time_budget_max,
                               expand_n_times=1,
                               expand_n_times_min=1,
                               expand_n_times_max=1,
                               sample_best_X_actions=5,
                               kappa=0.5,
                               alpha=0.2,
                               k=30,
                               radius_threshold=1.0,
                               progressive_widening_method="dispersion",
                               epsilon=0.1,
                               abstraction_refinement_exponent=-0.0,
                               c_uct=0.5,
                               plan_in_space_time=False,
                               use_controller=True,
                               tracking_tolerance=0.5,
                               yield_mode='N',
                               random_rollout_n_times=1,
                               random_rollout_length=4,
                               force_exact_update=False,
                               controller_expansion_check=False)



# %%
episode_over = False
total_reward = 0

planner.reset()
try:
    # planner.plan(iterations=20, expand_n_times=10) # Bootstrapping
    pass
except Exception as e:
    print("Exception during bootstrapping planning:")
    print(e)
i = 0
while not episode_over:
    print(f"--- Episode step {i} ---")

    # planner.plan(iterations=250, expand_n_times=1)
    print("Agent state:", planner.env.agent.state)
    observation, reward, terminated, truncated, info = planner.plan_online(render=False, time=True, computational=False, use_hardware=False)
    print("Agent state after step:", planner.env.agent.state)
    # sleep(1.0) 
    # planner.render()
    print('root node id', planner.root_node_id)
    try:
        # get_stats()
        plan = planner.planned_trajectory['node_ids']
        plan_action_list = planner.planned_trajectory['action_list']
        plan_dt_list = planner.planned_trajectory['dt_list']
        print(f"Episode step {i}: current state: {planner.env.agent.state}, plan: {plan}, actions: {plan_action_list}, dts: {plan_dt_list}")
        print(f"Computational budget used: {planner.computational_budget_used}/{planner.computational_budget_total}")
        print(f"Time budget used: {planner.time_budget_used:.4f}/{planner.time_budget_total:.4f}")
        print(f"Episode step {i}: took action {plan_action_list[0]} to {observation}, got reward {reward}, terminated={terminated}, truncated={truncated}")
    except Exception as e:
        print(f"Episode step {i}: current state: {planner.env.agent.state}, no plan found!")
        print(e)

    total_reward += reward
    episode_over = terminated or truncated
    i += 1
    # if i >0:
    #     break
    sleep(.01)
    planner.render()
print(f"Episode finished! Total reward: {total_reward} after {i} iterations with computational budget used {planner.computational_budget_used}/{planner.computational_budget_total}")

env.close()
