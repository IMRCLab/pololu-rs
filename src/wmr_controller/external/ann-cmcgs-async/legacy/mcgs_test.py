# %%
from __future__ import annotations

import numpy as np
import matplotlib.pyplot as plt
import logging

from tools.robots.robot import Robot
from tools.robots.double_integrator import DoubleIntegrator
from tools.robots.single_integrator import SingleIntegrator

from tools.planners.mcgs import MCGSPlanner, NodePayload, EdgePayload
from tools.envs.frozen_lake_continuous import FrozenLakeEnv

from tools.utils.plot_graph import plot_graph_2d, plot_spacetime_graph_3d, plot_graph_2d_live

import rustworkx as rx
from rustworkx.visualization import mpl_draw
# from rustworkx import Pos2DMapping



# %%
# only from mcgs.py
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')


# %%
def reward_function(state):
    # Example reward function: negative distance to the goal position
    goal_position = 10*np.array([1.0, 1.0])  # Define goal position
    return 0 - 1*state['time'] - np.linalg.norm(state['position'] - goal_position, ord=2)

# def reward_function(state):
#     # Example reward function: negative distance to the goal position
#     goal_position = 10*np.array([1.0, 1.0])  # Define goal position
#     return - np.linalg.norm(state['position'] - goal_position, ord=2)




# %%
max_velocity = 1.0
rob = SingleIntegrator(id=0, 
                       name="SingleIntegrator", 
                       dt=1.0, 
                       max_velocity=max_velocity,
                       state_goal={'position': 10*np.array([1.0, 1.0]), 'time': np.array(20.0)},
                       action_sampler_method='directed_random',)

root_state = {'position': np.array([0.0, 0.0]), 'time': np.array(0.0)}

rob.state = root_state
print(rob)

root_encoded_state = rob.encode_state(root_state)
print("Encoded root state:", root_encoded_state)
print("Encoded goal state:", rob.encode_state(rob.state_goal))


# %%
mcgs_planner = MCGSPlanner(robot=rob, 
                        reward_function=reward_function,
                        max_iterations=100, 
                        kappa=1.6, 
                        alpha=1.0,
                        k=8,
                        radius_threshold=2,
                        c_uct=0.8)

# %%
mcgs_planner.reset_graph()
mcgs_planner.plan(iterations=250)


# %%
plot_graph_2d(mcgs_planner.graph, plot_labels=True, scatter_mode='Q', plot_values=False)
plot_spacetime_graph_3d(mcgs_planner.graph, mcgs_planner.robot, plot_labels=True, plot_time=True)

# %%
atol = 0.0
rtol = .2

plan = mcgs_planner.yield_plan(atol=atol, rtol=rtol)
print(plan)
print(mcgs_planner.graph[plan[-1]].state)
print("Finished: ", rob.is_finished(atol=atol, rtol=rtol, state=mcgs_planner.graph[plan[-1]].state))

# %%
mcgs_planner.reset_graph()


# %%
def step(action: EdgePayload):
    current_state = mcgs_planner.robot.state
    mcgs_planner.robot.step(action.action + 0.2 *np.random.normal(0, 1, size=action.action.shape))
    new_state = mcgs_planner.robot.state
    print("Current state:", current_state)
    print("New state:", new_state)

# %%
mcgs_planner.reset_graph()


atol = 0.0
rtol = .2
for i in range(10):
    print("Iteration:", i)
    mcgs_planner.plan(iterations=10)
    plan = mcgs_planner.yield_plan(atol=atol, rtol=rtol)
    print(plan)
    assert len(plan) > 1, "Plan does not include successor node!"
    optimal_action = mcgs_planner.graph.get_edge_data(mcgs_planner.root_node_id, plan[1])
    print("Optimal action:", optimal_action.action, " from ", mcgs_planner.root_node_id, " to ", plan[1], " at ", mcgs_planner.graph[plan[1]].state)
    step(optimal_action)
    mcgs_planner.update_planner()
    print("New node:", mcgs_planner.root_node_id, "with successors: ", mcgs_planner.graph.successor_indices(mcgs_planner.root_node_id))
    mcgs_planner.garbage_collector.collect_garbage(timestamp_threshold=mcgs_planner.robot.state['time'], root_node_id=mcgs_planner.root_node_id)
    plot_graph_2d(mcgs_planner.graph, plot_labels=True, scatter_mode='Q', plot_values=False)


# %%



