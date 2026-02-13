from __future__ import annotations

import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import gymnasium as gym


# Add parent directory to sys.path
sys.path.append(os.path.abspath(os.path.join(os.getcwd(), '..')))

from tools.robots.robot import Robot
from tools.robots.double_integrator import DoubleIntegrator
from tools.robots.single_integrator import SingleIntegrator

from tools.planners.mcgs import MCGSPlanner, NodePayload, EdgePayload
from tools.envs.frozen_lake_continuous import FrozenLakeEnv

from tools.utils.plot_graph import plot_graph_2d, plot_spacetime_graph_3d

import rustworkx as rx
from rustworkx.visualization import mpl_draw

import unittest
import logging

logger = logging.getLogger(__name__)

class TestPathFinding(unittest.TestCase):

    def setUp(self):
        print("SETUP")
        # Register the environment
        try:
            gym.register(
            id="gymnasium_env/FrozenLakeContinuous-v0",
            entry_point="tools.envs.frozen_lake_continuous_wrapper:FrozenLakeSingleRobotEnv",
            )
        except Exception as e:
            print(f"Setup failed: {e}")
        pass

        self.env = gym.make("gymnasium_env/FrozenLakeContinuous-v0", 
            render_mode="human",
            size=20,
            n_puddles=0,
            n_obstacles=0,
            dt=1.0,
            max_velocity=1.0,
            slippery_everywhere=0.0,
            atol=0.0,
            rtol=0.2)
        
        self.env.reset()
        

                
        

    def test_mcts(self, atol=0.0, rtol=.2):
        """Tests MCGS in MCTS mode"""
        # Test case for finding the goal
        print("TEST MCTS")
        # Reset environment
        self.env.reset(seed=10)
        # Setup

        self.mcts_planner = MCGSPlanner(robot=self.env.unwrapped.robot,
                    reward_function=self.env.unwrapped.reward_function,
                    max_iterations=1000, 
                    expand_n_times=1,
                    kappa=.5, 
                    alpha=.9,
                    k=0,
                    radius_threshold=0.0,
                    abstraction_refinement_exponent=-0.1,
                    c_uct=0.4,
                    plan_in_space_time=False)

        self.env.unwrapped.connect_planner(self.mcts_planner)

        # Run planner
        self.mcts_planner.plan()

        plan, plan_actions = self.mcts_planner.yield_plan(atol=atol, rtol=rtol)

        print("Plan:", plan)
        print("States:", [self.mcts_planner.graph[node_id].state for node_id in plan])

        assert(len(plan) > 0), "Plan does not include successor node"

        self.mcts_planner.update_trajectory_plan(plan)

        final_state = self.mcts_planner.graph[plan[-1]].state
        self.assertTrue(self.env.unwrapped.robot.is_finished(state=final_state), f"Final state {final_state} does not meet the tolerance condition.")

        for action in plan_actions:
            observation, reward, terminated, truncated, info = self.env.step(action)

        # Check if the robot is close to the goal
        self.assertTrue(terminated, f"Robot is not close to the goal {self.mcts_planner.graph[plan[-1]].state}")

    def test_mcgs(self, atol=0.0, rtol=.2):
        """Tests if MCGS finds the goal"""
        print("TEST MCGS")
        # Reset environment
        self.env.reset(seed=10)

        self.mcgs_planner = MCGSPlanner(robot=self.env.unwrapped.robot,
                    reward_function=self.env.unwrapped.reward_function,
                    max_iterations=500,
                    expand_n_times=1,
                    kappa=1.6, # 1.6
                    alpha=0.3, # 0.3
                    k=8,
                    radius_threshold=1.3, # 1.8,
                    abstraction_refinement_exponent=-0.1,
                    c_uct=0.4,
                    plan_in_space_time=False)

        self.env.unwrapped.connect_planner(self.mcgs_planner)

        # Run planner
        self.mcgs_planner.plan()

        plan, plan_actions = self.mcgs_planner.yield_plan(atol=atol, rtol=rtol)
        print("Plan:", plan)
        print("States:", [self.mcgs_planner.graph[node_id].state for node_id in plan])

        self.assertTrue(len(plan) > 0), "Plan does not include successor node"
        self.assertTrue(len(plan_actions) == len(plan) -1, "Number of actions does not match number of nodes in the plan (edges = nodes - 1)")

        final_state = self.mcgs_planner.graph[plan[-1]].state
        self.assertTrue(self.env.unwrapped.robot.is_finished(state=final_state), f"Final state {final_state} does not meet the tolerance condition.")

        self.mcgs_planner.update_trajectory_plan(plan)

        for i, action in enumerate(plan_actions):
            print("Iteration: ",i)
            print("action:", action)
            observation, reward, terminated, truncated, info = self.env.step(action)
            print(observation[:3], self.mcgs_planner.graph[plan[i+1]].state)
            self.assertTrue(np.allclose(observation[:3], self.mcgs_planner.graph[plan[i+1]].state, atol=1e-1, rtol=1e-1), f"Observation {observation[:3]} does not match planned state {self.mcgs_planner.graph[plan[i+1]].state}")

        # Check if the robot is close to the goal
        print("Final state:", self.mcgs_planner.graph[plan[-1]].state)
        print("Goal state:", self.env.unwrapped.robot.state_goal)
        self.assertTrue(terminated, f"Robot is not close to the goal {self.mcgs_planner.graph[plan[-1]].state}")


    def test_astar(self, relative_path_length_tolerance=0.2):
        """Tests the length of the plan against an optimal plan using A*"""
        print("TEST A*")
        # Reset environment
        self.env.reset(seed=10)

        self.mcgs_planner = MCGSPlanner(robot=self.env.unwrapped.robot,
                    reward_function=self.env.unwrapped.reward_function,
                    max_iterations=500,
                    kappa=1.6,
                    alpha=1.0,
                    k=8,
                    radius_threshold=2.0,
                    c_uct=0.8)

        self.env.unwrapped.connect_planner(self.mcgs_planner)



        # Run planner
        self.mcgs_planner.plan()

        # Define cost functions for A*
        all_nodes = self.mcgs_planner.graph.nodes()
        best_node = max(all_nodes, key=lambda n: n.U) # Best node is the one with the highest Utility
        logger.debug(best_node.id)
        
        def edge_cost_fn(edge: EdgePayload) -> float:
            """Compute the cost of the edge."""
            return 1.0  # Uniform cost for all edges

        def estimate_cost_fn(node: NodePayload) -> float:
                """Estimate the cost to reach the goal from the given node."""
                return float(np.linalg.norm(node.state[:2] - best_node.state[:2]))

        atol = 1e-2
        rtol = 1e-2
        max_tolerance_iterations = 10
        for i in range(max_tolerance_iterations):
            # Update goal_fn for the current tolerance value
            def goal_fn(node: NodePayload) -> bool:
                return self.env.unwrapped.robot.is_finished(atol=atol, rtol=rtol, state=node.state)

            try:
                optimal_plan = rx.digraph_astar_shortest_path(graph=self.mcgs_planner.graph,
                                                            node=self.mcgs_planner.root_node_id,
                                                            goal_fn=goal_fn,
                                                            edge_cost_fn=edge_cost_fn,
                                                            estimate_cost_fn=estimate_cost_fn,
                                                            )


                plan, plan_actions = self.mcgs_planner.yield_plan(atol=atol, rtol=rtol)

                # Test path length
                plan_length_diff = len(optimal_plan) - len(plan)
                self.assertLessEqual(plan_length_diff, relative_path_length_tolerance * len(optimal_plan), f"Optimal plan length {len(optimal_plan)} differs from MCGS plan length {len(plan)} by {100*plan_length_diff/len(optimal_plan)} %.")

            except rx.NoPathFound:
                logger.info(f"No path found with abs. tolerance {atol:.2f} / rel. tolerance {rtol:.2f}. Increasing tolerance.")
                atol += 0.5
                rtol += 0.1
                continue

    def test_yield_plan(self, atol=0.0, rtol=.2):
        """Tests the yield_plan method of the MCGSPlanner."""
        print("TEST YIELD PLAN")
        # Setup
        self.mcgs_planner = MCGSPlanner(robot=self.env.unwrapped.robot,
                    reward_function=self.env.unwrapped.reward_function,
                    max_iterations=500,
                    kappa=1.6,
                    alpha=1.0,
                    k=8,
                    radius_threshold=2.0,
                    c_uct=0.8)

        self.env.unwrapped.connect_planner(self.mcgs_planner)

        # Reset environment
        self.env.reset(seed=10)

        # Run planner
        self.mcgs_planner.plan()

        plan, plan_actions = self.mcgs_planner.yield_plan(atol=atol, rtol=rtol)

        assert(len(plan) > 0), "Plan does not include successor node"

        self.mcgs_planner.update_trajectory_plan(plan)

        # Check if the robot is close to the goal
        tolerance_met_list = [self.env.unwrapped.robot.is_finished(atol=atol, rtol=rtol, state=self.mcgs_planner.graph[node_id].state) for node_id in plan]
        logger.debug(tolerance_met_list)
        self.assertEqual(sum(tolerance_met_list), 1, "Yielded plan does not meet the tolerance condition for exactly one node.")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    unittest.main()
