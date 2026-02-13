import numpy as np
import rustworkx as rx
import hnswlib
from abc import ABC
from typing import Callable, Optional
from copy import deepcopy
import logging
from scipy.spatial.distance import pdist
from time import perf_counter, sleep

from tools.planners.planner import Planner
from tools.robots.robot import Robot
from tools.envs.gym_robot_plan_env import GymRobotPlanEnv
from tools.utils.plot_graph import plot_graph_2d



logger = logging.getLogger(__name__)


        
class NodePayload(ABC):
    """
    Abstract class for payloads in the MCTS tree.
    Each payload represents a state in the search tree.
    It can be used to store information about the state, such as the number of times it has been visited (N) and its value (V).
    """
    def __init__(self, state, state_encoded, U=0, is_finished=False, is_truncated=False):
        self.id = None  # Index in the graph
        self.state = deepcopy(state)  # Ensure state is not shared
        self.state_encoded = deepcopy(state_encoded)  # Encoded state for HNSW index
        self.is_finished = is_finished  # Whether the state is finished
        self.is_truncated = is_truncated  # Whether the state is truncated
        self.N: int = 0  # Number of times the state has been visited
        self.Q = U  # Best action value (Q) for the state
        self.U = U  # Value of the state
        self.X: int = 0  # Number of times the state has been attempted to expand
        self.dispersion: float = 0.0  # Dispersion of child nodes

    def is_terminal(self):
        return self.is_finished or self.is_truncated

    def __str__(self):
        # Return a string representation of the state with 1 decimal place
        return self.__repr__()

    def __repr__(self):
        return f"NodePayload(id={self.id}, state={self.state}, N={self.N}, U={self.U}, Q={self.Q}, is_finished={self.is_finished}, is_truncated={self.is_truncated})"

    def _short_value_repr(self):
        """Short representation for debugging."""
        return f"[id: {self.id}, N: {self.N}, Q: {self.Q:.2f}, U: {self.U:.2f}, X: {self.X}, F: {self.is_finished}, T: {self.is_truncated}]"

class EdgePayload(ABC):
    """
    Abstract class for actions in the MCTS tree.
    Each action represents a transition from one state to another.
    It can be used to store information about the action, such as the action value (Q).
    """
    def __init__(self, action_list, dt_list, controller_created: bool = False):
        self.id = None  # Index in the graph
        self.action_list = deepcopy(action_list)  # Ensure action is not shared
        self.dt_list = deepcopy(dt_list)  # Ensure dt_list is not shared
        self.N = 0  # Number of times the action has been taken
        self.Q = None
        self.controller_created = controller_created  # Whether the action was created by a controller

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return f"EdgePayload(actions={self.action_list}, dt={self.dt_list}, Q={self.Q})"

    def _short_value_repr(self):
        """Short representation for debugging."""
        return f"[N: {self.N}, Q: {self.Q}]"

class MCGSPlanner(Planner):
    def __init__(self, env: GymRobotPlanEnv, budget_per_step:int=2500, computational_budget_max:int=2500, time_budget_max:float=20.0, expand_n_times:int=1, expand_n_times_min:int=1, expand_n_times_max:int=3, sample_best_X_actions:int=1, alpha:float=0.5, kappa:float=2.0, gamma:float=0.99, k:int=5, radius_threshold: float=0.1, progressive_widening_method='default', abstraction_refinement_exponent: float=-0.0, c_uct: float=1.0, epsilon: float=0.1, plan_in_space_time: bool = True, use_controller: bool = True, tracking_tolerance: float = 0.1, yield_mode: str = 'N', random_rollout_n_times: int = 0, random_rollout_length: int=1, save_replay: bool = True, save_graph: bool = False, force_exact_update: bool=True, controller_expansion_check: bool=True):
        super().__init__(env=env, computational_budget_max=computational_budget_max, time_budget_max=time_budget_max, save_replay=save_replay)
        self.budget_per_step = budget_per_step # Maximum number of iterations per planning step
        self.expand_n_times = expand_n_times
        self.expand_n_times_min = expand_n_times_min
        self.expand_n_times_max = expand_n_times_max
        self.expand_variably = (expand_n_times_min != expand_n_times_max)
        self.sample_best_X_actions = sample_best_X_actions # Number of best actions to sample from when expanding a node
        self.force_exact_update = force_exact_update # If True, always perform insertion without kNN after each step. This can lead to small cycles in the graph but ensures that each trajectory is feasible.
        self.yield_mode = yield_mode  # One of ['Q', 'U', 'N']
        assert yield_mode in ['Q', 'U', 'N'], "Yield mode must be one of ['Q', 'U', 'N']"

        self.alpha = alpha  # Exploration parameter
        self.kappa = kappa  # Progressive widening parameter
        self.k = k # Number of nearest neighbors to consider for insertion
        self.epsilon = epsilon # Epsilon for epsilon-greedy progressive widening
        self.gamma = gamma

        progressive_widening_methods_supported = ['default', 'log', 'playout', 'expansion_attempts', 'epsilon_greedy', 'dispersion']
        self.progressive_widening_method = progressive_widening_method # One out of ['default', 'log', 'playout', 'expansion_attempts']:
        assert self.progressive_widening_method in progressive_widening_methods_supported, f"Progressive widening method must be one of {progressive_widening_methods_supported}"
        # Abstraction refinement parameters
        self.radius_threshold = radius_threshold # Used if abstraction refinement is disabled
        self.use_abstraction_refinement = True # Equivalent to self.abstraction_refinement_exponent not equal 0
        self.abstraction_refinement_exponent = abstraction_refinement_exponent # Exponent
        self.tracking_tolerance = tracking_tolerance # Tolerance for tracking a node 


        self.c_uct = c_uct # UCT constant


        self.random_rollout_n_times = random_rollout_n_times # Number of random rollouts to perform for value estimation
        self.random_rollout_length = random_rollout_length # Length of each random rollout

        # Plan in space-time if True, otherwise only in space
        self.plan_in_space_time = plan_in_space_time
        self.use_controller = use_controller # Use controller to check feasibility of edges
        self.controller_expansion_check = controller_expansion_check # Use controller to check feasibility during expansion (slows down planning)


        self.planned_trajectory = []

        # Telemetry / logging
        self.save_graph = save_graph
        self.graph_replay = []



        self.reset_graph()

    def reset(self, seed: Optional[int] = None):
        """Reset the planner and the environment."""
        super().reset(seed=seed)
        self.reset_graph()

    def reset_graph(self):
        """Reset the graph and HNSW index."""
        self.graph = rx.PyDiGraph()
        if self.env.agent.state is None:
            raise ValueError("Agent state must be initialized before planning.")
        initial_reward = self.env.reward_function(self.env.agent.state) + self.rollout(self.env.agent.state)
        root_node = NodePayload(state=self.env.agent.state, 
                                state_encoded=self.env.agent.encode_state(self.env.agent.state),
                                U=initial_reward,
                                is_finished=self.env.agent.is_finished(state=self.env.agent.state),
                                is_truncated=self.env.is_truncated(state=self.env.agent.state))


        self.hnsw_index = hnswlib.Index(space='l2', 
                                        dim=root_node.state_encoded.shape[0],)
        
        self.hnsw_index.init_index(max_elements=10000, 
                                    ef_construction=200, 
                                    M=16, 
                                    allow_replace_deleted=False,
                                )
        
        self.garbage_collector = GarbageCollector(graph=self.graph, hnsw_index=self.hnsw_index)


        self.root_node_id = self.insert_node(root_node)  # Insert the root node into the graph

        self.update_node_value(self.root_node_id)  # Initialize root node value
        
        self.update_planned_trajectory() # Initialize planned trajectory

    def plan_and_step(self, computational: bool = True, time: bool = False, render: bool = False):
        """Plan while within the computational budget or time budget, then execute the first step from the plan."""
        self.plan_while_in_budget(computational=computational, time=time)
        post_processing_time_start = perf_counter()
        plan, plan_action_list, plan_dt_list = self.yield_plan(max_length=10)
        # print(f"Plan yielded: {plan} with actions {plan_action_list} and dts {plan_dt_list}")
        self.update_planned_trajectory(plan, plan_action_list, plan_dt_list)
        if len(plan) == 1: # Only root node in plan, no action to take
            logger.warning("No plan found, executing random action.")
            random_action = self.env.action_space.sample()
            new_state = self.env.agent.transition_model(self.env.agent.state, random_action, self.env.agent.dt)
            plan = [self.root_node_id, self.insert_node(NodePayload(state=new_state, state_encoded=self.env.agent.encode_state(new_state), U=0, is_finished=self.env.agent.is_finished(new_state), is_truncated=self.env.is_truncated(new_state)))]
            plan_action_list = [[random_action]]
            plan_dt_list = [[self.env.agent.dt]]
        assert len(plan) > 0, "Plan is empty. Current state: " + str(self.env.agent.state) + ", Root node: " + str(self.graph[self.root_node_id]) + ", Nodes in graph: " + str(len(self.graph.nodes()))
        # assert len(plan_action_list) > 0, "Plan action list is empty: " + str(self.env.agent.state) + ", Root node: " + str(self.graph[self.root_node_id]) + ", Nodes in graph: " + str(len(self.graph.nodes()))
        best_action_list = plan_action_list[0]
        best_dt_list = plan_dt_list[0]
        # print(f"Executing actions: {best_action_list} with dts: {best_dt_list}")

        if self.save_replay:
            self.action_replay.append(best_action_list)
            self.dt_replay.append(best_dt_list)
            self.observation_replay.append(self.env._get_obs())
            self.plan_replay.append(self.planned_trajectory)
        if self.save_graph:
            self.graph_replay.append(self.graph)

        if render:
            self.render()

        # Execute the entire action sequence in the environment
        observation, reward, terminated, truncated, info = self.env.multi_step(best_action_list, best_dt_list, render=False)
        # Or retreive new info from hardware robot <- TO BE IMPLEMENTED
        self.update_planner() # this needs to be reworked to handle new state info either from simulation or hardware
        self.time_budget_total += perf_counter() - post_processing_time_start


        return observation, reward, terminated, truncated, info
    

    

    def plan_online(self, computational: bool = True, time: bool = False, render: bool = False, use_hardware: bool = True):
        """Plan while continuously retrieving new state info from a hardware robot or more abstract simulator. 
        Enables concurrent planning and execution: publishes setpoint immediately, then continues planning 
        while robot executes in background thread."""
        # Receive latest state once at start of planning cycle
        new_state = self.receive_latest_state(use_hardware=use_hardware)
        if use_hardware:
            # Only update planner if new state is different from current state
            if not np.allclose(np.array(new_state)[:-1], np.array(self.env.agent.state)[:-1], atol=self.env.agent.atol, rtol=self.env.agent.rtol): # exclude time dimension from comparison
                # If yes, update the agent & planner with the new state info
                logger.info(f"New state received from hardware/simulator: {new_state}, previous state: {self.env.agent.state}")
                self.update_planner()
        else: # Always update planner in simulator mode, because the state is updated directly in the env
            logger.info(f"Updating planner with simulator state: {new_state}, previous state: {self.env.agent.state}")
            self.update_planner()
        
        # Get information that would be handled by the gym environment: reward, terminated, truncated, info 
        observation = self.env._get_obs()
        reward = self.env.reward_function(self.env.agent.state)
        finished = self.env.agent.is_finished(state=self.env.agent.state)
        truncated = self.env.is_truncated(state=self.env.agent.state)
        info = {}
        
        if finished or truncated: # return immediately
            if render:
                self.render()
            return observation, reward, finished, truncated, info
        
        # Check if robot is still executing from previous cycle
        if not self.is_executing():
            logger.debug("Robot is idle, can publish new setpoint")
            
            # Minimal bootstrap if no viable plan exists yet
            plan, plan_action_list, plan_dt_list = self.yield_plan(max_length=10)
            if len(plan) <= 1:
                logger.info("No plan found, performing minimal bootstrap planning...")
                for _ in range(5):  # 5 planning iterations for minimal bootstrap
                    self.plan()
                    plan, plan_action_list, plan_dt_list = self.yield_plan(max_length=10)
                    if len(plan) > 1:
                        break
            
            # Select and publish setpoint (starts execution in background)
            if len(plan) <= 1: # Only root node in plan, no action to take
                logger.warning("No plan found after bootstrap, holding position.")
                setpoint_state = self.env.agent.state # The robot should hold position
            else:
                self.update_planned_trajectory(plan, plan_action_list, plan_dt_list) # Update planned trajectory
                # Select next setpoint from planned trajectory
                setpoint_state = self.planned_trajectory['state_list'][1]            
                # Publish setpoint to hardware robot or simulator (non-blocking - starts background execution)
                self.publish_setpoint(setpoint_state, use_hardware=use_hardware)
                logger.info(f"Published setpoint {setpoint_state} to hardware={use_hardware}, execution started in background")
        else:
            logger.debug("Robot still executing, skipping setpoint publication")
        
        # Continue planning while robot executes (or is idle) in background
        logger.debug(f"Starting planning...")
        self.plan_while_in_budget(computational=computational, time=time)
        logger.debug(f"Planning completed")
        
        # Render current state (robot may still be executing)
        if render:
            self.render()
        
        return observation, reward, finished, truncated, info



    def plan(self, iterations: Optional[int] = None, expand_n_times: Optional[int] = None, expand_n_times_min: Optional[int] = None, expand_n_times_max: Optional[int] = None):
        """Plan the trajectory for the robot using MCGS."""
        if iterations is None:
            iterations = 1
        for i in range(iterations):
                logger.debug(f"---- Iteration {i+1}/{iterations} ----")
                self.expand_graph(expand_n_times=expand_n_times)
        logger.debug(f"Finished planning for {iterations} iterations.")


    def expand_graph(self, expand_n_times: Optional[int] = None, expand_n_times_min: Optional[int] = None, expand_n_times_max: Optional[int] = None, sample_best_X_actions: Optional[int] = None):
        """Expand the graph by selecting a node and inserting new nodes and edges."""
        logger.debug(f"Expanding graph at computational budget used: {self.computational_budget_used}/{self.computational_budget_max}")
        logger.debug(f"Expanding graph with time in budget used: {self.time_budget_used:.4f}/{self.time_budget_max:.4f}")
        # Select a node to expand
        selected_node_id, parent_node_ids_visited = self.select_node(self.root_node_id) # Parent nodes do not include the selected node!
        starting_node_id = deepcopy(selected_node_id)
        logger.debug(f"Selected node: {selected_node_id} via {parent_node_ids_visited}")
        
        ### Expand N-1 times without KNN search
        # inserted_edges_by_ids = [] # List of tuples (from_id, to_id)
        node_pay = self.graph[selected_node_id]
        if sample_best_X_actions is None:
            sample_best_X_actions = self.sample_best_X_actions
        if expand_n_times is None:
            expand_n_times = self.expand_n_times
        if expand_n_times_min is not None and expand_n_times_max is not None:
            assert expand_n_times_min <= expand_n_times_max, "expand_n_times_min must be less than or equal to expand_n_times_max"
            expand_n_times = np.random.randint(expand_n_times_min, expand_n_times_max + 1)
        elif self.expand_variably:
            expand_n_times = np.random.randint(self.expand_n_times_min, self.expand_n_times_max + 1)
        logger.debug(f"Expanding {expand_n_times} times")
        for i in range(expand_n_times-1):
            node_pay, edge_pay = self.expand_node(node_pay, sample_best_X_actions=sample_best_X_actions) # Generate new child node
            if node_pay.is_finished or node_pay.is_truncated:
                break
            else:
                child_node_id = self.insert_node(node_pay, edge_pay, parent_node_id=selected_node_id) # Without KNN
                parent_node_ids_visited.append(selected_node_id) # Add intermediate nodes to parent nodes for backprop
                selected_node_id = child_node_id # Child becomes new selected node
                logger.debug(f"Forced expand iteration {i+1}: {selected_node_id} via parents {parent_node_ids_visited}")

            # inserted_edges_by_ids.append((parent_node_id, child_node_id))
        logger.debug(f"Final selected node: {selected_node_id} via parents {parent_node_ids_visited}")

        # Get final node as candidate for new state and action
        node_pay, edge_pay = self.expand_node(self.graph[selected_node_id], sample_best_X_actions=sample_best_X_actions) # Generate new child node

        # Abstraction refinement (0.0 equals no abstraction refinement)
        # Abstraction refinement with multiple expansions uses the number of visits to the original selected node (as all new ones could not have been visited yet)
        radius_threshold = self.radius_threshold * (self.graph[starting_node_id].N ** self.abstraction_refinement_exponent)
        ### Insert N-th element using KNN search
        child_node_ids = self.insert_with_knn(selected_node_id, 
                                     node_pay, 
                                     edge_pay, 
                                     k=self.k, 
                                     radius_threshold=radius_threshold,
                                     mark_for_garbage_collection=self.plan_in_space_time) # Node is marked for garbage collection if planning in space-time
        # inserted_edges_by_ids.extend([(parent_node_id, child_node_id) for child_node_id in child_node_ids])
        logger.debug(f"Expanded to node(s): {child_node_ids} via {selected_node_id} and parents {parent_node_ids_visited}")
        for child_node_id in child_node_ids:
            # Update the value of the child node
            self.update_node_value(child_node_id) 
            # Update the edge value from the last selected node to the child node
            self.update_edge_value(selected_node_id, child_node_id) 

        # Backpropagate the values from the last selected node to the root node
        self.backpropagate(selected_node_id, parent_node_ids_visited)



    def yield_plan(self, max_length:int=100) -> tuple[list[int], list[np.ndarray], list[float]]:
        """Yield the plan as a sequence of states."""
        # Start from the root node and traverse the graph
        plan = [self.root_node_id]
        plan_action_list = []
        plan_dt_list = []
        current_node_id = self.root_node_id

        while current_node_id is not None:
            current_node = self.graph[current_node_id]
            edge_tuple_list = self.graph.out_edges(current_node_id) # tuple (from_id, to_id, edge_data)            
            logging.debug(f"Current node {current_node_id} has {len(edge_tuple_list)} children.")
            # Stop yield conditions



            # if self.env.agent.is_finished(state=current_node.state): # also handled in is_terminal
            #     return plan, plan_action_list, plan_dt_list
            if current_node.is_terminal():
                return plan, plan_action_list, plan_dt_list
            if len(plan) >= max_length:
                return plan, plan_action_list, plan_dt_list
            # Filter out nodes in playout path if not planning in space time (to avoid cycles)
            if not self.plan_in_space_time:
                edge_tuple_list = [edge_tuple for edge_tuple in edge_tuple_list if edge_tuple[1] not in plan]
            if current_node_id == self.root_node_id: # At root node, real robot state can deviate from root node state (if force_exact_update is disabled)
                if not self.force_exact_update and self.use_controller: # If updates are not exact agent state, the first edges are not accurate and must be calculated from scratch
                    # Recalculate edge payloads
                    logger.debug(f"Recalculating edge payloads from root node {current_node_id} based on current agent state {self.env.agent.state}")
                    new_edge_tuple_list = []
                    for i, edge_tuple in enumerate(edge_tuple_list):
                        from_id, to_id, old_edge_pay = edge_tuple
                        correct_agent_state = self.env.agent.state
                        setpoint_state = self.graph[to_id].state
                        success, new_edge_pay = self.create_edge_with_controller(correct_agent_state, setpoint_state)
                        if success:
                            new_edge_tuple_list.append((from_id, to_id, new_edge_pay))
                            logger.debug(f"Created edge from {correct_agent_state} to node {to_id}:{setpoint_state} out of {i}/{len(edge_tuple_list)} children during plan yield.")
                        else:
                            logger.debug(f"Could not create edge from {correct_agent_state} to node {to_id}:{setpoint_state} out of {i}/{len(edge_tuple_list)} children during plan yield.")
                    edge_tuple_list = new_edge_tuple_list
            if self.force_exact_update: # Remove the originally planned node from the plan that the exact update node "duplicated" (to avoid cycles where the node selection could select the duplicated node again)
                planned_new_root_node = self.planned_trajectory['previous_plan_node_ids'][1] if len(self.planned_trajectory['previous_plan_node_ids']) > 1 else None
                if planned_new_root_node is not None:
                    edge_tuple_list = [edge_tuple for edge_tuple in edge_tuple_list if edge_tuple[1] != planned_new_root_node] # Remove the edges that contain the planned new root node
            if self.planned_trajectory and 'previous_plan_node_ids' in self.planned_trajectory: # Remove the last root node
                last_root_node = self.planned_trajectory['previous_plan_node_ids'][0] if len(self.planned_trajectory['previous_plan_node_ids']) > 0 else None
                if last_root_node is not None:
                    edge_tuple_list = [edge_tuple for edge_tuple in edge_tuple_list if edge_tuple[1] != last_root_node] # Remove the edges that contain the last root node
            if len(edge_tuple_list) == 0:
                return plan, plan_action_list, plan_dt_list
            # If there are finished nodes, select one of them randomly
            finished_node_tuples = [edge_tup for edge_tup in edge_tuple_list if self.graph[edge_tup[1]].is_finished]
            if len(finished_node_tuples) > 0:
                random_tuple = np.random.randint(len(finished_node_tuples))
                best_edge_tuple = finished_node_tuples[random_tuple] # Select the child with the highest N value, tie break with Q, finally randomly
            else:
                match self.yield_mode:
                    case 'Q':
                        best_edge_tuple = edge_tuple_list[np.argmax([self.graph[edge_tuple[1]].Q for edge_tuple in edge_tuple_list])] # Select the child with the highest Q value
                    case 'U':
                        best_edge_tuple = edge_tuple_list[np.argmax([self.graph[edge_tuple[1]].U for edge_tuple in edge_tuple_list])] # Select the child with the highest U value
                    case 'N':
                        max_N_val = np.max([edge_tuple[2].N for edge_tuple in edge_tuple_list])
                        best_edge_tuples = [edge_tuple for edge_tuple in edge_tuple_list if np.isclose(edge_tuple[2].N, max_N_val)]
                        if len(best_edge_tuples) > 1:
                            # Tie break with Q value
                            max_Q_val = np.max([self.graph[edge_tuple[1]].Q for edge_tuple in best_edge_tuples])
                            best_edge_tuples = [edge_tuple for edge_tuple in best_edge_tuples if np.isclose(self.graph[edge_tuple[1]].Q, max_Q_val)]
                        # print(len(best_edge_tuples))
                        random_tuple = np.random.randint(len(best_edge_tuples))
                        best_edge_tuple = best_edge_tuples[random_tuple] # Select the child with the highest N value, tie break with Q, finally randomly

                # if best_child_node.is_truncated:
                #     break
            # print("Best edge tuple:", best_edge_tuple)
            plan.extend([best_edge_tuple[1]])
            plan_action_list.extend([best_edge_tuple[2].action_list])
            plan_dt_list.extend([best_edge_tuple[2].dt_list])
            current_node_id = best_edge_tuple[1]

        # If we reach here, selection process is finished
        logger.debug(f"Yielded plan: {plan} with actions {plan_action_list} and dts {plan_dt_list}")
        return plan, plan_action_list, plan_dt_list

    def update_planned_trajectory(self, plan=[], plan_action_list=[], plan_dt_list=[]):
        """Manages the updates of the planned trajectory.
        Currently, it replaces the entire trajectory with the new plan."""
        plan = {
            'root_node_id': self.root_node_id,
            'node_ids': plan,
            'state_list': [self.graph[node_id].state for node_id in plan],
            'action_list': plan_action_list,
            'dt_list': plan_dt_list,
            'previous_plan_node_ids': self.planned_trajectory['node_ids'] if self.planned_trajectory and 'node_ids' in self.planned_trajectory else []
        }
        self.planned_trajectory = plan

    def update_planner(self, collect_garbage=True):
        """Update the planner by finding current by inserting the current state into the graph and collecting garbage."""
        # Retreive current state from the environment
        current_state = self.env.agent.state
        # Insert the current state into the graph
        current_state_encoded = self.env.agent.encode_state(current_state)
        initial_reward = self.env.reward_function(current_state) + self.rollout(current_state)
        # If dynamic model is not deterministic, the root state is a new state and the controller must be used
        node_pay = NodePayload(state=current_state, 
                            state_encoded=current_state_encoded, 
                            U=initial_reward,
                            is_finished=self.env.agent.is_finished(state=current_state),
                            is_truncated=self.env.is_truncated(state=current_state)
        )

        
        # Insert the current state into the graph as new root
        mark_for_garbage_collection = self.plan_in_space_time or self.force_exact_update

        if self.force_exact_update:
            self.root_node_id = self.insert_node(node_pay, mark_for_garbage_collection=mark_for_garbage_collection)
        else:
            labels, distances = self.hnsw_index.knn_query(node_pay.state_encoded, k=1)
            if distances[0][0] < self.tracking_tolerance:
                self.root_node_id = labels[0][0]
                self.update_node_value(self.root_node_id)
                # print("Updated to existing root node:", self.root_node_id, "with distance:", distances[0][0])
                return # no further edges have to be added
            else: 
                self.root_node_id = self.insert_node(node_pay, mark_for_garbage_collection=mark_for_garbage_collection)
                # print("Brand new state: ", self.root_node_id, "with distance:", distances[0][0])
        self.update_node_value(self.root_node_id)
        if self.use_controller:
            attachment_node_ids = self.find_attachments_for_root_node()
            for attachment_node_id in attachment_node_ids:
                attachment_state = self.graph[attachment_node_id].state
                success, edge_payload = self.create_edge_with_controller(node_pay.state, attachment_state)
                if success:
                    self.graph.add_edge(self.root_node_id, attachment_node_id, edge_payload)
                    # Update the value of the child node
                    self.update_node_value(attachment_node_id) # 
                    # Update the edge value for all attachment nodes so that the UCT values are correct
                    self.update_edge_value(self.root_node_id, attachment_node_id)
                    # print(f"Connected new root node {self.root_node_id} to attachment node {attachment_node_id} via controller.")
            logger.debug(f"Added new root node {self.root_node_id} with attachment to nodes {attachment_node_ids}")

        else:
            # Root node is the node at the same position in the graph
            pass # Connections will be made in the next planning step


        if self.plan_in_space_time or not self.force_exact_update or collect_garbage:
            # Collect garbage nodes that are older than a certain threshold
            self.garbage_collector.collect_garbage(timestamp_threshold=current_state[-1], # Assuming time is the last element in the state
                                                   root_node_id=self.root_node_id) 

    def select_node(self, root_node_id: int) -> tuple[int, list[int]]:
        """Select a node to expand using UCT. This method calls _select_node_with_playout_path for recursion.
        Args:
            root_node_id (int): The ID of the root node to start the selection from.
        Returns:
            tuple[int, list[int]]: The ID of the selected node and the list of parent nodes visited in the current selection path (excluding the selected node).
        """
        return self._select_node_with_playout_path(root_node_id, [])

    def _select_node_with_playout_path(self, node_id: int, playout_path: list[int] = []) -> tuple[int, list[int]]:
        """Actual selection of the node to expand using UCT using recursive selection. Not meant to be called directly.
        Args:
            node_id (int): The ID of the current node to select.
            playout_path (list[int]): List of parent node IDs visited in the current selection path.
        Returns:
            tuple[int, list[int]]: The ID of the selected node and the list of parent nodes visited in the current selection path (excluding the selected node).
        """
        node = self.graph[node_id]
        successors_ids = self.graph.successor_indices(node_id)

        if node.is_truncated: # Truncated node reached, this should never happen and indicates a problem in the actual selection process
            self.render(render_mode='human')
            logging.debug(f"No further expansion possible, truncated node reached at node {node_id} with state {node.state} and {len(successors_ids)} children with playout path {playout_path} during selection.")
            
        # Use progressive widening to check if child count insufficient
        progressive_widening_condition = False
        # print(f"Progressive widening method: {self.progressive_widening_method}")
        match self.progressive_widening_method:
            case 'default':
                # print(f"Node {node_id} has {len(successors_ids)} children, N={node.N}, kappa={self.kappa}, alpha={self.alpha}, condition={self.kappa * (node.N ** self.alpha)}")
                if len(successors_ids) < self.kappa * (node.N ** self.alpha):
                    progressive_widening_condition = True
            case 'log': # Use logarithm to counteract exponential nature of node visits
                if len(successors_ids) < self.kappa * (np.log(node.N) ** self.alpha):
                    progressive_widening_condition = True
            case 'playout': # Use playout length to prevent selection too far in the future
                if node.N < self.kappa * (len(playout_path) + 1) ** self.alpha:
                    progressive_widening_condition = True
            case 'expansion_attempts': # Alternative progressive widening based on node expansion attempts
                if node.X < self.kappa * (node.N ** self.alpha):
                    progressive_widening_condition = True
            case 'epsilon_greedy': # Epsilon-greedy selection with a minimum of kappa children
                assert hasattr(self, 'epsilon'), "Epsilon not set for epsilon-greedy selection"
                if len(successors_ids) < self.kappa or np.random.rand() < self.epsilon:
                    progressive_widening_condition = True
            case 'dispersion': # Progressive widening based on dispersion of child nodes
                if node.dispersion < self.kappa * (node.N ** self.alpha):
                    progressive_widening_condition = True
                    

        # print(f"Node {node_id} with Q={node.Q}, U={node.U}, N={node.N}, X={node.X}, children={len(successors_ids)}: progressive_widening_condition={progressive_widening_condition}")
        if progressive_widening_condition:
            # Progressive widening condition met, select the node
            return node_id, playout_path
        else:
            # print(f"Current node: {node_id} with successors {successors_ids}")
            # print(f"Truncated: {[id for id in successors_ids if self.graph[id].is_truncated]}")
            # print(f"Not truncated: {[id for id in successors_ids if not self.graph[id].is_truncated]}")
            # Select best child via UCT if at least one non_truncated child exists
            # If there are non-truncated children, select the best one based on UCT
            # print(f"Selecting child of node {node_id} with N={node.N}, X={node.X}, children={len(successors_ids)}")
            uct_array = self.get_uct_values(node_id=node_id, non_truncated_only=True)
            # print(f"UCT values: {uct_array}")
            # logger.debug(f"UCT values: {uct_array}")
            if not self.plan_in_space_time and len(uct_array) > 0:
                # If not planning in space-time, check to only select nodes not already in the playout path (to avoid loops)
                uct_array = uct_array[~np.isin(uct_array[:,0], playout_path), :]

            if len(self.planned_trajectory['previous_plan_node_ids']) > 0 and len(uct_array) > 0: # Remove the last root node from the possible children to select (to avoid cycles)
                exclude_nodes = []
                exclude_nodes.append(self.planned_trajectory['previous_plan_node_ids'][0]) # Last root node
                if self.force_exact_update and len(self.planned_trajectory['previous_plan_node_ids']) > 1:
                    exclude_nodes.append(self.planned_trajectory['previous_plan_node_ids'][1]) # The node the agent tried to reach in the last step
                if len(exclude_nodes) > 0:
                    uct_array = uct_array[~np.isin(uct_array[:,0], exclude_nodes), :]
            # print(f"UCT values after playout path check: {uct_array}")
            if len(uct_array) > 0:
                max_uct_value = np.max(uct_array[:,1])
                best_child_ids = uct_array[np.isclose(uct_array[:,1], max_uct_value), 0] # Avoid rounding inaccuracies
                best_child_id = int(np.random.choice(best_child_ids))  # Randomly select one of the best children
                assert self.graph[best_child_id].is_truncated == False
                # print(f"Selected child node {best_child_id} with UCT value {max_uct_value}")
                playout_path.append(node_id) # Insert node id as parent in nodes visited (for backprop)
                return self._select_node_with_playout_path(best_child_id, playout_path)  # Recursively select the best child node
            else: # No children to traverse, return the current node
                return node_id, playout_path

    def get_child_dispersion(self, node_id: int, use_mean=False) -> float:
        """Calculate the dispersion of the children of the given node."""
        edge_list = self.graph.out_edges(node_id) # list of tuples of the form: `(node_index, child_index, edge_data)`
        if len(edge_list) == 0:
            return 0.0  # No children to calculate dispersion
        encoded_states = np.array([self.graph[edge_tuple[1]].state_encoded for edge_tuple in edge_list])
        if len(encoded_states) < 2:
            return 0.0  # Dispersion is zero if there is only one child
        # Calculate pairwise distances
        pairwise_distances = pdist(encoded_states, metric='euclidean')
        if use_mean:
            dispersion = np.mean(pairwise_distances)
        else:
            dispersion = np.max(pairwise_distances)
        return dispersion

    def get_uct_values(self, node_id: int, non_truncated_only=True) ->  np.typing.NDArray[np.float64]:
        """Calculate the UCT values for the children of the given node."""
        edge_list = self.graph.out_edges(node_id) # list of tuples of the form: `(node_index, child_index, edge_data)`
        if non_truncated_only:
            edge_list = [edge_tuple for edge_tuple in edge_list if not self.graph[edge_tuple[1]].is_truncated]
        if len(edge_list) == 0:
            return np.empty((0, 2), dtype=np.float64)  # No children to calculate UCT values
        
        N_parent = self.graph[node_id].N # Parent visit count
        uct_values = []
        for parent_node_id, child_node_id, edge_payload in edge_list:
            # print(f"Calculating UCT for edge from {parent_node_id} to {child_node_id} with edge payload {edge_payload._short_value_repr()}")
            Q_child = self.graph[child_node_id].Q
            N_action = edge_payload.N # Important: Use edge visit count for action count
            uct = Q_child + self.c_uct * np.sqrt(np.log(N_parent) / N_action)
            uct_values.append([child_node_id, uct])
        # print(uct_values)
        return np.array(uct_values) if len(uct_values) > 0 else np.empty((0, 2), dtype=np.float64)

    def rollout(self, state) -> float:
        """Perform a random rollout from the given state to estimate the expected future reward."""
        rollout_rewards = []
        for i in range(self.random_rollout_n_times):
            rollout_state = deepcopy(state)
            is_finished_temp = False
            is_truncated_temp = False
            discounted_return = 0.0
            for j in range(self.random_rollout_length):
                if not (is_finished_temp or is_truncated_temp):
                    action = self.env.agent.sample_action(state=rollout_state)
                    # print(f"Rollout {i+1}, step {j+1}: state={rollout_state}, action={action}, dt={self.env.agent.dt}")
                    rollout_state = self.env.agent.transition_model(rollout_state, action, self.env.agent.dt)
                    # print(f"Rollout {i+1}, step {j+1}: new state={rollout_state}")
                    is_finished_temp = self.env.agent.is_finished(state=rollout_state)
                    is_truncated_temp = self.env.is_truncated(state=rollout_state)
                    discounted_return += (self.env.reward_function(rollout_state)) * (self.gamma ** j)
                else:
                    # print(f"Rollout terminated early at step {j} for state {rollout_state}")
                    break
            # print(f"Rollout {i+1}/{self.random_rollout_n_times} from state {state} ended in state {rollout_state} with discounted return {discounted_return}")
            rollout_rewards.append(discounted_return)
        expected_future_reward = float(np.mean(rollout_rewards)) if len(rollout_rewards) > 0 else 0.0
        # print(f"Performed rollout from state {state}, expected future reward: {expected_future_reward}")
        return expected_future_reward

    def expand_node(self, node_pay: NodePayload, sample_best_X_actions: int=1) -> tuple[NodePayload, EdgePayload]:
        """Expand the given node by sampling a candidate for new state and action."""
        # Increase node expansion attempt counter
        node_pay.X += 1
        # Extract the encoded states of the children for maximising dispersion
        encoded_states_children = np.array([self.graph[child_id].state_encoded for child_id in self.graph.successor_indices(node_pay.id)])
        # Sample multiple actions and select the one that maximises the minimum distance to existing children
        candidate_node_list = []
        candidate_edge_list = []
        candidate_min_distances = []
        if len(encoded_states_children) == 0:
            sample_best_X_actions = 1 # No need to sample multiple if no children exist
        for i in range(sample_best_X_actions):
            node, edge = self.sample_successor_candidate_pair(node_pay)
            candidate_node_list.append(node)
            candidate_edge_list.append(edge)
            if len(encoded_states_children) > 0:
                candidate_min_distances.append(np.min(np.linalg.norm(encoded_states_children - node.state_encoded, axis=1)))
        # Select the best node based on diversity
        if len(encoded_states_children) > 0:
            best_index = int(np.argmax(candidate_min_distances))
            node = candidate_node_list[best_index]
            edge = candidate_edge_list[best_index]
        else:
            node = candidate_node_list[0]
            edge = candidate_edge_list[0]
        # print(f"Expanded candidate node {node_pay.id} to new node with state {new_state}, reward={reward}, is_finished={is_finished}, is_truncated={is_truncated}")
        ### Logging
        self.computational_budget_used += 1
        # logger.debug(f"Expanded node {node_pay.id} to new node with state {new_state}, reward={reward}, is_finished={is_finished}, is_truncated={is_truncated

        # logger.debug(node, edge)
        return (node, edge)
    
    def sample_successor_candidate_pair(self, parent_node: NodePayload) -> tuple[NodePayload, EdgePayload]:
        # Sample actions for the current node
        action = self.env.agent.sample_action(state=parent_node.state)
        action_list = [action] * self.env.multi_step_count
        dt_list = [self.env.agent.dt / self.env.multi_step_count] * self.env.multi_step_count
        # Propagate new state
        new_state = deepcopy(parent_node.state)
        for i in range(len(action_list)):
            new_state = self.env.agent.transition_model(new_state, action_list[i], dt_list[i])
            is_finished = self.env.agent.is_finished(state=new_state)
            is_truncated = self.env.is_truncated(state=new_state)
            if is_finished or is_truncated:
                break
        # Optionally use controller for actually reachable states
        if self.controller_expansion_check:
            # Check if controller can reach the new state from the parent state
            action_list, dt_list = self.env.agent.controller(state=parent_node.state, setpoint_state=new_state, match_time=False, N=self.env.multi_step_count, T=2*self.env.agent.dt)
            # Re-propagate new state with controller actions
            new_state = deepcopy(parent_node.state)
            for i in range(len(action_list)):
                new_state = self.env.agent.transition_model(new_state, action_list[i], dt_list[i])
                is_finished = self.env.agent.is_finished(state=new_state)
                is_truncated = self.env.is_truncated(state=new_state)
                if is_finished or is_truncated:
                    break
            # For debug, check if create_edge_with_controller would succeed
            success, _ = self.create_edge_with_controller(parent_node.state, new_state)
            if not success:
                logger.debug(f"Controller could not reach sampled state {new_state} from parent state {parent_node.state}, resampling action.")

        expected_future_reward = 0.0
        if not (is_finished or is_truncated):
            expected_future_reward = self.rollout(new_state)
        reward = self.env.reward_function(new_state) + expected_future_reward
        # Create new node and edge payloads
        new_state_encoded = self.env.agent.encode_state(new_state)

        node = NodePayload(state=new_state, 
                        state_encoded=new_state_encoded,
                        U=reward,
                        is_finished=is_finished,
                        is_truncated=is_truncated)
        edge = EdgePayload(action_list=action_list, dt_list=dt_list, controller_created=False)
        # logger.info(f"Sampled successor candidate pair from node {parent_node.id} to new node with state {new_state}, reward={reward}, is_finished={is_finished}, is_truncated={is_truncated}")
        return node, edge

    def create_edge_with_controller(self, start_state, setpoint_state, step_divisor=1) -> tuple[bool, EdgePayload]:
        """Create an edge between two nodes using the controller."""
        def in_tracking_tolerance(state, setpoint_state):
            return np.allclose(self.env.agent.encode_state(state), self.env.agent.encode_state(setpoint_state), atol=self.tracking_tolerance)
        # Use the agent's controller to get the action and dt
        if in_tracking_tolerance(start_state, setpoint_state):
            return False, None # Do not connect to nodes in the past or at the same time, already in tracking tolerance
        else:
            try:
                if self.plan_in_space_time:
                    action_list, dt_list = self.env.agent.controller(state=start_state, setpoint_state=setpoint_state, match_time=True, step_divisor=step_divisor)
                else:
                    action_list, dt_list = self.env.agent.controller(state=start_state, setpoint_state=setpoint_state, match_time=False, N=self.env.multi_step_count, T=2*self.env.agent.dt, step_divisor=step_divisor)
            except Exception as e:
                print(f"Error in controller: {e}")
                return False, None
        current_state = deepcopy(start_state)
        would_truncate = False
        # print(f"Controller from {start_state} to {setpoint_state} with actions {action_list} and dts {dt_list}")
        for i in range(len(action_list)):
            current_state = self.env.agent.transition_model(current_state, action_list[i], dt_list[i], step_divisor=step_divisor)
            # Check if truncated
            would_truncate = self.env.is_truncated(state=current_state)
            if would_truncate:
                break
        success = in_tracking_tolerance(current_state, setpoint_state) and not would_truncate
        # logger.debug(f"Success: {success}, n_actions: {len(action_list)}, sum_dt: {np.sum(dt_list)}, start_state: {start_state}, setpoint_state: {setpoint_state}, final_state: {current_state}, would_truncate: {would_truncate}")
        if success:
            # assert np.sum(np.linalg.norm(np.array(action_list), axis=1)) > 0, "Controller returned zero actions"
            edge = EdgePayload(action_list=action_list, dt_list=dt_list, controller_created=True)
        else:
            edge = None
        # print(success, "Start:", start_state, "Controller:", current_state, "Setpoint:", setpoint_state, "Would Truncate:", would_truncate)
        return success, edge

        # generate new graph using HNSW for nearest neighbor search
    def _create_edge_with_controller(self, start_state, setpoint_state, step_divisor=1) -> tuple[bool, Optional['EdgePayload']]:
        """Override from Planner to implement MCGS-specific controller edge creation."""
        return self.create_edge_with_controller(start_state, setpoint_state, step_divisor)
    
    def insert_with_knn(self, parent_node_id: int, candidate_node_pay: NodePayload, candidate_edge_pay: EdgePayload, k:int, radius_threshold: float, mark_for_garbage_collection: bool = True) -> np.typing.NDArray[np.int64]:
        """Insert a new state-action pair into the graph using HNSW for nearest neighbor search.
        Args:
            parent_node_id (int): The ID of the parent node in the graph.
            node_pay (NodePayload): The payload containing the new state and its encoded representation.
            edge_pay (EdgePayload): The payload containing the action associated with the new state.
            k (int): The number of nearest neighbors to consider for insertion.
            radius_threshold (float): The distance threshold for considering neighbors.
        Returns:
            np.ndarray: An array of child node IDs that were connected to the new node. (No nodes returns means no *new* edges were added)
        
        """

        # Find k nearest neighbors in HNSW index
        can_track_ids = []
        edge_payloads = []
        try:
            k = k if k is not None else self.k
            k = min(k, self.hnsw_index.get_current_count()) # Ensure k does not exceed number of elements in index
            # use knn_query to find the nearest neighbors around predicted state (candidate_node_pay.state_encoded)
            labels, distances = self.hnsw_index.knn_query(candidate_node_pay.state_encoded, k=k)
            # emulate radius search: Check which neighbors are in radius (changes with abstraction refinement)
            radius_threshold = radius_threshold
            ids_in_radius = distances[0] < radius_threshold
            if self.use_controller:
                # Check if neighbors are trackable via controller from parent node
                start_state = deepcopy(self.graph[parent_node_id].state)
                
                for neighbor_id in labels[0][ids_in_radius]:
                    if self.graph.has_edge(parent_node_id, neighbor_id):
                        # Already connected, skip
                        can_track_ids.append(neighbor_id)
                        edge_payloads.append(self.graph.get_edge_data(parent_node_id, neighbor_id))
                    else:
                        neighbor_state = self.graph[neighbor_id].state
                        success, edge_payload = self.create_edge_with_controller(start_state, neighbor_state)
                        if success:
                            can_track_ids.append(neighbor_id)
                            edge_payloads.append(edge_payload)
                can_track_ids = np.array(can_track_ids, dtype=np.int64)
            else:    
                # If not using controller, all neighbors in radius are considered trackable via candidate action
                can_track_ids = labels[0][ids_in_radius]
                edge_payloads = [candidate_edge_pay] * np.sum(ids_in_radius)
        except Exception as e:
            # No neighbors found or error in HNSW query
            logger.warning(f"Error in HNSW query (probably no neighbors to be found): {e}")
            # raise ValueError(f"Error in HNSW query: {e}")
        # Check if new node should be added or if existing neighbors should be connected
        child_node_ids = np.empty(0, dtype=np.int64)
        logger.debug(f"Found {len(can_track_ids)} neighbors: {can_track_ids} for candidate node with state {candidate_node_pay.state} from parent node {parent_node_id}")
        # Iterate over neighbors and connect them to the new node if suitable
        for i in range(len(can_track_ids)):
            neighbor_id = int(can_track_ids[i])  # Ensure neighbor_id is an integer
            neighbor_edge_pay = edge_payloads[i]
            if neighbor_id != parent_node_id:
                # Check if edge already exists
                if not self.graph.has_edge(parent_node_id, neighbor_id): 
                    if self.plan_in_space_time:
                        # Check if neighbor is in the past (avoid cycles)
                        if self.graph[neighbor_id].state[-1] <= self.graph[parent_node_id].state[-1] + 1e-3: # Assuming time is the last element in the state
                            continue
                    # Add edge to the graph
                    self.graph.add_edge(parent_node_id, neighbor_id, neighbor_edge_pay)
                else:
                    # No action necessary, edge visit will be increased during backpropagation
                    pass
                # Add new connected or existing neighbors as child nodes for backpropagation
                child_node_ids = np.append(child_node_ids, neighbor_id)

        # Check if at least one node is similar to candidate in regards to finished, truncated to increase diversity
        # Important, as important gaps between nodes could be missed otherwise
        is_similar = False
        for id in child_node_ids:
            if (self.graph[id].is_finished == candidate_node_pay.is_finished):
                is_similar = True
            if (self.graph[id].is_truncated == candidate_node_pay.is_truncated):
                is_similar = True
        # If no similar neighbors were found, add the original candidate node as a new node
        if len(child_node_ids) == 0 or not is_similar:
            new_node_id = self.insert_node(candidate_node_pay, candidate_edge_pay, parent_node_id=parent_node_id, mark_for_garbage_collection=mark_for_garbage_collection)
            # Add the new node as a child for backpropagation
            child_node_ids = np.append(child_node_ids, new_node_id)
        
        return child_node_ids

    def insert_node(self, node_pay: NodePayload, edge_pay: Optional[EdgePayload] = None, parent_node_id: Optional[int] = None, mark_for_garbage_collection: bool = True) -> int:
        """Insert a new node into the graph and updating the HNSW index."""
        # Add node to graph
        new_node_id = self.graph.add_node(node_pay)
        self.graph[new_node_id].id = new_node_id
        if edge_pay is not None and parent_node_id is not None:
            self.graph.add_edge(parent_node_id, new_node_id, edge_pay)
        # Add node to HNSW Index
        self.hnsw_index.add_items(node_pay.state_encoded, new_node_id)
        if mark_for_garbage_collection:
            # Add node to garbage collector with its timestamp
            self.garbage_collector.add_node(new_node_id, node_pay.state[-1]) # Assuming time is the last element in the state
        return new_node_id

    def find_attachments_for_root_node(self) -> list[int]:
        """Find attachments for the root node using HNSW nearest neighbor search.
        Returns:
            List[int]: A list of node IDs that are attached to the root node.
        """
        # Get the root node payload
        root_node = self.graph[self.root_node_id]
        attachment_node_ids = []

        # Find k nearest neighbors in HNSW index
        try:
            labels, distances = self.hnsw_index.knn_query(root_node.state_encoded, k=min(self.k, self.hnsw_index.get_current_count()))
            # Iterate over neighbors and connect them to the root node if suitable
            for neighbor_id, distance in zip(labels[0], distances[0]):
                if distance < self.radius_threshold:
                    neighbor_id = int(neighbor_id)
                    if self.plan_in_space_time:
                        # Check if neighbor is in the past (avoid cycles)
                        if self.graph[neighbor_id].state[-1] <= root_node.state[-1] + 1e-3: # Assuming time is the last element in the state
                            continue
                    attachment_node_ids.append(neighbor_id)
        except:
            logger.warning(f"Error in HNSW query (probably no neighbors to be found): {e}")

        return attachment_node_ids

    def update_node_value(self, node_id: int):
        node = self.graph[node_id]
        edge_list = self.graph.out_edges(node_id) # list of tuples of the form: `(node_index, child_index, edge_data)`
        # Update the value of the current node
        node.N = int(1 + np.sum([edge_pay.N for node_id, child_id, edge_pay in edge_list]))
        node.Q = (1/node.N) * (node.U + np.sum([edge_pay.N * self.graph[child_id].Q for node_id, child_id, edge_pay in edge_list]))
        node.dispersion = self.get_child_dispersion(node_id)
        # logger.debug(f"Node {node_id} updated: N={node.N}, Q={node.Q:.2f}, U={node.U:.2f}")
        # print(f"Node {node_id} updated: N={node.N}, Q={node.Q:.2f}, U={node.U:.2f}")
    
    def update_edge_value(self, node_id: int, child_node_id: int):
        logger.debug(f"Updating edge value from node {node_id} to child node {child_node_id}")
        edge_id = self.graph.edge_indices_from_endpoints(node_id, child_node_id)
        assert len(edge_id) == 1, "There should be exactly one edge from the parent to the current node."
        edge_data = self.graph.get_edge_data_by_index(edge_id[0])
        edge_data.N += 1  # Increment the edge visit count

    def backpropagate(self, node_id: int, playout_node_ids: list[int]):
        """Backpropagate the values from the selected node to the root node."""
        # print(f"Backpropagating from node {node_id} to parent nodes {playout_node_ids}")
        self.update_node_value(node_id)
        # Walk up the tree to update parent nodes
        if playout_node_ids == []:
            return
        parent_node_id = playout_node_ids[-1]
        self.update_edge_value(parent_node_id, node_id)

        self.backpropagate(parent_node_id, playout_node_ids[:-1])

    def render(self, render_mode=None, scatter_mode='Q', render_tree=True):
        """Render the graph"""
        self.env.render(planner=self, render_mode=render_mode, scatter_mode=scatter_mode, render_tree=render_tree)


class GarbageCollector:
    """Garbage collector for MCGSPlanner to clean up unused nodes."""
    def __init__(self, graph: rx.PyDiGraph, hnsw_index: hnswlib.Index):
        self.graph = graph
        self.hnsw_index = hnsw_index
        self.node_dict = dict()  # Maps node IDs to timestamps for garbage collection

    def add_node(self, node_id: int, timestamp: float):
        """Add a node to the garbage collector with its timestamp."""
        timestamp = float(timestamp) # Make sure it is hashable
        if timestamp not in self.node_dict:
            self.node_dict[timestamp] = []
        self.node_dict[timestamp].append(node_id)

    def collect_garbage(self, timestamp_threshold: float, root_node_id: int):
        """Collect garbage nodes that are older than the threshold."""
        to_remove = []
        # Find all nodes that are older than the threshold
        for time, node_ids in self.node_dict.items(): # TODO: This could be a Queue
            if time < timestamp_threshold:
                to_remove.extend(node_ids)
        # Find nodes that are not attached to root node with some threshold
        # connected_node_ids_set = rx.node_connected_component(self.graph, root_node_id)

        # Recover root node
        assert root_node_id not in to_remove # Root node must not be removed
        
        print(f"Removing nodes: {to_remove}")
        # Remove nodes from the graph and HNSW index
        self.graph.remove_nodes_from(to_remove)
        for node_id in to_remove:
            # Mark as deleted, as HNSW does not support direct removal
            self.hnsw_index.mark_deleted(node_id)
        # Clean up the dictionary
        for time in list(self.node_dict.keys()):
            if time < timestamp_threshold:
                del self.node_dict[time]


    def check_overflow(self):
        """Check for overflow in the HNSW Index."""
        if self.hnsw_index.element_count >= self.hnsw_index.get_max_elements():
            # Warning
            raise Exception("HNSW Index overflow detected.")