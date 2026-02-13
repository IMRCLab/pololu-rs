from __future__ import annotations
from abc import ABC
import numpy as np

from tools.robots.robot import Robot

from treelib import Tree as TreelibTree
import base64
import hashlib



class CODecMCTS:
    def __init__(self, robot: Robot, max_level: int, iterations: int, weight_vector_state, weight_vector_action, kappa: float = .3, alpha: float = .5, gamma: float = 1.0):
        self.robot = robot
        self.max_level = max_level
        self.iterations = iterations
        self.weight_vector_state = weight_vector_state
        self.weight_vector_action = weight_vector_action

        self.kappa = kappa
        self.alpha = alpha
        self.gamma = gamma  # Discount factor for well explored action nodes during yield
        self.reset_tree()  # Initialize the tree with the initial state

    def reset_tree(self):
        """Reset the tree to the initial state."""
        self.root_node = StateNode(level=0, k=0, state=self.robot.current_state, robot=self.robot, parent=None)


    def expand_tree(self, expand_node_n_times: int = 1):
        """Expand the tree by selecting a node, expanding it, and performing a rollout."""
        node = self.select_node(self.root_node)
        if node is None:  # No further expansion possible, return
            return
        else:
            for _ in range(expand_node_n_times):
                child = node.expand()
                if isinstance(child, StateNode):  
                    child.rollout(self.weight_vector_state) 
                    node.backpropagate() # Backpropagate the value from the (current) StateNode
                elif isinstance(child, ActionNode):
                    # If the child is an ActionNode, expand it to get the next StateNode for avoiding reward imbalance
                    for _ in range(expand_node_n_times):
                        state_child = child.expand()
                        state_child.rollout(self.weight_vector_state) 
                    child.backpropagate()  # Backpropagate the value from the (next) StateNode
                else:
                    pass
        


    
    def select_node(self, node: Node) -> Node:
        """Select a node based on the UCT algorithm and progressive widening."""
        children = node.children
        children_non_terminal = [child for child in children if not child.is_terminal]

        if node.is_terminal: # Terminal node reached
            return None # No further expansion possible
        if len(children) < self.kappa * (node.visits ** self.alpha):
            # Progressive widening condition not met, expand the node
            return node
        if len(children_non_terminal) > 0:
            # If there are non-terminal children, select the best one based on UCT

            # Select the child with tendendy towards lower values (ActionNode) or higher values (StateNode)
            uct_values = node.get_uct_values(non_terminal_only=True)
            # print(f'nodetype: {'State' if isinstance(node, StateNode) else 'Action'}, uct values: ')
            for child, uct in zip(children_non_terminal, uct_values):
                # print(child, uct)
                pass
            max_uct_value = max(uct_values)
            best_children = [child for child, uct in zip(children_non_terminal, uct_values) if uct == max_uct_value]
            best_child = np.random.choice(best_children)  # Randomly select one of the best children
            return self.select_node(best_child)  # Recursively select the best child node
        else: # No children to traverse, return the current node
            return node
        
    def yield_best_action(self, node: StateNode):
        """Yield the best action node based on the minimum yield score of its children."""
        # c_values = np.array([child.value for child in node.children])
        children_non_terminal = [child for child in node.children if not child.is_terminal]
        # print(f'Robot {node.robot.name} at level {node.level} has {len(children_non_terminal)} non-terminal children.')
        # print([f'k={child.k}: {child.action} ' for child in children_non_terminal])
        c_values = np.array([child.get_yield_score(self.gamma) for child in children_non_terminal])
        c_min = np.argmin(c_values) # Get the index of the child with the minimum value
        return children_non_terminal[c_min]

    def yield_next_state(self, node: ActionNode):
        """Yield the next state node based on the median value of its children."""
        assert np.sum([child.is_terminal for child in node.children]) == 0, "No terminal children should be present in the ActionNode"
                       
        # c_values = [child.value for child in node.children]
        # c_median = np.argpartition(c_values, len(c_values) // 2)[len(c_values) // 2] # Get the median value index
        
        # plan for the bad cases but bias towards better explored values
        c_values = np.array([child.get_yield_score(self.gamma) for child in node.children])
        c_max = np.argmax(c_values) # Get the index of the child with the maximum value
        
        return node.children[c_max]
    
    def yield_plan(self) -> tuple[list[dict], dict]:
        """Yield the planned trajectory as a list of (complete) state-action pairs."""
        assert isinstance(self.root_node, StateNode), "Ensure the root node is a StateNode"
        planned_trajectory = []
        next_state_node = self.root_node
    
        # Traverse the tree to find the best state action pairs sequence
        while len([child for child in next_state_node.children if not child.is_terminal]) > 0: # At least one action for the current state
            best_action_node = self.yield_best_action(next_state_node)
            planned_trajectory.append({'state': next_state_node.state, 'action': best_action_node.action})
            next_state_node = self.yield_next_state(best_action_node)
            assert next_state_node.is_terminal == False
            if len([child for child in best_action_node.children if not child.is_terminal]) == 0: # There are no children to traverse in the same level
                break  # No more children to traverse, exit the loop
        return planned_trajectory, next_state_node.state  # Return the planned trajectory and the final state

class Node(ABC):
    def __init__(self, robot: Robot, level: int, k: int, parent: Node=None):
        self.robot = robot
        self.level = level
        self.k = k # node id w/ respect to the parent node
        self.parent = parent

        self.children = list() # List of child nodes
        self.visits = 0
        self.cost_vector = None  # Cost vector for the state node, used for rollout
        self.value = 0.0
        self.is_terminal = False


    def update_value(self, value):
        self.value = value  # Cost vector for the state node, used for rollout
        self.visits += 1

    def expand(self) -> Node:
        raise NotImplementedError("This method should be implemented by subclasses.")

    def add_child(self, child_node):
        self.children.append(child_node)

    def get_value(self, cost_vector, weight_vector) -> float:
        """Calculate the value of the state node based on the cost vector and weight vector."""
        return cost_vector @ weight_vector

    def rollout(self, weight_vector):
        raise NotImplementedError("This method should be implemented by subclasses.")
    
    def backpropagate(self, value):
        raise NotImplementedError("This method should be implemented by subclasses.")

    def get_uct_values(self, non_terminal_only=False, sign='positive') -> float:
        """Calculate the UCT values for the children of the given node."""
        children = self.children
        if non_terminal_only:
            children = [child for child in children if not child.is_terminal]
        if len(children) == 0:
            return 0.0  # No children to calculate UCT values

        if sign == 'positive':
            uct_values = [child.value + np.sqrt(2 * np.log(self.visits) / child.visits) for child in children]
        elif sign == 'negative':
            uct_values = [- child.value + np.sqrt(2 * np.log(self.visits) / child.visits) for child in children]
        else:
            raise ValueError("Sign must be either 'positive' or 'negative'.")
        return uct_values

    def __repr__(self):
        return f"Node(level={self.level}, k={self.k}, robot={self.robot.name}, visits={self.visits}, value={self.value:5.1f})"
    
class StateNode(Node):
    def __init__(self, robot: Robot, level: int, k: int, state, residual_force=None, parent: Node=None):
        super().__init__(robot=robot, level=level, k=k, parent=parent)
        self.state = state
        self.residual_force = residual_force
        self.state_value = 0.0  # Value of the state node, used for backpropagation
        self.yield_score = 0.0  # Score during the yield phase, updated only after get_yield_score()

    def expand(self) -> ActionNode:
        action = self.sample_action()
        # print(f'Expanding node at level {self.level} with state {self.state} and action {action}')
        
        child = None
        if action is not None:
            child = ActionNode(robot=self.robot, 
                               level=self.level, # Action node is at the same level as the state node
                               k=len(self.children),
                               action=action,
                               parent=self)
            # print(f'Child action node created: {child}')
            self.add_child(child)
        else:
            self.is_terminal = True
        return child

    def sample_action(self) -> np.ndarray:
        # print(self.robot.planned_trajectory, self.level)
        if len(self.robot.planned_trajectory) > self.level + 1:
            action = self.robot.sample_action(state_node=self, next_planned_position=self.robot.planned_trajectory[self.level + 1]['state']['position']) # Sample action based on the current state and next planned position
        else: 
            action = self.robot.sample_action(state_node=self, next_planned_position=None)
        return action

    def backpropagate(self):
        value_min = min([child.value for child in self.children])
        # self.update_value([value_min, self.state_value])
        # self.update_value([value_min, -.3*len(self.children)])  # Update value with the minimum child value and the number of children
        self.update_value(max([value_min, self.state_value]))
        if self.parent is None: # No parent means this is the root node
            return
        else:
            self.parent.backpropagate() # Recursively backpropagate to the parent node



    def get_cost_vector(self, collision_threshold=0.1, border_threshold=1.0):
        c_border = self.border_check(self.state, border_threshold=border_threshold) # Border collision check
        c_collision = self.get_collision_cost(self.state, self.robot.planned_trajectory_others, collision_threshold=collision_threshold) # Collision cost based on the current state and planned trajectory of others
        c_tracking = self.get_tracking_cost(self.state, self.robot.tracking_targets) # Tracking cost based on the current state and tracking target
        c_residual = self.get_residual_cost(self.residual_force) # Residual cost based on the residual force
        c_reward = self.get_reward_cost(self.state)  # Reward cost based on the current state
        c_replanning = self.get_replanning_cost(self.state, self.robot.planned_trajectory)
        return np.array([c_border, c_collision, c_tracking, c_residual, c_reward, c_replanning])

    def random_rollouts(self,  n_rollouts, m_length, weight_vector, collision_threshold=2.0, border_threshold=1.0 ):
        """Perform multiple rollouts from the current state node by random action-sequences."""
        rollout_values = np.empty((n_rollouts))  # Store the rollouts for debugging or analysis
        for i in range(n_rollouts):
            state_node = self
            for _ in range(m_length):
                action = state_node.sample_action()  # Sample a random action
                if action is None:
                    break  # No action available, stop the rollout
                else:
                    action_node = ActionNode(robot=self.robot, 
                               level=state_node.level, # Action node is at the same level as the state node
                               k=len(state_node.children),
                               action=action,
                               parent=state_node)
                state, residual_force = action_node.sample_state()  # Sample the next state based on the current state and action
                
                state_node = StateNode(robot=self.robot, 
                                       level=action_node.level + 1, # State node is one level deeper than the action node
                                       k=len(action_node.children),
                                       state=state, 
                                       residual_force=residual_force,  # Residual force is not used in this case
                                       parent=action_node)
            # after random trajectory, perform a rollout
            state_node.rollout(weight_vector, 
                               collision_threshold=collision_threshold, 
                               border_threshold=border_threshold,
                               do_random_rollouts=False)  # Perform a rollout from the last state node
            rollout_values[i] = state_node.value  # Store the state node after the rollout

        self.update_value(np.mean(rollout_values))  # Update the value of the state node with the average value of the rollouts


    def rollout(self, weight_vector, collision_threshold=2.0, border_threshold=1.0, do_random_rollouts=False):
        

        cost_vector = self.get_cost_vector(collision_threshold=collision_threshold,
                                    border_threshold=border_threshold)

         # Store the state value for this state node
        value = self.get_value(cost_vector, weight_vector)  # Update the value of the node based on the cost vector and weight vector
        self.cost_vector = cost_vector
        self.update_value(value)
        self.state_value = value 

        if np.sum(cost_vector[:2]) > 0: # Collision detected, mark as terminal
            self.is_terminal = True
        elif do_random_rollouts:
            # Do random rollouts to estimate the value of the state node better
            self.random_rollouts(n_rollouts=0,
                                 m_length=0,
                                 weight_vector=weight_vector, 
                                 collision_threshold=collision_threshold, 
                                 border_threshold=border_threshold, 
                                 )

    def border_check(self, state, border_threshold) -> float:
        """Check if the state collides with the bounding box of the robot."""
        if self.robot.bbox is None:
            return False  # No bounding box defined, no collision check needed
        position = state['position']
        bbox = self.robot.bbox # bbox is an array [x_min, y_min, x_max, y_max]
        c_left = position[0] < bbox[0] + border_threshold
        c_right = position[0] > bbox[2] - border_threshold
        c_top = position[1] < bbox[1] + border_threshold
        c_bottom = position[1] > bbox[3] - border_threshold
        return (c_left or c_right or c_top or c_bottom)  # Return True if any of the conditions are met (collision with the bounding box)

    def collision_check(self, state_1, state_2, collision_threshold) -> float:
        """Check if two states are in collision based on their distance."""
        return np.linalg.norm(state_1['position'] - state_2['position']) < collision_threshold

    def get_collision_cost(self, state, planned_trajectory_others=list(list()), collision_threshold=.1) -> float:
        return np.sum([self.collision_check(state, traj[self.level+1]['state'], collision_threshold=collision_threshold) if len(traj) > self.level +1 else 0.0  for traj in planned_trajectory_others])

    def get_tracking_cost(self, state, tracking_targets) -> float:
        # Implement the logic to calculate the tracking cost based on the state and tracking target
        if len(self.robot.tracking_targets) > self.level:     
            return np.linalg.norm(state['position'] - tracking_targets[[self.level]])
        else: 
            return 0.0 

    def get_residual_cost(self, residual_force) -> float:
        # Implement the logic to calculate the residual cost based on the residual force
        if self.residual_force is not None: 
            return np.linalg.norm(residual_force)
        else:
            return 0.0
    
    def get_reward_cost(self, state) -> float:
        return self.robot.get_reward(state)
    
    def get_replanning_cost(self, state, planned_trajectory) -> float:
        """Replanning cost based on the action and next action with decreasing weights."""
        if len(planned_trajectory) > self.level +1:
            diff = state['position'] - planned_trajectory[self.level+1]['state']['position']
            inverse_weights = 1 / np.arange(1, len(diff) + 1)**2  # Inverse weights for the L2-Norm
            weighted_diff = diff * inverse_weights  # Apply inverse weights to the difference
            return np.linalg.norm(weighted_diff)  # Calculate the L2-Norm of the
        else:
            return 0.0


    def get_uct_values(self, non_terminal_only=False) -> list[float]:
        """Calculate the UCT values for the children of the given node."""
        children = self.children
        if non_terminal_only:
            children = [child for child in children if not child.is_terminal]
        if len(children) == 0:
            return 0.0  # No children to calculate UCT values
        # Use negative sign to minimize cost of ActionNode children
        uct_values = [- child.value + np.sqrt(2 * np.log(self.visits) / child.visits) for child in children]
        return uct_values
    
    def get_yield_score(self, gamma: float = 1.0) -> float:
        """Calculate the score for the state node based on its value and visits."""
        score = self.value - gamma / np.sqrt(self.visits)
        self.yield_score = score
        return score

    def __repr__(self) -> str:
        return f"StateNode(level={self.level}, k={self.k}, visits={self.visits}, value={self.value:5.1f},  is_terminal={self.is_terminal}, children={len(self.children)}, parent={get_identifier(self.parent)[:8]}, state={self.state}, residual={self.residual_force}, robot={self.robot.name}, )"
    

class ActionNode(Node):
    def __init__(self, robot: Robot, level: int, k: int, action, parent: Node=None):
        super().__init__(robot=robot, level=level, k=k, parent=parent)
        self.action = action
        self.action_value = 0.0
        self.yield_score = 0.0  # Score during the yield phase, updated only after get_yield_score()

    def expand(self) -> StateNode:
        """Expand the action node by sampling a new state."""
        state, residual_force = self.sample_state()
        
        child = None
        if state is not None:
            child = StateNode(robot=self.robot, 
                              level=self.level + 1, # State node is one level deeper than the action node
                              k=len(self.children),
                              state=state, 
                              residual_force=residual_force,
                              parent=self)
            self.add_child(child)
        else:
            self.is_terminal = True
        return child

    def sample_state(self) -> tuple[StateNode, np.ndarray]:
        """Interface with the robot's dynamic model to sample a new state based on the current state and action."""
        state, residual_force = self.robot.sample_state(self.parent.state, self.action)
        return state, residual_force

    def backpropagate(self):
        """Backpropagate the value from the action node to the parent state node."""
        value_max = max([child.value for child in self.children])
        # self.update_value([value_max, self.action_value])  # Update value with the action value and the maximum child value
        # self.update_value([value_max, -.3*len(self.children)])
        self.update_value(value_max)

        if any(child.is_terminal for child in self.children):  # Worst case scenario: Action is terminal if any child is terminal
            self.is_terminal = True

        if self.parent is None: # No parent means this is the root node
            return
        else: 
            self.parent.backpropagate() # Recursively backpropagate to the parent node

    def get_cost_vector(self):
        c_replanning = self.replanning_cost(self.action, self.robot.planned_trajectory)
        return np.array([c_replanning])


    def rollout(self, weight_vector):
                  
        self.cost_vector = self.get_cost_vector()

        # Store the state value for this action node
        value = self.get_value(self.cost_vector, weight_vector)  # Update the value of the node based on the cost vector and weight vector
        self.update_value(value)
        self.action_value = value 


    def replanning_cost(self, action, planned_trajectory) -> float:
        # Implement the logic to calculate the replanning cost based on the action and next action
        if len(planned_trajectory) > self.level +1:
            return np.linalg.norm(action - planned_trajectory[self.level+1]['action'])
        else:
            return 0.0


    def get_uct_values(self, non_terminal_only=False) -> list[float]:
        """Calculate the UCT values for the children of the given node."""
        children = self.children
        if non_terminal_only:
            children = [child for child in children if not child.is_terminal]
        if len(children) == 0:
            return 0.0  # No children to calculate UCT values
        # Use positive sign to maximize cost of StateNode children
        uct_values = [child.value + np.sqrt(2 * np.log(self.visits) / child.visits) for child in children] 
        return uct_values

    def get_yield_score(self, gamma: float = 1.0) -> float:
        """Calculate the score for the action node based on its value and visits."""
        score = self.value + gamma / np.sqrt(self.visits)
        self.yield_score = score
        return score

    def __repr__(self) -> str:
        return f"ActionNode(level={self.level}, k={self.k}, visits={self.visits}, value={self.value:5.1f}, last_yield_score={self.yield_score:5.1f}, is_terminal={self.is_terminal}, children={len(self.children)}, parent={get_identifier(self.parent)[:8]}, action={self.action}, robot={self.robot.name}, )"
    



### printing methods

def check_node_type(node):
    if isinstance(node, ActionNode):
        node_type = 'a'
    elif isinstance(node, StateNode):
        node_type = 's'
    else:
        node_type = 'n'
    return node_type

def get_identifier(node):
    unique_id = hashlib.md5(node.__repr__().encode('ascii')).digest()
    unique_id = base64.b64encode(unique_id)
    return unique_id
    #return f'{check_node_type(node)}_{node.level}_{node.k}'

def get_tag(node):
    return f'{check_node_type(node)}_{node.level}_{node.k}_{node.visits}_{node.value:5.1f}_{node.__repr__()}'

def append_nodes(tree, node):
    children = node.children
    for child in children:
        tree.create_node(tag=get_tag(child), 
                         identifier=get_identifier(child),
                         parent=get_identifier(node),
                        #  data=child
                        )
        append_nodes(tree, child)

def print_tree(robot):
    print("Tree structure:")
    tree = TreelibTree()
    root = robot.tree.root_node
    tree.create_node(tag=get_tag(root), 
                     identifier=get_identifier(root),
                    #  data=root
                     )
    append_nodes(tree, root)
    tree.show()