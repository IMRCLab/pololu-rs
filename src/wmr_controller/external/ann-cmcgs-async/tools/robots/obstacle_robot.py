import numpy as np

from .robot import Robot


class ObstacleRobot(Robot):
    def __init__(self, name, rgb_color=None, state=None):

        # self.weight_vector_state = np.array([0.0, 0.0, 0.0, 0.0])

        # self.weight_vector_action = np.array([0.0])

        self.dynamics_model = lambda state, action, residual_force: state # No dynamics for obstacle robot
        self.residual_model = lambda state, action: np.zeros_like(action)  # No residual force for obstacle robot
        self.sample_action = lambda state, rel_next_target: 0.0   # Uniform action sampler

        
        super().__init__(
            id=None,  # Obstacle robots do not have an ID
            name=name,
            dt=0.0,  # No time step for obstacle robots
            state=state,
            bbox=None,  # No bounding box for obstacle robots
            color="#FF0000"  # Default color for obstacle robots
        )

        
