from .gym_agent import GymAgent, GymState
from .robot import Robot
import gymnasium as gym

class GymRobot(GymAgent, Robot):
    """Robot that delegates dynamics to a Gym environment"""
    
    def __init__(self, id, name, env: gym.Env):
        super().__init__(id=id, name=name, env=env)

    def controller(self, state, setpoint_state):
        """Controller to compute action based on current state and setpoint state."""
        raise NotImplementedError("Controller must be implemented in the subclass.")