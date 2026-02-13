import numpy as np
from typing import Optional
from abc import ABC
from gymnasium import Space, spaces


class Agent(ABC):
    def __init__(self, id, name, state_space: Space, action_space: Space, state=None):
        self.id = id  # Unique identifier for the agent
        self.name = name
        self.state_space = state_space
        self.action_space = action_space
        
        self.reset(state=state)

    def reset(self, rng: np.random.Generator | None = None, seed: Optional[int] = None, state=None, state_goal=None) -> None:
        if rng is not None:
            self.np_random = rng
        else:
            self.np_random = np.random.default_rng(seed)
            self.state_space.seed(seed)
            self.action_space.seed(seed)

        if state is None:
            self.state = self.sample_random_state()  # Sample a random state
        else:
            self.state = state

        self.state_goal = state_goal  # Goal state can be None

    def encode_state(self, state) -> np.typing.NDArray:
        """Encode the state into a format suitable for nearest neighbor search."""
        raise NotImplementedError("State encoding must be implemented in the subclass.")

    def step(self, action):
        """Applies action"""
        self.state = self.transition_model(self.state, action)
        return self.state
    
    def set_state(self, state):
        """Set the agent's state."""
        self.state = state

    def transition_model(self, state, action):
        """Transition model of the agent. This should be implemented in the subclass."""
        raise NotImplementedError("Transition model must be implemented in the subclass.")

    def is_finished(self, state=None) -> bool:
        """Check if the agent has reached the goal state."""
        raise NotImplementedError("is_finished must be implemented in the subclass.")
    
    def is_truncated(self, state=None) -> bool:
        """Check if the agent has been truncated."""
        raise NotImplementedError("is_truncated must be implemented in the subclass.")

    def sample_random_state(self):
        return self.state_space.sample()

    def sample_action(self, state=None):
        """Sample an action based on the current state and the next planned position."""
        raise NotImplementedError("Action sampling must be implemented in the subclass.")
    
    def __repr__(self):
        return f"Agent(id={self.id}, name={self.name}, state={self.state})"

    def __str__(self):
        return f"Agent {self.name} (ID: {self.id}) with state {self.state}"