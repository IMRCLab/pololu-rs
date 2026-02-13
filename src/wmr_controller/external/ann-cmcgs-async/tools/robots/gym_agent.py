import numpy as np
from typing import Optional
from tools.robots.agent import Agent
import gymnasium as gym
import copy

class GymState():
    """Wrapper for Gym state representation. Contains not only observation but also copy of env and other info."""
    def __init__(self, env: gym.Env, observation, reward, terminated, truncated, info):
        self.set_state(env, observation, reward, terminated, truncated, info)

    def set_state(self, env: gym.Env, observation, reward, terminated, truncated, info):
        self.env = copy.deepcopy(env)
        self.observation = observation
        self.reward = reward
        self.terminated = terminated
        self.truncated = truncated
        self.info = info

    def clone(self):
        """Create a deep copy of the GymState."""
        return GymState(copy.deepcopy(self.env), copy.deepcopy(self.observation), self.reward, self.terminated, self.truncated, copy.deepcopy(self.info))

    def to_components(self):
        """Extract the components of the GymState using a cloned environment."""
        clone = self.clone()
        return clone.env, clone.observation, clone.reward, clone.terminated, clone.truncated, clone.info

    def reset(self, seed: Optional[int] = None):
        """Reset the GymState to its initial state."""
        self.observation, self.info = self.env.reset(seed=seed)
        self.reward = 0.0
        self.terminated = False
        self.truncated = False
        return self

class GymAgent(Agent):
    """Agent that delegates dynamics to a Gym environment."""

    def __init__(self, id, name, env: gym.Env):
        state = GymState(env, None, 0.0, False, False, {})
        super().__init__(id=id, 
                         name=name, 
                         state_space=env.observation_space,
                         action_space=env.action_space,
                         state=state)
        self.reset(state=state)

    def reset(self, rng: np.random.Generator | None = None, seed: int | None = None, state=None) -> None:
        """Reset both agent RNG and the Gym environment."""
        super().reset(rng, seed, state)
        self.state = self.state.reset(seed) # Reset the Gym environment

    def encode_state(self, state):
        """Get the observation as the encoded state."""
        return np.array(state.observation, dtype=np.float32)

    def transition_model(self, state, action, dt=None):
        """Use the gym env to simulate next state."""
        # Set current state
        temp_env, _, _, _, _, _ = state.to_components()
        # Step in gym environment
        # beware: RNG will not be advanced! We assume env is fully deterministic.
        obs, reward, terminated, truncated, info = temp_env.step(action)
        next_state = GymState(temp_env, obs, reward, terminated, truncated, info)
        return next_state

    def step(self, action):
        """Step the agent by applying the action in the gym environment."""
        self.state = self.transition_model(self.state, action)
        return self.state

    def is_finished(self, state=None) -> bool:
        """Check if the agent has reached the goal state."""
        if state is None:
            state = self.state
        terminated = state.terminated
        return terminated
    
    def is_truncated(self, state=None) -> bool:
        """Check if the agent has been truncated."""
        if state is None:
            state = self.state
        truncated = state.truncated
        return truncated

    def sample_action(self, state=None):
        """Random action from gym env action space."""
        if state is None:
            state = self.state

        temp_env, obs, reward, terminated, truncated, info = state.to_components()
        action = temp_env.action_space.sample()
 
        # necessary to advance RNG
        state.set_state(temp_env, obs, reward, terminated, truncated, info) # restore state
        
        return action

    def sample_random_state(self):
        """Sample random initial state by resetting env."""
        return GymState(self.state.env, None, 0.0, False, False, {}).reset()
