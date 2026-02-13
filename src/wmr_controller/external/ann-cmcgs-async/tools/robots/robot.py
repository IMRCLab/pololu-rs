import numpy as np
from typing import Optional
from .agent import Agent
from gymnasium import Space


class Robot(Agent):
    """Base class for robots in the simulation environment. Here, a robot is an agent with a controller."""
    def __init__(self, id, name, state_space: Space, action_space: Space, state=None, dt: float = 1.0, atol: float = 0.1, rtol: float = 0.0, default_step_divisor: int = 1, plan_in_space_time: bool = False):
        super().__init__(id=id, name=name, state_space=state_space, action_space=action_space, state=state)
        self.dt = dt
        self.atol = atol
        self.rtol = rtol
        self.default_step_divisor = default_step_divisor
        self.plan_in_space_time = plan_in_space_time

    def step(self, action, dt=None, residual=None, step_divisor=None):
        """Applies action"""
        self.state = self.transition_model(self.state, action, dt, residual, step_divisor=step_divisor)
        return self.state

    def transition_model(self, state, action, dt=None, residual=None, step_divisor=None):
        """Transition model of the agent. This should be implemented in the subclass."""
        raise NotImplementedError("Transition model must be implemented in the subclass.")
    
    def multi_step_transition_model(self, state, action_list: list, dt_list: Optional[list] = None, residual_list: Optional[list] = None, step_divisor: Optional[int] = None):
        """Apply multiple steps of the transition model with the given list of actions."""
        assert len(action_list) > 0, "Action list must contain at least one action."
        current_state = state
        for i, action in enumerate(action_list):
            current_state = self.transition_model(current_state, action, dt=dt_list[i] if dt_list else None, residual=residual_list[i] if residual_list else None, step_divisor=step_divisor)
        return current_state

    def is_finished(self, state=None, state_goal=None, atol=None, rtol=None) -> bool:
        """Check if the robot has reached the goal state."""
        if state is None:
            state = self.state
        if state_goal is None:
            state_goal = self.state_goal
        if atol is None:
            atol = self.atol
        if rtol is None:
            rtol = self.rtol
        a = self.encode_state(state)
        b = self.encode_state(state_goal)
        mask = ~(np.isnan(a) | np.isnan(b)) # ignore NaN values (e.g., time if reaching at any time is allowed)
        return np.allclose(a[mask], b[mask], atol=atol, rtol=rtol)

    def controller(self, state, setpoint_state) -> tuple[list, list]:
        """Controller to compute action based on current state and setpoint state.
        Return action_list and dt_list to apply the actions."""
        raise NotImplementedError("Controller must be implemented in the subclass.")

    def __repr__(self):
        return f"Robot(id={self.id}, name={self.name}, state={self.state})"

    def __str__(self):
        return f"Robot {self.name} (ID: {self.id}) with state {self.state}"