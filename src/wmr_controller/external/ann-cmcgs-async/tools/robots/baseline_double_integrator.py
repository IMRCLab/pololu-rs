import numpy as np
from typing import Optional
from gymnasium import Space, spaces
from .robot import Robot

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

class BaselineDoubleIntegrator(Robot):
    def __init__(self, id, name, state_space: Optional[spaces.Box] = None, action_space: Optional[spaces.Box] = None, state=None, dt: float = 1.0, max_velocity=1.0, max_acceleration=1.0, color="#FFFF00", state_goal=None, action_sampler_method='uniform', atol=0.1, rtol=0.1, seed: Optional[int] = None, default_step_divisor: int = 1, plan_in_space_time: bool = True):
        """Initialize the double integrator robot with fixed velocity in x from the CMCGS baseline."""
        logger.info("Starting initialization of BaselineDoubleIntegrator")
        if state_space is None:
            # Use a default state space if none is provided
            state_space = spaces.Box(low=np.array([-np.inf, -max_velocity, 0.0]), 
                                     high=np.array([np.inf, max_velocity, np.inf]), 
                                     dtype=np.float32)  # [y, dy, t]

        if action_space is None:
            # Use ddx, ddy for uniform sampling with max_acc
            action_space = spaces.Box(low=np.array([-max_acceleration]), 
                                      high=np.array([max_acceleration]), 
                                      dtype=np.float32)  # [ddy]

        super().__init__(id=id, name=name, state_space=state_space, action_space=action_space, state=state, dt=dt, atol=atol, rtol=rtol, plan_in_space_time=plan_in_space_time, default_step_divisor=default_step_divisor)

        # self.state_space = state_space # just to suppress type warning down the line, the super class handles this as well
        # self.action_space = action_space 

        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration

        self.color = color

        # Set the action sampler method
        assert action_sampler_method in ['uniform',
                                         'directed',
                                         'bang_bang',
                                         'directed_random',
                                         'directed_bang_bang'], "Invalid action sampler method. Choose from ['uniform', 'directed', 'bang_bang', 'directed_random', 'directed_bang_bang']"
        self.action_sampler_method = action_sampler_method

    def reset(self, rng: np.random.Generator | None = None, seed: Optional[int] = None, state=None, state_goal=None):
        if rng is not None:
            self.np_random = rng
        else:
            # print("Resetting DoubleIntegrator with seed:", seed)
            self.np_random = np.random.default_rng(seed)
            self.state_space.seed(seed)
            self.action_space.seed(seed)
        
        # print("DoubleIntegrator:", self.np_random.uniform(0,1,1))
        if state is None:
            self.state = self.sample_random_state(time=0.0)  # Sample a random state at time 0.0
        else:
            self.state = state
        if state_goal is None:
            self.state_goal = self.sample_random_state(time=np.nan)  # Sample a random goal state at any time
        else:
            self.state_goal = state_goal


    def encode_state(self, state):
        """Transform the state [y, v, t] to a vector representation."""
        y = state[0]
        v = state[1]
        t = state[-1]

        pos_transformed = y / (self.dt * self.max_velocity)
        vel_transformed = v / (self.dt * self.max_acceleration)
        time_transformed = t / self.dt
        if self.plan_in_space_time:
            return np.array([pos_transformed, vel_transformed, time_transformed])
        else:
            return np.array([pos_transformed, vel_transformed])


    def transition_model(self, state, action, dt=None, residual=None, step_divisor: Optional[int] = None):
        y = state[0]
        v = state[1]
        t = state[-1]
        u = action[0]  # scalar
        f = residual[0] if residual is not None else 0.0

        if step_divisor is None:
            step_divisor = self.default_step_divisor
        if dt is None:
            if self.dt is None:
                raise ValueError("Time step dt must be provided or set in the robot.")
            else:
                dt = self.dt

        dt = dt / step_divisor  # for substepping
        for _ in range(step_divisor):
            y = y + v * dt
            v = v + (u + f) * dt
            v = np.clip(v, self.state_space.low[1], self.state_space.high[1])  # Clip velocity to max limits

        if y >= self.state_space.high[0] and v > 0:
            v = 0.0
        if y <= self.state_space.low[0] and v < 0:
            v = 0.0 # stop at boundaries
        y = np.clip(y, self.state_space.low[0], self.state_space.high[0])  # Clip position to max limits
        t = t + dt
        # print("Transition model: state:", state, "action:", action, "-> new state:", [y_new, v_new, t_new])
        return np.concatenate([np.atleast_1d(y),
                               np.atleast_1d(v),
                               np.atleast_1d(t)], axis=-1)


    def sample_random_state(self, time=None):
        """Sample a random state for the robot."""
        state = self.state_space.sample()
        if time is not None:
            state[-1] = time
        return np.atleast_1d(state)
    

    def sample_action(self, state=None):
        """Sample an action based on the current state and next planned position."""
        rel_next_target = None
        if state is None:
            state = self.state

        if self.action_sampler_method == 'uniform':
            return self.action_sampler_uniform()
        if self.action_sampler_method == 'bang_bang':
            return self.action_sampler_bang_bang()
        else:
            raise ValueError(f"Unknown action sampler method: {self.action_sampler_method}")

    def controller(self, state, setpoint_state, match_time: bool = True, N: int = 2, T: Optional[float]=None, step_divisor: int = 1) -> tuple[list, list]:
        """
        Bang-bang controller for 1D discrete-time double integrator (Euler-style dynamics)
        """
        y0, v0, t0 = state
        y_f, v_f, t_f = setpoint_state

        if T is None:
            T = t_f - t0
        if T <= 0:
            return [np.array([0.0])], [0.0]  # no time left

        tau = 1/2 * T

        # Linear system for two-phase acceleration
        A = np.array([
            [tau,         T-tau],      # velocity constraint
            [tau*(T-tau), 0    ]       # position constraint (Euler-style)

        ])
        b = np.array([
            v_f - v0,
            y_f - y0 - v0 * T
        ])
        
        a1, a2 = np.linalg.solve(A, b)

        # a1 = np.clip(a1, self.action_space.low[0], self.action_space.high[0])
        # a2 = np.clip(a2, self.action_space.low[0], self.action_space.high[0])

        return [np.array([a1]), np.array([a2])], [tau, T - tau]


    def action_sampler_uniform(self):
            # random action
            action = self.action_space.sample()
            return action
    
    def action_sampler_bang_bang(self):
            # bang-bang action
            a_x = self.np_random.choice([self.action_space.low[0], self.action_space.high[0]])
            a_y = self.np_random.choice([self.action_space.low[1], self.action_space.high[1]])
            action = np.array([a_x, a_y])
            return action