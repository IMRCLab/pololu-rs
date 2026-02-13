import numpy as np
from typing import Optional
from gymnasium import Space, spaces
from .robot import Robot
import casadi as ca

class DoubleIntegrator(Robot):
    def __init__(self, id, name, state_space: Optional[spaces.Box] = None, action_space: Optional[spaces.Box] = None, state=None, dt: float = 1.0, max_velocity=1.0, max_acceleration=1.0, color="#FFFF00", state_goal=None, action_sampler_method='uniform', atol=0.1, rtol=0.1, seed: Optional[int] = None, default_step_divisor: int = 1, plan_in_space_time: bool = True):
        """Initialize the double integrator robot."""
        print("Starting initialization of DoubleIntegrator")
        if state_space is None:
            # Use a default state space if none is provided
            state_space = spaces.Box(low=np.array([-np.inf, -np.inf, -max_velocity, -max_velocity, 0.0]), 
                                     high=np.array([np.inf, np.inf, max_velocity, max_velocity, np.inf]), dtype=np.float32)  # [x, y, dx, dy, time]

        if action_space is None:
            # Use ddx, ddy for uniform sampling with max_acc
            action_space = spaces.Box(low=np.array([-max_acceleration, -max_acceleration]), 
                                      high=np.array([max_acceleration, max_acceleration]), dtype=np.float32)  # [ddx, ddy]

        super().__init__(id=id, name=name, state_space=state_space, action_space=action_space, state=state, dt=dt, atol=atol, rtol=rtol, default_step_divisor=default_step_divisor, plan_in_space_time=plan_in_space_time)

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
        """Transform the state to a vector representation of spacetime distance."""
        x = state[:2]
        v = state[2:4]
        t = state[-1] 

        pos_transformed = 1/self.dt/self.max_velocity * np.array(x)  # Scale position by max velocity
        vel_transformed = 1/self.dt/self.max_acceleration * np.array(v)
        time_transformed = 1/self.dt * np.array(t)
        if self.plan_in_space_time:
            return np.concatenate([pos_transformed.ravel(), vel_transformed.ravel(), time_transformed.ravel()])
        else:
            return np.concatenate([pos_transformed.ravel(), vel_transformed.ravel()])

    # double integrator dynamics model 
    def transition_model(self, state, action, dt=None, residual=None, step_divisor=None):
        x = state[:2]
        v = state[2:4]
        t = state[-1]
        u = action
        f = residual if residual is not None else np.zeros_like(u)

        if dt is None:
            if self.dt is None:
                raise ValueError("Time step dt must be provided or set in the robot.")
            else:
                dt = self.dt

        if step_divisor is None:
            step_divisor = self.default_step_divisor
        dt = dt / step_divisor  # smaller steps for numerical stability
        for _ in range(step_divisor):
            # Update position velocity and time
            x = x + v * dt
            v = v + (u + f) * dt
            t = t + dt
        # # Ensure next state is within bounds
        # if self.state_space.is_bounded():
        #     x_new = np.clip(x_new, self.state_space.low[:2], self.state_space.high[:2])
        #     v_new = np.clip(v_new, -self.max_velocity, self.max_velocity)
        return np.concatenate([np.atleast_1d(x),
                               np.atleast_1d(v),
                               np.atleast_1d(t)], axis=-1)


    def sample_random_state(self, time=None):
        """Sample a random state for the robot."""
        state = self.state_space.sample()
        if time is not None:
            state[-1] = time
        return state
    

    def sample_action(self, state=None):
        """Sample an action based on the current state and next planned position."""
        rel_next_target = None
        if state is None:
            state = self.state

        if self.action_sampler_method == 'uniform':
            return self.action_sampler_uniform()
        # elif self.action_sampler_method == 'directed':
        #     return self.action_sampler_directed(rel_next_target=rel_next_target, scale_along=1.0)
        # elif self.action_sampler_method == 'bang_bang':
        #     return self.action_sampler_bang_bang(scale_along=1.0)
        # elif self.action_sampler_method == 'directed_random':
        #     return self.action_sampler_directed_random(rel_next_target=rel_next_target, scale_random=0.5, scale_directed=0.1)
        # elif self.action_sampler_method == 'directed_bang_bang':
        #     return self.action_sampler_directed_bang_bang(rel_next_target=rel_next_target, scale_random=0.5, scale_directed=0.1)
        else:
            raise ValueError(f"Unknown action sampler method: {self.action_sampler_method}")

    def controller(self, state, setpoint_state, match_time: bool = True, N: int = 10, T: Optional[float] = None, step_divisor: Optional[int] = None) -> tuple[list, list]:
        def check_success(action_list, dt_list):
            # print("Checking controller success with action_list:", action_list, "dt_list:", dt_list)
            return self.is_finished(self.multi_step_transition_model(state.copy(), action_list, dt_list, step_divisor=step_divisor), setpoint_state)
        
        # try default controller first
        action_list, dt_list = self.controller_casadi(state, setpoint_state, match_time, N=N, T=T, step_divisor=step_divisor)
        if check_success(action_list, dt_list):
            return action_list, dt_list
        else:  # try something else
            pass
        return action_list, dt_list

    def controller_casadi(self, state, setpoint_state, match_time: bool = True, N: int = 3, T: Optional[float] = None, step_divisor: Optional[int] = None) -> tuple[list, list]:
        """
        Nonlinear optimization-based controller using CasADi + IPOPT.
        Uses the robot's own transition_model to roll out candidate trajectories.
        """
        assert step_divisor == 1, "Only step_divisor=1 is currently supported for controller with CasADi."

        if T is None:
            t0 = state[-1]
            tf = setpoint_state[-1]
            T = tf - t0

        if T <= 0:
            return [np.array([0.0, 0.0])], [0.0]


        # Decision variables
        ax = ca.SX.sym("ax", N)   # acceleration in x
        ay = ca.SX.sym("ay", N)   # acceleration in y
        dt = ca.SX.sym("dt", N)   # durations
        vars_ = ca.vertcat(ax, ay, dt)

        # Roll out dynamics symbolically using CasADi operations
        x, y, vx, vy, t = state[0], state[1], state[2], state[3], state[4]
        for i in range(N):
            # Double integrator dynamics in CasADi
            x = x + vx * dt[i] + 0.5 * ax[i] * dt[i]**2
            y = y + vy * dt[i] + 0.5 * ay[i] * dt[i]**2
            vx = vx + ax[i] * dt[i]
            vy = vy + ay[i] * dt[i]
            t = t + dt[i]
        final_state = ca.vertcat(x, y, vx, vy, t)

        # Cost: position + velocity error
        pos_err2 = (final_state[0] - setpoint_state[0])**2 + (final_state[1] - setpoint_state[1])**2
        vel_err2 = (final_state[2] - setpoint_state[2])**2 + (final_state[3] - setpoint_state[3])**2
        cost = pos_err2 + 0.5 * vel_err2

        if match_time:
            total_time = ca.sum1(dt)
            cost += (total_time - T)**2

        # Bounds
        lbax = [self.action_space.low[0]] * N
        ubax = [self.action_space.high[0]] * N
        lbay = [self.action_space.low[1]] * N
        ubay = [self.action_space.high[1]] * N
        lbdt = [0.01] * N
        ubdt = [T] * N
        lbx = lbax + lbay + lbdt
        ubx = ubax + ubay + ubdt

        # Initial guess
        pos_diff = setpoint_state[:2] - state[:2]
        vel_diff = setpoint_state[2:4] - state[2:4]
        
        # Simple initial guess for accelerations based on required velocity change
        ax0 = np.clip(vel_diff[0] / T, self.action_space.low[0], self.action_space.high[0])
        ay0 = np.clip(vel_diff[1] / T, self.action_space.low[1], self.action_space.high[1])
        
        x0_guess = [ax0] * N + [ay0] * N + [T / N] * N

        # Solve
        nlp = {"x": vars_, "f": cost}
        solver = ca.nlpsol("solver", "ipopt", nlp,
                           {"ipopt.print_level": 0, "print_time": 0})
        sol = solver(x0=x0_guess, lbx=lbx, ubx=ubx)
        sol_vars = np.array(sol["x"]).flatten()

        ax_opt = sol_vars[:N]
        ay_opt = sol_vars[N:2*N]
        dt_opt = sol_vars[2*N:]

        actions = [np.array([ax_opt[i], ay_opt[i]]) for i in range(N)]
        dts = list(dt_opt)
        # print("Controller optimization success:", solver.stats()['return_status'])
        # print("Optimized actions:", actions, "dts:", dts)

        return actions, dts

    def controller_two_step(self, state, setpoint_state, match_time: bool=True, T: Optional[float] = None, N: Optional[int] = 2, step_divisor: Optional[int] = None) -> tuple[list, list]:
        """
        Controller for double integrator: computes a 2-phase constant-acceleration
        sequence that drives the system from (x,v,t) to (x_f,v_f,t_f).

        Args:
            state          : current state [x, v, t]
            setpoint_state : desired state [x_f, v_f, t_f]
            match_time     : if True, ensure final time matches t_f exactly

        Returns:
            action_list : list of accelerations
            dt_list     : list of durations
        """
        # unpack
        x0 = state[:2]
        v0 = state[2:4]
        t0 = state[-1]

        x_f = setpoint_state[:2]
        v_f = setpoint_state[2:4]
        t_f = setpoint_state[-1]

        T = t_f - t0
        if T <= 0:
            return [np.zeros_like(v0)], [0.0]  # no time left, no action

        # symmetric split
        tau = 0.5 * T

        # linear system coefficients
        A = np.array([
            [tau, T - tau],                          # velocity constraint
            [0.5*tau**2 + tau*(T - tau), 0.5*(T - tau)**2]  # position constraint
        ])

        a1_list, a2_list = [], []
        for i in range(len(x0)):
            b = np.array([
                v_f[i] - v0[i],
                x_f[i] - x0[i] - v0[i]*T
            ])
            a1, a2 = np.linalg.solve(A, b)
            a1_list.append(a1)
            a2_list.append(a2)

        a1 = np.array(a1_list)
        a2 = np.array(a2_list)

        if not match_time:
            # just return first phase (minimal correction in one dt step)
            return [a1], [self.dt]
        else:
            # standard 2-phase profile
            return [a1, a2], [tau, T - tau]

    def action_sampler_uniform(self):
            # random action
            action = self.action_space.sample()
            return action
"""
##############

    def action_sampler_directed(self, rel_next_target=None, scale_along=1.0):
        # Sample an action in the direction of the next target
        random_action = self.action_sampler_uniform(scale_along=scale_along)
        if rel_next_target is None:
            return random_action

        # Normalize the direction vector
        d_hat = rel_next_target / np.linalg.norm(rel_next_target) if np.linalg.norm(rel_next_target) > 1e-8 else np.zeros_like(rel_next_target)
        # Create a directed action
        directed_action = d_hat * scale_along

        return directed_action * self.max_velocity if np.linalg.norm(directed_action) > 1e-8 else np.zeros_like(directed_action)

    def action_sampler(self, rel_next_target=None, scale_along=1.0, scale_perp=0.5):
        # Sample an action in the direction of the next target
        d = rel_next_target if rel_next_target is not None else self.action_sampler_uniform(scale_along=scale_along)
        # Normalize the direction vector
        d_hat = d / np.linalg.norm(d) if np.linalg.norm(d) > 1e-8 else np.zeros_like(d)
        # Create a perpendicular vector
        perp = np.array([-d_hat[1], d_hat[0]])

        # Sample in the basis of [d_hat, perp]
        coeffs = np.random.normal([1.0, 0.0], [scale_along, scale_perp])
        action = coeffs[0] * d_hat + coeffs[1] * perp
        return action


    def action_sampler_uniform(self, scale_along=1.0):
        # random action
        angle = np.random.uniform(0, 2 * np.pi)  # Random angle
        radius = np.random.uniform(0, self.max_velocity)  # Random radius in [0, max_velocity]
        action = np.array([radius * np.cos(angle), radius * np.sin(angle)])
        return action * scale_along  # Scale the action uniformly

    def action_sampler_bang_bang(self, scale_along=1.0):
        # Bang-bang control: either full speed in one direction or zero
        angle = np.random.uniform(0, 2 * np.pi)  # Random angle
        radius = np.random.choice([self.max_velocity, 0], p=[1.0, 0.0])  # 100% velocity probability
        action = np.array([radius * np.cos(angle), radius * np.sin(angle)])
        return action * scale_along  # Scale the action uniformly


    def action_sampler_directed_random(self, rel_next_target=None, scale_random=1.0, scale_directed=0.0):
        # Sample an action in the direction of the next target
        
        random_action = self.action_sampler_uniform(scale_along=scale_random)
        # Create a directed action
        directed_action = self.action_sampler_directed(rel_next_target=rel_next_target, scale_along=scale_directed)

        # Combine random and directed action
        action = random_action + directed_action
        return action / np.linalg.norm(action) * self.max_velocity if np.linalg.norm(action) > 1e-8 else np.zeros_like(action)


    def action_sampler_directed_bang_bang(self, rel_next_target=None, scale_random=1.0, scale_directed=0.3):
        # Sample an action in the direction of the next target with mixed scaling
        random_action = self.action_sampler_bang_bang(scale_along=scale_random)
        directed_action = self.action_sampler_directed(rel_next_target=rel_next_target, scale_along=scale_directed)
        # Combine random and directed action
        action = random_action + directed_action
        return action / np.linalg.norm(action) * self.max_velocity if np.linalg.norm(action) > 1e-8 else np.zeros_like(action)


#### LEGACY

def action_sampler_clip_vel(state, dt=1.0, max_acceleration=1.0):
    # Random action but reduce the velocity to a maximum of 1.0
    random_action = np.random.uniform(-max_acceleration, max_acceleration, 2)
    # Clip the action to ensure the velocity does not exceed 1.0
    action = random_action * dt  # Scale by dt to get the velocity
    a_norm = np.linalg.norm(action)
    v_norm = np.linalg.norm(state['velocity'])
    # a_norm + v_norm <= 1.0
    if a_norm + v_norm > 1.0:
        action = action / a_norm * (1.0 - v_norm)
    return action

def action_sampler_adaptive(state, rel_next_target=None, scale_along=1.0, scale_perp=1.0):
    # Random sampling, but bias towards perpendicular actions if the velocity is high
    v = state['velocity']
    # If velocity is high, bias towards perpendicular actions
    if np.linalg.norm(v) > 0.8:
        # Sample a perpendicular action and brake
        action =  action_sampler(rel_next_target, scale_along=scale_along/2, scale_perp=scale_perp*2)
        return action - 0.5 * v  # Apply a braking force

    elif np.linalg.norm(v) > 0.5:
        # Sample a perpendicular action
        return action_sampler(rel_next_target, scale_along=scale_along/2, scale_perp=scale_perp*2)
    else:
        # Sample a normal action
        return action_sampler(rel_next_target, scale_along=scale_along, scale_perp=scale_perp)

def action_sampler_cautious(state, standard_dev=2):
    # Sample actions from normal distribution, but with bias towards lower velocities
    # If velocity is high, bias towards braking actions
    v = state['velocity']
    # Modify the loc of the normal distribution based on velocity
    action = np.random.normal(-v, [standard_dev, standard_dev])
    assert action.shape == (2,), f"Action shape mismatch: {action.shape}"
    clip_action = np.clip(action, -1.0, 1.0)  # Clip the action to [-1, 1] range
    # If velocity is high, sample a braking action
    return clip_action

"""