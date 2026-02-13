import logging
import numpy as np
from typing import Optional
from gymnasium import Space, spaces
from .robot import Robot
import casadi as ca

logger = logging.getLogger(__name__)


class SingleIntegrator(Robot):
    def __init__(self, id, name, state_space: Optional[spaces.Box] = None, action_space: Optional[Space] = None, state=None, dt: float = 1.0, max_velocity=1.0, color="#FFFF00", state_goal=None, action_sampler_method='uniform', atol=0.1, rtol=0.1, seed: Optional[int] = None, default_step_divisor: int = 1, plan_in_space_time: bool = False):
        """Initialize the single integrator robot."""
        print("Starting initialization of SingleIntegrator")
        if state_space is None:
            # Use a default state space if none is provided
            state_space = spaces.Box(low=np.array([-np.inf, -np.inf, 0.0]), high=np.array([np.inf, np.inf, np.inf]), dtype=np.float32)  # [x, y, time]

        if action_space is None:
            # Use dx, dy for uniform sampling with max_velocity
            action_space = spaces.Box(low=np.array([-max_velocity, -max_velocity]), high=np.array([max_velocity, max_velocity]), dtype=np.float32)  # [dx, dy]

        super().__init__(id=id, name=name, state_space=state_space, action_space=action_space, state=state, dt=dt, atol=atol, rtol=rtol, default_step_divisor=default_step_divisor, plan_in_space_time=plan_in_space_time)

        # self.state_space = state_space # just to suppress type warning down the line, the super class handles this as well
        # self.action_space = action_space 

        self.max_velocity = max_velocity
        self.color = color

        # Set the action sampler method
        assert action_sampler_method in ['uniform',
                                         'directed',
                                         'bang_bang',
                                         'directed_random',
                                         'directed_bang_bang'], "Invalid action sampler method. Choose from ['uniform', 'directed', 'bang_bang', 'directed_random', 'directed_bang_bang']"
        self.action_sampler_method = action_sampler_method

        self.reset(seed=seed, state=state, state_goal=state_goal)
        print("Finished initialization of SingleIntegrator")


    def reset(self, rng: np.random.Generator | None = None, seed: Optional[int] = None, state=None, state_goal=None):
        if rng is not None:
            self.np_random = rng
        else:
            # print("Resetting SingleIntegrator with seed:", seed)
            self.np_random = np.random.default_rng(seed)
            self.state_space.seed(seed)
            self.action_space.seed(seed)
        
        # print("SingleIntegrator:", self.np_random.uniform(0,1,1))
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
        # print("Encoding state:", state)
        position = state[:-1]
        time = state[-1]

        pos_transformed = 1/self.dt/self.max_velocity * np.array(position)  # Scale position by max velocity
        time_transformed = 1/self.dt * np.array(time) # Scale time by dt
        if self.plan_in_space_time:
            return np.concatenate([pos_transformed.ravel(), time_transformed.ravel()])
        else:
            return np.concatenate([pos_transformed.ravel()])

    def transition_model(self, state, action, dt=None, residual=None, step_divisor:Optional[int]=None):
        x = state[:-1]
        t = state[-1]
        u = action
        f = residual if residual is not None else np.zeros_like(u)

        # print(f"x: {x}, t: {t}, u: {u}, f: {f}")

        if dt is None:
            if self.dt is None:
                raise ValueError("Time step dt must be provided or set in the robot.")
            else:
                dt = self.dt

        if step_divisor is None:
            step_divisor = self.default_step_divisor
        dt = dt / step_divisor

        # Update position and time
        for _ in range(step_divisor):
            x = x + (u + f) * dt
            t = t + dt

        # # Ensure next state is within bounds
        # if self.state_space.is_bounded():
        #     x_new = np.clip(x_new, self.state_space.low[:2], self.state_space.high[:2])
        return np.concatenate([np.atleast_1d(x), np.atleast_1d(t)], axis=-1)

    def sample_random_state(self, time=None):
        """Sample a random state for the robot."""
        state = self.state_space.sample()
        if time is not None:
            state[-1] = time
        return state


    def sample_action(self, state=None): # next_planned_position=None):
        """Sample an action based on the current state and next planned position."""
        rel_next_target = None
        if state is None:
            state = self.state
        # if next_planned_position is not None and state is not None:
        #     rel_next_target = next_planned_position - state['position']
        # elif self.state_goal is not None and state is not None:
        #     rel_next_target = self.state_goal['position'] - state['position']

        if self.action_sampler_method == 'uniform':
            return self.action_sampler_uniform()
        # elif self.action_sampler_method == 'directed':
        #     return self.action_sampler_directed(rel_next_target=rel_next_target, scale_along=1.0)
        elif self.action_sampler_method == 'bang_bang':
            return self.action_sampler_bang_bang(scale_along=1.0)
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
            logger.warning("Controller with CasADi failed to reach the setpoint.")
            pass
        return action_list, dt_list

    def controller_casadi(self, state, setpoint_state, match_time: bool = True, N: int = 3, T: Optional[float] = None, step_divisor: Optional[int] = None) -> tuple[list, list]:
        """
        Nonlinear optimization-based controller using CasADi + IPOPT.
        Uses the robot's own transition_model to roll out candidate trajectories.
        """
        if step_divisor is None:
            step_divisor = self.default_step_divisor
        assert step_divisor == 1, "Only step_divisor=1 is currently supported for controller with CasADi."

        if T is None:
            t0 = state[-1]
            tf = setpoint_state[-1]
            T = tf - t0

        if T <= 0:
            return [np.array([0.0, 0.0])], [0.0]


        # Decision variables
        vx = ca.SX.sym("vx", N)   # velocity in x
        vy = ca.SX.sym("vy", N)   # velocity in y
        dt = ca.SX.sym("dt", N)   # durations
        vars_ = ca.vertcat(vx, vy, dt)

        # Roll out dynamics symbolically using CasADi operations
        x, y, t = state[0], state[1], state[2]
        for i in range(N):
            # Single integrator dynamics in CasADi
            x = x + vx[i] * dt[i]
            y = y + vy[i] * dt[i]
            t = t + dt[i]
        final_state = ca.vertcat(x, y, t)

        # Cost: position error only
        pos_err2 = (final_state[0] - setpoint_state[0])**2 + (final_state[1] - setpoint_state[1])**2
        cost = pos_err2

        if match_time:
            total_time = ca.sum1(dt)
            cost += (total_time - T)**2

        # Bounds
        lbvx = [self.action_space.low[0]] * N
        ubvx = [self.action_space.high[0]] * N
        lbvy = [self.action_space.low[1]] * N
        ubvy = [self.action_space.high[1]] * N
        lbdt = [0.01] * N
        ubdt = [T] * N
        lbx = lbvx + lbvy + lbdt
        ubx = ubvx + ubvy + ubdt

        # Initial guess
        pos_diff = setpoint_state[:2] - state[:2]
        
        # Simple initial guess for velocities based on required position change
        vx0 = np.clip(pos_diff[0] / T, self.action_space.low[0], self.action_space.high[0])
        vy0 = np.clip(pos_diff[1] / T, self.action_space.low[1], self.action_space.high[1])
        
        x0_guess = [vx0] * N + [vy0] * N + [T / N] * N

        # Solve
        nlp = {"x": vars_, "f": cost}
        solver = ca.nlpsol("solver", "ipopt", nlp,
                           {"ipopt.print_level": 0, "print_time": 0})
        sol = solver(x0=x0_guess, lbx=lbx, ubx=ubx)
        sol_vars = np.array(sol["x"]).flatten()

        vx_opt = sol_vars[:N]
        vy_opt = sol_vars[N:2*N]
        dt_opt = sol_vars[2*N:]

        actions = [np.array([vx_opt[i], vy_opt[i]]) for i in range(N)]
        dts = list(dt_opt)
        # print("Controller optimization success:", solver.stats()['return_status'])
        # print("Optimized actions:", actions, "dts:", dts)

        return actions, dts

    def controller_one_step(self, state, setpoint_state, match_time:bool=True, N: int = 3, T: Optional[float] = None, step_divisor: Optional[int] = None ) -> tuple[list, list]:
        """Very simple controller to compute action based on current state and setpoint state."""
        # print("SingleIntegrator controller called with state:", state, "and setpoint_state:", setpoint_state, "match_time:", match_time)
        
        p0_x, p0_y, t0 = state
        p_f_x, p_f_y, t_f = setpoint_state

        T = t_f - t0
        p_err = np.array([p_f_x - p0_x, p_f_y - p0_y])
        if T <= 0:
            return [np.array([0.0, 0.0])], [0.0]  # no time left
        n_min_actions_p = int(np.ceil(max([np.abs(p_err[0]), np.abs(p_err[1])]) / (self.max_velocity * self.dt)))
        n_min_actions_t = int(np.ceil((T / self.dt)))
        n_min_actions = max(n_min_actions_p, n_min_actions_t, 1)
        # print("n_min_actions_p:", n_min_actions_p, "n_min_actions_t:", n_min_actions_t, "-> n_min_actions:", n_min_actions)

        action = (p_err) / self.dt
        # print("Unsplit action:", action)
        # Split action according to n_min_actions
        action_list = [action/n_min_actions] * n_min_actions
        dt_list = [T/n_min_actions] * n_min_actions
        # print("Computed actions:", action_list, "with dts:", dt_list)
        
        if not match_time:
            return [action], [self.dt] # Only a single action with dt are necessary to track
        else:
            return action_list, dt_list
        
    def action_sampler_uniform(self):
        # random action
        action = self.action_space.sample()
        return action
    
    def action_sampler_bang_bang(self, scale_along=1.0):
        # Bang-bang control: full control in either direction for each dimension
        # Sample min or max independently for each dimension
        action = np.zeros_like(self.action_space.low)
        for i in range(len(action)):
            action[i] = np.random.choice([self.action_space.low[i], self.action_space.high[i]])
        return action * scale_along  # Scale the action uniformly


"""
    def action_sampler(self, rel_next_target=None, scale_along=1.0, scale_perp=0.5):
        # Sample an action in the direction of the next target
        d = rel_next_target if rel_next_target is not None else self.action_sampler_uniform(scale_along=scale_along)
        # Normalize the direction vector
        d_hat = d / np.linalg.norm(d) if np.linalg.norm(d) > 1e-8 else np.zeros_like(d)
        # Create a perpendicular vector
        perp = np.array([-d_hat[1], d_hat[0]])

        # Sample in the basis of [d_hat, perp]
        coeffs = self.np_random.normal([1.0, 0.0], [scale_along, scale_perp])
        action = coeffs[0] * d_hat + coeffs[1] * perp
        return action

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


    def action_sampler_bang_bang(self, scale_along=1.0):
        # Bang-bang control: either full speed in one direction or zero
        angle = self.np_random.uniform(0, 2 * np.pi)  # Random angle
        radius = self.np_random.choice([self.max_velocity, 0], p=[1.0, 0.0])  # 100% velocity probability
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
"""