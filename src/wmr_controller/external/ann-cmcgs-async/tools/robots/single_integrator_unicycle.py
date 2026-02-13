import numpy as np
from typing import Optional
from gymnasium import Space, spaces
from gymnasium.spaces import Box
import logging
from scipy.optimize import minimize
import casadi as ca


import dubins

from .robot import Robot
from .single_integrator import SingleIntegrator


logger = logging.getLogger(__name__)

class SingleIntegratorUnicycle(SingleIntegrator):
    def __init__(self, id, name, state_space: Optional[spaces.Box] = None, action_space: Optional[Space] = None, state=None, dt: float = 1.0, max_velocity=1.0, max_angular=1.0, color="#FFFF00", state_goal=None, action_sampler_method='uniform', atol=0.1, rtol=0.1, seed: Optional[int] = None, default_step_divisor: int = 10, plan_in_space_time: bool = True):
        """Initialize the single integrator robot."""
        logger.info("Starting initialization of SingleIntegratorUnicycle")
        if state_space is None:
            # Use a default state space if none is provided
            state_space = spaces.Box(low=np.array([-np.inf, -np.inf, -np.pi, 0.0]),
                                     high=np.array([np.inf, np.inf, np.pi, np.inf]), dtype=np.float32)  # [x, y, theta, time]

        if action_space is None:
            # Use dr, dtheta for uniform sampling with max_velocity
            action_space = spaces.Box(low=np.array([0.0, -max_angular]), 
                                      high=np.array([max_velocity, max_angular]), dtype=np.float32)  # [dr, dtheta]

        super().__init__(id=id, name=name, state_space=state_space, action_space=action_space, state=state, dt=dt, color=color, state_goal=state_goal, atol=atol, rtol=rtol, seed=seed, default_step_divisor=default_step_divisor, plan_in_space_time=plan_in_space_time)

        # self.state_space = state_space # just to suppress type warning down the line, the super class handles this as well
        # self.action_space = action_space 

        self.max_velocity = max_velocity
        self.max_angular = max_angular

        # Set the action sampler method
        assert action_sampler_method in ['uniform',
                                         'bang_bang',
                                         'uniform_steering'], "Invalid action sampler method. Choose from ['uniform', 'bang_bang', 'uniform_steering']"
        self.action_sampler_method = action_sampler_method

        self.reset(seed=seed, state=state, state_goal=state_goal)
        logger.info("Finished initialization of SingleIntegratorUnicycle")

    def _wrap_to_state_space_angle(self, angle):
        """Wrap angle in relation to how state space is defined."""
        max_angle = self.state_space.high[2]
        min_angle = self.state_space.low[2]
        range_angle = max_angle - min_angle
        return (angle - min_angle) % range_angle + min_angle

    def _wrap_to_2pi_positive(self, angle):
        return (angle % (2*np.pi))
    
    def _angle_diff(self, theta, theta_setpoint):
        """Smallest signed difference a - b in [-pi, pi)."""
        return (theta_setpoint - theta + np.pi) % (2*np.pi) - np.pi


    def encode_state(self, state):
        """Transform the state to a vector representation of spacetime distance. The heuristic must be optimistic as it is supposed to be used for radius search before actually querying the controller."""
        # print("Encoding state:", state)
        x, y, theta, t = state
        pos_transformed = np.array([x, y]) / (self.dt * self.max_velocity)
        theta_transformed = np.array([np.sin(theta), np.cos(theta)]) / (2 * np.sin(self.dt * self.max_angular / 2)) # Embedding so that the euclidean distance corresponds to minimum angular difference
        time_transformed = t / self.dt
        if self.plan_in_space_time:
            features = [*pos_transformed, *theta_transformed, time_transformed]
        else:
            features = [*pos_transformed, *theta_transformed]
        return np.array(features)
    
    def is_finished(self, state=None, state_goal=None, atol=None, rtol=None) -> bool:
        # Apply angle wrapping to theta components before checking closeness
        if state is None:
            state = self.state.copy()  # Use copy to avoid modifying original state
        else:
            state = state.copy()
        if state_goal is None:
            state_goal = self.state_goal.copy()
        else:
            state_goal = state_goal.copy()

        state[2] = self._wrap_to_state_space_angle(state[2])
        state_goal[2] = self._wrap_to_state_space_angle(state_goal[2])
        return super().is_finished(state, state_goal, atol, rtol)
        

    def transition_model(self, state, action, dt=None, residual=None, step_divisor:Optional[int]=None):
        
        # state = [x, y, theta, t]
        x, y, theta, t = state
        v, w = action
        f = residual if residual is not None else np.zeros_like(action)
        # Add residuals (if used for modeling errors/disturbances)
        v += f[0]
        w += f[1]

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
            # Unicycle dynamics
            x = x + v * np.cos(theta) * dt
            y = y + v * np.sin(theta) * dt
            theta = self._wrap_to_state_space_angle(theta + w * dt)  # Use _wrap_to_state_space_angle instead of modulo
            t = t + dt

        # print(f"Transition from state {state} with action {action} over dt {dt} to new state {[x, y, theta, t]}")
        return np.array([x, y, theta, t])
    

    def sample_action(self, state=None): # next_planned_position=None):
        """Sample an action based on the current state and next planned position."""
        if state is None:
            state = self.state
        if self.action_sampler_method == 'uniform':
            return self.action_sampler_uniform()
        elif self.action_sampler_method == 'bang_bang':
            return self.action_sampler_bang_bang(scale_along=1.0)
        elif self.action_sampler_method == 'uniform_steering':
            return self.action_sampler_uniform_steering()
        else:
            raise ValueError(f"Unknown action sampler method: {self.action_sampler_method}")

    def action_sampler_uniform_steering(self):
            # maximum velocity (for/backwards) or zero, uniform steering
            v = np.random.choice([self.action_space.low[0], 0.0, self.action_space.high[0]])
            omega = self.np_random.uniform(self.action_space.low[1], self.action_space.high[1])
            action = np.array([v, omega])
            return action


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
        v = ca.SX.sym("v", N)   # linear velocities
        w = ca.SX.sym("w", N)   # angular velocities
        dt = ca.SX.sym("dt", N) # durations
        vars_ = ca.vertcat(v, w, dt)

        # Roll out dynamics symbolically using CasADi operations
        x, y, theta, t = state[0], state[1], state[2], state[3]
        for i in range(N):
            # Unicycle dynamics in CasADi
            x = x + v[i] * ca.cos(theta) * dt[i]
            y = y + v[i] * ca.sin(theta) * dt[i] 
            theta = theta + w[i] * dt[i]
            t = t + dt[i]
        final_state = ca.vertcat(x, y, theta, t)

        # Cost: position + orientation error
        pos_err2 = (final_state[0] - setpoint_state[0])**2 + (final_state[1] - setpoint_state[1])**2
        ang_err2 = (ca.atan2(ca.sin(final_state[2] - setpoint_state[2]),
                             ca.cos(final_state[2] - setpoint_state[2])))**2
        cost = pos_err2 + 0.5 * ang_err2

        if match_time:
            total_time = ca.sum1(dt)
            cost += (total_time - T)**2

        # Bounds
        lbv = [self.action_space.low[0]] * N
        ubv = [self.action_space.high[0]] * N
        lbw = [self.action_space.low[1]] * N
        ubw = [self.action_space.high[1]] * N
        lbdt = [0.01] * N
        ubdt = [T] * N
        lbx = lbv + lbw + lbdt
        ubx = ubv + ubw + ubdt

        # Initial guess
        dist = np.linalg.norm(setpoint_state[:2] - state[:2])
        v0 = np.clip(dist / T, self.action_space.low[0], self.action_space.high[0])
        angle_diff = self._angle_diff(setpoint_state[2], state[2])
        if abs(angle_diff) < 1e-3 or np.isclose(abs(angle_diff), np.pi, atol=.1):  # straight if no turning or 180° turn necessary
            w0 = 0.0
        else:
            w0 = np.clip(self._angle_diff(setpoint_state[2], state[2]) / T,
                        self.action_space.low[1], self.action_space.high[1])
        # print("Error: ", "angle_diff:", self._angle_diff(setpoint_state[2], state[2]), "dist:", dist, "T:", T)
        # print("Initial guess v0, w0, T/N:", v0, w0, T/N)
        x0_guess = [v0] * N + [w0] * N + [T / N] * N

        # Solve
        nlp = {"x": vars_, "f": cost}
        solver = ca.nlpsol("solver", "ipopt", nlp,
                           {"ipopt.print_level": 0, "print_time": 0})
        sol = solver(x0=x0_guess, lbx=lbx, ubx=ubx)
        sol_vars = np.array(sol["x"]).flatten()

        v_opt = sol_vars[:N]
        w_opt = sol_vars[N:2*N]
        dt_opt = sol_vars[2*N:]

        actions = [np.array([v_opt[i], w_opt[i]]) for i in range(N)]
        dts = list(dt_opt)
        # print("Controller optimization success:", solver.stats()['return_status'])
        # print("Optimized actions:", actions, "dts:", dts)

        return actions, dts


    def controller_dubins(self, state, setpoint_state, match_time: bool = True, N: int = 10) -> tuple[list, list]:
        """
        Dubins-based controller for the unicycle model.

        Parameters:
            state: current state [x, y, theta, t]
            setpoint_state: desired state [x, y, theta, t]
            match_time: if True, scale durations to match desired final time
            N: ignored (kept for compatibility)
        Returns:
            action_list: list of [v, w] controls
            dt_list: list of durations for each control
        """

        t0 = state[-1]
        tf = setpoint_state[-1]
        T = tf - t0

        if T <= 0:
            return [np.array([0.0, 0.0])], [0.0]

        # Dubins path params
        q0 = (state[0], state[1], state[2])       # (x, y, theta)
        qf = (setpoint_state[0], setpoint_state[1], setpoint_state[2])
        rho = self.max_velocity / self.max_angular  # turning radius = v_max / w_max

        path = dubins.shortest_path(q0, qf, rho)

        # Sample along path
        sample_step = 1/N  # resolution proportional to turning radius
        configurations, _ = path.sample_many(sample_step)

        action_list = []
        dt_list = []

        for i in range(len(configurations) - 1):
            x0, y0, th0 = configurations[i]
            x1, y1, th1 = configurations[i + 1]

            dx, dy = x1 - x0, y1 - y0
            dtheta = self._angle_diff(th1, th0)

            dist = np.hypot(dx, dy)
            dt = dist / self.max_velocity if dist > 1e-6 else abs(dtheta) / self.max_angular
            dt = max(dt, 1e-3)

            v = dist / dt
            w = dtheta / dt

            v = np.clip(v, self.action_space.low[0], self.action_space.high[0])
            w = np.clip(w, self.action_space.low[1], self.action_space.high[1])

            action_list.append(np.array([v, w]))
            dt_list.append(dt)

        # # Optionally scale to match desired final time
        # if match_time:
        #     total_time = np.sum(dt_list)
        #     if total_time > 0 and T > 0:
        #         scaling = T / total_time
        #         dt_list = [dt * scaling for dt in dt_list]

        return action_list, dt_list


    def controller_optimization(self, state, setpoint_state, match_time:bool=True, N=3) -> tuple[list, list]:
        """
        Produces a sequence of controls and durations to track a setpoint state for a unicycle.

        Parameters:
            state: current state [x, y, theta, t]
            setpoint_state: desired state [x, y, theta, t]
            match_time: if True, penalize mismatch in time
            N: number of constant-control segments
        Returns:
            action_list: list of [v, w] controls
            dt_list: list of durations for each control
        """

        t0 = state[-1]
        tf = setpoint_state[-1]
        T = tf - t0

        if T <= 0:
            return [np.array([0.0, 0.0])], [0.0]

        def objective(vars_):
            controls = vars_[:2*N].reshape(N, 2)
            dts = vars_[2*N:]
            curr_state = state.copy()

            for u, dt in zip(controls, dts):
                curr_state = self.transition_model(curr_state, u, dt)

            pos_err2 = np.linalg.norm(curr_state[:2] - setpoint_state[:2])**2
            ang_err2 = self._angle_diff(curr_state[2], setpoint_state[2])**2

            cost = pos_err2 + 1.0 * ang_err2
            if match_time:
                total_time = np.sum(dts)
                cost += 1.0 * (total_time - T)**2  # penalty for time mismatch
            return cost

        # Initial guess
        pos_err = setpoint_state[:2] - state[:2]
        dist = np.linalg.norm(pos_err)
        theta_err = self._angle_diff(setpoint_state[2], state[2])
        # print("Initial position error:", pos_err, "Distance:", dist, "Theta error:", theta_err)

        if abs(theta_err) > 0.5:   # large heading error
            v0, w0 = 0.0, theta_err / T
        else:                      # small heading error
            v0, w0 = dist / T, 0.0

        v0 = np.clip(v0, self.action_space.low[0], self.action_space.high[0])
        w0 = np.clip(w0, self.action_space.low[1], self.action_space.high[1])

        v0 = 1.0
        w0 = np.pi/8

        controls_init = np.tile([v0, w0], N)
        dts_init = np.full(N, T / N)
        x0_vars = np.concatenate([controls_init, dts_init])
        print("Initial guess for optimization:", x0_vars)

        # Bounds
        control_bounds = [(-self.action_space.low[0], self.action_space.high[0]), (-self.action_space.low[1], self.action_space.high[1])] * N
        dt_bounds = [(0.01, T)] * N
        bounds = control_bounds + dt_bounds
        result = minimize(objective, x0_vars, bounds=bounds, method='L-BFGS-B', options={"maxiter": 200}, tol=1e-3)
        if not result.success:
            return [np.array([v0, w0])], [T]

        opt_controls = result.x[:2*N].reshape(N, 2)
        opt_dts = result.x[2*N:]
        return [np.array(c) for c in opt_controls], list(opt_dts)
    