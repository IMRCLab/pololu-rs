import numpy as np
from typing import Optional
from gymnasium import Space, spaces
import logging
from scipy.optimize import minimize
import casadi as ca

from .robot import Robot
from .single_integrator import SingleIntegrator
from .single_integrator_unicycle import SingleIntegratorUnicycle


logger = logging.getLogger(__name__)

class SingleIntegratorUnicycleBrokenRudder(SingleIntegratorUnicycle):
    def __init__(self, id, name, state_space: Optional[spaces.Box] = None, action_space: Optional[spaces.Box] = None, state=None, dt: float = 1.0, max_velocity=1.0, max_angular=1.0, color="#FFFF00", state_goal=None, action_sampler_method='uniform', atol=0.1, rtol=0.1, seed: Optional[int] = None, default_step_divisor: int = 10, plan_in_space_time: bool = True):
        """Initialize the broken rudder robot."""
        logger.info("Starting initialization of SingleIntegratorUnicycleBrokenRudder")
        if state_space is None:
            # Use a default state space if none is provided
            state_space = spaces.Box(low=np.array([-np.inf, -np.inf, -np.pi, 0.0]),
                                     high=np.array([np.inf, np.inf, np.pi, np.inf]), dtype=np.float32)  # [x, y, theta, time]

        if action_space is None:
            # Use dr, dtheta for uniform sampling with max_velocity
            action_space = spaces.Box(low=np.array([-max_velocity, 0]), 
                                      high=np.array([max_velocity, max_angular]), dtype=np.float32)  # [dr, dtheta]

        super().__init__(id=id, name=name, state_space=state_space, action_space=action_space, state=state, dt=dt, max_velocity=max_velocity, max_angular=max_angular, color=color, state_goal=state_goal, action_sampler_method=action_sampler_method, atol=atol, rtol=rtol, seed=seed, default_step_divisor=default_step_divisor, plan_in_space_time=plan_in_space_time)

        self.reset(seed=seed, state=state, state_goal=state_goal)
        logger.info("Finished initialization of SingleIntegratorUnicycleBrokenRudder")


    def encode_state(self, state):
        """Transform the state to a vector representation of spacetime distance."""
        # print("Encoding state:", state)
        x, y, theta, t = state
        pos_transformed = np.array([x, y]) / (self.dt * self.max_velocity)
        theta_transformed = self._wrap_to_2pi_positive(theta) / (self.dt * self.max_angular)
        time_transformed = t / self.dt
        if self.plan_in_space_time:
            features = [*pos_transformed, theta_transformed, time_transformed]
        else:
            features = [*pos_transformed, theta_transformed]
        return np.array(features)


    def _angle_diff(self, theta, theta_setpoint):
        """Oriented angle difference because of broken rudder."""
        return (theta_setpoint - theta) % (2*np.pi)

    # def controller(self, state, setpoint_state, match_time:bool=True, N=3) -> tuple[list, list]:
    #     """
    #     Produces a sequence of controls and durations to track a setpoint state for a unicycle.

    #     Parameters:
    #         state: current state [x, y, theta, t]
    #         setpoint_state: desired state [x, y, theta, t]
    #         match_time: if True, penalize mismatch in time
    #         N: number of constant-control segments
    #     Returns:
    #         action_list: list of [v, w] controls
    #         dt_list: list of durations for each control
    #     """

    #     t0 = state[-1]
    #     tf = setpoint_state[-1]
    #     T = tf - t0

    #     if T <= 0:
    #         return [np.array([0.0, 0.0])], [0.0]

    #     def objective(vars_):
    #         controls = vars_[:2*N].reshape(N, 2)
    #         dts = vars_[2*N:]
    #         curr_state = state.copy()

    #         for u, dt in zip(controls, dts):
    #             curr_state = self.transition_model(curr_state, u, dt)

    #         pos_err2 = (curr_state[0] - setpoint_state[0])**2 + (curr_state[1] - setpoint_state[1])**2
    #         ang_err2 = self._angle_diff(curr_state[2],  setpoint_state[2])**2

    #         cost = pos_err2 + 2.0 * ang_err2
    #         if match_time:
    #             total_time = np.sum(dts)
    #             cost += 10.0 * (total_time - T)**2  # penalty for time mismatch
    #         return cost

    #     # Initial guess
    #     pos_err = setpoint_state[:2] - state[:2]
    #     dist = np.linalg.norm(pos_err)
    #     v0 = dist / T if T > 0 else 0.0
    #     theta_err = (setpoint_state[2] - state[2]) % (2*np.pi)  # always positive
    #     w0 = theta_err / T if T > 0 else 0.0
    #     v0 = np.clip(v0, self.action_space.low[0], self.action_space.high[0])
    #     w0 = np.clip(w0, self.action_space.low[1], self.action_space.high[1])

    #     controls_init = np.tile([v0, w0], N)
    #     dts_init = np.full(N, T / N)
    #     x0_vars = np.concatenate([controls_init, dts_init])

    #     # Bounds
    #     control_bounds = [(-self.max_velocity, self.max_velocity), (0.0, self.max_angular)] * N
    #     dt_bounds = [(0.01, T)] * N
    #     bounds = control_bounds + dt_bounds

    #     result = minimize(objective, x0_vars, bounds=bounds, method='L-BFGS-B', options={"maxiter": 2000}, tol=self.atol)

    #     if not result.success:
    #         return [np.array([v0, w0])], [T]

    #     opt_controls = result.x[:2*N].reshape(N, 2)
    #     opt_dts = result.x[2*N:]
    #     return [np.array(c) for c in opt_controls], list(opt_dts)
    
    
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
        
        # Check if turn is too large for max angular velocity
        angle_diff = self._angle_diff(state[2], setpoint_state[2])  # Oriented angle difference
        min_time_for_turn = angle_diff / self.max_angular
        if T < min_time_for_turn - 1e-3:  # small tolerance
            # Cannot make the turn in the given time
            logger.warning(f"Controller warning: Cannot achieve desired orientation change of {angle_diff:.2f} rad in {T:.2f} s with max angular velocity {self.max_angular:.2f} rad/s. Minimum time required is {min_time_for_turn:.2f} s. Returning zero control.")
            return [np.array([0.0, 0.0])], [T]


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

        # Cost: position + orientation error (adapted for broken rudder)
        pos_err2 = (final_state[0] - setpoint_state[0])**2 + (final_state[1] - setpoint_state[1])**2
        # For broken rudder: use oriented angle difference (can only turn one way)
        # Use CasADi-compatible modulo operation: fmod for floating point remainder
        angle_diff_raw = setpoint_state[2] - final_state[2]
        angle_diff = ca.fmod(angle_diff_raw + 2*ca.pi, 2*ca.pi)  # Ensure positive angle
        ang_err2 = angle_diff**2
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

        # Initial guess (adapted for broken rudder)
        dist = np.linalg.norm(setpoint_state[:2] - state[:2])
        v0 = np.clip(dist / T, self.action_space.low[0], self.action_space.high[0])
        
        # For broken rudder: use oriented angle difference (only positive turns)
        angle_diff = self._angle_diff(state[2], setpoint_state[2])  # Note: order swapped for proper orientation
        if angle_diff < 1e-3:  # already aligned
            w0 = 0.0
        else:
            w0 = np.clip(angle_diff / T, self.action_space.low[1], self.action_space.high[1])
        
        x0_guess = [v0] * N + [w0] * N + [T / N] * N

        # Solve
        nlp = {"x": vars_, "f": cost}
        solver = ca.nlpsol("solver", "ipopt", nlp,
                           {"ipopt.print_level": 0, "print_time": 0, "ipopt.max_iter": 500, "ipopt.tol": 1e-1})
        sol = solver(x0=x0_guess, lbx=lbx, ubx=ubx)
        sol_vars = np.array(sol["x"]).flatten()

        v_opt = sol_vars[:N]
        w_opt = sol_vars[N:2*N]
        dt_opt = sol_vars[2*N:]

        actions = [np.array([v_opt[i], w_opt[i]]) for i in range(N)]
        dts = list(dt_opt)
        print("Controller optimization success:", solver.stats()['return_status'])
        # print("Optimized actions:", actions, "dts:", dts)

        return actions, dts
