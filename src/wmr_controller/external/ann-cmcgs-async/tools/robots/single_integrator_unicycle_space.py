import numpy as np
from typing import Optional
from gymnasium import Space, spaces
from gymnasium.spaces import Box
import logging
from scipy.optimize import minimize
import casadi as ca


import dubins

from .robot import Robot
from .single_integrator_unicycle import SingleIntegratorUnicycle


logger = logging.getLogger(__name__)

class SingleIntegratorUnicycleSpace(SingleIntegratorUnicycle):
    def __init__(self, id, name, state_space: Optional[spaces.Box] = None, action_space: Optional[Space] = None, state=None, dt: float = 1.0, max_velocity=1.0, max_angular=1.0, color="#FFFF00", state_goal=None, action_sampler_method='uniform', atol=0.1, rtol=0.1, seed: Optional[int] = None, default_step_divisor: int = 10):
        """Initialize the single integrator robot."""
        logger.info("Starting initialization of SingleIntegratorUnicycleSpace")


        super().__init__(id=id, name=name, state_space=state_space, action_space=action_space, state=state, dt=dt, color=color, max_velocity=max_velocity, max_angular=max_angular, state_goal=state_goal, action_sampler_method=action_sampler_method, atol=atol, rtol=rtol, seed=seed, default_step_divisor=default_step_divisor)

        logger.info("Finished initialization of SingleIntegratorUnicycleSpace")


    def encode_state(self, state):
        """Transform the state to a vector representation of spacetime distance. The heuristic must be optimistic as it is supposed to be used for radius search before actually querying the controller."""
        # print("Encoding state:", state)
        x, y, theta, t = state
        pos_transformed = np.array([x, y]) / (self.dt * self.max_velocity)
        theta_transformed = np.array([np.sin(theta), np.cos(theta)]) / (2 * np.sin(self.dt * self.max_angular / 2)) # Embedding so that the euclidean distance corresponds to minimum angular difference
        features = [*pos_transformed, *theta_transformed]
        return np.array(features)
    
    def action_sampler_bang_bang(self, scale_along=1.0):
        """Sample action using bang-bang control."""
        v = self.action_space.high[0] * scale_along
        omega = self.np_random.choice([self.action_space.low[1], self.action_space.high[1]]) * scale_along
        return np.array([v, omega])

    def controller(self, state, setpoint_state, match_time: bool = True, N: int = 3, T: Optional[float] = None, step_divisor: Optional[int] = None) -> tuple[list, list]:
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
