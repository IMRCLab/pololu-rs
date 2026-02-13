import numpy as np
from typing import Optional
from gymnasium import Space, spaces
from gymnasium.spaces import Box
import logging
from scipy.optimize import minimize
import casadi as ca


import dubins

from .robot import Robot
from .single_integrator_unicycle_space import SingleIntegratorUnicycleSpace


logger = logging.getLogger(__name__)

class Pololu(SingleIntegratorUnicycleSpace):
    def __init__(self, id, name, state_space: Optional[spaces.Box] = None, action_space: Optional[Space] = None, state=None, dt: float = 1.0, max_velocity=1.0, max_angular=1.0, color="#FFFF00", state_goal=None, action_sampler_method='uniform', atol=0.1, rtol=0.1, seed: Optional[int] = None, default_step_divisor: int = 10, plan_in_space_time: bool = True):
        """Initialize the Pololu robot."""
        logger.info("Starting initialization of Pololu")
        if state_space is None:
            # Use a default state space if none is provided
            state_space = spaces.Box(low=np.array([-np.inf, -np.inf, -np.pi, 0.0]),
                                     high=np.array([np.inf, np.inf, np.pi, np.inf]), dtype=np.float32)  # [x, y, theta, time]

        if action_space is None:
            # Use dr, dtheta for uniform sampling with max_velocity
            action_space = spaces.Box(low=np.array([0.0, -max_angular]), 
                                      high=np.array([max_velocity, max_angular]), dtype=np.float32)  # [dr, dtheta]

        super().__init__(id=id, name=name, state_space=state_space, action_space=action_space, state=state, dt=dt, color=color, state_goal=state_goal, atol=atol, rtol=rtol, seed=seed, default_step_divisor=default_step_divisor)

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

    def controller(self, state, setpoint_state, match_time: bool = True, N: int = 5, T: Optional[float] = None, step_divisor: Optional[int] = None) -> tuple[list, list]:
            def check_success(action_list, dt_list):
                # print("Checking controller success with action_list:", action_list, "dt_list:", dt_list)
                return self.is_finished(self.multi_step_transition_model(state.copy(), action_list, dt_list, step_divisor=step_divisor), setpoint_state)
            
            # try default controller first
            # action_list, dt_list = self.controller_pololu(state, setpoint_state, N=N)
            action_list, dt_list = self.controller_casadi(state, setpoint_state, match_time=match_time, N=N, T=T, step_divisor=1)
            if check_success(action_list, dt_list):
                return action_list, dt_list
            else:  # try something else
                pass
            return action_list, dt_list


    def controller_pololu(self, state, setpoint_state, N: int = 5, T: Optional[float] = None, step_divisor: Optional[int] = None) -> tuple[list, list]:
        """
        Pololu-specific cascade controller implementation.
        Uses feedback linearization with proportional gains for tracking.
        """
        def low_level_controller(robot_state, setpoint):
            """
            Cascade control law for differential drive robot tracking a setpoint.
            
            Implements feedback linearization:
            1. Compute error in body frame
            2. Apply PD control to generate v, w
            3. Convert to wheel speeds
            
            Args:
                robot_state: [x, y, theta, time]
                setpoint: [x_des, y_des, theta_des, vdes, wdes] (or dict with keys 'x', 'y', 'theta', 'vdes', 'wdes')
            
            Returns:
                (ul, ur, xerror, yerror, therror) - left/right wheel speeds and errors
            """
            # Extract state
            x, y, theta, t = robot_state[0], robot_state[1], robot_state[2], robot_state[3]
            

            x_des, y_des, theta_des = setpoint[0], setpoint[1], setpoint[2]
            v_des, w_des = 0.0, 0.0
            
            # Controller gains (tuning parameters)
            kx = 1.0     # Proportional gain for x error
            ky = 1.0     # Proportional gain for y error
            kth = 1.0    # Proportional gain for theta error
            
            # Robot parameters
            l = 0.1      # Distance between wheels (half-width)
            r = 0.02     # Wheel radius
            
            # Compute error in body frame (rotation to body-centered coordinates)
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            dx = x_des - x
            dy = y_des - y
            
            xerror = cos_theta * dx + sin_theta * dy
            yerror = -sin_theta * dx + cos_theta * dy
            
            # Angle error (wrapped to [-pi, pi])
            therror = np.arctan2(np.sin(theta_des - theta), np.cos(theta_des - theta))
            
            # Compute desired feedforward velocity based on distance to target
            # This allows the robot to maintain motion while turning
            distance_to_target = np.sqrt(dx**2 + dy**2)
            
            # Feedforward velocity: scaled by distance, reduced if turning sharply
            if distance_to_target < 0.05:
                # Very close to target, move slowly
                v_des = 0.1 * distance_to_target
            else:
                # Farther away, move faster but scale down if angle error is large
                angle_alignment = np.cos(therror)  # 1 if aligned, 0 if perpendicular
                v_des = self.max_velocity * min(1.0, distance_to_target) * max(0.1, angle_alignment)
            
            # Feedback linearization control law
            # v = vdes * cos(therror) + kx * xerror
            # w = wdes + vdes * (ky * yerror + sin(therror) * kth)
            v = v_des * np.cos(therror) + kx * xerror
            w = w_des + v_des * (ky * yerror + np.sin(therror) * kth)
            
            # Convert to wheel speeds
            # ur = (2*v + l*w) / (2*r)
            # ul = (2*v - l*w) / (2*r)
            # ur = (2.0 * v + 1.0 * l * w) / (2.0 * r)
            # ul = (2.0 * v - 1.0 * l * w) / (2.0 * r)
            
            return v, w, xerror, yerror, therror
        
        # Generate action sequence using cascade controller
        # Use N steps to reach setpoint
        action_list = []
        dt_list = []
        current_state = state.copy()
        dt = self.dt / N
        for i in range(N):
            # Compute desired velocities to reach setpoint in remaining steps
            # Get control action
            v, w, xerror, yerror, therror = low_level_controller(current_state, setpoint_state)
            
            # Clip to max velocities
            v = np.clip(v, 0.0, self.max_velocity)
            w = np.clip(w, -self.max_angular, self.max_angular)
            
            action = np.array([v, w])
            action_list.append(action)
            dt_list.append(dt)
            
            # Simulate one step forward
            current_state = self.transition_model(current_state, action, dt)
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
