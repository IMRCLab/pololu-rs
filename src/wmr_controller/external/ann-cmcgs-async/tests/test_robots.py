import unittest
import logging
import os

import gymnasium as gym
from gymnasium.spaces import Box
import numpy as np

from tools import robots
from tools.utils.visualization import (
    plot_trajectory_2d,
    plot_trajectory_1d,
    plot_unicycle_trajectory, 
    plot_tracking_error,
    plot_tracking_error_1d,
    create_trajectory_visualization
)

logger = logging.getLogger(__name__)

class TestRobots(unittest.TestCase):
    """Unit tests for the robots dynamics and controllers."""
    
    def setUp(self):
        # Create output directory for test visualizations
        self.output_dir = "logs/tests/visualizations"
        os.makedirs(self.output_dir, exist_ok=True)
    
    def _assert_box_space_bounds(self, space, expected_low, expected_high):
        """Helper method to assert Box space bounds with proper type checking."""
        self.assertIsInstance(space, Box)
        if len(expected_low.shape) == 0 or expected_low.shape[0] <= 4:
            self.assertTrue(np.allclose(space.low, expected_low))
            self.assertTrue(np.allclose(space.high, expected_high))
        else:
            self.assertTrue(np.array_equal(space.low, expected_low))
            self.assertTrue(np.array_equal(space.high, expected_high))
    
    def test_baseline_double_integrator(self):
        robot = robots.BaselineDoubleIntegrator(
            id=0,
            name="TestDoubleIntegrator",
            state_space=gym.spaces.Box(low=np.array([-np.inf, -1.0, 0.0]), high=np.array([np.inf, 1.0, np.inf]), dtype=np.float32),  # [x, vx, t]
            action_space=gym.spaces.Box(low=np.array([-1.0]), high=np.array([1.0]), dtype=np.float32),  # [ax]
            dt=1.0,
            max_velocity=1.0,
            max_acceleration=1.0,
            action_sampler_method='uniform',
            color="#00FF00",
            atol=0.1,
            rtol=0.1
        )
        self.assertIsNotNone(robot)
        self.assertEqual(robot.id, 0)
        self.assertEqual(robot.name, "TestDoubleIntegrator")
        self._assert_box_space_bounds(robot.state_space, np.array([-np.inf, -1.0, 0.0]), np.array([np.inf, 1.0, np.inf]))
        self._assert_box_space_bounds(robot.action_space, np.array([-1.0]), np.array([1.0]))
        self.assertEqual(robot.dt, 1.0)
        self.assertEqual(robot.max_velocity, 1.0)
        self.assertEqual(robot.max_acceleration, 1.0)
        self.assertEqual(robot.action_sampler_method, 'uniform')
        self.assertEqual(robot.color, "#00FF00")
        self.assertEqual(robot.atol, 0.1)
        self.assertEqual(robot.rtol, 0.1)

        # Test state update
        initial_state = np.array([0.0, 0.0, 0.0])  # [y=0.0, vy=0.0, t=0.0]

        action = np.array([1.0])  # [u=1.0]
        next_state = robot.transition_model(initial_state, action)
        expected_next_state = np.array([0.0, 1.0, 1.0])  # [y=1.0, vy=1.0, t=1.0]

        self.assertTrue(robot.is_finished(next_state, expected_next_state), f"Expected {expected_next_state}, got {next_state}")
        print("BaselineDoubleIntegrator state update test passed.")

        # Test controller
        # Single step
        action_list, dt_list = robot.controller(state=initial_state, setpoint_state=expected_next_state, match_time=True)
        controller_state = robot.multi_step_transition_model(state=initial_state, action_list=action_list, dt_list=dt_list)
        self.assertTrue(robot.is_finished(controller_state, expected_next_state), f"Controller single step failed. Expected {expected_next_state}, got {controller_state}")
        print("Single step controller test passed.")

        # Multi step
        tracking_actions = [np.array([0.5]), np.array([1.0]), np.array([0.0])]
        tracking_dts = [1.0, 1.0, 1.0]
        expected_tracking_state = robot.multi_step_transition_model(state=initial_state, action_list=tracking_actions, dt_list=tracking_dts)
        controller_actions, controller_dts = robot.controller(state=initial_state, setpoint_state=expected_tracking_state, match_time=True)
        controller_tracking_state = robot.multi_step_transition_model(state=initial_state, action_list=controller_actions, dt_list=controller_dts)
        
        # Generate visualization for controller tracking
        trajectory_states = [initial_state]
        current_state = initial_state.copy()
        for action, dt in zip(controller_actions, controller_dts):
            current_state = robot.transition_model(current_state, action, dt)
            trajectory_states.append(current_state.copy())
        
        # Generate target trajectory for comparison
        target_trajectory = [initial_state]
        current_target = initial_state.copy()
        for action, dt in zip(tracking_actions, tracking_dts):
            current_target = robot.transition_model(current_target, action, dt)
            target_trajectory.append(current_target.copy())
        
        # Create 1D visualizations for baseline double integrator
        plot_trajectory_1d(trajectory_states, "Controller Trajectory Tracking", 
                           "baseline_double_integrator_trajectory.png", "BaselineDoubleIntegrator", 
                           output_dir=self.output_dir, setpoint_states=target_trajectory, actions=controller_actions)
        
        # For 1D robots, we can still use tracking error but need to adapt it
        plot_tracking_error_1d(trajectory_states, target_trajectory, "Controller Tracking Error",
                              "baseline_double_integrator_tracking_error.png", "BaselineDoubleIntegrator", 
                              output_dir=self.output_dir)
        
        self.assertTrue(robot.is_finished(controller_tracking_state, expected_tracking_state), f"Controller multi step failed. Expected {expected_tracking_state}, got {controller_tracking_state}")
        print(f"Multi step controller test passed: Initial state: {initial_state}, Expected tracking state: {expected_tracking_state}, Controller tracking state: {controller_tracking_state}")
        
        print("BaselineDoubleIntegrator: All tests passed.")

    def test_single_integrator(self):
        robot = robots.SingleIntegrator(
            id=0,
            name="TestSingleIntegrator",
            state_space=gym.spaces.Box(low=np.array([-10.0, -10.0, 0.0]), high=np.array([10.0, 10.0, np.inf]), dtype=np.float32),  # [x, y, t]
            action_space=gym.spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32),  # [dx, dy]
            dt=1.0,
            max_velocity=1.0,
            action_sampler_method='uniform',
            color="#00FF00",
            atol=0.1,
            rtol=0.1
        )
        self.assertIsNotNone(robot) 
        self.assertEqual(robot.id, 0)
        self.assertEqual(robot.name, "TestSingleIntegrator")
        self._assert_box_space_bounds(robot.state_space, np.array([-10.0, -10.0, 0.0]), np.array([10.0, 10.0, np.inf]))
        self._assert_box_space_bounds(robot.action_space, np.array([-1.0, -1.0]), np.array([1.0, 1.0]))
        self.assertEqual(robot.dt, 1.0)
        self.assertEqual(robot.max_velocity, 1.0)
        self.assertEqual(robot.action_sampler_method, 'uniform')
        self.assertEqual(robot.color, "#00FF00")
        self.assertEqual(robot.atol, 0.1)
        self.assertEqual(robot.rtol, 0.1)

        # Test state update
        initial_state = np.array([0.0, 0.0, 0.0])  # [x=0.0, y=0.0, t=0.0]

        action = np.array([1.0, 1.0])  # [dx=1.0, dy=1.0]
        next_state = robot.transition_model(initial_state, action)
        expected_next_state = np.array([1.0, 1.0, 1.0])  # [x=1.0, y=1.0, t=1.0]    

        self.assertTrue(robot.is_finished(next_state, expected_next_state), f"Expected {expected_next_state}, got {next_state}")
        print("SingleIntegrator state update test passed.")

        # Test controller
        # Single step
        action_list, dt_list = robot.controller(state=initial_state, setpoint_state=expected_next_state, match_time=True, N=1, step_divisor=1)
        controller_state = robot.multi_step_transition_model(state=initial_state, action_list=action_list, dt_list=dt_list, step_divisor=1)
        self.assertTrue(robot.is_finished(controller_state, expected_next_state), f"Controller single step failed. Expected {expected_next_state}, got {controller_state}")
        print("Single step controller test passed.")

        # Multi step
        tracking_actions = [np.array([0.5, 0.5]), np.array([1.0, 1.0]), np.array([0.0, 0.0])]
        tracking_dts = [1.0, 1.0, 1.0]
        expected_tracking_state = robot.multi_step_transition_model(state=initial_state, action_list=tracking_actions, dt_list=tracking_dts)
        controller_actions, controller_dts = robot.controller(state=initial_state, setpoint_state=expected_tracking_state, match_time=True)
        controller_tracking_state = robot.multi_step_transition_model(state=initial_state, action_list=controller_actions, dt_list=controller_dts)

        # Generate visualization for controller tracking
        trajectory_states = [initial_state]
        current_state = initial_state.copy()
        for action, dt in zip(controller_actions, controller_dts):
            current_state = robot.transition_model(current_state, action, dt)
            trajectory_states.append(current_state.copy())
        
        # Generate target trajectory for comparison
        target_trajectory = [initial_state]
        current_target = initial_state.copy()
        for action, dt in zip(tracking_actions, tracking_dts):
            current_target = robot.transition_model(current_target, action, dt)
            target_trajectory.append(current_target.copy())
        
        # Create visualizations
        plot_trajectory_2d(trajectory_states, "Controller Trajectory Tracking",
                           "double_integrator_trajectory.png", "DoubleIntegrator",
                           output_dir=self.output_dir, setpoint_states=target_trajectory, actions=controller_actions)

        plot_tracking_error(trajectory_states, target_trajectory, "Controller Tracking Error",
                           "double_integrator_tracking_error.png", "DoubleIntegrator",
                           output_dir=self.output_dir)
        
        self.assertTrue(robot.is_finished(controller_tracking_state, expected_tracking_state), f"Controller multi step failed. Expected {expected_tracking_state}, got {controller_tracking_state}")
        print(f"Multi step controller test passed: Initial state: {initial_state}, Expected tracking state: {expected_tracking_state}, Controller tracking state: {controller_tracking_state}")

        print("SingleIntegrator: All tests passed.")

    def test_single_integrator_unicycle(self):
        robot = robots.SingleIntegratorUnicycle(
            id=0,
            name="TestSingleIntegratorUnicycle",
            state_space=gym.spaces.Box(low=np.array([-10.0, -10.0, -np.pi, 0.0]), high=np.array([10.0, 10.0, np.pi, np.inf]), dtype=np.float64),  # [x, y, theta, t]
            action_space=gym.spaces.Box(low=np.array([0.0, -np.pi/4]), high=np.array([1.0, np.pi/4]), dtype=np.float64),  # [dr, dtheta]
            dt=1.0,
            max_velocity=1.0,
            max_angular=np.pi/4,
            action_sampler_method='uniform',
            color="#00FF00",
            atol=0.1,
            rtol=0.1,
            default_step_divisor=20
        )
        self.assertIsNotNone(robot)
        self.assertEqual(robot.id, 0)
        self.assertEqual(robot.name, "TestSingleIntegratorUnicycle")
        self._assert_box_space_bounds(robot.state_space, np.array([-10.0, -10.0, -np.pi, 0.0]), np.array([10.0, 10.0, np.pi, np.inf]))
        self._assert_box_space_bounds(robot.action_space, np.array([0.0, -np.pi/4]), np.array([1.0, np.pi/4]))
        self.assertEqual(robot.dt, 1.0)
        self.assertEqual(robot.max_velocity, 1.0)
        self.assertEqual(robot.max_angular, np.pi/4)
        self.assertEqual(robot.action_sampler_method, 'uniform')
        self.assertEqual(robot.color, "#00FF00")
        self.assertEqual(robot.atol, 0.1)
        self.assertEqual(robot.rtol, 0.1)

        # Test state update
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x=0.0, y=0.0, theta=0.0, t=0.0]

        action = np.array([1.0, np.pi/8], dtype=np.float64)  # [dr=1.0, dtheta=pi/4]
        assert robot.action_space.contains(action), f"Action {action} is out of bounds for action space {robot.action_space}"
        next_state = robot.transition_model(initial_state, action, step_divisor=1)
        expected_next_state = np.array([1.0, 0.0, np.pi/8, 1.0])  # [x=1.0, y=0.0, theta=pi/4, t=1.0]   

        self.assertTrue(robot.is_finished(next_state, expected_next_state), f"Expected {expected_next_state}, got {next_state}")
        print("SingleIntegratorUnicycle state update test passed.")

        expected_next_state = robot.transition_model(initial_state, robot.sample_action())
        # Test controller
        # Single step
        print("Testing controller from initial state:", initial_state, "to expected next state:", expected_next_state)
        action_list, dt_list = robot.controller(state=initial_state, setpoint_state=expected_next_state, match_time=True, N=10, step_divisor=1)
        print("Controller action_list:", action_list, "dt_list:", dt_list)
        controller_state = robot.multi_step_transition_model(state=initial_state, action_list=action_list, dt_list=dt_list, step_divisor=1)
         # Step divisor is 1 as the controller currently does not support step division internally
        self.assertTrue(robot.is_finished(controller_state, expected_next_state), f"Controller single step failed. Expected {expected_next_state}, got {controller_state}")
        print("Single step controller test passed.")

        # Multi step
        tracking_actions = [np.array([0.5, 0.5]), np.array([1.0,1.0]), np.array([0.0, 0.0])]
        tracking_dts = [1.0, 1.0, 1.0]
        expected_tracking_state = robot.multi_step_transition_model(state=initial_state, action_list=tracking_actions, dt_list=tracking_dts)
        controller_actions, controller_dts = robot.controller(state=initial_state, setpoint_state=expected_tracking_state, match_time=True, N=15, step_divisor=1)
        controller_tracking_state = robot.multi_step_transition_model(state=initial_state, action_list=controller_actions, dt_list=controller_dts, step_divisor=1)
        
        # Generate visualization for controller tracking
        trajectory_states = [initial_state]
        current_state = initial_state.copy()
        for action, dt in zip(controller_actions, controller_dts):
            current_state = robot.transition_model(current_state, action, dt)
            trajectory_states.append(current_state.copy())
        
        # Generate target trajectory for comparison
        target_trajectory = [initial_state]
        current_target = initial_state.copy()
        for action, dt in zip(tracking_actions, tracking_dts):
            current_target = robot.transition_model(current_target, action, dt)
            target_trajectory.append(current_target.copy())
        
        # Create visualizations
        plot_unicycle_trajectory(trajectory_states, "Controller Trajectory Tracking",
                                 "unicycle_trajectory.png", "Unicycle",
                                 output_dir=self.output_dir, setpoint_states=target_trajectory, actions=controller_actions)

        plot_tracking_error(trajectory_states, target_trajectory, "Controller Tracking Error",
                           "unicycle_tracking_error.png", "Unicycle",
                           output_dir=self.output_dir)
        
        self.assertTrue(robot.is_finished(controller_tracking_state, expected_tracking_state), f"Controller multi step failed. Expected {expected_tracking_state}, got {controller_tracking_state}")
        print(f"Multi step controller test passed: Initial state: {initial_state}, Expected tracking state: {expected_tracking_state}, Controller tracking state: {controller_tracking_state}")

        print("SingleIntegratorUnicycle: All tests passed.")

    def test_single_integrator_unicycle_broken_rudder(self):
        robot = robots.SingleIntegratorUnicycleBrokenRudder(
            id=0,
            name="TestSingleIntegratorUnicycleBrokenRudder",
            state_space=gym.spaces.Box(low=np.array([-10.0, -10.0, -np.pi, 0.0]), high=np.array([10.0, 10.0, np.pi, np.inf]), dtype=np.float32),  # [x, y, theta, t]
            action_space=gym.spaces.Box(low=np.array([0.0, 0.0]), high=np.array([1.0, np.pi/4]), dtype=np.float32),  # [dr, dtheta] with broken rudder (no negative angular velocity)
            dt=1.0,
            max_velocity=1.0,
            max_angular=np.pi/4,
            action_sampler_method='uniform',
            color="#00FF00",
            atol=0.1,
            rtol=0.1,
            default_step_divisor=20
        )
        self.assertIsNotNone(robot)
        self.assertEqual(robot.id, 0)
        self.assertEqual(robot.name, "TestSingleIntegratorUnicycleBrokenRudder")
        self._assert_box_space_bounds(robot.state_space, np.array([-10.0, -10.0, -np.pi, 0.0]), np.array([10.0, 10.0, np.pi, np.inf]))
        self._assert_box_space_bounds(robot.action_space, np.array([0.0, 0.0]), np.array([1.0, np.pi/4]))
        self.assertEqual(robot.dt, 1.0)
        self.assertEqual(robot.max_velocity, 1.0)
        self.assertEqual(robot.max_angular, np.pi/4)
        self.assertEqual(robot.action_sampler_method, 'uniform')
        self.assertEqual(robot.color, "#00FF00")
        self.assertEqual(robot.atol, 0.1)
        self.assertEqual(robot.rtol, 0.1)

        # Test state update
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x=0.0, y=0.0, theta=0.0, t=0.0]

        action = np.array([1.0, np.pi/4-0.5])  # [dr=1.0, dtheta=pi/4]
        next_state = robot.transition_model(initial_state, action, step_divisor=1)
        expected_next_state = np.array([1.0, 0.0, np.pi/4-0.5, 1.0])  # [x=1.0, y=0.0, theta=pi/4, t=1.0]

        self.assertTrue(robot.is_finished(next_state, expected_next_state), f"Expected {expected_next_state}, got {next_state}")
        print("SingleIntegratorUnicycleBrokenRudder state update test passed.")

        # Test controller
        # Single step
        action_list, dt_list = robot.controller(state=initial_state, setpoint_state=expected_next_state, match_time=True, N=1*5, step_divisor=1)
        controller_state = robot.multi_step_transition_model(state=initial_state, action_list=action_list, dt_list=dt_list, step_divisor=1)
        self.assertTrue(robot.is_finished(controller_state, expected_next_state), f"Controller single step failed. Expected {expected_next_state}, got {controller_state}")
        print("Single step controller test passed.")

        # Multi step
        tracking_actions = [np.array([0.5, 0.5]), np.array([1.0, 1.0]), np.array([0.0, 0.0])]
        tracking_dts = [1.0, 1.0, 1.0]
        expected_tracking_state = robot.multi_step_transition_model(state=initial_state, action_list=tracking_actions, dt_list=tracking_dts, step_divisor=1)
        controller_actions, controller_dts = robot.controller(state=initial_state, setpoint_state=expected_tracking_state, match_time=True, N=len(tracking_actions), step_divisor=1)
        controller_tracking_state = robot.multi_step_transition_model(state=initial_state, action_list=controller_actions, dt_list=controller_dts, step_divisor=1)

        # Generate visualization for controller tracking
        trajectory_states = [initial_state]
        current_state = initial_state.copy()
        for action, dt in zip(controller_actions, controller_dts):
            current_state = robot.transition_model(current_state, action, dt)
            trajectory_states.append(current_state.copy())
        
        # Generate target trajectory for comparison
        target_trajectory = [initial_state]
        current_target = initial_state.copy()
        for action, dt in zip(tracking_actions, tracking_dts):
            current_target = robot.transition_model(current_target, action, dt)
            target_trajectory.append(current_target.copy())
        
        # Create visualizations
        plot_unicycle_trajectory(trajectory_states, "Broken Rudder Controller Trajectory", 
                                 "broken_rudder_trajectory.png", "SingleIntegratorUnicycleBrokenRudder", 
                                 output_dir=self.output_dir, setpoint_states=target_trajectory, actions=controller_actions)
        
        plot_tracking_error(trajectory_states, target_trajectory, "Broken Rudder Tracking Error",
                           "broken_rudder_tracking_error.png", "SingleIntegratorUnicycleBrokenRudder",
                           output_dir=self.output_dir)
        
        self.assertTrue(robot.is_finished(controller_tracking_state, expected_tracking_state), f"Controller multi step failed. Expected {expected_tracking_state}, got {controller_tracking_state}")
        print(f"Multi step controller test passed: Initial state: {initial_state}, Expected tracking state: {expected_tracking_state}, Controller tracking state: {controller_tracking_state}")

        print("SingleIntegratorUnicycleBrokenRudder: All tests passed.")

    def test_single_integrator_unicycle_space(self):
        robot = robots.SingleIntegratorUnicycleSpace(
            id=0,
            name="TestSingleIntegratorUnicycleSpace",
            state_space=gym.spaces.Box(low=np.array([-np.inf, -np.inf, -np.pi, 0.0]), high=np.array([np.inf, np.inf, np.pi, np.inf]), dtype=np.float64),  # [x, y, theta, t]
            action_space=gym.spaces.Box(low=np.array([0.0, -np.pi]), high=np.array([1.0, np.pi]), dtype=np.float64),  # [dr, dtheta]
            dt=1.0,
            max_velocity=1.0,
            max_angular=np.pi,
            action_sampler_method='bang_bang',
            color="#00FF00",
            atol=0.1,
            rtol=0.1,
            default_step_divisor=100
        )

        self.assertIsNotNone(robot)
        self.assertEqual(robot.id, 0)
        self.assertEqual(robot.name, "TestSingleIntegratorUnicycleSpace")
        self._assert_box_space_bounds(robot.state_space, np.array([-np.inf, -np.inf, -np.pi, 0.0]), np.array([np.inf, np.inf, np.pi, np.inf]))
        self._assert_box_space_bounds(robot.action_space, np.array([0.0, -np.pi]), np.array([1.0, np.pi]))
        self.assertEqual(robot.dt, 1.0)
        self.assertEqual(robot.max_velocity, 1.0)
        self.assertEqual(robot.max_angular, np.pi)
        self.assertEqual(robot.action_sampler_method, 'bang_bang')
        self.assertEqual(robot.color, "#00FF00")
        self.assertEqual(robot.atol, 0.1)
        self.assertEqual(robot.rtol, 0.1)   


        # Test state update
        initial_state = np.array([0.0, 0.0, robot._wrap_to_state_space_angle(np.pi/2), 0.0])  # [x=0.0, y=0.0, theta=0.0, t=0.0]
        action = np.array([1.0, np.pi])  # [linear_velocity, angular_velocity]
        dt = 1.0
        updated_state = robot.transition_model(initial_state, action, dt, step_divisor=int(1e5))
        expected_state = np.array([-2/np.pi, 0.0, robot._wrap_to_state_space_angle(-np.pi/2), 1.0])  # two radii around the circle to the left, radius is v/w
        np.testing.assert_almost_equal(updated_state, expected_state, decimal=5)

        # Test controller
        action_list, dt_list = robot.controller(state=initial_state, setpoint_state=expected_state, match_time=False, N=10, T=robot.dt, step_divisor=1)  # Step divisor is 1 as the controller currently does not support step division internally
        print("Controller action_list:", action_list, "dt_list:", dt_list)
        controller_state = robot.multi_step_transition_model(state=initial_state, action_list=action_list, dt_list=dt_list, step_divisor=1)  # Step divisor is 1 as the controller currently does not support step division internally
        self.assertTrue(robot.is_finished(controller_state, expected_state), f"Controller failed. Expected {expected_state}, got {controller_state}")
        print("Controller test passed.")

if __name__ == '__main__':
    unittest.main()