"""
Test suite for Pololu robot controller.

Tests the cascade controller implementation with feedback linearization
for differential drive robots.
"""

import unittest
import numpy as np
import logging
import os

from tools.robots.pololu import Pololu
from tools.utils.visualization import plot_unicycle_trajectory, plot_tracking_error


logger = logging.getLogger(__name__)


class TestPoluController(unittest.TestCase):
    """Test cases for Pololu cascade controller."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.output_dir = "logs/tests/visualizations"
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Create a Pololu robot instance
        self.robot = Pololu(
            id=0,
            name="TestPololu",
            dt=1.0,
            max_velocity=1.0,
            max_angular=np.pi,
            state_goal=None,
            atol=0.1,
            rtol=0.1,
            seed=42
        )
    
    def test_pololu_initialization(self):
        """Test that Pololu robot initializes correctly."""
        self.assertIsNotNone(self.robot)
        self.assertEqual(self.robot.name, "TestPololu")
        self.assertEqual(self.robot.dt, 1.0)
        self.assertEqual(self.robot.max_velocity, 1.0)
        self.assertEqual(self.robot.max_angular, np.pi)
        self.assertIsNotNone(self.robot.state)
        print("✓ Pololu initialization test passed")
    
    def test_controller_straight_line(self):
        """Test controller tracking straight ahead to a nearby point."""
        # Start at origin pointing right
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, theta, t]
        
        # Target: 1.0 unit ahead (straight line)
        target_state = np.array([1.0, 0.0, 0.0, np.inf])  # [x_des, y_des, theta_des, t_des]
        
        # Get controller actions
        action_list, dt_list = self.robot.controller_pololu(
            initial_state, target_state, N=10
        )
        
        # Verify we got actions
        self.assertGreater(len(action_list), 0, "Controller should generate actions")
        self.assertEqual(len(action_list), len(dt_list), "Action and dt lists should match")
        
        # Simulate trajectory
        trajectory = [initial_state.copy()]
        current_state = initial_state.copy()
        for action, dt in zip(action_list, dt_list):
            next_state = self.robot.transition_model(current_state, action, dt)
            trajectory.append(next_state.copy())
            current_state = next_state
        
        final_state = trajectory[-1]
        
        # Check that we got closer to target (x should increase)
        self.assertGreater(final_state[0], initial_state[0], 
                          "Robot should move forward")
        
        # Check that theta didn't change much (we're going straight)
        self.assertLess(abs(final_state[2] - initial_state[2]), 0.2,
                       "Angle should not change significantly for straight line")
        
        logger.info(f"Initial state: {initial_state}")
        logger.info(f"Final state: {final_state}")
        logger.info(f"Target state: {target_state[:3]}")
        print("✓ Straight line tracking test passed")
    
    def test_controller_turn_in_place(self):
        """Test controller rotating to face a different direction."""
        # Start at origin
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, theta, t]
        
        # Target: same position, rotated 90 degrees
        target_state = np.array([0.0, 0.0, np.pi/2, np.inf])
        
        # Get controller actions
        action_list, dt_list = self.robot.controller_pololu(
            initial_state, target_state, N=5
        )
        
        self.assertGreater(len(action_list), 0)
        
        # Simulate trajectory
        trajectory = [initial_state.copy()]
        current_state = initial_state.copy()
        for action, dt in zip(action_list, dt_list):
            next_state = self.robot.transition_model(current_state, action, dt)
            trajectory.append(next_state.copy())
            current_state = next_state
        
        final_state = trajectory[-1]
        
        # Check that position didn't change much
        self.assertLess(np.linalg.norm(final_state[:2] - initial_state[:2]), 0.2,
                       "Position should not change for in-place rotation")
        
        # Check that we rotated
        angle_diff = np.arctan2(np.sin(final_state[2] - initial_state[2]),
                               np.cos(final_state[2] - initial_state[2]))
        self.assertGreater(abs(angle_diff), 0.2,
                         "Robot should rotate significantly")
        
        logger.info(f"Initial angle: {initial_state[2]:.4f}")
        logger.info(f"Final angle: {final_state[2]:.4f}")
        logger.info(f"Target angle: {target_state[2]:.4f}")
        print("✓ In-place rotation test passed")
    
    def test_controller_diagonal_movement(self):
        """Test controller tracking to a diagonal position."""
        # Start at origin pointing right
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])
        
        # Target: diagonal (1, 1) pointing northeast
        target_state = np.array([1.0, 1.0, np.pi/4, np.inf])
        
        # Get controller actions
        action_list, dt_list = self.robot.controller_pololu(
            initial_state, target_state, N=15
        )
        
        self.assertGreater(len(action_list), 0)
        
        # Simulate trajectory
        trajectory = [initial_state.copy()]
        current_state = initial_state.copy()
        for action, dt in zip(action_list, dt_list):
            next_state = self.robot.transition_model(current_state, action, dt)
            trajectory.append(next_state.copy())
            current_state = next_state
        
        final_state = trajectory[-1]
        
        # Check that we moved diagonally
        self.assertGreater(final_state[0], initial_state[0], "X should increase")
        self.assertGreater(final_state[1], initial_state[1], "Y should increase")
        
        # Check that we're closer to target than initial
        initial_dist = np.linalg.norm(target_state[:2] - initial_state[:2])
        final_dist = np.linalg.norm(target_state[:2] - final_state[:2])
        self.assertLess(final_dist, initial_dist,
                       "Should move closer to target")
        
        logger.info(f"Initial distance to target: {initial_dist:.4f}")
        logger.info(f"Final distance to target: {final_dist:.4f}")
        logger.info(f"Initial pos: {initial_state[:2]}, Final pos: {final_state[:2]}, Target: {target_state[:2]}")
        print("✓ Diagonal movement test passed")
    
    def test_controller_returns_valid_actions(self):
        """Test that controller actions are within bounds."""
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])
        target_state = np.array([2.0, 2.0, np.pi/4, np.inf])
        
        action_list, dt_list = self.robot.controller_pololu(
            initial_state, target_state, N=20
        )
        
        # Check action bounds
        for i, action in enumerate(action_list):
            v, w = action
            self.assertGreaterEqual(v, self.robot.action_space.low[0],
                                  f"Action {i}: v={v} below lower bound")
            self.assertLessEqual(v, self.robot.action_space.high[0],
                               f"Action {i}: v={v} above upper bound")
            self.assertGreaterEqual(w, self.robot.action_space.low[1],
                                  f"Action {i}: w={w} below lower bound")
            self.assertLessEqual(w, self.robot.action_space.high[1],
                               f"Action {i}: w={w} above upper bound")
        
        # Check dt values
        for i, dt in enumerate(dt_list):
            self.assertGreater(dt, 0, f"dt[{i}] should be positive")
            self.assertLess(dt, 10.0, f"dt[{i}] should be reasonable")
        
        print("✓ Action bounds test passed")
    
    def test_controller_consistency(self):
        """Test that controller produces consistent outputs for same inputs."""
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])
        target_state = np.array([1.0, 0.5, np.pi/6, np.inf])
        
        # Run controller twice
        action_list_1, dt_list_1 = self.robot.controller_pololu(
            initial_state, target_state, N=5
        )
        action_list_2, dt_list_2 = self.robot.controller_pololu(
            initial_state, target_state, N=5
        )
        
        # Results should be identical (no randomness in cascade controller)
        self.assertEqual(len(action_list_1), len(action_list_2))
        for a1, a2 in zip(action_list_1, action_list_2):
            np.testing.assert_array_almost_equal(a1, a2, decimal=6)
        
        for dt1, dt2 in zip(dt_list_1, dt_list_2):
            self.assertAlmostEqual(dt1, dt2, places=6)
        
        print("✓ Consistency test passed")
    
    def test_controller_integration_with_base_class(self):
        """Test controller through base class interface."""
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])
        target_state = np.array([1.0, 1.0, 0.0, np.inf])
        
        # Call through public controller method
        action_list, dt_list = self.robot.controller(
            initial_state, target_state, N=10
        )
        
        self.assertGreater(len(action_list), 0)
        
        # Simulate
        current_state = initial_state.copy()
        for action, dt in zip(action_list, dt_list):
            current_state = self.robot.transition_model(current_state, action, dt)
        
        # Check we moved
        self.assertGreater(np.linalg.norm(current_state[:2] - initial_state[:2]), 0,
                         "Robot should have moved")
        
        print("✓ Base class integration test passed")
    
    def test_controller_edge_case_already_at_target(self):
        """Test controller when already at target."""
        state = np.array([0.0, 0.0, 0.0, 0.0])
        
        # Target is same as current state
        action_list, dt_list = self.robot.controller_pololu(
            state, state, N=5
        )
        
        self.assertGreater(len(action_list), 0)
        
        # Actions should be minimal (ideally zero or small)
        for action in action_list:
            v, w = action
            # Should move slowly since already at target
            self.assertLess(v, 0.5, "Velocity should be small when at target")
        
        print("✓ Already at target edge case test passed")


class TestPoluControllerTrajectory(unittest.TestCase):
    """Test Pololu controller with trajectory visualization."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.output_dir = "logs/tests/visualizations"
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.robot = Pololu(
            id=0,
            name="TestPololu",
            dt=0.1,
            max_velocity=1.0,
            max_angular=np.pi,
            seed=42
        )
    
    def test_trajectory_visualization(self):
        """Generate and save trajectory visualization."""
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])
        target_state = np.array([2.0, 1.0, np.pi/6, np.inf])
        
        # Get controller actions
        action_list, dt_list = self.robot.controller_pololu(
            initial_state, target_state, N=20
        )
        
        # Simulate trajectory
        trajectory_states = [initial_state.copy()]
        current_state = initial_state.copy()
        
        for action, dt in zip(action_list, dt_list):
            next_state = self.robot.transition_model(current_state, action, dt)
            trajectory_states.append(next_state.copy())
            current_state = next_state
        
        trajectory_array = np.array(trajectory_states)
        
        # Visualize
        try:
            fig = plot_unicycle_trajectory(
                trajectory_array,
                goal_state=target_state,
                title="Pololu Controller Trajectory"
            )
            fig.savefig(f"{self.output_dir}/pololu_trajectory.png", dpi=100, bbox_inches='tight')
            logger.info(f"Saved trajectory plot to {self.output_dir}/pololu_trajectory.png")
            print("✓ Trajectory visualization test passed")
        except Exception as e:
            logger.warning(f"Could not save trajectory visualization: {e}")
            print("⚠ Trajectory visualization test skipped (visualization not available)")


if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    unittest.main()
