"""
Visual tests for Pololu controller.

Generates plots showing controller behavior for various scenarios.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch, Circle
import logging

from tools.robots.pololu import Pololu


logger = logging.getLogger(__name__)


class PololuVisualTests:
    """Visual test suite for Pololu controller."""
    
    def __init__(self, output_dir: str = "logs/tests/visualizations"):
        """Initialize visual tests."""
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.robot = Pololu(
            id=0,
            name="VisualTestPololu",
            dt=3.0,
            max_velocity=1.0,
            max_angular=np.pi,
            seed=42
        )
    
    def _plot_robot(self, ax, state, color='blue', label=None, scale=0.1):
        """Helper to plot robot pose on axis."""
        x, y, theta = state[0], state[1], state[2]
        
        # Draw circle at position
        circle = Circle((x, y), scale/2, color=color, alpha=0.3, label=label)
        ax.add_patch(circle)
        
        # Draw arrow for orientation
        dx = scale * np.cos(theta)
        dy = scale * np.sin(theta)
        ax.arrow(x, y, dx, dy, head_width=scale/3, head_length=scale/3, 
                fc=color, ec=color, linewidth=2)
    
    def _plot_trajectory(self, initial_state, target_state, action_list, dt_list, 
                        title="", filename=""):
        """Helper to plot a trajectory with errors."""
        # Simulate trajectory
        trajectory = [initial_state.copy()]
        current_state = initial_state.copy()
        
        for action, dt in zip(action_list, dt_list):
            next_state = self.robot.transition_model(current_state, action, dt)
            trajectory.append(next_state.copy())
            current_state = next_state
        
        trajectory = np.array(trajectory)
        
        # Extract components
        x = trajectory[:, 0]
        y = trajectory[:, 1]
        theta = trajectory[:, 2]
        
        # Compute position errors
        target_x, target_y = target_state[0], target_state[1]
        pos_errors = np.sqrt((x - target_x)**2 + (y - target_y)**2)
        
        # Compute angle errors
        target_theta = target_state[2]
        angle_diffs = theta - target_theta
        angle_errors = np.arctan2(np.sin(angle_diffs), np.cos(angle_diffs))
        
        # Create figure with subplots
        fig = plt.figure(figsize=(14, 10))
        
        # Subplot 1: Trajectory in 2D space
        ax1 = plt.subplot(2, 3, 1)
        ax1.plot(x, y, 'b-', linewidth=2, label='Trajectory')
        ax1.plot(x[0], y[0], 'go', markersize=10, label='Start')
        ax1.plot(x[-1], y[-1], 'rs', markersize=10, label='End')
        ax1.plot(target_x, target_y, 'r*', markersize=20, label='Target')
        # Visualize target heading with an arrow at the goal pose
        self._plot_robot(ax1, target_state, color='orange', label='Target heading', scale=0.2)
        
        # Plot robot poses at intervals
        for i in range(0, len(trajectory), max(1, len(trajectory)//5)):
            self._plot_robot(ax1, trajectory[i], color=plt.cm.viridis(i/len(trajectory)))
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_title('Trajectory in 2D Space')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        ax1.axis('equal')
        
        # Subplot 2: Position error over time
        ax2 = plt.subplot(2, 3, 2)
        ax2.plot(pos_errors, 'b-', linewidth=2)
        ax2.axhline(y=self.robot.atol, color='g', linestyle='--', label=f'Tolerance ({self.robot.atol})')
        ax2.set_xlabel('Step')
        ax2.set_ylabel('Position Error (m)')
        ax2.set_title('Position Error vs Time')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # Subplot 3: Angle error over time
        ax3 = plt.subplot(2, 3, 3)
        ax3.plot(np.degrees(angle_errors), 'r-', linewidth=2)
        ax3.axhline(y=5.7, color='g', linestyle='--', label='Tolerance (≈5.7°)')
        ax3.set_xlabel('Step')
        ax3.set_ylabel('Angle Error (degrees)')
        ax3.set_title('Angle Error vs Time')
        ax3.grid(True, alpha=0.3)
        ax3.legend()
        
        # Subplot 4: Control inputs
        ax4 = plt.subplot(2, 3, 4)
        velocities = [a[0] for a in action_list]
        angular_vels = [a[1] for a in action_list]
        ax4.plot(velocities, 'b-', linewidth=2, label='Linear velocity')
        ax4.plot(angular_vels, 'r-', linewidth=2, label='Angular velocity')
        ax4.set_xlabel('Step')
        ax4.set_ylabel('Velocity (m/s, rad/s)')
        ax4.set_title('Control Inputs')
        ax4.grid(True, alpha=0.3)
        ax4.legend()
        
        # Subplot 5: X and Y position
        ax5 = plt.subplot(2, 3, 5)
        ax5.plot(x, 'b-', linewidth=2, label='X position')
        ax5.plot(y, 'g-', linewidth=2, label='Y position')
        ax5.axhline(y=target_x, color='b', linestyle='--', alpha=0.5, label=f'Target X={target_x}')
        ax5.axhline(y=target_y, color='g', linestyle='--', alpha=0.5, label=f'Target Y={target_y}')
        ax5.set_xlabel('Step')
        ax5.set_ylabel('Position (m)')
        ax5.set_title('Position Components')
        ax5.grid(True, alpha=0.3)
        ax5.legend()
        
        # Subplot 6: Orientation
        ax6 = plt.subplot(2, 3, 6)
        ax6.plot(np.degrees(theta), 'purple', linewidth=2, label='Actual')
        ax6.axhline(y=np.degrees(target_theta), color='orange', linestyle='--', 
                   label=f'Target {np.degrees(target_theta):.1f}°')
        ax6.set_xlabel('Step')
        ax6.set_ylabel('Angle (degrees)')
        ax6.set_title('Orientation')
        ax6.grid(True, alpha=0.3)
        ax6.legend()
        
        plt.suptitle(title, fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        if filename:
            filepath = os.path.join(self.output_dir, filename)
            plt.savefig(filepath, dpi=150, bbox_inches='tight')
            logger.info(f"Saved plot to {filepath}")
            print(f"✓ Saved: {filepath}")
        
        return fig
    
    def test_straight_line_tracking(self):
        """Test tracking a point straight ahead."""
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])
        target_state = np.array([2.0, 0.0, 0.0, np.inf])
        
        action_list, dt_list = self.robot.controller_pololu(
            initial_state, target_state, N=30
        )
        
        fig = self._plot_trajectory(
            initial_state, target_state, action_list, dt_list,
            title="Test 1: Straight Line Tracking (Forward)",
            filename="test_1_straight_line.png"
        )
        plt.show()
    
    def test_diagonal_tracking(self):
        """Test tracking a diagonal target."""
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])
        target_state = np.array([2.0, 1.5, np.pi/6, np.inf])
        
        action_list, dt_list = self.robot.controller_pololu(
            initial_state, target_state, N=40
        )
        
        fig = self._plot_trajectory(
            initial_state, target_state, action_list, dt_list,
            title="Test 2: Diagonal Tracking",
            filename="test_2_diagonal.png"
        )
        plt.show()
    
    def test_rotation_only(self):
        """Test rotating in place."""
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])
        target_state = np.array([0.0, 0.0, np.pi/2, np.inf])
        
        action_list, dt_list = self.robot.controller_pololu(
            initial_state, target_state, N=20
        )
        
        fig = self._plot_trajectory(
            initial_state, target_state, action_list, dt_list,
            title="Test 3: Rotation in Place (90°)",
            filename="test_3_rotation.png"
        )
        plt.show()
    
    def test_complex_path(self):
        """Test tracking a complex path with turns."""
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])
        target_state = np.array([3.0, 2.0, -np.pi/4, np.inf])
        
        action_list, dt_list = self.robot.controller_pololu(
            initial_state, target_state, N=50
        )
        
        fig = self._plot_trajectory(
            initial_state, target_state, action_list, dt_list,
            title="Test 4: Complex Path with Turns",
            filename="test_4_complex_path.png"
        )
        plt.show()
    
    def test_reverse_direction(self):
        """Test tracking backward."""
        initial_state = np.array([0.0, 0.0, np.pi, 0.0])  # Facing backward
        target_state = np.array([-2.0, 0.0, np.pi, np.inf])
        
        action_list, dt_list = self.robot.controller_pololu(
            initial_state, target_state, N=30
        )
        
        fig = self._plot_trajectory(
            initial_state, target_state, action_list, dt_list,
            title="Test 5: Backward Motion",
            filename="test_5_backward.png"
        )
        plt.show()
    
    def test_multiple_trajectories_comparison(self):
        """Compare controller behavior on multiple targets from same start."""
        initial_state = np.array([0.0, 0.0, 0.0, 0.0])
        
        targets = [
            (np.array([2.0, 0.0, 0.0, np.inf]), "Forward"),
            (np.array([0.0, 2.0, np.pi/2, np.inf]), "Left"),
            (np.array([2.0, 2.0, np.pi/4, np.inf]), "Diagonal"),
        ]
        
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        
        for idx, (target_state, label) in enumerate(targets):
            action_list, dt_list = self.robot.controller_pololu(
                initial_state, target_state, N=30
            )
            
            # Simulate trajectory
            trajectory = [initial_state.copy()]
            current_state = initial_state.copy()
            
            for action, dt in zip(action_list, dt_list):
                next_state = self.robot.transition_model(current_state, action, dt)
                trajectory.append(next_state.copy())
                current_state = next_state
            
            trajectory = np.array(trajectory)
            x, y = trajectory[:, 0], trajectory[:, 1]
            
            ax = axes[idx]
            ax.plot(x, y, 'b-', linewidth=2, label='Trajectory')
            ax.plot(x[0], y[0], 'go', markersize=10, label='Start')
            ax.plot(x[-1], y[-1], 'rs', markersize=10, label='End')
            ax.plot(target_state[0], target_state[1], 'r*', markersize=20, label='Target')
            
            # Plot robot at intervals
            for i in range(0, len(trajectory), max(1, len(trajectory)//5)):
                self._plot_robot(ax, trajectory[i], color=plt.cm.viridis(i/len(trajectory)))
            
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_title(f'Target: {label}')
            ax.grid(True, alpha=0.3)
            ax.legend()
            ax.axis('equal')
        
        plt.suptitle('Multiple Target Comparison', fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        filepath = os.path.join(self.output_dir, "test_6_comparison.png")
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        logger.info(f"Saved plot to {filepath}")
        print(f"✓ Saved: {filepath}")
        plt.show()
    
    def run_all_tests(self):
        """Run all visual tests."""
        print("\n" + "="*60)
        print("POLOLU CONTROLLER VISUAL TESTS")
        print("="*60 + "\n")
        
        tests = [
            ("Straight Line Tracking", self.test_straight_line_tracking),
            ("Diagonal Tracking", self.test_diagonal_tracking),
            ("Rotation in Place", self.test_rotation_only),
            ("Complex Path", self.test_complex_path),
            ("Backward Motion", self.test_reverse_direction),
            ("Multiple Targets Comparison", self.test_multiple_trajectories_comparison),
        ]
        
        for test_name, test_func in tests:
            try:
                print(f"\n▶ Running: {test_name}")
                test_func()
                print(f"✓ {test_name} completed")
            except Exception as e:
                logger.error(f"Error in {test_name}: {e}", exc_info=True)
                print(f"✗ {test_name} failed: {e}")
        
        print("\n" + "="*60)
        print(f"All tests completed! Results saved to: {self.output_dir}")
        print("="*60 + "\n")


if __name__ == '__main__':
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    visual_tests = PololuVisualTests()
    visual_tests.run_all_tests()
