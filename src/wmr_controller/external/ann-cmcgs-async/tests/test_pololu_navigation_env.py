import numpy as np
import unittest
import time

from tools.envs.navigation_envs.navigation_env_pololu import NavigationEnvPololu


class TestPololuNavigationEnv(unittest.TestCase):
    """Integration test: Pololu controller inside navigation gym env."""

    def test_controller_reaches_goal(self):
        # Deterministic setup
        env = NavigationEnvPololu(dt=0.1, sparse_reward=False, obstacle_mode="none", render_mode="human")
        obs, info = env.reset(seed=0)

        # Remove any randomly created obstacles for a clean test track
        if hasattr(env, "obstacles"):
            env.obstacles = []
        # Disable truncation checks (collision/time) to focus on controller tracking
        env.is_truncated = lambda state: False

        # Place robot at origin, facing +x; set a simple goal at (1, 0)
        env.agent.state = np.array([0.0, 0.0, 0.0, env.start_t], dtype=np.float32)
        env.goal_pos = np.array([1.0, 0.0], dtype=np.float32)

        # Build setpoint state: [x, y, theta, t]
        target_theta = 0.0
        setpoint_state = np.array([
            env.goal_pos[0],
            env.goal_pos[1],
            target_theta,
            env.agent.state[3] + 5.0,
        ], dtype=np.float32)

        # Run controller to generate actions
        action_list, dt_list = env.agent.controller_pololu(
            env.agent.state, setpoint_state, N=60
        )

        # Apply actions in the environment (truncation disabled above)
        obs, reward, terminated, truncated, info = env.multi_step(
            action_list, dt_list, render=True
        )

        # Validate: reached goal (truncation forced off)
        self.assertTrue(
            env.agent.is_finished(env.agent.state, setpoint_state, atol=env.atol, rtol=env.rtol),
            f"Agent did not reach goal. Final state: {env.agent.state}, goal: {setpoint_state}"
        )

        # Keep window open for 10 seconds to view trajectory
        print("Keeping render window open for 10 seconds...")
        time.sleep(10)
        env.close()



if __name__ == "__main__":
    unittest.main()
