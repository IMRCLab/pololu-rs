import unittest
import logging

import gymnasium as gym
import numpy as np

from gymnasium.utils.env_checker import check_env


logger = logging.getLogger(__name__)



class TestGymEnvs(unittest.TestCase):

    def setUp(self):
        try:
            gym.register(
                id="gymnasium_env/FrozenLakeContinuous-v0",
                entry_point="tools.envs.frozen_lake_continuous_wrapper:FrozenLakeSingleRobotEnv",
            )
        except Exception as e:
            print(f"Setup failed: {e}")
        pass

    def make(self):
        self.env = gym.make("gymnasium_env/FrozenLakeContinuous-v0", 
               render_mode="human",
               size=20,
               n_puddles=8,
               n_obstacles=0,
               dt=0.1,
               max_velocity=1.0,
               slippery_everywhere=0.0)
        self.env.reset()

    def test_env(self):
        self.setUp()

        self.make()

        passed = False
        try:
            check_env(self.env.unwrapped)
            print("Environment passes all checks!")
            passed = True
        except Exception as e:
            print(f"Environment has issues: {e}")
        self.assertTrue(passed)


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    unittest.main()
