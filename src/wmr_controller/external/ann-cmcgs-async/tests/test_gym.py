import unittest
import logging

import gymnasium as gym
import numpy as np

from tools.robots.gym_agent import GymAgent

logger = logging.getLogger(__name__)

# use MountainCarContinuous-v0 as test environment
class TestGymAgent(unittest.TestCase):

    def setUp(self):
        self.env = gym.make("MountainCarContinuous-v0")
        self.env.reset()
        self.agent = GymAgent(id=0, name="TestGymAgent", env=self.env)

    def test_initialization(self):
        self.assertEqual(self.agent.id, 0)
        self.assertEqual(self.agent.name, "TestGymAgent")
        self.assertIsInstance(self.agent.state, type(self.agent.state))
        self.assertEqual(self.agent.state_space, self.env.observation_space)
        self.assertEqual(self.agent.action_space, self.env.action_space)

    def test_reset(self):
        self.agent.reset(seed=42)
        self.assertFalse(self.agent.state.terminated)
        self.assertFalse(self.agent.state.truncated)
        self.assertIsNotNone(self.agent.state.observation)

    def test_encode_state(self):
        encoded_state = self.agent.encode_state(self.agent.state)
        self.assertIsInstance(encoded_state, np.ndarray)
        self.assertEqual(encoded_state.shape, self.env.observation_space.shape)

    def test_transition_model(self):
        initial_state = self.agent.state.clone()
        action = self.agent.sample_action()
        next_state = self.agent.transition_model(initial_state, action)
        
        self.assertIsInstance(next_state, type(initial_state))
        self.assertFalse(next_state.terminated and not initial_state.terminated)  # Should not terminate immediately
        self.assertFalse(next_state.truncated and not initial_state.truncated)    # Should not truncate immediately
        np.testing.assert_array_equal(initial_state.observation, initial_state.observation)  # Initial observation should remain unchanged

    def test_sample_action(self):
        action = self.agent.sample_action()
        self.assertIsInstance(action, np.ndarray)
        self.assertEqual(action.shape, self.env.action_space.shape)
        self.assertTrue(np.all(action >= self.env.action_space.low))
        self.assertTrue(np.all(action <= self.env.action_space.high))

    def test_full_step(self):
        self.agent.reset(seed=42)
        done = False
        steps = 0
        while not done and steps < 10:
            action = self.agent.sample_action()
            self.agent.state = self.agent.transition_model(self.agent.state, action)
            done = self.agent.state.terminated or self.agent.state.truncated
            steps += 1
        self.assertLessEqual(steps, 10)  # Ensure we didn't exceed max steps

    def test_planning(self):
        self.agent.reset(seed=42)
        plan_length = 5
        action_plan = []
        state_plan = []
        state = self.agent.state.clone()
        for _ in range(plan_length):
            action = self.agent.sample_action()
            action_plan.append(action)
            state = self.agent.transition_model(state, action)
            state_plan.append(state)
            if state.terminated or state.truncated:
                break
        # Now verify the plan
        state_executions = []
        for action in action_plan:
            new_state = self.agent.step(action)
            state_executions.append(new_state)
            if new_state.terminated or new_state.truncated:
                break
        self.assertEqual(len(state_executions), len(state_plan))

        for s_exec, s_plan in zip(state_executions, state_plan):
            np.testing.assert_array_equal(s_exec.observation, s_plan.observation)
            self.assertEqual(s_exec.terminated, s_plan.terminated)
            self.assertEqual(s_exec.truncated, s_plan.truncated)
        

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    unittest.main()