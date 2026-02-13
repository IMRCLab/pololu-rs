"""Environment wrapper base class. Reference https://github.com/deepmind/acme/blob/e0d53ce3f8090b3520b24b2c585f6a6a2dfb799b/acme/wrappers/base.py"""

from typing import Callable, Sequence, Tuple, Dict, Any
import numpy as np
import gymnasium as gym


class EnvWrapper(gym.Env):
  """Environment that wraps another environment.
  This exposes the wrapped environment with the `.environment` property and also
  defines `__getattr__` so that attributes are invisibly forwarded to the
  wrapped environment (and hence enabling duck-typing).
  """

  _env: gym.Env

  def __init__(self, env: gym.Env):
    self._env = env

  def __getattr__(self, attr: str):
    # Delegates attribute calls to the wrapped environment.
    return getattr(self._env, attr)

  # Getting/setting of state is necessary so that getattr doesn't delegate them
  # to the wrapped environment. This makes sure pickling a wrapped environment
  # works as expected.

  def __getstate__(self):
    return self.__dict__

  def __setstate__(self, state):
    self.__dict__.update(state)

  @property
  def env(self) -> gym.Env:
    return self._env

  # The following lines are necessary because methods defined in
  # `dm_env.Environment` are not delegated through `__getattr__`, which would
  # only be used to expose methods or properties that are not defined in the
  # base `dm_env.Environment` class.

  def step(self, action) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
    return self._env.step(action)

  def reset(self) -> np.ndarray:
    return self._env.reset()

  @property
  def observation_space(self):
    return self._env.observation_space

  @property
  def action_space(self):
    return self._env.action_space

  def close(self):
    return self._env.close()
