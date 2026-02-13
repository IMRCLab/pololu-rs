import numpy as np
from typing import Callable, Optional
from abc import ABC
from time import perf_counter, sleep
import logging
import threading
logger = logging.getLogger(__name__)

from tools.robots.agent import Agent
from tools.envs.gym_robot_plan_env import GymRobotPlanEnv


class Planner(ABC):
    def __init__(self, env: GymRobotPlanEnv, computational_budget_max:int=2500, time_budget_max:float=20.0, save_replay: bool = True):
        """Initialize the planner."""
        self.env = env

        self.planned_trajectory = {
            'state_list': [],
            'action_list': [],
            'dt_list': [],
        }

        ### Telemetry / logging
        self.computational_budget_max = computational_budget_max
        self.computational_budget_total = 0
        self.computational_budget_used = 0

        self.time_budget_max = time_budget_max
        self.time_budget_total = 0.0
        self.time_budget_used = 0.0
        self.timer_start: float = None
        self.timer_end: float = None

        self.save_replay = save_replay
        self.action_replay = []
        self.dt_replay = []
        self.observation_replay = []
        self.plan_replay = []
        
        ### Execution tracking for concurrent operations
        self._execution_thread = None
        self._execution_complete = threading.Event()
        self._execution_complete.set()  # Initially no execution running
        self._state_lock = threading.Lock()  # Lock for thread-safe state updates in hardware mode

    def reset(self, seed: Optional[int] = None):
        """Reset the planner and the environment."""
        self.env.reset(seed=seed)
        self.planned_trajectory = {
            'state_list': [],
            'action_list': [],
            'dt_list': [],
        }
        ### Telemetry / logging
        self.computational_budget_used = 0
        self.timer_start = perf_counter()

    def plan(self):
        """Plan the trajectory for the agent."""
        raise NotImplementedError("Planning must be implemented in the subclass.")
    
    def plan_while_in_budget(self, computational: bool = True, time: bool = False):
        """Plan while within the computational budget or time budget."""
        logger.debug(f"Starting planning while in budget with time budget {self.time_budget_max} and computational budget {self.computational_budget_max}.")
        logger.debug(f"...using stopping criteria: computational={computational}, time={time}")
        self.computational_budget_used = 0
        self.time_budget_used = 0.0
        self.timer_start = perf_counter()
        while True:
            self.time_budget_used = perf_counter() - self.timer_start
            logger.debug(f"Time used: {self.time_budget_used:.4f}/{self.time_budget_max:.4f}")
            if time:
                # logger.debug(f"Time used: {self.time_budget_used:.4f}/{self.time_budget_max:.4f}")
                if self.time_budget_used >= self.time_budget_max:
                    logger.debug("Time budget exceeded, stopping planning.")
                    break
            if computational:
                logger.debug(f"Computational used: {self.computational_budget_used}/{self.computational_budget_max}")
                if self.computational_budget_used >= self.computational_budget_max:
                    break
            self.plan()
            # Yield CPU to allow concurrent execution thread to run
            sleep(0.001)
        self.time_budget_total += perf_counter() - self.timer_start
        self.computational_budget_total += self.computational_budget_used
        logger.info(f"Planning finished. Time used: {self.time_budget_used:.4f}/{self.time_budget_max:.4f}, Computational used: {self.computational_budget_used}/{self.computational_budget_max}")
        # sleep(1.0)  # Small sleep to ensure print statements complete

    def get_best_action(self) -> np.ndarray:
        """Yield the current planned trajectory."""
        raise NotImplementedError("Yielding the plan must be implemented in the subclass.")

    def plan_best_action(self, computational: bool = True, time: bool = False):
        """Plan while within the computational budget or time budget, then execute the first step from the plan."""
        self.plan_while_in_budget(computational=computational, time=time)
        best_action = self.get_best_action()
        
        return best_action

    def plan_and_step(self, computational: bool = True, time: bool = False, render: bool = False):
        """Plan while within the computational budget or time budget, then execute the first step from the plan."""
        best_action = self.plan_best_action(computational=computational, time=time)
        
        if self.save_replay:
            self.action_replay.append(best_action)
            self.dt_replay.append(self.env.agent.dt)
            self.observation_replay.append(self.env._get_obs())
            self.plan_replay.append(self.planned_trajectory)
        
        observation, reward, terminated, truncated, info = self.env.step(best_action)

        if render:
            self.render()
        return observation, reward, terminated, truncated, info

    def render(self, render_mode: Optional[str] = None):
        """Render the plan using the environment"""
        raise NotImplementedError("Rendering must be implemented in the subclass.")

    def receive_latest_state(self, use_hardware: bool = True):
        """Receive the latest state from hardware robot or simulator."""
        if use_hardware:
            raise NotImplementedError("Receiving latest state from hardware is not implemented yet.")
        else:
            logger.info(f"Receiving latest state from simulator: {self.env.agent.state}")
            new_state = self.env.agent.state
            # add noise
            # new_state = new_state + np.random.normal(0, 0.01, size=new_state.shape)
            return new_state
    
    def publish_setpoint(self, setpoint_state, use_hardware: bool = True):
        """Publish the setpoint to hardware robot or simulator (non-blocking)."""
        if use_hardware:
            raise NotImplementedError("Publishing setpoint to hardware is not implemented yet.")
            success, edge_pay = self._create_edge_with_controller(self.env.agent.state, setpoint_state)
            if success:
                action_list, dt_list = edge_pay.action_list, edge_pay.dt_list
                # Workaround: directly send actions here since proper hardware controller is not implemented yet
            else:
                logger.info("Controller returned zero total dt, failed to reach the setpoint.")
        else:
            logger.info(f"Published setpoint {setpoint_state}.")
            # query controller - to be implemented by subclass
            success, edge_pay = self._create_edge_with_controller(self.env.agent.state, setpoint_state)
            if success:
                action_list, dt_list = edge_pay.action_list, edge_pay.dt_list
                # Start execution in background thread
                self._execution_complete.clear()
                self._execution_thread = threading.Thread(
                    target=self._execute_actions_realtime,
                    args=(action_list, dt_list)
                )
                self._execution_thread.daemon = True
                self._execution_thread.start()
                logger.info(f"Started execution toward setpoint {setpoint_state}")
            else:
                logger.info("Controller returned zero total dt, failed to reach the setpoint.")
    
    def _execute_actions_realtime(self, action_list, dt_list):
        """Execute actions with real-time dt delays (runs in background thread)."""
        try:
            for action, dt in zip(action_list, dt_list):
                self.env.agent.step(action, dt=dt)
                sleep(dt)
            logger.info(f"Execution complete. New agent state: {self.env.agent.state}")
        finally:
            self._execution_complete.set()
    
    def is_executing(self):
        """Check if robot is still executing actions."""
        return self._execution_thread is not None and self._execution_thread.is_alive()
    
    def wait_for_completion(self, timeout: Optional[float] = None):
        """Block until execution completes. Returns True if completed, False if timeout."""
        return self._execution_complete.wait(timeout)
    
    def _create_edge_with_controller(self, start_state, setpoint_state):
        """Hook for subclasses to implement controller-based edge creation."""
        raise NotImplementedError("_create_edge_with_controller must be implemented in the subclass.")