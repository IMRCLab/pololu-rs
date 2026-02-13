import unittest
import logging
import yaml
import tempfile
import os

import gymnasium as gym
import numpy as np
import matplotlib.pyplot as plt

import tools.robots as robots
import tools.envs.navigation_envs as navigation_envs
import tools.planners as planners

from tools.utils.plot_graph import plot_graph_2d

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)

class TestPlanners(unittest.TestCase):

    def setUp(self):
        try:
            gym.register(
            id='NavigationEnvBaseline-v0',
            entry_point='tools.envs.navigation_envs.navigation_env_baseline:NavigationEnvBaseline',)
        except Exception as e:
            print(f"Setup failed: {e}")

        self.env = gym.make('NavigationEnvBaseline-v0', 
               render_mode="human",
               dt=1.0,
               atol=0.1,
               rtol=0.0,
               multi_step_count=3,
               obstacle_mode="None")
        self.env.reset()
        
        # Create MCGS Planner
        self.mcgs_planner = planners.MCGSPlanner(env=self.env.unwrapped,
                               budget_per_step=150,
                               computational_budget_total=2500,
                               expand_n_times=5,
                               kappa=1.3, # 1.6
                               alpha=0.8, # 0.3
                               k=32,
                               radius_threshold=np.inf, # 1.4,
                               progressive_widening_method='default',
                               abstraction_refinement_exponent=-0.0,
                               c_uct=0.8,
                               plan_in_space_time=True,
                               use_controller=True,
                               tracking_tolerance=0.9)
        
        # Create CMCGS Planner with configuration
        cmcgs_config = {
            'horizon': 10,
            'action_dim': self.env.action_space.shape[0],
            'population_size': 50,
            'elite_fraction': 0.2,
            'iterations': 5,
            'alpha': 0.1
        }
        
        # Create temporary config file for CMCGS
        self.temp_config_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
        yaml.dump(cmcgs_config, self.temp_config_file)
        self.temp_config_file.close()
        
        self.cmcgs_planner = planners.CMCGSWrapperPlanner(
            computational_budget_max=100,
            cmcgs_config=self.temp_config_file.name
        )
        
        # Create CEM Planner
        self.cem_planner = planners.CEMPlanner(
            env=self.env.unwrapped,
            planning_horizon=10,
            population_size=50,
            elite_fraction=0.2,
            noise=0.1,
            warm_start=True
        )
        
        # Set default planner for backwards compatibility
        self.planner = self.mcgs_planner
        self.planner.reset()
        print(self.planner.env.obstacles)
    
    def tearDown(self):
        # Clean up temporary config file
        if hasattr(self, 'temp_config_file'):
            try:
                os.unlink(self.temp_config_file.name)
            except FileNotFoundError:
                pass

        

    def test_expansion(self):
        self.setUp()
        self.planner.plan(iterations=50, expand_n_times=2)
        fig = plot_graph_2d(self.planner.graph, plot_labels=False, scatter_mode='Q', plot_values=False, bbox=self.planner.env.pos_limits, scatter_size=15, get_pos=self.planner.env._get_pos, get_t=self.planner.env._get_t, get_vel=self.planner.env._get_vel)
                      
        plt.show()
    
    def test_mcgs(self):
        pass

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    unittest.main()