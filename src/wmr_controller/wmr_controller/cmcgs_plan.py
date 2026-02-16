#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist, Vector3, Pose2D
from motion_capture_tracking_interfaces.msg import NamedPoseArray
import numpy as np
import sys
import os

# Add wmr-simulator scripts to path
# When installed, the scripts should be in share/wmr_controller/deps/wmr-simulator/scripts
import ament_index_python.packages
try:
    package_share_directory = ament_index_python.packages.get_package_share_directory('wmr_controller')
    wmr_sim_path = os.path.join(package_share_directory, 'deps/wmr-simulator/scripts')
except Exception:
    wmr_sim_path = None

# If not found in share (e.g. not installed yet or running from source differently), try relative path
if not wmr_sim_path or not os.path.exists(wmr_sim_path):
     # When running from source: src/wmr_controller/wmr_controller/wmr_controller_node.py
     # Target: src/wmr_controller/deps/wmr-simulator/scripts
     wmr_sim_path = os.path.join(os.path.dirname(__file__), '../deps/wmr-simulator/scripts')

if os.path.exists(wmr_sim_path):
    sys.path.insert(0, wmr_sim_path)
    # from controller import Controller  (Unused in plan)
else:
    print(f"Error: Could not find wmr-simulator scripts at {wmr_sim_path}")

# Add ann-cmcgs-async to path
ann_cmcgs_path = None
try:
    # Try finding in share (installed mode)
    package_share_directory = ament_index_python.packages.get_package_share_directory('wmr_controller')
    ann_cmcgs_path = os.path.join(package_share_directory, 'external/ann-cmcgs')
except Exception:
    pass

if not ann_cmcgs_path or not os.path.exists(os.path.join(ann_cmcgs_path, 'tools')):
    # Try relative to source file (dev mode)
    ann_cmcgs_path = os.path.join(os.path.dirname(__file__), '../external/ann-cmcgs')

if os.path.exists(ann_cmcgs_path):
    sys.path.insert(0, ann_cmcgs_path)

    
    # We prioritize the local path vs openai baseline
    sys.path.insert(0, ann_cmcgs_path)
    
    # Register envs
    import gymnasium as gym
    import tools.planners as planners
    from tools.planners import MCGSPlanner
    from tools.envs.navigation_envs.navigation_env_pololu import NavigationEnvPololu
    
    gym.register(
        id='NavigationEnvPololu-v0',
        entry_point='tools.envs.navigation_envs.navigation_env_pololu:NavigationEnvPololu',)
else:
    print(f"Error: Could not find ann-cmcgs-async at {ann_cmcgs_path}")


class ROSMCGSPlanner(MCGSPlanner):
    def __init__(self, node, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.node = node

    def is_executing(self):
        """Check if robot is still executing actions (driving to setpoint)."""
        if self.node.active_setpoint_state is None:
            return False
            
        if self.node.latest_pose is None:
            return False

        # Calculate distance to active setpoint
        sp_x = self.node.active_setpoint_state[0]
        sp_y = self.node.active_setpoint_state[1]
        curr_x = self.node.latest_pose.position.x
        curr_y = self.node.latest_pose.position.y
        dist = np.sqrt((sp_x - curr_x)**2 + (sp_y - curr_y)**2)
        
        return dist > self.node.plan_tolerance

    def receive_latest_state(self, use_hardware: bool = True):
        if use_hardware:
            # Determine start state for the planner
            
            # If we are executing, plan from the target we are driving to
            if self.is_executing():
                state = self.node.active_setpoint_state.copy()
                current_time = self.node.get_clock().now().nanoseconds / 1e9 - self.node.start_time
                state[3] = current_time
                return state

            # If not executing (idle/reached), plan from current robot pose
            if self.node.latest_pose is None:
                return np.zeros(4) 
            
            # Construct state [x, y, theta, time]
            x = self.node.latest_pose.position.x
            y = self.node.latest_pose.position.y
            
            q = self.node.latest_pose.orientation
            theta = np.arctan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
            
            current_time = self.node.get_clock().now().nanoseconds / 1e9 - self.node.start_time
            
            new_state = np.array([x, y, theta, current_time], dtype=np.float32)
            #self.node.get_logger().info(f"ROSMCGSPlanner: New state received from hardware: {new_state}")
            
            # --- CRITICAL FIX ------------------------------------
            # The base MCGSPlanner.update_planner() reads state directly from self.env.agent.state
            # It ignores the return value of receive_latest_state() when updating internal graph!
            # So we MUST update the environment state here manually.
            if hasattr(self.env, 'agent'):
                self.env.agent.state = new_state
            # -----------------------------------------------------

            return new_state
        else:
            return super().receive_latest_state(use_hardware=False)

    def publish_setpoint(self, setpoint_state, use_hardware: bool = True):
        self.node.get_logger().info(f"Published setpoint: {setpoint_state}")
        if use_hardware:
            # Update active setpoint
            self.node.active_setpoint_state = setpoint_state
            
            # Publish to ROS
            msg = Pose2D()
            msg.x = float(setpoint_state[0])
            msg.y = float(setpoint_state[1])
            msg.theta = float(setpoint_state[2])
            self.node.plan_pub.publish(msg)
            # self.node.get_logger().info(f"Published setpoint: {msg.x:.2f}, {msg.y:.2f}")
        else:
            super().publish_setpoint(setpoint_state, use_hardware=False)

class CMCGSPlanNode(Node):
    def __init__(self):
        super().__init__('cmcgs_plan')
        
        # Parameters
        self.declare_parameter('robot_name', 'Pololu08')
        self.declare_parameter('plan_tolerance', 0.05)  # 5cm tolerance for reaching waypoint
        #obstacles are a list of x1,z1,r1 x2,z2,r2 -> a list of circels
        self.declare_parameter('obstacles', [
            -0.25, -0.5, 0.245,     #P01
            0.0, 0.75, 0.245,       #P02
            0.5, 0.75, 0.245,       #P03
            -0.5, -0.25, 0.3        #P04
            ])
        # Manual goal setting
        self.declare_parameter('goal_x', 0.75)
        self.declare_parameter('goal_y', 1.75)
        self.declare_parameter('goal_radius', 0.05)

        self.robot_name = self.get_parameter('robot_name').value
        self.plan_tolerance = self.get_parameter('plan_tolerance').value
        obstacles_param = self.get_parameter('obstacles').value
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_radius = self.get_parameter('goal_radius').value
        
        # ROS Communication
        mocap_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.mocap_sub = self.create_subscription(
            NamedPoseArray,
            '/poses',
            self.poses_callback,
            mocap_qos
        )
        
        # Plan publisher - keep only the latest plan (size 1)
        self.plan_pub = self.create_publisher(Pose2D, 'plan', 1)

        # Planning State
        self.latest_pose = None
        self.initialized = False
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Receding Horizon State
        self.active_setpoint_state = None  # The setpoint we are currently driving to

        # Initialize Environment
        self.env = gym.make('NavigationEnvPololu-v0', 
               render_mode='human', # Enable rendering
               dt=1.0,
               atol=0.3,
               rtol=0.0,
               multi_step_count=10,
               obstacle_mode="circles",
               render_sleep=0.0,
               sparse_reward=True,
        )
        self.env.reset(seed=42)

        # --------------------------------------------------------------------------
        # MANUALLY OVERRIDE GOAL 
        # For real world experiments, we want a fixed goal, not a random one.
        # We access the underlying environment to set 'goal_pos' and 'goal_radius'.
        # --------------------------------------------------------------------------
        try:
             # Unwrap if necessary
            if hasattr(self.env, 'unwrapped'):
                env_unwrapped = self.env.unwrapped
            else:
                env_unwrapped = self.env
            
            # The environment expects goal_pos as a numpy array or list [x, y]
            # Use object.__setattr__ just in case it's a frozen dataclass or similar
            object.__setattr__(env_unwrapped, 'goal_pos', np.array([self.goal_x, self.goal_y]))
            object.__setattr__(env_unwrapped, 'goal_radius', self.goal_radius)

            # Fix for Observation Space Limits causing Truncation
            # Original limits were derived from hardcoded start/end pos [0,3] -> [9,4]
            # Robot is starting at [0,0] which is outside Y range [1, 5]
            
            # Create new expanded limits: [-5, 10, -5, 10]
            # Structure: [x_min, x_max, y_min, y_max]
            new_pos_limits = [-5.0, 10.0, -5.0, 10.0]
            object.__setattr__(env_unwrapped, 'pos_limits', new_pos_limits)

            # Re-create observation space with new limits
            # State: [x, y, theta, time, goal_x, goal_y]
            # Assuming NavigationEnvPololu structure
            if isinstance(env_unwrapped, gym.Env):
                 # Manually construct box based on known structure from NavigationEnvPololu
                 low_obs = np.array([new_pos_limits[0], new_pos_limits[2], -np.pi, 0.0, new_pos_limits[0], new_pos_limits[2]], dtype=np.float32)
                 high_obs = np.array([new_pos_limits[1], new_pos_limits[3], np.pi, np.inf, new_pos_limits[1], new_pos_limits[3]], dtype=np.float32)
                 
                 new_obs_space = gym.spaces.Box(low=low_obs, high=high_obs, dtype=np.float32)
                 object.__setattr__(env_unwrapped, 'observation_space', new_obs_space)
                 self.get_logger().info(f"Expanded environment observation space to {new_pos_limits}")

            self.get_logger().info(f"Manually set Goal: ({self.goal_x}, {self.goal_y}) Radius: {self.goal_radius}")
        except Exception as e:
            self.get_logger().error(f"Failed to manually set goal/limits: {e}")

        # Update obstacles in environment
        if obstacles_param:
            self.update_environment_obstacles(obstacles_param)

        # Initialize Planner
        computational_budget_max = 500
        time_budget_max = 2.0
        
        self.planner = ROSMCGSPlanner(
            node=self,
            env=self.env.unwrapped,
            budget_per_step=1,
            computational_budget_max=computational_budget_max,
            time_budget_max=time_budget_max,
            expand_n_times=1,
            sample_best_X_actions=5,
            kappa=0.5,
            alpha=0.2,
            k=30,
            radius_threshold=1.0,
            progressive_widening_method="dispersion",
            epsilon=0.1,
            c_uct=0.5,
            plan_in_space_time=False,
            use_controller=True,
            tracking_tolerance=0.5,
            yield_mode='N',
            random_rollout_n_times=1,
            random_rollout_length=4,
            force_exact_update=False,
            controller_expansion_check=False
        )
        # Bootstrap planning
        try:
             self.planner.reset() # Important to init the graph
             # self.planner.plan(iterations=20) 
        except Exception as e:
             self.get_logger().error(f"Error init planner: {e}")

        # Timer for planning loop (10Hz)
        self.timer = self.create_timer(0.1, self.planning_loop)
        
        self.get_logger().info(f'CMCGS Planner started for robot: {self.robot_name}')

    def update_environment_obstacles(self, obstacles_list):
        """Update obstacles in the environment. Expects list [x1, y1, r1, x2, y2, r2...]"""
        if not obstacles_list or len(obstacles_list) % 3 != 0:
            self.get_logger().warn("Obstacles parameter must be a list of floats length multiple of 3 [x, y, r, ...]")
            return
            
        new_obstacles = []
        for i in range(0, len(obstacles_list), 3):
            x = float(obstacles_list[i])
            y = float(obstacles_list[i+1])
            r = float(obstacles_list[i+2])
            new_obstacles.append([x, y, r])
            
        try:
            # Try to get the underlying environment if it's wrapped
            if hasattr(self.env, 'unwrapped'):
                env = self.env.unwrapped
            else:
                env = self.env
            
            # Use setattr to bypass any read-only properties if necessary
            object.__setattr__(env, 'obstacles', new_obstacles)
            self.get_logger().info(f"Updated environment with {len(new_obstacles)} obstacles")
        except Exception as e:
            self.get_logger().error(f"Failed to update obstacles: {e}")

    def planning_loop(self):
        if not self.initialized or self.latest_pose is None:
            self.get_logger().warn('Waiting for first mocap pose...', throttle_duration_sec=2.0)
            return

        # Continuous planning loop
        # The ROSMCGSPlanner.plan_online() method handles:
        # 1. Checking if we are executing (using is_executing override)
        # 2. Publishing new setpoints if idle (using publish_setpoint override)
        # 3. Refining the plan while driving (using budget)
        
        try:
            self.planner.plan_online(
                computational=False,
                time=True,
                render=False, # Disable rendering to avoid blocking/overhead
                use_hardware=True 
            )
        except Exception as e:
            self.get_logger().error(f"Planning error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())

    def poses_callback(self, msg: NamedPoseArray):
        """Callback for motion capture poses"""
        for pose in msg.poses:
            if pose.name == self.robot_name:
                self.latest_pose = pose.pose
                # self.get_logger().info(f"Mocap Poses Callback update: {pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}")
                if not self.initialized:
                    self.initialized = True
                    self.get_logger().info('Planner received first pose')
                break

def main(args=None):
    rclpy.init(args=args)
    node = CMCGSPlanNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

