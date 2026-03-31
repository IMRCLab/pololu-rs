# WMR Controller

Hardware-agnostic Model Predictive Control (MPC)  git@github.com:IMRCLab/wmr-simulator.git controller for wheeled mobile robots (unicycle) with motion capture feedback (TODO: implement abstraction layer to use with or with/out mocap)

## Overview

This package adapts the 

## Architecture

```
Motion Capture System (OptiTrack/Vicon/VRPN)
    ↓
motion_capture_tracking_node
    ↓ publishes to /poses (NamedPoseArray)
    ↓
wmr_controller_node (this package)
    ↓ publishes to /cmd_unicycle (Vector3: v, w, 0) 
    ↓
Hardware Interface (pololu/controller_interface)
    ↓
robot
```

### Data Flow

1. **Motion Capture Data Source**:
   - **Origin**: External motion capture system (OptiTrack Motive, Vicon, VRPN, or mock)
   - **Node**: `motion_capture_tracking_node` (from `motion_capture_tracking` package)
   - **Processing**: 
     - Connects to mocap hardware via `libmotioncapture`
     - Tracks rigid bodies using `librigidbodytracker`
     - Publishes robot poses with position (x, y, z) and orientation (quaternion) 
   - **Topic**: `/poses` (type: `motion_capture_tracking_interfaces/NamedPoseArray`)
   - **QoS**: BEST_EFFORT, KEEP_LAST(10)
   - **Frame**: World/mocap coordinate frame

2. **Controller (this package)**:
   - **Node**: `wmr_controller_node`
   - **Subscribes**: `/poses` for real-time robot pose feedback
   - **Control Loop** (10 Hz currently):
     ```
     1. Get true pose from mocap (x, y, θ from quaternion)
     2. Get true wheel speeds (use prev cmd values, because we do not support encoder reading logs to ROS yet)
     (3. estimate the pose with a Kalman Filter using the wheel speeds)
     4. Compute control action from ref state of trajectory:
        ur_cmd, ul_cmd = controller.compute(ref_state, pose_est, wheel_est)
     5. Convert wheel speeds to (v, w) unicycle model
     6. Publish command to /cmd_unicycle
     ```
   - **Publishes**: `/cmd_unicycle` (type: `geometry_msgs/Vector3`, x=v, y=w, z=0/unused)
   - **QoS**: BEST_EFFORT, KEEP_LAST(1) for minimal latency
   - **Controller**: MPC from wmr-simulator

3. **Hardware Interface** (separate package):
   - **Example**: `pololu_ros/controller_interface` XBox controller interface for Pololu 3pi+ 2040 robots
   - **Subscribes**: `/cmd_unicycle` with matching QoS
   - **Action**: Forwards (v, w) commands to physical robots via Crazy Radio

## Dependencies

### External (Git Submodule)
- **wmr-simulator**: Controller and Estimator implementations
  - Location: `deps/wmr-simulator/`
  - Repository: https://github.com/IMRCLab/wmr-simulator
  - Commit: 42b63da

### ROS 2 Packages
- `rclpy`: Python ROS 2 client library
- `geometry_msgs`: For Vector3 velocity commands
- `motion_capture_tracking_interfaces`: For NamedPoseArray mocap data

## Robot Parameters

Configured for Pololu 3pi+ 2040 robots:
- Wheel radius: 16.5 mm (0.0165 m)
- Base diameter: 85 mm (0.085 m)

Update these in `wmr_controller_node.py` for different robots.

## Nodes

### wmr_controller_node

Main MPC controller with closed-loop feedback adapted from git@github.com:IMRCLab/wmr-simulator.git

**Parameters**:
- `robot_name` (string, default: "cf10"): Name of the robot to control

**Subscribes**:
- `/poses` (motion_capture_tracking_interfaces/NamedPoseArray): Robot poses from mocap

**Publishes**:
- `/cmd_unicycle` (geometry_msgs/Vector3): Velocity commands (x=v, y=w)

**Control Frequency**: 10 Hz (100 ms intervals)

### reference_publisher_node

Open-loop reference trajectory publisher (example/testing).

**Publishes**:
- `/cmd_unicycle` (geometry_msgs/Vector3): Circular trajectory commands

**Subscribes** (for monitoring):
- `/poses` (motion_capture_tracking_interfaces/NamedPoseArray): Robot poses

**Control Frequency**: 10 Hz

### timing_monitor

Diagnostic tool for monitoring command timing.

**Subscribes**:
- `/cmd_unicycle` (geometry_msgs/Vector3): Commands to monitor

**Output**: Statistics every 5 seconds (avg/min/max intervals, std dev, Hz)

## Building

```bash
cd /path/to/workspace
colcon build --packages-select wmr_controller
source install/setup.bash
```


## Usage
TODO
point to robot firmware with support for control action execution

## ROS 2 to Robot Architecture
TODO


## Debugging
- make sure the robotname in wmr_controller_launch.py matches exactly that in the broadcasted mocap pose!!