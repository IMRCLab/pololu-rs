# Ground Robots ROS Workspace

This repository contains a ROS 2 workspace for ground robots.  
Some packages (e.g. [`crazyswarm2`](https://github.com/IMRCLab/crazyswarm2) and [`motion_capture_tracking`](https://github.com/IMRCLab/motion_capture_tracking)) are included as **git submodules**.

---

## How to Clone

Since this repository uses **submodules**, you must clone it recursively:

```bash
git clone --recurse-submodules -b workstation_ros <REPO_URL> <target_directory>
cd <target_directory>
```

If you already cloned without `--recurse-submodules`, run:
```bash
git submodule update --init --recursive
```

## How to Build
From the workspace root:
```bash
source /opt/ros/<ros_distro>/setup.bash
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
Source the environment before running any nodes:
```bash
source install/setup.bash
```

## Controller Interface

This workspace includes an Xbox controller interface for controlling multiple ground robots. The interface allows you to:
- Select and control individual robots (1-4)
- Choose different programs for each robot
- Send teleop commands to selected robots
- Start/stop robots individually or globally

### Hardware Requirements
- Xbox controller (wired or wireless with USB adapter)
- Compatible joystick device recognized by `joy_linux`

### Quick Start
1. **Connect your Xbox controller** to the system
2. **Launch the controller interface:**
   ```bash
   ros2 launch pololu controller_interface_launch.py
   ```

### Controller Mapping

#### Robot Selection & Program Management
- **D-Pad Left/Right**: Select robot (cycles through robots 1-4)
- **D-Pad Up/Down**: Select program for current robot
- **X-Button**: Confirm program selection and send to robot

#### Robot Control
- **A Button**: Start selected robot with chosen program
- **B Button**: Stop selected robot
- **X Button**: Send program to robot OR quit current program
- **START Button**: Start ALL robots simultaneously
- **BACK Button**: Stops ALL robots (emergency stop)

note: when stopping the robot, the program must be quit for each robot with the X button, before starting a program again.

#### Teleop Control
- **Left Stick**: Control selected robot when in teleop mode
  - Vertical axis: Linear velocity (forward/backward)
- **Right Stick**: Steering control
  - Horizontal axis: Steering angle (left/right)

### Available Programs
1. **Teleop**: Manual teleoperation mode (immediate control)
2. **Trajectory Following (Direct Duty)**: Follows pre-programmed trajectories with direct motor control
3. **Trajectory Following (MoCap)**: Follows trajectories using motion capture feedback (using position controller and speedcontrol)

### Usage Workflow
1. **Select Robot**: Use D-Pad Left/Right to choose which robot to control
2. **Choose Program**: Use D-Pad Up/Down to select the desired program
3. **Send Program**: Press X button to send the program to the robot
4. **Start Execution**: Press A button to start the program
5. **Control/Monitor**: 
   - For teleop: Use left stick to control the robot
   - For trajectories: Robot executes automatically
6. **Stop**: Press B to stop individual robot or BACK for emergency stop

### Safety Features
- Emergency stop functionality (BACK button stops all robots) - might be helpful when running multirobot experiments

### Configuration
Robot URIs and controller parameters can be configured in:
```
src/pololu_ros/config/controller_interface.yaml
```

## Notes on Submodules

- Submodules are pinned to specific commits to ensure reproducibility.
- If you need to update a submodule to a newer commit:

```bash
cd src/crazyswarm2
git checkout <new_commit_or_branch>
git pull
cd ../../
git add src/crazyswarm2
git commit -m "Update crazyswarm2 submodule"
```

Do the same for `motion_capture_tracking` if needed.

## Troubleshooting

### Controller Interface and Robot Issues

**Controller not detected:**
```bash
# Check if controller is recognized
ls /dev/input/js*
# Should show /dev/input/js0 or similar

# Test controller input
ros2 run joy joy_node
ros2 topic echo /joy
```

**No robot response:**
1. Check robot radio URIs in `controller_interface.yaml` Are the commands sent to the correct robot addresses?
2. Verify robots are flashed with correct firmware (to accept commands)
3. Could it be that the robot has no SD Card? Has the SD Card a trajectory (TR0001.JSN - use exactly THIS name)? Is the log full (capped at TR99)?
4. "Wild behavior" - Does the robot have proper mocap data? Does the robot have an SD Card with the right robot configuration on it (ROBOTCFG.CFG)?
5. Check ROS logs for connection errors:
   ```bash
   ros2 launch pololu controller_interface_launch.py --screen
   ```

**Build errors:**
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

**Submodule issues:**
```bash
# Reset all submodules and pull them again recursively
git submodule deinit --all
git submodule update --init --recursive
```
