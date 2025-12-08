<!--Here as well we could think of moving the troubleshooting to the troubleshooting section -->
# ROS Workspace Setup
This document explains how to build the ROS Workspace, the use cases for different nodes are described [here](usage.md).

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## How to Clone
The workstation_ros directory contains a ROS 2 workspace that uses **git submodules**. To clone it properly:

```bash
git clone --recurse-submodules -b workstation_ros https://github.com/IMRCLab/pololu3pi2040-rs.git <target_directory>
cd <target_directory>
```

If you already cloned without `--recurse-submodules`, run:
```bash
git submodule update --init --recursive
```

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

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

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Troubleshooting

---

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

1. Check robot radio URIs in `controller_interface.yaml` - Are the commands sent to the correct robot addresses?
2. Verify robots are flashed with correct firmware (to accept commands)
3. Could it be that the robot has no SD Card? Has the SD Card a trajectory (`TRJ0001.JSN` - use exactly THIS NAME)? Is the log full (capped at `TR99`)?
4. "Weird behavior" - Does the robot have proper mocap data? Does the robot have an SD Card with the right robot configuration on it (`ROBOTCFG_3Pi.CFG` or `ROBOTCFG_ZUMO.CFG`)?
5. Check ROS logs for connection errors:
   
        ros2 launch pololu controller_interface_launch.py --screen
    

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

---

### Safety Features
- Emergency stop functionality (BACK button stops all robots) - helpful when running multi-robot experiments
- Individual robot control prevents accidental commands to wrong robots