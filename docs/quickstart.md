# Quickstart

This page guides you through a quick first set-up of the system. It aims to provide some hands on experience without the need of extensive prior knowledge.

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

- Prepare the Robot:
    - Prepare the [nrf-Dongle]() and write down the dongle address <!-- what is the current workflow here?, where should I take the address from ??? -->
    - Connect the [Raspberry-Debug-Probe](debugging.md) and a USB-C cable to the robot
    - Flash the firmware using the following command (if you want to read on how the flashing works, read [here](build.md)):
    
            ./run 3pi menu

- Prepare the Trajectory:
    - Change the name of the corresponding robot configuration file in folder `cfg` to `ROBOTCFG.CFG` and paste the tuned robot configuration file into a micro sd card. Please **DO NOT CHANGE** the name of the configuration file, the file system depends on the file name to distinguish configuration file from other files.
    <!-- Where does the file come from, is it just in the workspace???-->
    - Change the trajectory file name to `TRJ0001.JSN` and paste it into a micro sd card. Your trajectory file should be in the same format as the [provided ones](https://github.com/IMRCLab/pololu3pi2040-rs/tree/main/TRAJs). Please **DO NOT CHANGE** the name of the trajectory file, the file system depends on the file name to distinguish trajectory file from other files. 
    <!-- Same question here, we should also put in a file with "trajectory generation" 
    
    Where are the files located -> pointer -->
- Prepare the Trajectory Following ROS Node:
    - Set up [ROS](ros.md)
    - Connect a Crazyradio PA or Crazyradio 2 <!-- should we explain those in the hardware section -->
    - Change the address for each robot in `workstation_ros/src/pololu_ros/config/controller_interface.yaml`. <!-- To what? -->
    - Open the workstation_ros folder and build the ros2 workspace using:

            colcon build
- Plug an Xbox controller (wired or wireless with USB adapter) to your PC
    
- Execute:
    - Open a terminal and run:

            source install/setup.bash
            ros2 run pololu_ros controller_interface

    - Use the buttons on the joystick to select different functionalities.
    ![joystick_menu](./images/joystick.png)

    - Select your robot and "trajectory following w/o MC"
    - Confirm with "X"
    - Start the motion with "A"
    - Stop your robot with "B"

Congrats! You successfully moved your robot!
<!-- Link to troubleshooting-->