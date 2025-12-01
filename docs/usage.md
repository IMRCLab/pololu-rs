# Usage
This page describes all available programms for the robots, including Universal Menu Interface, Teleoperation Control via Joystick and Trajectory Following with/without Mocap.

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Menu
- Prepare the Robots:
    - Prepare the nRF Dongle and write down the address for the robot(if multiple dongles are running together then each dongle should have different address)
    - Change the `robot_id` according to the Dongle address of the current robot in file `src/bin/programm_entrance.rs`, the `robot_id` can be found here:

            spawner.spawn(orchestrator(spawner, devices, UartCfg { robot_id: 10 })).unwrap();

    - Connect the Raspberry-Debug-Probe and a USB-C cable to the Pololu.
    - Flash the firmware to the robot using the following command:

            ./run 3pi menu

    - Change the name of the corresponding robot configuration file in folder `cfg` to `ROBOTCFG.CFG` and paste the tuned robot configuration file into a micro sd card. Please **DO NOT CHANGE** the name of the configuration file, the file system depends on the file name to distinguish configuration file from other files.
    - Change the trajectory file name to `TRJ0001.JSN` and paste it into a micro sd card. Please **DO NOT CHANGE** the name of the trajectory file, the file system depends on the file name to distinguish trajectory file from other files.
- Prepare the Trajectory Following ROS Node:
    - Connect a Crazyradio PA or Crazyradio 2 and a joystick to your PC.
    - Change the address for each robot in `workstation_ros/src/pololu_ros/config/controller_interface.yaml`.
    - Open workstation_ros folder and build the ros2 workspace using:

            colcon build

    - Open a terminal and run:

            source install/setup.bash
            ros2 run pololu_ros controller_interface

- Use the buttons on the joystick to select different functionalities.
  ![joystick_menu](./images/joystick.png)

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

**Note**: When stopping the robot, the program must be **QUIT** for each robot with the X button, before starting a program again.

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

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Tele-Operation for multiple robots
In addition to the controller interface node with the menu, there is another node that provides tele-operation functionality for multiple robots, who can simultaneously be controlled by with one controller for each robot respectively.

### Hardware Requirements
- Xbox controller (wired or wireless with USB adapter)
- Compatible joystick device recognized by `joy_linux`

### Setup Steps
- Prepare the Robots
    - Prepare the nRF Dongle and write down the address for the robot(if multiple dongles are running together then each dongle should have different address)
    - Connect the Raspberry-Debug-Probe and a USB-C cable to the Pololu.
    - Flash the firmware to the robot using the following command:

            ./run 3pi teleop

- Prepare the Joystick ROS Node
    - Connect a Crazyradio PA or Crazyradio 2 and 1 or more joysticks onto the PC.
    - Change the address for each robot in `workstation_ros/src/pololu_ros/config/teleop.yaml`.
    - Open workstation_ros folder and build the ros2 workspace using:

            colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
            source install/setup.bash

    - Run the `teleop` Node using:

            ros2 launch pololu_ros teleop_launch.py

  
<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Trajectory Following
Also take Polulu 3Pi as an example, PLEASE FOLLOW THE STEPS:

- Prepare the Robots:
    - Prepare the nRF Dongle and write down the address for the robot(if multiple dongles are running together then each dongle should have different address)
    - Change the `robot_id` according to the Dongle address of the current robot in file `src/bin/trajectory_following.rs`, the `robot_id` can be found here:

            spawner.spawn(uart_motioncap_receiving_task(
                    devices.uart,
                    UartCfg { robot_id: 10 },
            )).unwrap();

    - Connect the Raspberry-Debug-Probe and a USB-C cable to the Pololu.
    - Flash the firmware to the robot using the following command:

            ./run 3pi trajectory_following

    - Change the name of the corresponding robot configuration file in folder `cfg` to `ROBOTCFG.CFG` and paste the tuned robot configuration file into a micro sd card. Please **DO NOT CHANGE** the name of the configuration file, the file system depends on the file name to distinguish configuration file from other files.
    - Change the trajectory file name to `TRJ0001.JSN` and paste it into a micro sd card. Please **DO NOT CHANGE** the name of the trajectory file, the file system depends on the file name to distinguish trajectory file from other files.
    - Insert the sd card to the port on the robot.
- Prepare the Trajectory Following ROS Node:
    - Connect a Crazyradio PA or Crazyradio 2.
    - Change the address for each robot in `workstation_ros/src/pololu_ros/config/mocap_broadcast.yaml`.
    - Open `workstation_ros` folder and build the ros2 workspace using:

            colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

    - Open a terminal and run:
    
            source install/setup.bash
            ros2 run pololu_ros mocap_broadcast

    - Open a new terminal and run:

            source install/setup.bash
            ros2 launch motion_capture_tracking launch.py 

- Run Trajectory:
    - Put the robot on the correct starting position written in the trajectory file. 
    - Press `t` for starting the trajectory(for all robots); Press `s` for stop(for all robots).
    - The actual trajectory would be saved in a csv file in the sd card, some plotting function is also provided in folder `datavis`. User could use the following command to plot the trajectories and part of the error w.r.t the given trajectory:

            python3 my_display.py path/to/your/csv

- Notice for Rerunning the Trajectory:
    - If the robot is taken out of the flightspace then the 2 ros processes should be restarted.
    - Restart the robot manually so that the trajectory csv will not be overwritten.

