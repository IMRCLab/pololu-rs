# pololu3pi2040
This is a firmware written in Rust for the [Pololu 3pi+ 2040 robot](https://www.pololu.com/category/300/3pi-plus-2040-robot). It's a differential-drive robot that can move with up to 4 m/s very fast and has a RP2040 (RPI Pico)-based hardware.

## Hardware

The board seems to be an almost unmodified Rpi Pico, with a relatively small library on top for drivers. 

Detailed documentation including Pin mappings are available in the [User Guide](https://www.pololu.com/docs/0J86).

[C-Firmware](https://github.com/pololu/pololu-3pi-2040-robot/tree/master/c/pololu_3pi_2040_robot)
[Python-Firmware](https://github.com/pololu/pololu-3pi-2040-robot/tree/master/micropython_demo/pololu_3pi_2040_robot)
[Micropython](https://github.com/pololu/micropython-build)

### Yellow LED

The official blinky examples works - the yellow LED is connected to GPIO25.

[C driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/c/pololu_3pi_2040_robot/yellow_led.c)


### Motors

- GPIO10 sets direction of the right motor
- GPIO11 sets the direction of the left motor
- PWM7 channel a, Pin14 controls the right motor
- PWM7 channel b, Pin15 controls the left motor

[C driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/c/pololu_3pi_2040_robot/motors.c)
[Python driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/motors.py)

### quadrature encoders

No C driver
[Python driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/encoders.py) with [PIO](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/_lib/pio_quadrature_counter.py)

### IMU

- LSM6DSO (6-axis IMU)
- LIS3MDL (3-axis magnetometer)
- Connected over I2C (SCL: 5, SDA: 4)

No C driver
[Python driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/imu.py) [Low-Level LIS2MDL](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/_lib/lis3mdl.py) [Low-level LSM6DSO](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/_lib/lsm6dso.py)


### OLED Display

- SH1106 OLED
- Connected via SPI (data: 0, reset: 1, sck: 2, mosi: 3)
[Python driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/display.py)


### RGB LEDs
The 3pi+ 2040 control board also features six individually-addressable RGB LEDs. The RGB LEDs have integrated drivers compatible with the popular APA102 addressable LED, and they are chained together in alphabetical order (labeled A through F) and arranged counterclockwise on the board.

The control board uses SPI0, one of its two hardware SPI modules, on GP3 and GP6 (TX and SCK, respectively) to control the RGB LEDs. The 3pi+ 2040 libraries include functions that make it easier to control the RGB LEDs and use them together with the OLED display, which shares the SPI0 interface with the RGB LEDs. The display and RGB LEDs share a common pin for SPI0 TX (data), but use different pins for SPI0 SCK (clock), allowing them to be controlled separately.

- TX: GP3(SPI0)
- SCK: GP6(SPI0)

[C driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/c/pololu_3pi_2040_robot/rgb_leds.c)
[Python driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/rgb_leds.py)

### USB-C Interface

### Line sensors
The five line sensors are on the underside of the board along the front edge and can help the 3pi+ distinguish between light and dark surfaces. Each reflectance sensor consists of a down-facing infrared (IR) emitter LED paired with a phototransistor that can detect reflected infrared light from the LED. The reflectance sensors operate on the same principles as our RC-type QTR reflectance sensors: the RP2040 uses an I/O line to drive the sensor output high, and then measures the time for the output voltage to decay.
- Line sensor emitter control pin(DNE): PIN_26
- DN1: PIN_22
- DN2: PIN_21
- DN3: PIN_20
- DN4: PIN_19
- DN5: PIN_18

[C driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/c/pololu_3pi_2040_robot/ir_sensors.c)
[Python driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/ir_sensors.py)

### Bump sensors
The two bump sensors are also reflectance sensors, but rather than providing simple reflectance readings, these are designed to measure changes in reflected light as the corresponding bump sensor flaps on the front of the 3pi+’s bumper skirt are pressed (deflected). This allows the 3pi+ to detect when it has contacted another object in front of it and determine which side the contact is on.
- Bump sensors emitter control pin(BE): PIN_23
- BL: PIN_17
- BR: PIN_16
[C driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/c/pololu_3pi_2040_robot/ir_sensors.c)
[Python driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/ir_sensors.py)

### Buzzer
By default, it is connected to GP7, which can be configured as PWM3 B to produce hardware pulse width modulation.
- Buzzer: PIN_7

[Python driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/buzzer.py)

### Push Buttons
Pressing one of the user pushbuttons pulls the associated I/O pin to ground through a resistor.
- Button A: PIN_25
- Button B: QSPI_SS_N(PIN_1)
- Button C: PIN_0
Notice: Button A is conflict with the LED pin. User should select the function of the pin when initialize it, or change it in need.

[C driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/c/pololu_3pi_2040_robot/button.c)
[Python driver](https://github.com/pololu/pololu-3pi-2040-robot/blob/master/micropython_demo/pololu_3pi_2040_robot/buttons.py)

### Flash memory

### SD Card Deck
Adopted from [Crazyflie uSD deck](https://www.bitcraze.io/products/micro-sd-card-deck/). The deck use spi to write/read data to/from the micro SD Card. The deck now is assigned to SPI0, the pin mapping is shown as:
- SPI_SCK: PIN_18
- SPI_MOSI: PIN_19
- SPI_MISO: PIN_20
- SPI_CS: PIN_21
#### Notice
* The current connection is conflict with the line sensors, which means that we have to disable the line sensors if we would like to log data to SD Card.

## Rust

There are two major frameworks: [embedded-hal](https://github.com/rp-rs/rp-hal-boards/tree/main/boards/rp-pico) + [RTIC](https://rtic.rs) or [Embassy](https://embassy.dev)

[Blog-post](https://willhart.io/post/embedded-rust-options/)

## Flash

### Using the `./run` Script (Recommended)

The project includes a convenient `./run` script for building and flashing different robot configurations:

```bash
# Make executable (first time only)
chmod +x run

```
Then: 
* Press "B" + Reset on the Robot

```bash
# Examples:
./run                    # Default config, main binary
./run zumo              # Zumo config, main binary
./run zumo teleop       # Zumo config, teleop_control binary
./run 3pi trajectory    # 3Pi config, trajectory_following binary
```

### Manual Cargo Commands

* Press "B" + Reset on the Robot
* `cargo run --release` will flash and run the firmware with default configuration
* `cargo run --release --features zumo` for Zumo robot
* `cargo run --release --features 3pi` for 3Pi robot

See [README_BUILD_SYSTEM.md](README_BUILD_SYSTEM.md) for complete documentation.

## Multi-Robot Support

This firmware supports both **Zumo** and **3Pi** robots with different physical parameters and configurations:

### Robot Configurations

#### Zumo Robot (`--features zumo`)
- **Gear Ratio**: 75.81
- **Wheel Radius**: 0.02m
- **Wheel Base**: 0.099m
- **Motor Direction**: Reversed (`-duty_left`, `-duty_right`)
- **Encoder CPR**: 909.72 (75.81 × 12.0)

#### 3Pi Robot (`--features 3pi`)
- **Gear Ratio**: 29.86
- **Wheel Radius**: 0.016m
- **Wheel Base**: 0.0842m
- **Motor Direction**: Normal (`duty_left`, `duty_right`)
- **Encoder CPR**: 358.32 (29.86 × 12.0)

#### Default/Testing (no features)
- **Configuration**: Currently set to Zumo parameters for testing
- **Use Case**: Development and testing without robot-specific compilation

### Parameter Comparison Table

| Parameter | Zumo Robot | 3Pi Robot | Default/Testing |
|-----------|------------|-----------|-----------------|
| Gear Ratio | 75.81 | 29.86 | 75.81 |
| Wheel Radius | 0.02m | 0.016m | 0.02m |
| Wheel Base | 0.099m | 0.0842m | 0.099m |
| Motor Direction | Reversed | Normal | Reversed |
| Encoder CPR | 909.72 | 358.32 | 909.72 |

### Quick Start Examples:
```bash
./run zumo teleop       # Flash teleop control to Zumo
./run 3pi trajectory    # Flash trajectory following to 3Pi  
./run                   # Use default config for testing
```

## USB Logging

```
sudo apt install tio
tio /dev/ttyACM0
```
(use ctrl+t q to exit)

## Advanced Motor Control System

### PI Controller Implementation

The firmware features an advanced **PI (Proportional-Integral) controller** for precise RPM-based motor control:

#### Key Features:
- **Physical Unit Control**: Direct RPM control instead of arbitrary PWM values
- **Robot-Specific Tuning**: Optimized gains for different robot configurations
- **Anti-Windup Protection**: Prevents integrator saturation using back-calculation method
- **Complementary Filtering**: Noise reduction with configurable α = 0.3 for RPM feedback
- **Separate Integrator States**: Proper anti-windup implementation with independent left/right integrators

#### Controller Parameters:

| Robot Type | kp (Proportional) | ki (Integral) | Kaw (Anti-windup) | Sample Rate |
|------------|-------------------|---------------|-------------------|-------------|
| **3Pi Robot** | 60.0 | 1.0 | 0.1 | 20ms |
| **Zumo/Default** | 10.0 | 1.0 | 0.1 | 20ms |

#### Control Algorithm:
```rust
// PI Controller with anti-windup
P_inc = kp * (e_k - e_k1)                                    // Proportional increment
I_inc = ki * Ts * e_k + Kaw * (u_sat_prev - u_k_prev)      // Integral with anti-windup
u_k = u_k_prev + P_inc + I_inc                             // Control output
```

### Control Command Interface

The system supports multiple control interfaces through UART communication:

#### 1. **Legacy PWM Control** (`ControlCommand`)
- Direct left/right motor PWM control
- Range: ±10000 (normalized to ±1.0)
- Packet: `CmdLegacyPacketF32`

#### 2. **Unicycle Model Control** (`ControlCommandUnicycle`) 
- **Linear velocity** (`v`): Forward/backward speed [m/s]
- **Angular velocity** (`omega`): Rotation rate [rad/s]
- Automatic differential drive conversion
- Nonlinear exponential scaling for intuitive control feel

#### 3. **Advanced Teleop Control** (`CmdTeleopPacketMix`)
- **Linear Velocity**: ±20000 range with exponential scaling to 0.2 m/s max
- **Steering Angle**: Direct angle input with configurable turn time
- **Nonlinear Scaling**: Exponential curves for both linear and angular commands
- **Dead Zone Handling**: Managed by ROS node (1000-20000 active range)

### Nonlinear Control Scaling

Both linear and angular commands use exponential scaling for better control feel:

```rust
// Exponential scaling function
scaled_output = max_value * (exp(normalized_input) - 1.0) / (exp(1.0) - 1.0)
```

**Benefits:**
- Fine control at low speeds
- Smooth acceleration curves
- Intuitive joystick response
- Maximum performance when needed

## Project Structure
- `src/`: Source code
  - `main.rs`: The entry point of the project.
  - `init.rs`: Initialization for all devices (PWM frequency fixed to 10kHz for silent operation).
  - `lib.rs`: Integrated libraries with feature-based module selection.
  - `led.rs`: Default LED driver.
  - `buzzer.rs`: Buzzer driver.
  - `motor.rs`: Driver of both motors.
  - `encoder.rs`: Encoder driver for both encoders using PIO.
  - `uart.rs`: UART0 driver.
  - `packet.rs`: Extended packet definitions including `CmdTeleopPacketMix` for advanced control.

  **Advanced Motor Control System**
  - `joystick_control.rs`: 
    - **PI Controller**: RPM-based control with anti-windup protection
    - **Multiple Control Modes**: PWM, Unicycle, and Teleop interfaces
    - **Robot-Specific Parameters**: Conditional compilation for Zumo/3Pi configurations  
    - **Nonlinear Scaling**: Exponential curves for intuitive control response
    - **Complementary Filtering**: RPM noise reduction (α=0.3)
  - `bin/`: Binary targets
    - `teleop_control.rs`: Teleop control application with advanced PI controller
    - `trajectory_following.rs`: Trajectory following application
  - `imu/`: IMU library.
    - `lis3mdl.rs`: Driver for the 3-axis magnetometer.
    - `lsm6dso.rs`: Driver for the combined 3-axis accelerometer and 3-axis gyrometer.
    - `shared_i2c.rs`: A shared I2C bus for reusing the same I2C for the combined 3-axis accelerometer and 3-axis gyrometer.
    - `complementary_filter.rs`: Complementary filter to estimate Roll/Pitch/Yaw.
    - `madgwick.rs`: Madgwick filter to estimate Roll/Pitch/Yaw (under developing).
- `memory.x`: Defines the memory layout of RP2040 (SRAM and Flash).
- `.cargo/config.toml`: Specifies the target platform as `thumbv6m-none-eabi` for RP2040.
- `Cargo.toml`: Declares project metadata, Rust edition, dependencies, and feature flags for robot selection.
- `build.rs`: A custom build script to ensure the `memory.x` linker script is properly included during compilation.
- `run`: Build script for easy robot and binary selection (see [README_BUILD_SYSTEM.md](README_BUILD_SYSTEM.md)).
- `README_BUILD_SYSTEM.md`: Comprehensive documentation for the build system and robot configurations. `thumbv6m-none-eabi` for RP2040.
- `Cargo.toml`: Declares project metadata, Rust edition, and dependencies.
- `build.rs`: A custom build script to ensure the `memory.x` linker script is properly included during compilation.


## Logging with Raspberry Pi Debug Probe
The logging is mainly based on [probe-rs](https://probe.rs/docs/getting-started/installation/), [defmt](https://docs.rs/defmt/latest/defmt/) and [defmt-rtt](https://docs.rs/defmt-rtt/latest/defmt_rtt/).
There is also a useful [blog](https://murraytodd.medium.com/our-first-rust-blinky-program-on-raspberry-pi-pico-w-376211f1074d) to learn how to use them very quickly.

* Notice: The installation command on the home page of [probe-rs](https://probe.rs/) doesn't work for me due to some package conflicts. If so you could try following the [instructions](https://probe.rs/docs/getting-started/installation/) and install it from source.

### Debug Setup
* You will need a Raspberry Pi Debug Probe(for debugging) and a USB-C Cabel(for flashing).
* connect the probe properly to the Pololu. If the official probe is used, the connection should be:
```
Yellow(SWDIO) -> SWDIO
Green(SWCLK)  -> SWCLK
Black(GND)    -> GND 
``` 
Then connect the Pololu with the USB-C cabel to your PC, and the debug probe as well and uncomment in config.toml

```runner = "probe-rs run --chip RP2040 --protocol swd" ```

* If you would like to directly observed the debug information in the terminal, run:
```
cargo run --release
```
* If you would like to save the logging information into a csv file, run:
```
cargo run --release > file_name.csv
```

**OR** 

flash in bootloader mode and without debug information in the terminal by uncommenting 

```runner = "elf2uf2-rs -d"```

in config.toml and then press "B" and "Reset" on the robot
and flash with
```
cargo run --release
```
or use the recommended [build system](README_BUILD_SYSTEM.md)

### Print New Debug Information 
The code only print a test value(constant). When new debug information is needed, use:
```
info!("New Sensor Data: {}", value);
```

## Logging with sd card
The SD card logging is built based on [Crazyflie micro sd card deck](https://www.bitcraze.io/products/micro-sd-card-deck/). The deck uses spi protocol to log data in and read data out from the micro sd card. There are mutiple available file format including `csv`, `txt` and `binary file` and different logging functions and some preset structs is provided for testing which can be modified later on according to individual use cases.

### TODO:
When the sd card deck initialization function is called but no sd card is inserted in the deck, a segmentation fault will raises. Therefore, we need to modify the intialization function so that even no sd card is in the deck but the code can still run.


## Uart

### Control Command Protocols

The UART interface supports multiple communication protocols for different control requirements:

#### 1. **Legacy Packet Types** (`packet.rs`)
- **`CmdLegacyPacketU16`**: 4 × U16 values for basic integer commands
- **`CmdLegacyPacketF32`**: 4 × F32 values for direct PWM control with floating-point precision  
- **`CmdLegacyPacketMix`**: 2 × U16 + 2 × F32 values for mixed-type commands

#### 2. **Advanced Teleop Protocol** (`CmdTeleopPacketMix`)
- **Packet Size**: 10 bytes (header + 2 × F32 values)
- **Linear Velocity**: F32 range ±20000 with exponential scaling to ±0.2 m/s
- **Steering Angle**: F32 steering angle in degrees for intuitive control
- **Nonlinear Scaling**: Both commands use exponential curves for smooth control response
- **Dead Zone**: 1000-20000 active range (handled by ROS node)

#### 3. **Control Command Types**

**Direct Motor Control:**
```rust
ControlCommand {
    left_speed: f32,   // ±10000 range, normalized to ±1.0 PWM
    right_speed: f32,  // ±10000 range, normalized to ±1.0 PWM  
}
```

**Unicycle Model Control:**
```rust
ControlCommandUnicycle {
    v: f32,      // Linear velocity [m/s]
    omega: f32,  // Angular velocity [rad/s]
}
```

### Communication Features
- **Asynchronous Reception**: Non-blocking UART message processing
- **Timeout Protection**: 1000ms timeout with automatic motor stop
- **Packet Validation**: Header verification and length checking
- **Multiple Start Bytes**: Supports different packet types (0x02, 0x08, 0x09)
- **Buffer Management**: 32-byte circular buffer with automatic clearing

### Protocol Headers
- **Legacy Packets**: Header `0x3C` (modified from `0xFF` by channel/port settings)
- **Teleop Packets**: Header `0x3C` with specific start bytes for packet identification
- **Channel Configuration**: Channel 0x00, Port 0x03 (modifies header automatically)

### Real-Time Control Loop
- **Sample Rate**: 20ms (50Hz) for consistent control timing
- **PI Controller Integration**: Direct RPM feedback control
- **Motor Safety**: Automatic timeout stop prevents runaway conditions
- **Debug Logging**: Comprehensive telemetry via defmt logging

### Packet Type
### Legacy Packet Support
3 different legacy packet types remain supported in `packet.rs`:
- `CmdLegacyPacketU16`: including 4 U16 values.
- `CmdLegacyPacketF32`: including 4 F32 values.  
- `CmdLegacyPacketMix`: including 2 U16 values and 2 F32 values.

### Read Packet
Multiple asynchronous tasks handle different control protocols:

- **`robot_command_task`**: Processes legacy F32 packets for direct PWM control
- **`robot_command_control_task`**: Handles advanced teleop packets with PI controller integration  
- **Asynchronous Reception**: Non-blocking message processing with timeout protection
- **Automatic Failsafe**: 1000ms timeout triggers motor stop for safety

### Communication Protocol Details
- **Header Verification**: All packets validated against expected `0x3C` header
- **Length Checking**: Dynamic packet length validation based on first byte
- **Buffer Management**: 32-byte ring buffer with automatic overflow handling
- **Start Byte Filtering**: Multiple valid start bytes (0x02, 0x08, 0x09) for packet type identification

**Important Notes:**
- Header `0xFF` in Crazyflie-link examples becomes `0x3C` due to `set_Channel(0x00)` and `set_Port(0x03)` 
- Extra start bytes (0x09, etc.) are part of the Crazyflie-link protocol implementation
- All UART tasks operate in fully asynchronous mode for optimal performance

## Motor Control System

### Basic Motor Interface
The `motor.rs` module provides the fundamental `set_speed` function for direct PWM control:
```rust
motor.set_speed(left_duty: f32, right_duty: f32)  // Range: ±1.0
```

### Advanced PI Controller Integration  
The motor control system now features sophisticated closed-loop control:

#### **RPM-Based Control**
- Direct RPM target setting instead of open-loop PWM
- Real-time encoder feedback for precise speed control
- Robot-specific calibration for optimal performance

#### **Anti-Windup Protection**
- Prevents integrator saturation during motor saturation
- Back-calculation method: `I_inc = ki*Ts*error + Kaw*(u_sat_prev - u_k_prev)`
- Separate integrator states for left and right motors

#### **Complementary Filtering**
- RPM noise reduction with α=0.3 filter coefficient
- Improved control stability and reduced jitter
- Real-time filtering: `filtered_rpm = α*raw_rpm + (1-α)*filtered_rpm_prev`

#### **Robot-Specific Motor Direction Handling**
- **Zumo**: Reversed motor directions (`MOTOR_DIRECTION_LEFT/RIGHT = -1.0`)
- **3Pi**: Normal motor directions (`MOTOR_DIRECTION_LEFT/RIGHT = 1.0`)
- Automatic compensation in control loop for consistent behavior


## Encoder
Use Pio Module to build the encoder driver. Provide the user with position reading function and rotation speed (RPM) reading function. A asynchronous reading task is implemented in `encoder/encoder.rs`.


## IMU
Provide `read_imu_task` function to read and estimate the euler angles. The task should be registered in main using spawn.

### TODO:
 * Yaw is always drifting. Needs to be fixed.