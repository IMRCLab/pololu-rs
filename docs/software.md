# Software Overview

<!-- Here I have to little knowledge to actually improve somtehing. Jiaming and I discussed the inclusion of a "information flow" - graphic, and a shift in each modules explanation towards a more contextualized structure (e.g., how uses the LED function for what, why would this be interesting to somebody usign the lib)

Also: How are the module descriptions ordered? Is there any structure to this?

Also: Is this up to date?, are all libs documented?
 -->

This page describes the structure, modules, and control architecture of the Rust firmware for the  
**Pololu 3pi+ 2040** and **Pololu Zumo 2040** robots.

The firmware is fully async using the [Embassy](https://github.com/embassy-rs/embassy) framework and modularized for extensibility: motor control, IMU filtering, encoder reading, UART packet decoding, SD logging, tele-operation via joystick and trajectory execution.

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Firmware Architecture

The overall architecture consists of:
```
src/
├── main.rs                     # Entry point of the firmware
├── init.rs                     # Initialization for all devices
├── lib.rs                      # Feature-based module selection
├── led.rs                      # LED driver
├── buzzer.rs                   # Buzzer driver
├── diffdrive.rs                # Differential flatness & trajectory math
├── motor.rs                    # Motor driver
├── encoder.rs                  # Basic encoder (PIO) driver
├── encoder_lib.rs              # Higher-level encoder API built on encoder.rs
├── uart.rs                     # UART0 driver
├── packet.rs                   # Crazyflie-compatible packet definitions
├── sdlog.rs                    # SD logging + parameter loading
├── orchestrator_signal.rs      # Signals and channels for async tasks

│
├── joystick_control.rs         # Default/testing teleop configuration
├── trajectory_control.rs       # Main trajectory-following controller
├── trajectory_read.rs          # Load trajectory from JSON
├── trajectory_signal.rs        # Event/update signals
├── trajectory_uart.rs          # Receive MoCap poses via UART

│
├── imu/                        # IMU module
│   ├── lis3mdl.rs              # Magnetometer driver
│   ├── lsm6dso.rs              # Accel + Gyro driver
│   ├── shared_i2c.rs           # Shared I2C bus with Mutex
│   ├── complementary_filter.rs # Roll/Pitch/Yaw estimation (complementary filter)
│   ├── madgwick.rs             # Madgwick AHRS filter
│   ├── read_robot_config_from_sd.rs   # Load robot parameters from SD config file
│   └── robot_parameters_default.rs    # Default fallback robot parameters

│
└── bin/                        # Application binaries
    ├── programm_entrance.rs    # Unified program launcher
    ├── teleop_control.rs       # Teleoperation mode
    └── trajectory_following.rs # Cascade trajectory-following controller

memory.x                        # RP2040 memory layout
.cargo/
└── config.toml                 # Target: thumbv6m-none-eabi

Cargo.toml                      # Project metadata, dependencies, feature flags
build.rs                        # Ensures memory.x is included
run                             # Helper script for flashing/building
```

<div style="height:4px; background:#1e90ff; margin:32px 0;"></div>

## Modules Description

---

### LED Module

The LED module provides a simple interface for controlling the robot’s on-board LED (GPIO25 for 3pi/Pololu boards).  
It supports turning the LED on/off and asynchronous blinking.

---

#### Shared LED Handle
```rust
LED_SHARED: Mutex<Option<Led>>
```
A global mutex stores the LED instance so tasks can safely access it concurrently.

---

#### Led Structure
```rust
struct Led {
    pin: Output<'static>,
}
```

Created using:
```rust
Led::new(output_pin)
```

---

#### Methods
**Turn LED on**
```rust
led.on()
```

**Turn LED off**
```rust
led.off()
```

**Blink asynchronously**
```rust
led.blink(interval_ms, count).await
```

This alternates LED on/off with the given interval.

<div style="height:2px; background:#1e90ff; margin:32px 0;"></div>

### Button Module
The button module provides asynchronous handling of the robot’s physical buttons using Embassy’s async GPIO interface. Each button runs in its own lightweight task and triggers on falling edges.

---

#### Buttons Structure
```rust
pub struct Buttons {
    pub btn_a: Input<'static>,
    pub btn_b: Input<'static>,
    pub btn_c: Input<'static>,
}
```
Button A needs to be handled separately because it shares `GPIO25` with the LED on Pololu boards.

---

#### Button Tasks
Each button has a dedicated async task:

- Waits for a falling edge
- Confirms the pin is low (pressed)
- Prints a `defmt` debug message
- Includes a small delay for debouncing

Example task behavior (simplified):
```rust
wait_for_falling_edge().await;
if button.is_low() {
    info!("Button X pressed!");
}
```
Buttons supported:

- Button A (PIN 25)
- Button B (PIN 1)
- Button C (PIN 0)

---

#### Notes

- All buttons are active-low (pressed = low).
- Button A conflicts with the LED pin; LED should be disabled when using Button A.
- Tasks are spawned during initialization (`init_all()` or `main()`).

<div style="height:2px; background:#1e90ff; margin:32px 0;"></div>

### Buzzer Module

The buzzer module provides PWM-based audio playback for tones, beeps, and simple music.
It modifies the PWM frequency dynamically to generate musical notes.

---

#### Core Structures
```rust
pub struct Buzzer<'a> {
    pwm: Pwm<'a>,
    top: u16,
}
```
- `pwm`: PWM slice output
- `top`: PWM counter top value (controls resolution and tone accuracy)

A global mutex wraps the buzzer for async-safe access (BuzzerController).

#### Core Functions
```rust
pub fn tone(&mut self, freq_hz: u32, duty_ratio: f32)
```
Plays a tone by reconfiguring the PWM divider.

```rust
pub fn stop(&mut self)
```
Silences the buzzer.

```rust
pub async fn beep(&mut self, freq: u32, duty: f32, interval_ms: u64, count: usize)
```
Repeats the same tone several times.

#### Notes Provided
```
C4, D4, E4, F4, G4, A4, B4, C5
```
Useful for building scales or melodies.

#### Preset Startup Sound
```rust
play_startup_sound(&buzzer).await;
```
Plays a short sci-fi style boot sound (frequency sweep → oscillation → confirmation ping).

<div style="height:2px; background:#1e90ff; margin:32px 0;"></div>

### Motor Module

The motor module provides an async-safe and high-level interface for controlling the left and right DC motors of the robot. It handles PWM speed control, direction control, and shared access through Embassy mutexes.

---

#### Core Structures

##### **`Motor`**
Represents a single motor, which includes:

- `PwmOutput<'a>` — PWM channel for motor speed  
- `Output<'a>` — direction GPIO pin  
- `top: u16` — PWM counter maximum value  

**Responsibilities**:

- Clamp speed input to `[-1.0, 1.0]`
- Map speed magnitude to PWM duty cycle
- Switch direction pin based on speed sign

**Key Method**:
```rust
pub fn set_speed(&mut self, speed: f32)
```
Direction logic:

- `speed >= 0` → forward
- `speed < 0` → backward
  
PWM duty is proportional to `abs(speed)`.

---

##### **`MotorController`**

Controls both motors simultaneously using Embassy mutexes.
```rust
pub async fn set_speed(&self, left: f32, right: f32)
```
This method locks each motor asynchronously and applies speed commands. Used by teleoperation tasks and trajectory controllers.

---

#### Initialization

Motors are initialized using:
```rust
pub fn init_motor(pwm_left: PwmOutput<'static>, dir_left: Output<'static>, pwm_right: PwmOutput<'static>, dir_right: Output<'static>, top: u16,) -> MotorController
```

This function:

- Allocates static storage using `StaticCell`.
- Wraps each motor in `Mutex<ThreadModeRawMutex, Motor>`.
- Returns a `MotorController` managing both motors.

Designed to be called inside `init_all()`.

<div style="height:2px; background:#1e90ff; margin:32px 0;"></div>

### Encoder Module

The encoder module provides PIO-based quadrature decoding for the left and right wheel encoders.  
It maintains shared tick counters and provides helper functions for computing RPM and angular speed.

---

#### Core Structures

**`EncoderPair`**  
Holds two hardware PIO encoders:
- `encoder_left: PioEncoder<PIO0, SM0>`
- `encoder_right: PioEncoder<PIO0, SM1>`

Constructed using A/B pin pairs for each wheel.

**`EncoderCounters`**  
Shared tick counters stored in:
- `left: Mutex<i32>`
- `right: Mutex<i32>`

Initialized via:
```rust
pub fn init_encoder_counts() -> EncoderCounters
```
---

#### Quadrature Decoding

Quadrature transitions are decoded using a 4×4 lookup table (`LUT`).
```rust
const LUT: [[i8; 4]; 4] = [
    /*prev\curr: 00, 01, 10, 11 */
    [0, 1, -1, 0], // 00 ->
    [-1, 0, 0, 1], // 01 ->
    [1, 0, 0, -1], // 10 ->
    [0, -1, 1, 0], // 11 ->
];
```  
Each PIO FIFO entry is processed to accumulate tick deltas.

---

#### Async Encoder Tasks

Two tasks run continuously:
```rust
pub async fn encoder_left_task(
    mut encoder: PioEncoder<'static, PIO0, 0>,
    counter: &'static Mutex<NoopRawMutex, i32>,
)

pub async fn encoder_right_task(
    mut encoder: PioEncoder<'static, PIO0, 1>,
    counter: &'static Mutex<NoopRawMutex, i32>,
)
```


Each task:

- Reads AB state
- Computes tick delta using LUT
- Updates the shared counter with direction correction

---

#### High-Level Velocity Functions

**RPM:**
```rust
get_left_rpm(counter, cpr_wheel, interval_ms)

get_right_rpm(counter, cpr_wheel, interval_ms)

get_rpms(left, right, cpr_wheel, interval_ms)
```

**Angular velocity (rad/s):**
```rust
pub async fn get_wheel_speed_in_rad(
    left_counter: &'static Mutex<NoopRawMutex, i32>,
    right_counter: &'static Mutex<NoopRawMutex, i32>,
    cpr_wheel: f32,
    interval_ms: u64,
    dt: f32, // equal to interval_ms but in second, just for saving one divide calculation
) -> (f32, f32)
```

**Instantaneous speed (non-blocking):**
```rust
pub fn wheel_speed_from_counts_now(
    left_counter: &Mutex<NoopRawMutex, i32>,
    right_counter: &Mutex<NoopRawMutex, i32>,
    cpr_wheel: f32,
    prev_left: i32,
    prev_right: i32,
    dt: f32, // in sec
) -> ((f32, f32), (i32, i32))
```

---

#### Initialization

The quadrature encoders are initialized in two layers:

##### 1. Load the PIO Program
```rust
let program = PioEncoderProgram::new(&mut pio_common);
```

This loads the custom quadrature-decoder PIO assembly program into the PIO instruction memory.

##### 2. Create Left and Right `PioEncoder` Instances
```rust
let encoders = EncoderPair::new(&mut pio_common, sm0, sm1, pin_left_a, pin_left_b, pin_right_a, pin_right_b);
```

This:

- Converts GPIOs into PIO pins  
- Enables pull-ups on A/B lines  
- Sets pin directions to input  
- Configures FIFO as RX-only  
- Sets PIO clock divider  
- Binds each encoder to a state machine  
- Enables both state machines

`EncoderPair` now exposes:

- `encoder_left`
- `encoder_right`

Each is a fully configured quadrature reader.

##### 3. Initialize Shared Tick Counters
```rust
let counters = init_encoder_counts();
```

This allocates:

- `counters.left  → Mutex<i32>`
- `counters.right → Mutex<i32>`

Used by the async tasks.

##### 4. Start Async Encoder Tasks
```rust
spawner.spawn(encoder_left_task(encoders.encoder_left, counters.left));
spawner.spawn(encoder_right_task(encoders.encoder_right, counters.right));
```

Each task:

- Continuously reads A/B transitions  
- Uses LUT-based decoding  
- Updates the corresponding counter

<div style="height:2px; background:#1e90ff; margin:32px 0;"></div>

### IMU Module

The IMU module provides drivers and filtering for the 9-axis sensor suite:

- **LSM6DSO** (accelerometer + gyroscope)
- **LIS3MDL** (magnetometer)
- **Complementary filter** for roll/pitch/yaw estimation  
- Optional **Madgwick filter** (currently disabled)

A shared async I2C bus is used for all sensors, since both imu sensors share the same I2C Bus.

---

#### ImuPack

`ImuPack` bundles all IMU components:

- `i2c: Mutex<I2C>` — shared async I2C bus  
- `lsm6dso` — accelerometer + gyroscope driver  
- `lis3mdl` — magnetometer driver  
- `complementary` — global complementary filter instance  

Created using:
```rust
pub fn new(i2c: &'a Mutex<ThreadModeRawMutex, T>) -> Self
```

---

#### Initialization
```rust
pub async fn init(&mut self) -> Result<(), T::Error>
```

Initializes:

- LSM6DSO  
- LIS3MDL  

Returns an error if any sensor fails to respond.

---

#### Reading Sensor Data
```rust
pub async fn read_all(&mut self) -> Result<([f32; 3], [f32; 3], [f32; 3]), T::Error>
```

Each returned value is a `[f32; 3]` vector.

---

#### Orientation Filtering

The complementary filter is stored in a `StaticCell`:
```rust
complementary.update(gyro, accel, mag, dt)

complementary.get_angles_deg()
```

Madgwick filter code exists but is commented out.

---

#### IMU Task
```rust
pub async fn read_imu_task(mut imu: ImuPack<'static, I2c<'static, I2C0, Async>>)
```

Async task that:

- Initializes the IMU  
- Continuously reads accel/gyro/mag  
- Updates the complementary filter  
- Runs at 100 Hz (10 ms interval)

<div style="height:2px; background:#1e90ff; margin:32px 0;"></div>

### UART Module

The UART module provides:
- A low-level asynchronous UART hardware task  
- A packet-decoding task  
- TX/RX channels for inter-task communication  
- Helper functions for sending command / state packets  
- A loopback system for broadcasting robot states  

It is the communication backbone between the robot and external controllers (PC, joystick node, Mocap node, etc.).

---

#### Shared UART Handle
```rust
pub type SharedUart = &'a Mutex<Uart<Async>>;
```

Wrapped in a mutex for async-safe access.

---

#### Channels

- **UART_TX_CHANNEL**  
  Tasks → UART hardware layer (Sends byte buffers)

- **UART_RX_CHANNEL**  
  UART hardware layer → Decoder task (Receives 1 byte at a time)

---

#### Low-Level UART Hardware Task
```rust
uart_hw_task(uart)
```

This task continuously waits on two futures:

- Incoming TX request from `UART_TX_CHANNEL`
- Incoming RX byte from UART hardware

It uses `select()` to process whichever event arrives first.

Responsibilities:

- On TX: write bytes to UART
- On RX: forward byte to `UART_RX_CHANNEL`

#### Packet Receive Task
```rust
uart_receive_task()
```
A standalone task that:

- Reads bytes from `UART_RX_CHANNEL`
- Reconstructs framed packets based on their length
- Supports multiple packet formats:
    - `CmdLegacyPacketU16`
    - `CmdLegacyPacketMix`
    - `CmdLegacyPacketF32`
    - `MocapPosesPacketF32Test`

Packet structure:

- Byte 0 → packet length
- Byte 1 → header (`0x3C` expected)
- Remaining bytes → payload

When one full packet is collected, it is decoded and printed/debugged.

#### UART Send Functions
**Send raw bytes**
```rust
pub fn uart_send(data: &[u8])
```
Uses `UART_TX_CHANNEL.try_send()` to schedule transmission.

**Pack state-loopback messages**
```rust
pub fn pack_state_loopback(pkt: &StateLoopBackPacketF32) -> Vec<u8, 64>

pub fn uart_send_state_loopback(pkt: &StateLoopBackPacketF32)
```
Used to broadcast full robot state (pos, vel, quaternion).

**Robot State Loopback Task**
```rust
pub async fn state_loopback_task()
```
Reads robot state packets from `ROBOT_STATE_CH`, packages them, and sends them through UART.

Allows external systems (PC/ROS) to monitor:

- Position
- Velocity
- Orientation

in real-time.

<div style="height:2px; background:#1e90ff; margin:32px 0;"></div>

### SD Logging Module
The SD logging module provides:

- Full SD card initialization via SPI0
- FAT filesystem mounting (`embedded-sdmmc`)
- Automatic loading of:
    - Trajectory file (`TRJ0001.JSN`)
    - Robot config file (`ROBOTCFG_*.CFG`)

- Automatic allocation of a unique log file (`TR00–TR99`)
- CSV + binary logging utilities for:
    - robot motion
    - trajectory controller debugging

This module is required for trajectory-following experiments and logging-based debugging.

#### SD Card Initialization
SD card init is performed using:
```rust
pub fn init_sd_logger(
    spi: Peri<'static, SPI0>,
    sck: Peri<'static, PIN_18>,
    mosi: Peri<'static, PIN_19>,
    miso: Peri<'static, PIN_20>,
    cs: Peri<'static, PIN_21>,
) -> Result<(SdLogger, RobotConfig), SdError>
```

This function performs:

1. SPI0 setup
      - 400 kHz bootstrap frequency
      - 16 MHz normal operation
2. Volume manager creation
   use
   ```rust
   VolumeManager<SdCard, DummyClock>
   ```
3. Open FAT volume + root directory
4. Load Trajectory (`TRJ0001.JSN`)
   Uses a 48 KB scratch buffer for JSON.
5. Load Robot Configuration
6. Allocate a new log file
   The logger cycles from:
   ```
   TR00 → TR01 → … → TR99
   ```
   and picks the first unused slot.

Returns:

- `SdLogger` — handle used for writing
- `RobotConfig` — parsed config stored in SD

---

#### Code Structure
##### SdLogger
`SdLogger` wraps a FAT file object:
```rust
pub struct SdLogger {
    file: File<'static, Sd<'static>, DummyClock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
}
```
Provides file I/O for logging.

##### Basic Logging Functions
**Write raw bytes**
```rust
pub fn write(&mut self, data: &[u8])
```

**Flush the file**
```rust
pub fn flush(&mut self)
```
Must be called after all logging tasks finish.

**Read entire file (debug only)**
```rust
pub fn read_all(&mut self)
```

##### CSV Logging Helpers
**Write CSV header (trajectory control)**
```rust
pub fn write_traj_control_header(&mut self)
```

**Append one trajectory-control row (CSV)**
```rust
pub fn log_traj_control_as_csv(&mut self, data: &TrajControlLog)
```

##### Binary Logging Helpers
Binary logging is more compact and suitable for high-frequency data rates.
```rust
pub fn log_motion_as_bin(&mut self, log: &MotionLog)
```
Internally uses `mem::transmute` to convert the struct to a byte slice.

##### Log Data Structures
`TrajControlLog`

Full trajectory controller logging:
```rust
pub struct TrajControlLog {
    pub timestamp_ms: u32,
    pub target_x: f32, pub target_y: f32, pub target_theta: f32,
    pub actual_x: f32, pub actual_y: f32, pub actual_theta: f32,
    pub target_vx: f32, pub target_vy: f32, pub target_vz: f32,
    pub actual_vx: f32, pub actual_vy: f32, pub actual_vz: f32,
    pub target_qw: f32, pub target_qx: f32, pub target_qy: f32, pub target_qz: f32,
    pub actual_qw: f32, pub actual_qx: f32, pub actual_qy: f32, pub actual_qz: f32,
    pub xerror: f32, pub yerror: f32, pub thetaerror: f32,
    pub ul: f32, pub ur: f32, pub dutyl: f32, pub dutyr: f32,
}
```
This could be extended if needed.

#### File Naming Rules

| Purpose              | File Name        | Notes                         |
|----------------------|------------------|--------------------------------|
| Robot configuration  | `ROBOTCFG.CFG`   | Must follow naming format      |
| Trajectory           | `TRJ0001.JSN`    | Must exist for trajectory following |
| Log files            | `TR00`–`TR99`    | Automatically chosen           |

#### Notes

- JSON trajectory must fit into the 48 KB scratch buffer
- Must call `.flush()` before removing SD card
- SPI0 conflicts with line sensors; they must be disabled during SD use
- If robot configuration is missing, defaults are loaded

<div style="height:2px; background:#1e90ff; margin:32px 0;"></div>