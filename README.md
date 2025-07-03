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

### USB-C Interface

### IR sensors

### Bump sensors

### Buzzer

### Push Buttons

### Flash memory

## Rust

There are two major frameworks: [embedded-hal](https://github.com/rp-rs/rp-hal-boards/tree/main/boards/rp-pico) + [RTIC](https://rtic.rs) or [Embassy](https://embassy.dev)

[Blog-post](https://willhart.io/post/embedded-rust-options/)

## Flash

* Press "B" + Reset
* `cargo run --release` will flash and run the firmware


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
Then connect the Pololu with the USB-C cabel to your PC, and the debug probe as well.
* Press "B" + Reset to set the Pololu into bootloader mode.
* If you would like to directly observed the debug information in the terminal, run:
```
cargo run --release
```
* If you would like to save the logging information into a csv file, run:
```
cargo run --release > file_name.csv
```

### Print New Debug Information 
The code only print a test value(constant). When new debug information is needed, use:
```
info!("New Sensor Data: {}", value);
```