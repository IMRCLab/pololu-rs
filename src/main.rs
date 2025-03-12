//! # Pico Blinky Example
//!
//! Blinks the LED on a Pico board.
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for
//! the on-board LED.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]

use embedded_hal::pwm::SetDutyCycle;
// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::OutputPin;

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// Pull in any important traits
use rp_pico::hal::prelude::*;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;

/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then blinks the LED in an
/// infinite loop.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The delay object lets us wait for specified amounts of time (in
    // milliseconds)
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // See https://github.com/pololu/pololu-3pi-2040-robot/blob/master/c/pololu_3pi_2040_robot/yellow_led.c
    // See https://github.com/rp-rs/rp-hal-boards/blob/main/boards/rp-pico/examples/pico_blinky.rs

    // // Set the LED to be an output
    let mut led_pin = pins.led.into_push_pull_output();

    // // Blink the LED at 1 Hz
    // loop {
    //     led_pin.set_high().unwrap();
    //     delay.delay_ms(500);
    //     led_pin.set_low().unwrap();
    //     delay.delay_ms(500);
    // }

    // See https://github.com/pololu/pololu-3pi-2040-robot/blob/master/c/pololu_3pi_2040_robot/motors.c
    // See https://github.com/rp-rs/rp-hal-boards/blob/main/boards/rp-pico/examples/pico_pwm_servo.rs
    // 

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM7
    let pwm = &mut pwm_slices.pwm7;
    pwm.set_ph_correct();
    pwm.set_div_int(1); // 20.8 KhZ
    pwm.set_div_frac(0);
    pwm.set_top(/*MOTORS_MAX_SPEED - 1 */ 6000 - 1);
    pwm.enable();

    let mut direction_right = pins.gpio10.into_push_pull_output_in_state(hal::gpio::PinState::Low); // direction right
    let mut direction_left = pins.gpio11.into_push_pull_output_in_state(hal::gpio::PinState::Low); // direction left

    let channel_right = &mut pwm.channel_a;
    channel_right.output_to(pins.gpio14);

    let channel_left = &mut pwm.channel_b;
    channel_left.output_to(pins.gpio15);

    loop {
        channel_right.set_duty_cycle(3000).unwrap();
        direction_right.set_high().unwrap();

        channel_left.set_duty_cycle(1000).unwrap();
        direction_left.set_low().unwrap();

        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        led_pin.set_low().unwrap();
        delay.delay_ms(500);
    }

}

// End of file
