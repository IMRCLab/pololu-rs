//! This example test the RP Pico on board LED.
//!
//! It does not work with the RP Pico W board. See wifi_blinky.rs.

#![no_std]
#![no_main]

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

use embassy_executor::Spawner;
use embassy_rp::gpio;
use embassy_rp::pwm::{Config, Pwm, SetDutyCycle};
use embassy_time::Timer;
use gpio::{Level, Output};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let mut led = Output::new(p.PIN_25, Level::Low);

    let mut c = Config::default();
    c.top = 6000 - 1;
    c.phase_correct = true;
    c.divider = 1.into();

    let mut direction_right = Output::new(p.PIN_10, Level::Low);
    let mut direction_left = Output::new(p.PIN_11, Level::Low);

    let mut channel_right = Pwm::new_output_a(p.PWM_SLICE7, p.PIN_14, c.clone());

    channel_right.set_duty_cycle(3000).unwrap();
    direction_right.set_high();

    // let mut channel_left = Pwm::new_output_b(p.PWM_SLICE7, p.PIN_15, c.clone());

    // let channel = Pwm::new_output_ab(p.PWM_SLICE7, p.PIN_14, p.PIN_15, c.clone());
    // let (channel_left, channel_right) = channel.split();

    // channel_right.unwrap().set_duty_cycle(3000).unwrap();
    // direction_right.set_high();

    // channel_left.unwrap().set_duty_cycle(1000).unwrap();
    // direction_left.set_low();

    loop {
        led.set_high();
        Timer::after_secs(1).await;

        led.set_low();
        Timer::after_secs(1).await;
    }
}
