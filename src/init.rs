#![allow(static_mut_refs)]
use static_cell::StaticCell;

use crate::buzzer::Buzzer;
use crate::led::led::Led;
use crate::motor::motor::init_motor;
use embassy_rp::{
    gpio::{Input, Level, Output, Pull},
    peripherals::*,
    pwm::{Config as PWM_config, Pwm},
};

///
pub struct InitDevices {
    pub led: Led,
    pub buzzer: Buzzer,
}

/// init all used components
pub fn init_all(p: embassy_rp::Peripherals) -> InitDevices {
    // === LED Initialization ===
    let led_pin = Output::new(p.PIN_25, Level::Low);
    let led = Led::new(led_pin);

    // === Buzzer Initialization ===
    let buzzer_pin = Output::new(p.PIN_7, Level::High);
    let buzzer = Buzzer::new(buzzer_pin);

    // === Motor Initialization ===
    let mut config = PWM_config::default();
    config.top = 32768;

    let pwm = Pwm::new_output_ab(p.PWM_SLICE7, p.PIN_14, p.PIN_15, config.clone());
    let (pwm_a, pwm_b) = pwm.split();
    let pwm_right = pwm_a.expect("A Channel not initialized!");
    let pwm_left = pwm_b.expect("B Channel not initialized!");

    let dir_left = Output::new(p.PIN_11, Level::Low);
    let dir_right = Output::new(p.PIN_10, Level::Low);

    init_motor(pwm_left, dir_left, pwm_right, dir_right, config.top);

    InitDevices { led, buzzer }
}
