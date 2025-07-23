#![allow(static_mut_refs)]
#![allow(dead_code)]

use embassy_rp::gpio::Output;
use embassy_rp::pwm::{PwmOutput, SetDutyCycle};

pub struct Motor<'a> {
    pwm: PwmOutput<'a>,
    dir: Output<'a>,
    top: u16,
}

impl<'a> Motor<'a> {
    pub fn new(pwm: PwmOutput<'a>, dir: Output<'a>, top: u16) -> Self {
        Self { pwm, dir, top }
    }

    /// Set Speed（-1.0 ~ 1.0）
    pub fn set_speed(&mut self, speed: f32) {
        let clamped = speed.clamp(-1.0, 1.0);
        if clamped >= 0.0 {
            self.dir.set_high();
        } else {
            self.dir.set_low();
        }

        let duty = (clamped.abs() * self.top as f32) as u16;
        self.pwm.set_duty_cycle(duty).unwrap();
    }
}

static mut LEFT_MOTOR: Option<Motor> = None;
static mut RIGHT_MOTOR: Option<Motor> = None;

/// Initialization
pub fn init_motor(
    pwm_left: PwmOutput<'static>,
    dir_left: Output<'static>,
    pwm_right: PwmOutput<'static>,
    dir_right: Output<'static>,
    top: u16,
) {
    unsafe {
        LEFT_MOTOR = Some(Motor::new(pwm_left, dir_left, top));
        RIGHT_MOTOR = Some(Motor::new(pwm_right, dir_right, top));
    }
}

/// Set Speed for both motors
pub fn set_speed(left: f32, right: f32) {
    unsafe {
        if let Some(m) = LEFT_MOTOR.as_mut() {
            m.set_speed(left);
        }
        if let Some(m) = RIGHT_MOTOR.as_mut() {
            m.set_speed(right);
        }
    }
}
