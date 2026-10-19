//! Core 0 motor PWM driver.
//!
//! The motor controller has a single owner: the permanent Core 0 control task.
//! It therefore does not require a global StaticCell or an async mutex.

use embassy_rp::{
    gpio::Output,
    pwm::{
        PwmOutput,
        SetDutyCycle,
    },
};

pub struct Motor<'a> {
    pwm: PwmOutput<'a>,
    direction: Output<'a>,
    pwm_top: u16,
}

impl<'a> Motor<'a> {
    pub fn new(
        pwm: PwmOutput<'a>,
        direction: Output<'a>,
        pwm_top: u16,
    ) -> Self {
        Self {
            pwm,
            direction,
            pwm_top,
        }
    }

    /// Set normalized motor output in the range `[-1.0, 1.0]`.
    pub fn set_speed(&mut self, speed: f32) {
        let speed = speed.clamp(-1.0, 1.0);

        if speed >= 0.0 {
            self.direction.set_high();
        } else {
            self.direction.set_low();
        }

        let duty =
            (speed.abs() * self.pwm_top as f32)
                as u16;

        self.pwm
            .set_duty_cycle(duty)
            .unwrap();
    }

    /// Immediately force PWM output to zero.
    pub fn stop(&mut self) {
        self.pwm
            .set_duty_cycle(0)
            .unwrap();
    }
}

/// Both motors are exclusively owned by Core 0.
///
/// Unlike the legacy controller, this type contains no async mutex. Calling
/// `set_speed()` cannot yield to another Embassy task.
pub struct MotorController {
    left: Motor<'static>,
    right: Motor<'static>,
}

impl MotorController {
    pub fn new(
        left: Motor<'static>,
        right: Motor<'static>,
    ) -> Self {
        Self {
            left,
            right,
        }
    }

    /// Set left and right normalized motor outputs.
    ///
    /// This function is synchronous so the control task cannot be suspended
    /// between calculating and applying motor output.
    pub fn set_speed(
        &mut self,
        left: f32,
        right: f32,
    ) {
        self.left.set_speed(left);
        self.right.set_speed(right);
    }

    /// Immediately set both PWM outputs to zero.
    pub fn stop(&mut self) {
        self.left.stop();
        self.right.stop();
    }
}

pub fn init_motor(
    pwm_left: PwmOutput<'static>,
    direction_left: Output<'static>,

    pwm_right: PwmOutput<'static>,
    direction_right: Output<'static>,

    pwm_top: u16,
) -> MotorController {
    let left = Motor::new(
        pwm_left,
        direction_left,
        pwm_top,
    );

    let right = Motor::new(
        pwm_right,
        direction_right,
        pwm_top,
    );

    MotorController::new(left, right)
}