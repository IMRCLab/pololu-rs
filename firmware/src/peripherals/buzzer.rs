//! Buzzer PWM driver.

use embassy_rp::pwm::{
    Config as PwmConfig,
    Pwm,
};
use fixed::{
    traits::ToFixed,
    types::U12F4,
};

pub struct Buzzer {
    pwm: Pwm<'static>,
    pwm_top: u16,
}

impl Buzzer {
    pub fn new(
        pwm: Pwm<'static>,
        pwm_top: u16,
    ) -> Self {
        Self {
            pwm,
            pwm_top,
        }
    }

    pub fn tone(
        &mut self,
        frequency_hz: u32,
        duty_ratio: f32,
    ) {
        if frequency_hz == 0 {
            self.stop();
            return;
        }

        let duty_ratio =
            duty_ratio.clamp(0.0, 1.0);

        let divider = 125_000_000.0
            / (
                frequency_hz as f32
                    * (self.pwm_top as f32 + 1.0)
            );

        let mut config =
            PwmConfig::default();

        config.top = self.pwm_top;

        config.compare_b =
            (duty_ratio * self.pwm_top as f32)
                as u16;

        config.divider =
            divider.to_fixed::<U12F4>();

        self.pwm.set_config(&config);
    }

    pub fn stop(&mut self) {
        let mut config =
            PwmConfig::default();

        config.top = self.pwm_top;
        config.compare_b = 0;

        self.pwm.set_config(&config);
    }
}