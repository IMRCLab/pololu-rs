use panic_probe as _;
use rp_pico::hal;

use rp_pico::hal::{
    gpio::{
        bank0::Gpio10, bank0::Gpio11, bank0::Gpio14, bank0::Gpio15, FunctionNull, FunctionSio, Pin,
        PinState, PullDown, SioOutput,
    },
    pac::PWM,
    pwm,
    pwm::{Channel, FreeRunning, Pwm7, Slice, Slices},
};

use hal::pac::RESETS;

pub struct MotorPWM {
    // pub pwm7: Slice<Pwm7, FreeRunning>,
    pub left_channel: Channel<Slice<Pwm7, FreeRunning>, pwm::B>,
    pub right_channel: Channel<Slice<Pwm7, FreeRunning>, pwm::A>,
    pub left_direction: Pin<Gpio11, FunctionSio<SioOutput>, PullDown>,
    pub right_direction: Pin<Gpio10, FunctionSio<SioOutput>, PullDown>,
}

pub fn init_motor_pwm(
    pwm: PWM,
    resets: &mut RESETS,
    gpio14: Pin<Gpio14, FunctionNull, PullDown>,
    gpio15: Pin<Gpio15, FunctionNull, PullDown>,
    gpio10: Pin<Gpio10, FunctionNull, PullDown>,
    gpio11: Pin<Gpio11, FunctionNull, PullDown>,
) -> MotorPWM {
    let pwm_slices = Slices::new(pwm, resets);
    let mut pwm7 = pwm_slices.pwm7;

    pwm7.set_ph_correct();
    pwm7.set_div_int(1);
    pwm7.set_div_frac(0);
    pwm7.set_top(6000 - 1);
    pwm7.enable();

    let mut right_channel = pwm7.channel_a;
    let mut left_channel = pwm7.channel_b;

    right_channel.output_to(gpio14);
    left_channel.output_to(gpio15);

    /*
    pwm7.channel_a.output_to(gpio14);
    pwm7.channel_b.output_to(gpio15);
    */

    let right_direction = gpio10.into_push_pull_output_in_state(PinState::Low);
    let left_direction = gpio11.into_push_pull_output_in_state(PinState::Low);

    MotorPWM {
        left_channel,
        right_channel,
        left_direction,
        right_direction,
    }
}
