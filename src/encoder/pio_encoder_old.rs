use embassy_rp::peripherals::{PIN_12, PIN_13, PIN_8, PIN_9, PIO0};
use embassy_rp::pio::{Common, StateMachine};
use embassy_rp::pio_programs::rotary_encoder::{Direction, PioEncoder, PioEncoderProgram};
use embassy_rp::Peri;

pub struct EncoderPair<'a> {
    pub encoder_left: PioEncoder<'a, PIO0, 0>,
    pub encoder_right: PioEncoder<'a, PIO0, 1>,
}

impl<'a> EncoderPair<'a> {
    pub fn new(
        common: &mut Common<'a, PIO0>,
        sm0: StateMachine<'a, PIO0, 0>,
        sm1: StateMachine<'a, PIO0, 1>,
        pin_left_a: Peri<'a, PIN_12>,
        pin_left_b: Peri<'a, PIN_13>,
        pin_right_a: Peri<'a, PIN_8>,
        pin_right_b: Peri<'a, PIN_9>,
    ) -> Self {
        let prg = PioEncoderProgram::new(common);

        let encoder_left = PioEncoder::new(common, sm0, pin_left_a, pin_left_b, &prg);
        let encoder_right = PioEncoder::new(common, sm1, pin_right_a, pin_right_b, &prg);

        Self {
            encoder_left,
            encoder_right,
        }
    }
}

#[embassy_executor::task]
pub async fn encoder_left_task(mut encoder: PioEncoder<'static, PIO0, 0>) {
    let mut count = 0;
    loop {
        let dir = encoder.read().await;
        count += match dir {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };
        defmt::info!("ENC LEFT count: {}", count);
    }
}

#[embassy_executor::task]
pub async fn encoder_right_task(mut encoder: PioEncoder<'static, PIO0, 1>) {
    let mut count = 0;
    loop {
        let dir = encoder.read().await;
        count += match dir {
            Direction::Clockwise => 1,
            Direction::CounterClockwise => -1,
        };
        defmt::info!("ENC RIGHT count: {}", count);
    }
}
