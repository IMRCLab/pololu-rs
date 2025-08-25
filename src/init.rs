#![allow(static_mut_refs)]
use static_cell::StaticCell;

use crate::button::Buttons;
use crate::buzzer::Buzzer;
use crate::encoder::{EncoderCounters, EncoderPair, init_encoder_counts};
use crate::led::Led;
use crate::motor::{MotorController, init_motor};
use crate::robot_config::RobotConfig;
use crate::sdlog::{self, SdLogger};
use crate::uart::SharedUart;
use embassy_rp::peripherals::UART0;
use embassy_rp::uart::Uart;
use embassy_rp::{
    bind_interrupts,
    gpio::{Input, Level, Output, Pull},
    i2c::{self, I2c, InterruptHandler as I2C_INT_HDL},
    peripherals::*,
    pio::{InterruptHandler as PIO_INT_HDL, Pio},
    pwm::{Config as PWM_config, Pwm},
    uart::Async,
    uart::Config as UART_config,
    uart::InterruptHandler as UART_INT_HDL,
};

use crate::imu::ImuPack;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PIO_INT_HDL<PIO0>;
    I2C0_IRQ => I2C_INT_HDL<I2C0>;
    UART0_IRQ => UART_INT_HDL<UART0>;
});

static UART_CELL: StaticCell<Mutex<ThreadModeRawMutex, Uart<'static, Async>>> = StaticCell::new();
static I2C_CELL: StaticCell<Mutex<ThreadModeRawMutex, I2c<'static, I2C0, i2c::Async>>> =
    StaticCell::new();

pub struct InitDevices<'a> {
    pub led: Led,
    pub buzzer: Buzzer,
    pub buttons: Buttons,
    pub motor: MotorController,
    pub encoders: EncoderPair<'static>,
    pub encoder_counts: EncoderCounters,
    pub imu: ImuPack<'a, I2c<'a, I2C0, i2c::Async>>,
    pub uart: SharedUart<'static>,
    pub sdlogger: Option<SdLogger>,
    pub config: Option<RobotConfig>,
}

/// init all used components
pub fn init_all(p: embassy_rp::Peripherals) -> InitDevices<'static> {
    // === LED Initialization ===
    let led_pin = Output::new(p.PIN_25, Level::Low);
    let led = Led::new(led_pin);

    // === Buzzer Initialization ===
    let buzzer_pin = Output::new(p.PIN_7, Level::High);
    let buzzer = Buzzer::new(buzzer_pin);

    // === Buttons Initialization ===
    let btn_c = Input::new(p.PIN_0, Pull::Up);
    let btn_b = Input::new(p.PIN_1, Pull::Up);
    let buttons = Buttons { btn_c, btn_b };

    // === Motor Initialization ===
    let mut config = PWM_config::default();
    // config.top = 10000; // ~12.5kHz PWM frequency (125MHz / 10001) - above audible range?
    config.top = 32768; // ~12.5kHz PWM frequency (125MHz / 32769)

    let pwm = Pwm::new_output_ab(p.PWM_SLICE7, p.PIN_14, p.PIN_15, config.clone());
    let (pwm_a, pwm_b) = pwm.split();
    let pwm_right = pwm_a.expect("A Channel not initialized!");
    let pwm_left = pwm_b.expect("B Channel not initialized!");

    let dir_left = Output::new(p.PIN_11, Level::Low);
    let dir_right = Output::new(p.PIN_10, Level::Low);

    let motor = init_motor(pwm_left, dir_left, pwm_right, dir_right, config.top);

    // === Encoder Initialization ===
    let Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = Pio::new(p.PIO0, Irqs);
    let encoders = EncoderPair::new(&mut common, sm0, sm1, p.PIN_12, p.PIN_13, p.PIN_8, p.PIN_9);
    let encoder_counts = init_encoder_counts();

    // === IMU Initialization ===
    let i2c_dev = I2c::new_async(
        p.I2C0,
        p.PIN_5, // SCL
        p.PIN_4, // SDA
        Irqs,
        i2c::Config::default(),
    );
    let i2c = I2C_CELL.init(Mutex::new(i2c_dev));
    let imu = ImuPack::new(i2c);

    // === UART Initialization ===
    let mut config = UART_config::default();
    config.baudrate = 115200;
    let uart = Uart::new(
        p.UART0, p.PIN_28, p.PIN_29, Irqs, p.DMA_CH0, p.DMA_CH1, config,
    );
    let uart = UART_CELL.init(Mutex::new(uart));

    // === SD Logger ===
    let (sdlogger, config) =
        match sdlog::init_sd_logger(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_20, p.PIN_21) {
            Ok((logger, cfg)) => (Some(logger), Some(cfg)),
            Err(_e) => {
                defmt::info!("SD card initialization failed!!");
                (None, Some(RobotConfig::default()))
            }
        };

    InitDevices {
        led,
        buzzer,
        buttons,
        motor,
        encoders,
        encoder_counts,
        imu,
        uart,
        sdlogger,
        config,
    }
}
