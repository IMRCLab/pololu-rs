//! Dual-core RP2040 board initialization.
//!
//! Hardware ownership:
//!
//! Core 0:
//! - motor PWM
//! - motor direction GPIO
//! - encoder PIO and IRQ
//!
//! Core 1:
//! - UART0 and DMA
//! - SPI0 and SD card
//! - I2C0 and IMU
//! - LED
//! - buzzer
//! - buttons

use embassy_rp::{
    Peri,
    bind_interrupts,
    gpio::{
        Input,
        Level,
        Output,
        Pull,
    },
    i2c::{
        self,
        I2c,
    },
    peripherals::{
        DMA_CH0,
        DMA_CH1,
        I2C0,
        PIN_0,
        PIN_1,
        PIN_4,
        PIN_5,
        PIN_7,
        PIN_8,
        PIN_9,
        PIN_10,
        PIN_11,
        PIN_12,
        PIN_13,
        PIN_14,
        PIN_15,
        PIN_18,
        PIN_19,
        PIN_20,
        PIN_21,
        PIN_25,
        PIN_28,
        PIN_29,
        PIO0,
        PWM_SLICE3,
        PWM_SLICE7,
        SPI0,
        UART0,
    },
    pio::{
        InterruptHandler as PioInterruptHandler,
        Pio,
    },
    pwm::{
        Config as PwmConfig,
        Pwm,
    },
    uart::{
        self,
        Async,
        Uart,
    },
};

use super::{
    button::Buttons,
    buzzer::Buzzer,
    encoder::{
        EncoderCounters,
        EncoderPair,
        EncoderPioIrqHandler,
        init_encoder_counts,
        start_encoder_irq,
    },
    led::Led,
    motor::{
        MotorController,
        init_motor,
    },
    sdcard::{
        RobotConfig,
        SdLogger,
        init_sd_card,
    },
    uart::{
        UartRxIrqHandler,
        init_uart_rx_irq,
    },
};

#[cfg(feature = "dual-core")]
bind_interrupts!(pub struct DualCoreIrqs {
    PIO0_IRQ_0 =>
        PioInterruptHandler<PIO0>,
        EncoderPioIrqHandler;

    I2C0_IRQ =>
        i2c::InterruptHandler<I2C0>;

    UART0_IRQ =>
        uart::InterruptHandler<UART0>,
        UartRxIrqHandler;
});
#[cfg(not(feature = "dual-core"))]
pub use crate::init::Irqs as DualCoreIrqs;

/* ================= Device ownership ================= */

pub struct Core0Devices {
    pub motor: MotorController,
    pub encoder_counts: EncoderCounters,
}

pub struct Core1Devices {
    pub uart: Uart<'static, Async>,
    pub imu_i2c: I2c<'static, I2C0, i2c::Async>,

    pub sd_logger: Option<SdLogger>,
    pub robot_config: RobotConfig,

    pub led: Led,
    pub buzzer: Buzzer,
    pub buttons: Buttons,
}

/* ================= Core 0 initialization ================= */

#[allow(clippy::too_many_arguments)]
pub fn init_core0_devices(
    pwm_slice: Peri<'static, PWM_SLICE7>,
    pwm_left_pin: Peri<'static, PIN_15>,
    pwm_right_pin: Peri<'static, PIN_14>,
    motor_left_direction: Peri<'static, PIN_11>,
    motor_right_direction: Peri<'static, PIN_10>,

    pio0: Peri<'static, PIO0>,
    encoder_left_a: Peri<'static, PIN_12>,
    encoder_left_b: Peri<'static, PIN_13>,
    encoder_right_a: Peri<'static, PIN_8>,
    encoder_right_b: Peri<'static, PIN_9>,
) -> Core0Devices {
    /* ---------- Motor ---------- */

    let mut pwm_config =
        PwmConfig::default();

    pwm_config.top = 10_000;

    let pwm = Pwm::new_output_ab(
        pwm_slice,
        pwm_right_pin,
        pwm_left_pin,
        pwm_config.clone(),
    );

    let (pwm_right, pwm_left) =
        pwm.split();

    let mut motor = init_motor(
        pwm_left.expect(
            "left motor PWM channel missing",
        ),
        Output::new(
            motor_left_direction,
            Level::Low,
        ),

        pwm_right.expect(
            "right motor PWM channel missing",
        ),
        Output::new(
            motor_right_direction,
            Level::Low,
        ),

        pwm_config.top,
    );

    // Explicit safe startup state.
    motor.stop();

    /* ---------- Encoder ---------- */

    let Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = Pio::new(
        pio0,
        DualCoreIrqs,
    );

    let encoders = EncoderPair::new(
        &mut common,
        sm0,
        sm1,

        encoder_left_a,
        encoder_left_b,

        encoder_right_a,
        encoder_right_b,
    );

    let encoder_counts =
        init_encoder_counts();

    start_encoder_irq(encoders);

    Core0Devices {
        motor,
        encoder_counts,
    }
}

/* ================= Core 1 initialization ================= */

#[allow(clippy::too_many_arguments)]
pub fn init_core1_devices(
    // UART
    uart0: Peri<'static, UART0>,
    uart_tx: Peri<'static, PIN_28>,
    uart_rx: Peri<'static, PIN_29>,
    uart_dma_tx: Peri<'static, DMA_CH0>,
    uart_dma_rx: Peri<'static, DMA_CH1>,

    // SD card
    spi0: Peri<'static, SPI0>,
    sd_clock: Peri<'static, PIN_18>,
    sd_mosi: Peri<'static, PIN_19>,
    sd_miso: Peri<'static, PIN_20>,
    sd_chip_select: Peri<'static, PIN_21>,

    // IMU
    i2c0: Peri<'static, I2C0>,
    imu_scl: Peri<'static, PIN_5>,
    imu_sda: Peri<'static, PIN_4>,

    // UI
    led_pin: Peri<'static, PIN_25>,
    buzzer_pwm_slice: Peri<'static, PWM_SLICE3>,
    buzzer_pin: Peri<'static, PIN_7>,
    button_b_pin: Peri<'static, PIN_1>,
    button_c_pin: Peri<'static, PIN_0>,
) -> Core1Devices {
    /* ---------- UART ---------- */

    let mut uart_config =
        uart::Config::default();

    uart_config.baudrate = 115_200;

    let uart = Uart::new(
        uart0,
        uart_tx,
        uart_rx,
        DualCoreIrqs,
        uart_dma_tx,
        uart_dma_rx,
        uart_config,
    );

    // This function must execute from Core 1 so the UART IRQ is enabled for
    // the core that owns the receive path.
    init_uart_rx_irq();

    /* ---------- I2C / IMU ---------- */

    let imu_i2c = I2c::new_async(
        i2c0,
        imu_scl,
        imu_sda,
        DualCoreIrqs,
        i2c::Config::default(),
    );

    /* ---------- SD card ---------- */

    let (sd_logger, robot_config) =
        match init_sd_card(
            spi0,
            sd_clock,
            sd_mosi,
            sd_miso,
            sd_chip_select,
        ) {
            Ok((logger, config)) => {
                defmt::info!(
                    "Core 1 loaded robot configuration from SD"
                );

                (
                    Some(logger),
                    config,
                )
            }

            Err(_) => {
                defmt::warn!(
                    "Core 1 SD unavailable; using default configuration"
                );

                (
                    None,
                    RobotConfig::default(),
                )
            }
        };

    /* ---------- LED ---------- */

    let led = Led::new(
        Output::new(
            led_pin,
            Level::Low,
        ),
    );

    /* ---------- Buzzer ---------- */

    let mut buzzer_config =
        PwmConfig::default();

    buzzer_config.top = 10_000;

    let buzzer_pwm = Pwm::new_output_b(
        buzzer_pwm_slice,
        buzzer_pin,
        buzzer_config.clone(),
    );

    let mut buzzer = Buzzer::new(
        buzzer_pwm,
        buzzer_config.top,
    );

    buzzer.stop();

    /* ---------- Buttons ---------- */

    let buttons = Buttons::new(
        Input::new(
            button_b_pin,
            Pull::Up,
        ),
        Input::new(
            button_c_pin,
            Pull::Up,
        ),
    );

    Core1Devices {
        uart,
        imu_i2c,

        sd_logger,
        robot_config,

        led,
        buzzer,
        buttons,
    }
}