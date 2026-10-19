//! Dual-core hardware initialization.
//!
//! Core 0 owns deterministic control hardware:
//! - Motor PWM
//! - Encoder PIO
//!
//! Core 1 owns communication and storage hardware:
//! - UART0 and DMA
//! - SPI0 and SD card

use embassy_rp::{
    Peri,
    gpio::{Level, Output},
    peripherals::{
        DMA_CH0,
        DMA_CH1,
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
        PIN_28,
        PIN_29,
        PIO0,
        PWM_SLICE7,
        SPI0,
        UART0,
    },
    pio::Pio,
    pwm::{Config as PwmConfig, Pwm},
    uart::{Async, Config as UartConfig, Uart},
};

use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    mutex::Mutex,
};

use static_cell::StaticCell;

use crate::{
    encoder::{
        EncoderCounters,
        EncoderPair,
        init_encoder_counts,
        start_encoder_irq,
    },
    init::Irqs,
    motor::{
        MotorController,
        init_motor,
    },
    read_robot_config_from_sd::RobotConfig,
    sdlog::{
        SdLogger,
        init_sd_logger,
    },
    uart::{
        SharedUart,
        init_uart_rx_irq,
    },
};

static DUAL_CORE_UART_CELL: StaticCell<
    Mutex<
        ThreadModeRawMutex,
        Uart<'static, Async>,
    >,
> = StaticCell::new();

/// Hardware exclusively owned by Core 0.
pub struct Core0ControlDevices {
    pub motor: MotorController,
    pub encoder_counts: EncoderCounters,
}

/// Hardware exclusively owned by Core 1.
pub struct Core1IoDevices {
    pub uart: SharedUart<'static>,
    pub sdlogger: Option<SdLogger>,
    pub config: RobotConfig,
}

/// Initialize motor PWM and encoder PIO.
///
/// This function must be called from Core 0.
#[allow(clippy::too_many_arguments)]
pub fn init_core0_control(
    pwm_slice: Peri<'static, PWM_SLICE7>,
    pwm_left_pin: Peri<'static, PIN_15>,
    pwm_right_pin: Peri<'static, PIN_14>,
    direction_left_pin: Peri<'static, PIN_11>,
    direction_right_pin: Peri<'static, PIN_10>,

    pio0: Peri<'static, PIO0>,
    encoder_left_a: Peri<'static, PIN_12>,
    encoder_left_b: Peri<'static, PIN_13>,
    encoder_right_a: Peri<'static, PIN_8>,
    encoder_right_b: Peri<'static, PIN_9>,
) -> Core0ControlDevices {
    // ================= Motor PWM =================

    let mut pwm_config = PwmConfig::default();
    pwm_config.top = 10_000;

    // PWM channel A is PIN_14/right; channel B is PIN_15/left.
    let pwm = Pwm::new_output_ab(
        pwm_slice,
        pwm_right_pin,
        pwm_left_pin,
        pwm_config.clone(),
    );

    let (pwm_right, pwm_left) = pwm.split();

    let motor = init_motor(
        pwm_left.expect("left PWM channel"),
        Output::new(direction_left_pin, Level::Low),
        pwm_right.expect("right PWM channel"),
        Output::new(direction_right_pin, Level::Low),
        pwm_config.top,
    );

    // ================= Encoder PIO =================

    let Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = Pio::new(pio0, Irqs);

    let encoders = EncoderPair::new(
        &mut common,
        sm0,
        sm1,
        encoder_left_a,
        encoder_left_b,
        encoder_right_a,
        encoder_right_b,
    );

    let encoder_counts = init_encoder_counts();
    start_encoder_irq(encoders);

    Core0ControlDevices {
        motor,
        encoder_counts,
    }
}

/// Initialize UART and SD card.
///
/// This function must be called inside the Core 1 startup closure. UART IRQ
/// enablement therefore happens from Core 1.
///
/// SD initialization also loads `ROBOTCFG.CFG`. If initialization fails,
/// the returned configuration is `RobotConfig::default()`.
#[allow(clippy::too_many_arguments)]
pub fn init_core1_io(
    uart0: Peri<'static, UART0>,
    uart_tx: Peri<'static, PIN_28>,
    uart_rx: Peri<'static, PIN_29>,
    dma_tx: Peri<'static, DMA_CH0>,
    dma_rx: Peri<'static, DMA_CH1>,

    spi0: Peri<'static, SPI0>,
    sd_sck: Peri<'static, PIN_18>,
    sd_mosi: Peri<'static, PIN_19>,
    sd_miso: Peri<'static, PIN_20>,
    sd_cs: Peri<'static, PIN_21>,
) -> Core1IoDevices {
    // ================= UART =================

    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115_200;

    let uart = Uart::new(
        uart0,
        uart_tx,
        uart_rx,
        Irqs,
        dma_tx,
        dma_rx,
        uart_config,
    );

    init_uart_rx_irq();

    // ThreadModeRawMutex is acceptable here because this UART object is
    // exclusively accessed by Core 1 tasks. Cross-core communication must use
    // CriticalSectionRawMutex elsewhere.
    let uart = DUAL_CORE_UART_CELL.init(
        Mutex::new(uart)
    );

    // ================= SPI / SD =================

    let (sdlogger, config) = match init_sd_logger(
        spi0,
        sd_sck,
        sd_mosi,
        sd_miso,
        sd_cs,
    ) {
        Ok((logger, config)) => {
            defmt::info!(
                "Core 1: robot config loaded from SD"
            );

            (Some(logger), config)
        }

        Err(_) => {
            defmt::warn!(
                "Core 1: SD/config unavailable, using defaults"
            );

            (None, RobotConfig::default())
        }
    };

    Core1IoDevices {
        uart,
        sdlogger,
        config,
    }
}