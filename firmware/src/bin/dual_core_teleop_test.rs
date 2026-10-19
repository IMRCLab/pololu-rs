#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
};
use static_cell::StaticCell;

use pololu3pi2040_rs::{
    dual_core::{
        core0::control_task::core0_control_task,
        core1::{
            imu_task::core1_imu_task,
            logging_task::core1_logging_task,
            uart_tasks::{core1_uart_router_task, UartRouterConfig},
            ui_task::core1_ui_task,
            monitor_task::core1_monitor_task,
        },
        orchestrator::core0_orchestrator_task,
    },
    peripherals::{
        board::{
            Core0Devices,
            Core1Devices,
            init_core0_devices,
            init_core1_devices,
        },
        sdcard::RobotConfig,
        uart::uart_hw_task,
    },
};

static CORE1_STACK: StaticCell<Stack<32768>> = StaticCell::new();
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();

static CONFIG_CHANNEL:
    Channel<CriticalSectionRawMutex, RobotConfig, 1> = Channel::new();

#[embassy_executor::main]
async fn main(core0_spawner: embassy_executor::Spawner) {
    let peripherals = embassy_rp::init(Default::default());

    let Core0Devices {
        motor,
        encoder_counts,
    } = init_core0_devices(
        peripherals.PWM_SLICE7,
        peripherals.PIN_15,
        peripherals.PIN_14,
        peripherals.PIN_11,
        peripherals.PIN_10,
        peripherals.PIO0,
        peripherals.PIN_12,
        peripherals.PIN_13,
        peripherals.PIN_8,
        peripherals.PIN_9,
    );

    let core1_stack = CORE1_STACK.init(Stack::new());

    spawn_core1(peripherals.CORE1, core1_stack, move || {
        let Core1Devices {
            uart,
            imu_i2c,
            sd_logger,
            robot_config,
            led,
            buzzer,
            buttons: _buttons,
        } = init_core1_devices(
            peripherals.UART0,
            peripherals.PIN_28,
            peripherals.PIN_29,
            peripherals.DMA_CH0,
            peripherals.DMA_CH1,
            peripherals.SPI0,
            peripherals.PIN_18,
            peripherals.PIN_19,
            peripherals.PIN_20,
            peripherals.PIN_21,
            peripherals.I2C0,
            peripherals.PIN_5,
            peripherals.PIN_4,
            peripherals.PIN_25,
            peripherals.PWM_SLICE3,
            peripherals.PIN_7,
            peripherals.PIN_1,
            peripherals.PIN_0,
        );

        let uart_config = UartRouterConfig::from(robot_config);

        CONFIG_CHANNEL
            .try_send(robot_config)
            .unwrap();

        let executor = CORE1_EXECUTOR.init(Executor::new());

        executor.run(|spawner| {
            spawner.spawn(uart_hw_task(uart)).unwrap();

            spawner
                .spawn(core1_uart_router_task(uart_config))
                .unwrap();

            spawner
                .spawn(core1_logging_task(sd_logger))
                .unwrap();

            spawner
                .spawn(core1_imu_task(imu_i2c, 10))
                .unwrap();

            spawner
                .spawn(core1_ui_task(Some(led), buzzer))
                .unwrap();

            spawner.spawn(core1_monitor_task()).unwrap();
        })
    });

    let robot_config = CONFIG_CHANNEL.receive().await;

    defmt::info!(
        "Core 0 received config: dt={}ms kp={} ki={} max_duty=({}, {})",
        robot_config.joystick_control_dt_ms,
        robot_config.kp_inner,
        robot_config.ki_inner,
        robot_config.motor_max_duty_left,
        robot_config.motor_max_duty_right,
    );

    core0_spawner
        .spawn(core0_control_task(
            motor,
            encoder_counts,
            robot_config,
        ))
        .unwrap();

    core0_spawner
        .spawn(core0_orchestrator_task())
        .unwrap();

    defmt::info!("Dual-core TeleOp smoke test started");

    core::future::pending::<()>().await;
}