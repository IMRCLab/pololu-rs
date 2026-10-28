#![no_std]
#![no_main]

use core::sync::atomic::{AtomicBool, Ordering};

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_time::{Duration, with_timeout};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
};
use static_cell::StaticCell;

use pololu3pi2040_rs::{
    dual_core::{
        core0::control_task::core0_control_task,
        core0::setpoint::register_trajectory,
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
    trajectory_reading::Trajectory,
};
const STACK_SIZE: usize = 64 * 1024;
static CORE1_STACK: StaticCell<Stack<STACK_SIZE>> = StaticCell::new();
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();

static CONFIG_CHANNEL:
    Channel<
        CriticalSectionRawMutex,
        (RobotConfig, Option<Trajectory>),
        1,
    > = Channel::new();

static CONFIG_SEND_FAILED: AtomicBool = AtomicBool::new(false);

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

    let core1_stack = CORE1_STACK.init_with(|| Stack::new());
    defmt::info!("Initialized core1 stack");

    spawn_core1(peripherals.CORE1, core1_stack, move || {
        defmt::info!("Core 1: entered spawn closure");

        defmt::info!("Core 1: initializing devices");
        let Core1Devices {
            uart,
            imu_i2c,
            sd_logger,
            robot_config,
            trajectory,
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
        defmt::info!("Core 1: device initialization complete");
        
        let uart_config = UartRouterConfig::from(robot_config);

        defmt::info!("Core 1: sending config and trajectory");
        if CONFIG_CHANNEL
            .try_send((robot_config, trajectory))
            .is_err()
        {
            CONFIG_SEND_FAILED.store(true, Ordering::Release);
            loop {
                core::hint::spin_loop();
            }
        }
        defmt::info!("Core 1: config and trajectory sent");

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
    defmt::info!("Core 0: core1 spawn returned; waiting for config");

    let (robot_config, trajectory) = loop {
        if CONFIG_SEND_FAILED.load(Ordering::Acquire) {
            panic!("Core 1 failed to send config: CONFIG_CHANNEL is full");
        }

        if let Ok(config) = with_timeout(
            Duration::from_millis(100),
            CONFIG_CHANNEL.receive(),
        )
        .await
        {
            break config;
        }
    };
    defmt::info!("Core 0: config and trajectory received");

    if let Some(trajectory) = trajectory {
        register_trajectory(trajectory);
    }

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
        .spawn(core0_orchestrator_task(core0_spawner, robot_config))
        .unwrap();

    defmt::info!("Dual-core TeleOp smoke test started");

    core::future::pending::<()>().await;
}