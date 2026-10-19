#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Executor;
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_time::{Duration, Instant, Ticker};
use portable_atomic::{AtomicU32, AtomicI32, Ordering};
use static_cell::StaticCell;
use embassy_sync::blocking_mutex::{
    raw::CriticalSectionRawMutex,
};
use embassy_rp::uart::{
    Async,
    Config as UartConfig,
    Uart,
};
use embassy_rp::{
    gpio::{Level, Output},
    pio::Pio,
    pwm::{Config as PwmConfig, Pwm},
};

use embassy_sync::{
    channel::Channel,
    mutex::Mutex as AsyncMutex,
};

use pololu3pi2040_rs::encoder::wheel_speed_from_counts_now;
use pololu3pi2040_rs::motor::MotorController;
use pololu3pi2040_rs::read_robot_config_from_sd::RobotConfig;
use pololu3pi2040_rs::{
    init::Irqs,
    uart::{
        init_uart_rx_irq,
        uart_hw_task,
    },
};
use pololu3pi2040_rs::{
    encoder::{
        EncoderPair,
        init_encoder_counts,
        start_encoder_irq,
    },
    motor::init_motor,
    sdlog::{init_sd_logger, SdLogger},
};
use pololu3pi2040_rs::joystick_control_dual_core::{
    core1_teleop_uart_task,
    latest_command,
    take_runtime_metrics,
};
// Import the selected constants into the module scope
use pololu3pi2040_rs::robot_parameters_default::robot_constants::*;

const PI: f32 = core::f32::consts::PI;
const TELEOP_STOP_TARGET_RAD_S: f32 = 0.1;



// Create Signal for quitting teleop mode

/// Get the gear ratio for runtime identification
pub fn get_gear_ratio() -> f32 {
    GEAR_RATIO
}

static CORE1_STACK: StaticCell<Stack<32768>> = StaticCell::new();
static CORE1_EXECUTOR: StaticCell<Executor> = StaticCell::new();

static CONFIG_CH:
    Channel<CriticalSectionRawMutex, RobotConfig, 1> =
    Channel::new();

static UART_CELL: StaticCell<
    AsyncMutex<
        embassy_sync::blocking_mutex::raw::ThreadModeRawMutex,
        Uart<'static, Async>,
    >,
> = StaticCell::new();


#[derive(Clone, Copy)]
struct ControlLogFrame {
    t_us: u64,

    command_v: f32,
    command_omega: f32,

    omega_l_target: f32,
    omega_r_target: f32,

    omega_l_measured: f32,
    omega_r_measured: f32,

    duty_l: f32,
    duty_r: f32,

    control_gap_us: u32,
}
static CONTROL_LOG_CH:
    Channel<CriticalSectionRawMutex, ControlLogFrame, 256> =
    Channel::new();
static CONTROL_TICKS: AtomicU32 = AtomicU32::new(0);
static CONTROL_MAX_GAP_US: AtomicU32 = AtomicU32::new(0);
static CONTROL_MISSES: AtomicU32 = AtomicU32::new(0);
static DROPPED_LOG_FRAMES: AtomicU32 = AtomicU32::new(0);


#[embassy_executor::task]
async fn core1_sd_logging_task(mut logger: SdLogger) {
    logger.open_new_file();
    logger.flush();

    defmt::info!("Core 1: SD logging started");

    let mut frames_since_flush = 0u32;

    loop {
        let frame = CONTROL_LOG_CH.receive().await;

        logger.write_buffered(
            &frame.t_us.to_le_bytes()
        );

        for value in [
            frame.command_v,
            frame.command_omega,
            frame.omega_l_target,
            frame.omega_r_target,
            frame.omega_l_measured,
            frame.omega_r_measured,
            frame.duty_l,
            frame.duty_r,
        ] {
            logger.write_buffered(
                &value.to_le_bytes()
            );
        }

        logger.write_buffered(
            &frame.control_gap_us.to_le_bytes()
        );

        frames_since_flush += 1;

        // 50 Hz 时约每 5 秒同步一次。
        if frames_since_flush >= 250 {
            logger.flush();
            frames_since_flush = 0;
        }
    }
}


/* ========================== Joy Stick Speed Control Task =============================== */
#[embassy_executor::task]
pub async fn teleop_motor_control_task(
    //adapted from inner loop
    motor: MotorController,
    left_counter: &'static AtomicI32,
    right_counter: &'static AtomicI32,
    cfg: Option<RobotConfig>,
) {
    let robot_cfg: RobotConfig;
    if let Some(_) = cfg {
        defmt::info!("Load Robot Params from SD CARD!");
        robot_cfg = cfg.unwrap();
    } else {
        defmt::info!("Load Robot Params from DEFAULT!");
        robot_cfg = RobotConfig::default();
    }

    let mut ticker = Ticker::every(Duration::from_millis(robot_cfg.joystick_control_dt_ms));

    // PID state variables
    let (mut il, mut ir) = (0.0f32, 0.0f32);
    let (mut prev_el, mut prev_er) = (0.0f32, 0.0f32);
    let (kp, ki, kd) = (robot_cfg.kp_inner, robot_cfg.ki_inner, robot_cfg.kd_inner);

    // =========== Filter Parameters ==============
    let dt: f32 = robot_cfg.joystick_control_dt_ms as f32 / 1000.0;
    let fc_hz: f32 = 3.0; // Match trajectory control - integral term helps smooth quantization noise
    let tau: f32 = 1.0 / (2.0 * PI * fc_hz);
    let alpha: f32 = dt / (tau + dt);

    // CRITICAL: Initialize prev encoder counts with CURRENT values to avoid velocity spike
    let mut prev_l = left_counter.load(Ordering::Relaxed);
    let mut prev_r = right_counter.load(Ordering::Relaxed);

    let mut omega_l_lp: f32 = 0.0;
    let mut omega_r_lp: f32 = 0.0;

    let mut last_tick = Instant::now();

    let expected_period_us =
        robot_cfg.joystick_control_dt_ms as u32 * 1_000;

    loop {
        ticker.next().await;

        const COMMAND_TIMEOUT_MS: u64 = 300;

        let now = Instant::now();
        let control_gap_us =
            now.duration_since(last_tick).as_micros() as u32;

        last_tick = now;

        CONTROL_TICKS.fetch_add(1, Ordering::Relaxed);

        CONTROL_MAX_GAP_US.fetch_max(
            control_gap_us,
            Ordering::Relaxed,
        );

        if control_gap_us > expected_period_us * 3 / 2 {
            CONTROL_MISSES.fetch_add(1, Ordering::Relaxed);
        }


        let received = latest_command();

        let (v, omega) =
            if now.duration_since(received.stamp).as_millis() > COMMAND_TIMEOUT_MS {
                (0.0, 0.0)
            }
            else {
                (received.v, received.omega)
            };

        // let odom = robotstate::read_odom().await;

        // Convert unicycle commands to differential drive wheel velocities
        let v_left = v - omega * robot_cfg.wheel_base / 2.0;
        let v_right = v + omega * robot_cfg.wheel_base / 2.0;

        // Convert linear wheel velocity to angular velocity (rad/s)
        let omega_l_target = v_left / robot_cfg.wheel_radius;
        let omega_r_target = v_right / robot_cfg.wheel_radius;

        // Get raw angular velocity of the wheels using encoder counts
        let ((omega_l_raw, omega_r_raw), (ln, rn)) = wheel_speed_from_counts_now(
            left_counter,
            right_counter,
            robot_cfg.encoder_cpr,
            prev_l,
            prev_r,
            dt,
        )
        .await;
        prev_l = ln;
        prev_r = rn;

        // =========== Low Pass Filter ==============
        omega_l_lp = omega_l_lp + alpha * (omega_l_raw - omega_l_lp);
        omega_r_lp = omega_r_lp + alpha * (omega_r_raw - omega_r_lp);

        // Error calculation
        let el = omega_l_target - omega_l_lp;
        let er = omega_r_target - omega_r_lp;

        // A centred joystick is an explicit stop command. Do not let stored
        // integral action or the filtered residual wheel speed drive the
        // motors again after the command has returned to zero.
        let stop_requested = omega_l_target.abs() < TELEOP_STOP_TARGET_RAD_S
            && omega_r_target.abs() < TELEOP_STOP_TARGET_RAD_S;

        let (u_l, u_r) = if stop_requested {
            il = 0.0;
            ir = 0.0;
            prev_el = 0.0;
            prev_er = 0.0;
            (0.0, 0.0)
        } else {
            // Error integration with anti-windup (helps smooth encoder quantization)
            il = (il + ki * dt * el).clamp(-2.0, 2.0);
            ir = (ir + ki * dt * er).clamp(-2.0, 2.0);

            // Error differentiation
            let dl = (el - prev_el) / dt;
            let dr = (er - prev_er) / dt;

            prev_el = el;
            prev_er = er;

            // PID control (normally the D term is disabled, kd = 0.0)
            (
                (kp * el + il + kd * dl).clamp(-1.0, 1.0),
                (kp * er + ir + kd * dr).clamp(-1.0, 1.0),
            )
        };

        //apply robot-specific motor direction corrections
        let duty_l = u_l * robot_cfg.motor_direction_left;
        let duty_r = u_r * robot_cfg.motor_direction_right;

        /*
        if v.abs() > 0.001 || omega.abs() > 0.001 {
            defmt::info!("Teleop: Cmd=(v:{}, ω:{}) -> TargetWheel=(L:{}, R:{}) -> Duty=(L:{}, R:{})",
                v, omega, omega_l_target, omega_r_target, duty_l, duty_r);
        }
        */

        motor.set_speed(duty_l, duty_r).await;

        let log_frame = ControlLogFrame {
            t_us: now.as_micros(),
            command_v: v,
            command_omega: omega,
            omega_l_target,
            omega_r_target,
            omega_l_measured: omega_l_lp,
            omega_r_measured: omega_r_lp,
            duty_l,
            duty_r,
            control_gap_us,
        };

        if CONTROL_LOG_CH.try_send(log_frame).is_err() {
            DROPPED_LOG_FRAMES.fetch_add(1, Ordering::Relaxed);
        }

        //Timer::after(Duration::from_millis(robot_cfg.joystick_control_dt_ms)).await;
    }
}
/* ========================== Joy Stick Speed Control Task =============================== */


#[embassy_executor::main]
async fn main(core0_spawner: embassy_executor::Spawner) {
    let p = embassy_rp::init(Default::default());

    // ================= Core 0: motor PWM =================
    let mut pwm_config = PwmConfig::default();
    pwm_config.top = 10_000;

    let pwm = Pwm::new_output_ab(
        p.PWM_SLICE7,
        p.PIN_14,
        p.PIN_15,
        pwm_config.clone(),
    );

    let (pwm_a, pwm_b) = pwm.split();

    let motor = init_motor(
        pwm_b.expect("left PWM channel"),
        Output::new(p.PIN_11, Level::Low),
        pwm_a.expect("right PWM channel"),
        Output::new(p.PIN_10, Level::Low),
        pwm_config.top,
    );

    // 初始化后明确输出零，避免上电时意外动作。
    motor.set_speed(0.0, 0.0).await;

    // ================= Core 0: encoder PIO =================
    let Pio {
        mut common,
        sm0,
        sm1,
        ..
    } = Pio::new(p.PIO0, Irqs);

    let encoders = EncoderPair::new(
        &mut common,
        sm0,
        sm1,
        p.PIN_12,
        p.PIN_13,
        p.PIN_8,
        p.PIN_9,
    );

    let encoder_counts = init_encoder_counts();

    start_encoder_irq(encoders);

    defmt::info!("multicore teleop test started");

    let core1_stack = CORE1_STACK.init(Stack::new());

    spawn_core1(p.CORE1, core1_stack, move || {
        let mut uart_config = UartConfig::default();
        uart_config.baudrate = 115_200;

        let uart = Uart::new(
            p.UART0,
            p.PIN_28,
            p.PIN_29,
            Irqs,
            p.DMA_CH0,
            p.DMA_CH1,
            uart_config,
        );

        init_uart_rx_irq();

        let uart = UART_CELL.init(
            AsyncMutex::new(uart)
        );

        let (sdlogger, robot_config) = match init_sd_logger(
            p.SPI0,
            p.PIN_18,
            p.PIN_19,
            p.PIN_20,
            p.PIN_21,
        ) {
            Ok((logger, config)) => {
                defmt::info!("Core 1: robot config loaded from SD");
                (Some(logger), config)
            }
            Err(_) => {
                defmt::warn!(
                    "Core 1: SD/config unavailable, using defaults"
                );

                (None, RobotConfig::default())
            }
        };

        // 同时承担 core1 初始化完成的握手。
        CONFIG_CH.try_send(robot_config).unwrap();

        let executor = CORE1_EXECUTOR.init(Executor::new());

        executor.run(|spawner| {
            spawner
                .spawn(uart_hw_task(uart))
                .unwrap();

            spawner
                .spawn(core1_teleop_uart_task(robot_config))
                .unwrap();

            if let Some(logger) = sdlogger {
                spawner
                    .spawn(core1_sd_logging_task(logger))
                    .unwrap();
            }
        })
    });


    let left_counter = encoder_counts.left;
    let right_counter = encoder_counts.right;

    let robot_config = CONFIG_CH.receive().await;

    defmt::info!(
        "Core 0: config received, kp={}, ki={}, dt={}ms, max_duty=({}, {})",
        robot_config.kp_inner,
        robot_config.ki_inner,
        robot_config.joystick_control_dt_ms,
        robot_config.motor_max_duty_left,
        robot_config.motor_max_duty_right,
    );

    core0_spawner
        .spawn(teleop_motor_control_task(
            motor,
            left_counter,
            right_counter,
            Some(robot_config),
        ))
        .unwrap();

    let mut ticker = Ticker::every(Duration::from_millis(1000));
    let mut previous_control_ticks = 0u32;

    loop {
        ticker.next().await;

        let dual_core_metrics = take_runtime_metrics();
        let updates = dual_core_metrics.uart_packets;

        let control_ticks =
            CONTROL_TICKS.load(Ordering::Relaxed);

        let control_hz =
            control_ticks.wrapping_sub(previous_control_ticks);

        previous_control_ticks = control_ticks;

        let max_control_gap_us =
            CONTROL_MAX_GAP_US.swap(0, Ordering::Relaxed);

        let control_misses =
            CONTROL_MISSES.swap(0, Ordering::Relaxed);

        let dropped_logs =
            DROPPED_LOG_FRAMES.load(Ordering::Relaxed);

        let left_count =
            encoder_counts.left.load(Ordering::Relaxed);

        let right_count =
            encoder_counts.right.load(Ordering::Relaxed);

        let command = latest_command();
        let command_age_ms = Instant::now().duration_since(command.stamp).as_millis();

        defmt::info!(
            "UART={}pkt/s CTRL={}Hz max_gap={}us misses={} dropped={} v={} omega={} age={}ms enc_l={} enc_r={}",
            updates,
            control_hz,
            max_control_gap_us,
            control_misses,
            dropped_logs,
            command.v,
            command.omega,
            command_age_ms,
            left_count,
            right_count,
        );
    }
}
