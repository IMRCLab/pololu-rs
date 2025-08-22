#![no_std]
#![no_main]

use {defmt_rtt as _, panic_probe as _};

use embassy_executor::Spawner;
use embassy_rp::init;

use pololu3pi2040_rs::init::init_all;
use pololu3pi2040_rs::trajectory_uart::{UartCfg, uart_motioncap_receiving_task};

use pololu3pi2040_rs::diffdrive_cascade::{
    ControlMode, diffdrive_outer_loop, encoder_left_task_cascade, encoder_right_task_cascade,
    mocap_update_task, wheel_speed_inner_loop,
};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = init(Default::default());
    let devices = init_all(p);

    // 启动接收 mocap 的 UART 任务（它会更新 STATE_SIG / LAST_STATE）
    // Start uart task for receiving pose information from UART (will update STATE_SIG and LAST_STATE)
    spawner
        .spawn(uart_motioncap_receiving_task(
            devices.uart,
            UartCfg { robot_id: 8 },
        ))
        .unwrap();

    // 启动编码器读取（PioEncoder）——累加到 ENC_LEFT/RIGHT_DELTA
    // Start encoder tasks (will be accumulated to ENC_LEFT/RIGHT_DELTA)
    let enc = devices.encoders;
    spawner
        .spawn(encoder_left_task_cascade(enc.encoder_left))
        .unwrap();
    spawner
        .spawn(encoder_right_task_cascade(enc.encoder_right))
        .unwrap();

    // 启动 mocap 更新转发（把 STATE_SIG 的数据写回 LAST_STATE，供外环读取）
    // Start Mocap update task.
    spawner.spawn(mocap_update_task()).unwrap();

    // 启动内环：5 ms 速度闭环（用 encoder 追踪目标轮速，唯一写 PWM 的地方）
    // Start inner loop.
    spawner
        .spawn(wheel_speed_inner_loop(devices.motor))
        .unwrap();

    // 启动外环：50 ms 轨迹控制（算 ωl/ωr 并通过 WHEEL_CMD_CH 发给内环）
    // Start outer loop.
    spawner
        .spawn(diffdrive_outer_loop(None, ControlMode::WithInnerSpeedLoop))
        .unwrap();
    // spawner
    //     .spawn(diffdrive_outer_loop(
    //         Some(devices.motor),
    //         ControlMode::DirectDuty,
    //     ))
    //     .unwrap();
}
