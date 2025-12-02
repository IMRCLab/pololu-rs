use crate::sdlog::{Clock, Sd};
use defmt::warn;
use embedded_sdmmc::Mode;

use crate::robot_parameters_default::robot_constants::*;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct RobotConfig {
    pub robot_id: u8,
    pub joystick_control_dt_ms: u64,
    pub traj_following_dt_s: f32,
    pub wheel_radius: f32,
    pub wheel_base: f32,
    pub motor_direction_left: f32,
    pub motor_direction_right: f32,
    pub motor_max_duty_left: f32,
    pub motor_max_duty_right: f32,
    pub k_clip: f32,
    pub kp_inner: f32,
    pub ki_inner: f32,
    pub kd_inner: f32,
    pub kx_traj: f32,
    pub ky_traj: f32,
    pub ktheta_traj: f32,
    pub gear_ratio: f32,
    pub encoder_cpr: f32,
    pub max_speed: f32,
    pub max_omega: f32,
    pub wheel_max: f32,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            robot_id: 7,
            joystick_control_dt_ms: JOYSTICK_CONTROL_DT, // Should be a integer but written in f32
            traj_following_dt_s: TRAJ_FOLLOWING_DT_S,
            wheel_radius: WHEEL_RADIUS,
            wheel_base: WHEEL_BASE,
            motor_direction_left: MOTOR_DIRECTION_LEFT,
            motor_direction_right: MOTOR_DIRECTION_RIGHT,
            motor_max_duty_left: 1.0,
            motor_max_duty_right: 1.0,
            k_clip: K_CLIP,
            kp_inner: KP_INNER,
            ki_inner: KI_INNER,
            kd_inner: KD_INNER,
            kx_traj: KX_TRAJ,
            ky_traj: KY_TRAJ,
            ktheta_traj: KTHETA_TRAJ,
            gear_ratio: GEAR_RATIO,
            encoder_cpr: -GEAR_RATIO * 12.0,
            max_speed: MAX_SPEED,
            max_omega: MAX_OMEGA,
            wheel_max: WHEEL_MAX,
        }
    }
}

fn trim_space(mut s: &[u8]) -> &[u8] {
    while let Some((&b, rest)) = s.split_first() {
        if !is_space(b) {
            break;
        }
        s = rest;
    }
    while let Some((&b, _rest)) = s.split_last() {
        if !is_space(b) {
            break;
        }
        s = &s[..s.len() - 1];
    }
    s
}

#[inline]
fn is_space(b: u8) -> bool {
    matches!(b, b' ' | b'\t' | b'\r' | b'\n')
}

fn memchr(byte: u8, s: &[u8]) -> Option<usize> {
    for (i, &b) in s.iter().enumerate() {
        if b == byte {
            return Some(i);
        }
    }
    None
}

pub fn parse_robot_config_from_bytes(buf: &[u8]) -> RobotConfig {
    let mut cfg = RobotConfig::default();

    for line in buf.split(|&b| b == b'\n') {
        let line = trim_space(line);
        if line.is_empty() || line[0] == b'#' || line[0] == b';' {
            continue;
        }
        if let Some(eq_pos) = memchr(b'=', line) {
            let (k_bytes, v_bytes) = (&line[..eq_pos], &line[eq_pos + 1..]);
            let k = core::str::from_utf8(trim_space(k_bytes)).unwrap_or("");
            let v = core::str::from_utf8(trim_space(v_bytes)).unwrap_or("");

            let parse_f32 = |s: &str| -> Option<f32> { s.parse::<f32>().ok() };
            let parse_u64 = |s: &str| -> Option<u64> { s.parse::<u64>().ok() };

            match k {
                "robot_id" => {
                    if let Some(x) = parse_u64(v) {
                        cfg.robot_id = x as u8;
                    }
                }
                "joystick_control_dt_ms" => {
                    if let Some(x) = parse_u64(v) {
                        cfg.joystick_control_dt_ms = x;
                    }
                }
                "traj_following_dt_s" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.traj_following_dt_s = x;
                    }
                }
                "wheel_radius" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.wheel_radius = x;
                    }
                }
                "wheel_base" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.wheel_base = x;
                    }
                }
                "motor_direction_left" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.motor_direction_left = x;
                    }
                }
                "motor_direction_right" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.motor_direction_right = x;
                    }
                }
                "motor_max_duty_left" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.motor_max_duty_left = x;
                    }
                }
                "motor_max_duty_right" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.motor_max_duty_right = x;
                    }
                }
                "k_clip" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.k_clip = x;
                    }
                }
                "kp_inner" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.kp_inner = x;
                    }
                }
                "ki_inner" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.ki_inner = x;
                    }
                }
                "kd_inner" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.kd_inner = x;
                    }
                }
                "kx_traj" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.kx_traj = x;
                    }
                }
                "ky_traj" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.ky_traj = x;
                    }
                }
                "ktheta_traj" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.ktheta_traj = x;
                    }
                }
                "gear_ratio" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.gear_ratio = x;
                    }
                }
                "encoder_cpr" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.encoder_cpr = x;
                    }
                }
                "max_speed" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.max_speed = x;
                    }
                }
                "max_omega" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.max_omega = x;
                    }
                }
                "wheel_max" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.wheel_max = x;
                    }
                }
                _ => {
                    defmt::warn!("Unknown config key: {}", k);
                }
            }
        }
    }
    cfg
}

pub fn load_robot_config_with_dir<
    'a,
    SD,
    CLK,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    dir: &mut embedded_sdmmc::Directory<'a, Sd, Clock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    scratch: &mut [u8],
) -> RobotConfig {
    let fname = "ROBOTCFG.CFG";

    let mut total = 0usize;

    match dir.open_file_in_dir(fname, Mode::ReadOnly) {
        Ok(mut file) => {
            loop {
                if total >= scratch.len() {
                    warn!("Config too large, truncated at {} bytes", total);
                    break;
                }
                match file.read(&mut scratch[total..]) {
                    Ok(0) => break,
                    Ok(n) => total += n,
                    Err(e) => {
                        warn!("Config read error: {:?}", defmt::Debug2Format(&e));
                        break;
                    }
                }
            }

            let cfg = parse_robot_config_from_bytes(&scratch[..total]);
            defmt::info!(
                "RobotConfig loaded {} bytes: 
                joystick_dt_ms={}, traj_following_dt_s={}, 
                motor_max_duty_left={}, motor_max_duty_right={},
                wheel_radius={}, wheel_base={}, motor_dir_left={}, motor_dir_right={}, k_clip={},
                gear_ratio={}, encoder_cpr={}, max_speed={}, max_omega={}, wheel_max(rad/s)={},
                K_inner=({}, {}, {}), K_outer=({}, {}, {})",
                total,
                cfg.joystick_control_dt_ms,
                cfg.traj_following_dt_s,
                cfg.motor_max_duty_left,
                cfg.motor_max_duty_right,
                cfg.wheel_radius,
                cfg.wheel_base,
                cfg.motor_direction_left,
                cfg.motor_direction_right,
                cfg.k_clip,
                cfg.gear_ratio,
                cfg.encoder_cpr,
                cfg.max_speed,
                cfg.max_omega,
                cfg.wheel_max,
                cfg.kp_inner,
                cfg.ki_inner,
                cfg.kd_inner,
                cfg.kx_traj,
                cfg.ky_traj,
                cfg.ktheta_traj,
            );
            cfg
        }
        Err(e) => {
            warn!(
                "Config file '{}' not found/open fail: {:?}",
                fname,
                defmt::Debug2Format(&e)
            );
            RobotConfig::default()
        }
    }
}
