use crate::sdlog::{Clock, Sd};
use defmt::warn;
use embedded_sdmmc::Mode;

#[derive(Clone, Copy, Debug, defmt::Format)]
pub struct RobotConfig {
    pub dt_s: f32,
    pub wheel_base: f32,
    pub motor_direction_left: f32,
    pub motor_direction_right: f32,
    pub motor_max_duty_left: f32,
    pub motor_max_duty_right: f32,
    pub kx: f32,
    pub ky: f32,
    pub ktheta: f32,
}

impl Default for RobotConfig {
    fn default() -> Self {
        Self {
            dt_s: 0.1,
            wheel_base: 0.099,
            motor_direction_left: -1.0,
            motor_direction_right: -1.0,
            motor_max_duty_left: 0.8,
            motor_max_duty_right: 0.8,
            kx: 0.5,
            ky: 0.5,
            ktheta: 0.8,
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

            match k {
                "dt_s" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.dt_s = x;
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
                "kx" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.kx = x;
                    }
                }
                "ky" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.ky = x;
                    }
                }
                "ktheta" => {
                    if let Some(x) = parse_f32(v) {
                        cfg.ktheta = x;
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
                "RobotConfig loaded {} bytes: dt_s={}, wheel_base={}, K={}, {}, {}, max_duty={}, {}, motor_dir={}, {}",
                total,
                cfg.dt_s,
                cfg.wheel_base,
                cfg.kx,
                cfg.ky,
                cfg.ktheta,
                cfg.motor_max_duty_left,
                cfg.motor_max_duty_right,
                cfg.motor_direction_left,
                cfg.motor_direction_right
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
