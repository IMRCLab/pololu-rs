use crate::sdlog::{Clock, Sd};
use embedded_sdmmc::{Directory, Mode, ShortFileName};
use heapless::Vec;
use serde::Deserialize;

// Could not be too large, should use static memory to support longer trajectory
pub const MAX_POINTS: usize = 128;

#[derive(Clone, Copy, Debug, Default)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Action {
    pub v: f32,
    pub omega: f32,
}

#[derive(Clone, Debug, Default)]
pub struct Trajectory {
    pub states: Vec<Pose, MAX_POINTS>,
    pub actions: Vec<Action, MAX_POINTS>,
}

#[derive(Deserialize)]
struct Root {
    #[serde(default)]
    result: Vec<Record, 1>,
}
#[derive(Deserialize, Clone)]
struct Record {
    #[serde(default)]
    states: Vec<[f32; 3], MAX_POINTS>,
    #[serde(default)]
    actions: Vec<[f32; 2], MAX_POINTS>,
    #[serde(default)]
    _timestamp: Option<f32>,
    #[serde(default)]
    _cost: Option<f32>,
    #[serde(default)]
    _num_states: Option<usize>,
    #[serde(default)]
    _num_actions: Option<usize>,
    // #[serde(borrow)]
    // _phantom: core::marker::PhantomData<&'a ()>,
}

impl Trajectory {
    pub fn from_json(mut json: &[u8]) -> Result<Self, &'static str> {
        if json.len() >= 3 && &json[..3] == b"\xEF\xBB\xBF" {
            json = &json[3..];
        }

        let (root, _rem) = serde_json_core::from_slice::<Root>(json).map_err(|e| {
            defmt::warn!("serde err: {:?}", defmt::Debug2Format(&e));
            "JSON parse failed"
        })?;

        let first = root.result.first().ok_or("result empty")?.clone();

        let mut states: Vec<Pose, MAX_POINTS> = Vec::new();
        let mut actions: Vec<Action, MAX_POINTS> = Vec::new();

        for s in first.states.into_iter() {
            states
                .push(Pose {
                    x: s[0],
                    y: s[1],
                    yaw: s[2],
                })
                .map_err(|_| "states overflow")?;
        }
        for a in first.actions.into_iter() {
            actions
                .push(Action {
                    v: a[0],
                    omega: a[1],
                })
                .map_err(|_| "actions overflow")?;
        }
        if states.is_empty() || actions.is_empty() {
            return Err("empty states/actions");
        }
        Ok(Self { states, actions })
    }
}

/* ============= read trajectory file during the intialization ============== */
pub fn sd_read_file_into_8_3_with_dir<
    'a,
    SD,
    CLK,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    dir: &mut Directory<'a, Sd, Clock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    name_8_3: &str,
    buf: &mut [u8],
) -> Result<usize, &'static str> {
    let short = ShortFileName::create_from_str(name_8_3).map_err(|_| "short name invalid")?;

    let total = (|| -> Result<usize, &'static str> {
        let mut file = dir
            .open_file_in_dir(&short, Mode::ReadOnly)
            .map_err(|_| "open file failed")?;

        let mut total = 0usize;
        let mut chunk = [0u8; 512];
        loop {
            let n = file.read(&mut chunk).map_err(|_| "read failed")?;
            if n == 0 {
                break;
            }
            if total + n > buf.len() {
                return Err("buffer size too short");
            }
            buf[total..total + n].copy_from_slice(&chunk[..n]);
            total += n;
        }
        Ok(total)
    })()?;

    Ok(total)
}

pub fn load_trajectory_with_dir<
    'a,
    SD,
    CLK,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    dir: &mut embedded_sdmmc::Directory<'a, Sd, Clock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    name_8_3: &str,
    scratch: &mut [u8],
) -> Result<Trajectory, &'static str> {
    let len = file_len_with_dir::<SD, CLK, MAX_DIRS, MAX_FILES, MAX_VOLUMES>(dir, name_8_3)?;
    if (len as usize) > scratch.len() {
        return Err("trajectory too large for scratch");
    }
    let n = sd_read_file_into_8_3_with_dir::<SD, CLK, MAX_DIRS, MAX_FILES, MAX_VOLUMES>(
        dir,
        name_8_3,
        &mut scratch[..len as usize],
    )?;

    Trajectory::from_json(&scratch[..n])
}

/* ============ pre-check the length of the 8.3 json file (in bytes) =============== */
pub fn file_len_with_dir<
    'a,
    SD,
    CLK,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    dir: &mut Directory<'a, Sd, Clock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    name_8_3: &str,
) -> Result<u32, &'static str> {
    let short = ShortFileName::create_from_str(name_8_3).map_err(|_| "short name invalid")?;

    let len = {
        let f = dir
            .open_file_in_dir(&short, Mode::ReadOnly)
            .map_err(|_| "open file failed")?;
        f.length()
    };

    Ok(len)
}

/* After reading and parsing, return (Trajectory, number of states, number of actions) to make logging easier.” */
pub fn load_trajectory_with_dir_count<
    'a,
    SD,
    CLK,
    const MAX_DIRS: usize,
    const MAX_FILES: usize,
    const MAX_VOLUMES: usize,
>(
    dir: &mut Directory<'a, Sd, Clock, MAX_DIRS, MAX_FILES, MAX_VOLUMES>,
    name_8_3: &str,
    scratch: &mut [u8],
) -> Result<(Trajectory, usize, usize), &'static str> {
    let traj = load_trajectory_with_dir::<SD, CLK, MAX_DIRS, MAX_FILES, MAX_VOLUMES>(
        dir, name_8_3, scratch,
    )?;
    let n_s = traj.states.len();
    let n_a = traj.actions.len();

    Ok((traj, n_s, n_a))
}
