use core::cell::RefCell;
use core::f32::consts::PI;
use embassy_futures::block_on;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Instant, Ticker, Timer};
use libm::{cosf, sinf};
use static_cell::StaticCell;

use crate::motor::MotorController;
use crate::trajectory_reading::{Action, Pose, Trajectory};
use crate::trajectory_signal::{FIRST_MESSAGE, LAST_STATE, PoseAbs, START_EVENT, STATE_SIG};

// static POSE: Mutex<ThreadModeRawMutex, RefCell<Pose>> = Mutex::new(RefCell::new(Pose {
//     x: 0.0,
//     y: 0.0,
//     yaw: 0.0,
// }));

// =============================== Save Trajectory ================================
static TRAJ_REF: Mutex<ThreadModeRawMutex, RefCell<Option<&'static Trajectory>>> =
    Mutex::new(RefCell::new(None));
pub static TRAJ_READY: Signal<ThreadModeRawMutex, ()> = Signal::new();
static TRAJ_CELL: StaticCell<Trajectory> = StaticCell::new();

pub fn store_trajectory(traj: Trajectory) -> &'static Trajectory {
    TRAJ_CELL.init(traj)
}

pub fn register_trajectory(traj: &'static Trajectory) {
    block_on(async {
        let g = TRAJ_REF.lock().await;
        *g.borrow_mut() = Some(traj);
        TRAJ_READY.signal(());
    })
}

// =============================== Sample Struct ==================================
#[derive(Clone, Copy, Default)]
pub struct Sample {
    pub ts_ms: u32,
    pub v_d: f32,
    pub omega_d: f32,
    pub x: f32,
    pub y: f32,
    pub yaw: f32,
    pub u_l: f32,
    pub u_r: f32,
}
impl Sample {
    pub const fn zero() -> Self {
        Self {
            ts_ms: 0,
            v_d: 0.0,
            omega_d: 0.0,
            x: 0.0,
            y: 0.0,
            yaw: 0.0,
            u_l: 0.0,
            u_r: 0.0,
        }
    }
}
pub static SAMPLE: Mutex<ThreadModeRawMutex, RefCell<Sample>> =
    Mutex::new(RefCell::new(Sample::zero()));

// pub async fn update_pose(x: f32, y: f32, yaw: f32) {
//     let p = POSE.lock().await;
//     *p.borrow_mut() = Pose { x, y, yaw };
// }

/* ===================== Control Related Variables and Functions ================== */
pub struct Gains {
    pub kx: f32,
    pub ky: f32,
    pub ktheta: f32,
}

pub struct CtrlCfg {
    pub dt_s: f32,
    pub wheel_base_m: f32,
    pub gains: Gains,
}

#[embassy_executor::task]
pub async fn control_task(motor: MotorController, cfg: CtrlCfg) {
    TRAJ_READY.wait().await;

    let (states, actions) = {
        let g = TRAJ_REF.lock().await;
        let t = g.borrow();
        let tr = t.as_ref().expect("Trajectory is not set");
        (&tr.states, &tr.actions)
    };

    let len = core::cmp::min(states.len(), actions.len());
    defmt::info!("trajectory len={}", len);

    // ===== Wait for first pose and start event =====
    FIRST_MESSAGE.wait().await;
    START_EVENT.wait().await;

    let mut pose: PoseAbs = {
        let s = LAST_STATE.lock().await;
        *s
    };

    let mut ticker = Ticker::every(Duration::from_millis((cfg.dt_s * 1000.0) as u64));
    let start = Instant::now();

    loop {
        // check whether we have new pose or not, if not then keep waiting
        match select(STATE_SIG.wait(), ticker.next()).await {
            Either::First(p) => {
                pose = p;
                continue;
            }
            Either::Second(_) => {}
        }

        let t = Instant::now() - start;
        let t_sec = t.as_millis() as f32 / 1000.0;
        let mut i = (t_sec / cfg.dt_s) as usize;
        if i >= len {
            i = len - 1;
        }

        let Pose {
            x: x_d,
            y: y_d,
            yaw: theta_d,
        } = states[i];
        let Action {
            v: v_d,
            omega: om_d,
        } = actions[i];

        // let Pose { x, y, yaw: theta } = { *POSE.lock().await.borrow() };
        let x = pose.x;
        let y = pose.y;
        let theta = pose.yaw;

        /* =========== control code ============= */
        defmt::info!(
            "state x: {}; state y: {}; state theta: {}",
            x_d,
            y_d,
            theta_d,
        );
        let dx = x_d - x;
        let dy = y_d - y;
        let x_error = dx * cosf(theta) + dy * sinf(theta);
        let y_error = -dx * sinf(theta) + dy * cosf(theta);
        let delta_theta = wrap_angle(theta_d - theta);

        let v_ctrl = v_d * cosf(delta_theta) + cfg.gains.kx * x_error;
        let omega_ctrl = om_d
            + v_d * (cfg.gains.ky * y_error + cfg.gains.ktheta * sinf(delta_theta))
            + cfg.gains.ktheta * delta_theta;

        let u_l = v_ctrl - omega_ctrl * cfg.wheel_base_m * 0.5;
        let u_r = v_ctrl + omega_ctrl * cfg.wheel_base_m * 0.5;

        motor.set_speed(u_l, u_r).await;
        defmt::info!(
            "i={}, ul={}, ur={}, pose=({},{},{})",
            i,
            u_l,
            u_r,
            x,
            y,
            theta
        );

        {
            let s = SAMPLE.lock().await;
            *s.borrow_mut() = Sample {
                ts_ms: (t.as_millis() as u32),
                v_d,
                omega_d: om_d,
                x,
                y,
                yaw: theta,
                u_l,
                u_r,
            };
        }

        Timer::after(Duration::from_millis((cfg.dt_s * 1000.0) as u64)).await;
    }
}

#[inline]
fn wrap_angle(a: f32) -> f32 {
    let mut x = (a + PI) % (2.0 * PI);
    if x < 0.0 {
        x += 2.0 * PI;
    }
    x - PI
}
