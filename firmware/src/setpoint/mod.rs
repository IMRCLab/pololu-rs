use crate::control_types::{DiffdriveCascade, DiffdriveSetpointCascade, PointCascade};
use crate::trajectory_reading::{Action, Pose, Trajectory};
use core::cell::RefCell;
use embassy_futures::block_on;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use static_cell::StaticCell;

// =============================== Save Trajectory ================================
pub static TRAJ_REF: Mutex<ThreadModeRawMutex, RefCell<Option<&'static Trajectory>>> =
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

pub async fn trajectory_start_pose() -> Option<(f32, f32, f32)> {
    let g = TRAJ_REF.lock().await;
    let t = g.borrow();
    t.as_ref().and_then(|tr| {
        tr.states.first().map(|s| (s.x, s.y, s.yaw))
    })
}
// =================================================================================

pub enum SetpointFinder {
    Figure8 {
        duration: f32,
        ax: f32,
        ay: f32,
        x0: f32,
        y0: f32,
        phi0: f32,
    },
    SdCard {
        states: &'static [Pose],
        actions: &'static [Action],
        dt_s: f32,
    },
    Spin {
        duration: f32,
        wd_spin: f32,
        x0: f32,
        y0: f32,
        phi0: f32,
    },
    GoTo {
        bezier: PointCascade,
        duration: f32,
    },
}

impl SetpointFinder {
    pub fn get_setpoint(
        &self,
        robot: &DiffdriveCascade,
        t: f32,
    ) -> DiffdriveSetpointCascade {
        match self {
            Self::Figure8 { duration, ax, ay, x0, y0, phi0 } => {
                robot.figure8_reference(t, *duration, *ax, *ay, *x0, *y0, *phi0)
            }
            Self::SdCard { states, actions, dt_s } => {
                let mut idx = (t / dt_s) as usize;
                if idx >= actions.len() {
                    idx = actions.len() - 1;
                }
                
                let setpoint_pose = states[idx + 1];
                let setpoint_action = actions[idx];
                
                DiffdriveSetpointCascade {
                    des: crate::control_types::DiffdriveStateCascade {
                        x: setpoint_pose.x,
                        y: setpoint_pose.y,
                        theta: crate::math::SO2::new(setpoint_pose.yaw),
                    },
                    vdes: setpoint_action.v,
                    wdes: setpoint_action.omega,
                }
            }
            Self::Spin { wd_spin, x0, y0, phi0, .. } => {
                robot.spinning_at_wd(*wd_spin, t, *x0, *y0, *phi0)
            }
            Self::GoTo { bezier, duration } => {
                robot.beziercurve(bezier.clone(), t, *duration)
            }
        }
    }

    pub fn duration(&self) -> Option<f32> {
        match self {
            Self::Figure8 { duration, .. } => Some(*duration),
            Self::SdCard { states, dt_s, .. } => Some(states.len() as f32 * dt_s),
            Self::Spin { duration, .. } => Some(*duration),
            Self::GoTo { duration, .. } => Some(*duration),
        }
    }
}
