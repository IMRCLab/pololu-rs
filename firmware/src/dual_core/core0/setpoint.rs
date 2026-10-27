use core::cell::RefCell;

use embassy_sync::blocking_mutex::{
    Mutex as BlockingMutex,
    raw::CriticalSectionRawMutex,
};
use static_cell::StaticCell;

use crate::math::SO2;
use crate::trajectory_reading::{Action, Pose, Trajectory};

use super::control_types::{
    DiffdriveCascade,
    DiffdriveSetpointCascade,
    DiffdriveStateCascade,
    PointCascade,
};

static TRAJECTORY_STORAGE: StaticCell<Trajectory> = StaticCell::new();
static TRAJECTORY_REF: BlockingMutex<
    CriticalSectionRawMutex,
    RefCell<Option<&'static Trajectory>>,
> = BlockingMutex::new(RefCell::new(None));

/// Store the trajectory received during Core 0 startup for later mode runs.
pub fn register_trajectory(trajectory: Trajectory) {
    let trajectory = TRAJECTORY_STORAGE.init(trajectory);
    TRAJECTORY_REF.lock(|slot| {
        *slot.borrow_mut() = Some(trajectory);
    });
}

pub fn trajectory() -> Option<&'static Trajectory> {
    TRAJECTORY_REF.lock(|slot| {
        *slot.borrow()
    })
}

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
            Self::Figure8 {
                duration,
                ax,
                ay,
                x0,
                y0,
                phi0,
            } => robot.figure8_reference(t, *duration, *ax, *ay, *x0, *y0, *phi0),
            Self::SdCard {
                states,
                actions,
                dt_s,
            } => {
                let mut idx = (t / dt_s) as usize;
                if idx >= actions.len() {
                    idx = actions.len() - 1;
                }

                let setpoint_pose = states[idx + 1];
                let setpoint_action = actions[idx];

                DiffdriveSetpointCascade {
                    des: DiffdriveStateCascade {
                        x: setpoint_pose.x,
                        y: setpoint_pose.y,
                        theta: SO2::new(setpoint_pose.yaw),
                    },
                    vdes: setpoint_action.v,
                    wdes: setpoint_action.omega,
                }
            }
            Self::Spin {
                wd_spin,
                x0,
                y0,
                phi0,
                ..
            } => robot.spinning_at_wd(*wd_spin, t, *x0, *y0, *phi0),
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
