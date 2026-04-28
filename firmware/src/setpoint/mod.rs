use core::cell::RefCell;
use embassy_futures::block_on;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use static_cell::StaticCell;

use crate::trajectory_reading::Trajectory;

pub mod providers;

// =============================== Save Trajectory ================================
pub static TRAJ_REF: Mutex<ThreadModeRawMutex, RefCell<Option<&'static Trajectory>>> =
    Mutex::new(RefCell::new(None));
pub static TRAJ_READY: Signal<ThreadModeRawMutex, ()> = Signal::new();
pub static TRAJ_CELL: StaticCell<Trajectory> = StaticCell::new();

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

/// Read the first state from the loaded trajectory (for EKF init fallback).
/// Returns `None` if no trajectory has been registered yet.
pub async fn trajectory_start_pose() -> Option<(f32, f32, f32)> {
    let g = TRAJ_REF.lock().await;
    let t = g.borrow();
    t.as_ref().and_then(|tr| {
        tr.states.first().map(|s| (s.x, s.y, s.yaw))
    })
}
