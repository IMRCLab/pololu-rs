use embassy_sync::channel::Channel;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex as Raw, mutex::Mutex, signal::Signal};
use embassy_time::Instant;
use heapless::Vec;

#[derive(Clone, Copy, Debug, Default, defmt::Format)]
pub struct PoseAbs {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub roll: f32,  // rad
    pub pitch: f32, // rad
    pub yaw: f32,   // rad
}

// ====== EVENT DEFINITIONS ========
pub static FIRST_MESSAGE: Signal<Raw, ()> = Signal::new();
pub static START_EVENT: Signal<Raw, ()> = Signal::new();
pub static TRAJECTORY_CONTROL_EVENT: Signal<Raw, bool> = Signal::new(); // true = start, false = stop

// ====== STATE (PUSHED EACH TIME A NEW FRAME COMES IN) ======
pub static STATE_SIG: Signal<Raw, PoseAbs> = Signal::new();

// ====== LAST STATE =======
// pub static LAST_STATE: Mutex<Raw, PoseAbs> = Mutex::new(PoseAbs {
//     x: 0.0,
//     y: 0.0,
//     z: 0.0,
//     roll: 0.0,
//     pitch: 0.0,
//     yaw: 0.0,
// });
// ====== LAST STATE =======
pub static LAST_STATE: Mutex<Raw, RobotState> = Mutex::new(RobotState {
    pose: PoseAbs {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        roll: 0.0,
        pitch: 0.0,
        yaw: 0.0,
    },
    other_robots: Vec::new(),
});

// ====== COMPREHENSIVE ROBOT STATE (includes own pose + other robots' info) ======
// ====== COMPREHENSIVE ROBOT STATE (includes own pose + other robots' info) ======
// Robot information states for the 3 tracked robots (corresponding to robot IDs 7, 9, 10)
pub static ROBOT1: Mutex<Raw, RobotInfo> = Mutex::new(RobotInfo {
    robot_id: 7,
    distance_x: 0.0,
    distance_y: 0.0,
    vel_x: 0.0,
    vel_y: 0.0,
    angle: 0.0,
});
pub static ROBOT2: Mutex<Raw, RobotInfo> = Mutex::new(RobotInfo {
    robot_id: 9,
    distance_x: 0.0,
    distance_y: 0.0,
    vel_x: 0.0,
    vel_y: 0.0,
    angle: 0.0,
});
pub static ROBOT3: Mutex<Raw, RobotInfo> = Mutex::new(RobotInfo {
    robot_id: 10,
    distance_x: 0.0,
    distance_y: 0.0,
    vel_x: 0.0,
    vel_y: 0.0,
    angle: 0.0,
});

// ===== ROBOT DISTANCES ======
#[derive(Copy, Clone, Default, Debug, defmt::Format)]
pub struct RobotDistance {
    pub id: u8,
    pub x: f32,     // relative distance in x
    pub y: f32,     // relative distance in y
    pub angle: f32, // absolute angle (yaw) of the other robot
}

// ====== WHEEL COMMAND =======
#[derive(Copy, Clone, Debug)]
pub struct WheelCmd {
    pub omega_l: f32,
    pub omega_r: f32,
    pub stamp: Instant,
}

pub static WHEEL_CMD_CH: Channel<Raw, WheelCmd, 4> = Channel::new();

// ====== VELOCITY ======
#[derive(Copy, Clone, Default, Debug, defmt::Format)]
pub struct RobotInfo {
    pub robot_id: u8,    // Robot ID (7, 9, 10, etc.)
    pub distance_x: f32, // relative distance in x
    pub distance_y: f32, // relative distance in y
    pub vel_x: f32,      // relative velocity in x
    pub vel_y: f32,      // relative velocity in y
    pub angle: f32,      // other robot's yaw angle (global frame)
}

const MAX_TRACKED_ROBOTS: usize = 10; // Maximum number of robots we can track

#[derive(Clone, Debug)]
pub struct RobotState {
    pub pose: PoseAbs,                                    // Own robot's pose
    pub other_robots: Vec<RobotInfo, MAX_TRACKED_ROBOTS>, // Variable number of other robots
}

impl Default for RobotState {
    fn default() -> Self {
        Self {
            pose: PoseAbs::default(),
            other_robots: Vec::new(),
        }
    }
}

impl RobotState {
    /// Add or update a robot's information
    pub fn update_robot(&mut self, robot_info: RobotInfo) {
        // Find if robot already exists
        if let Some(existing) = self
            .other_robots
            .iter_mut()
            .find(|r| r.robot_id == robot_info.robot_id)
        {
            *existing = robot_info;
        } else {
            // Add new robot if we have space
            let _ = self.other_robots.push(robot_info);
        }
    }

    /// Get robot info by ID
    pub fn get_robot(&self, robot_id: u8) -> Option<&RobotInfo> {
        self.other_robots.iter().find(|r| r.robot_id == robot_id)
    }

    /// Remove a robot by ID
    pub fn remove_robot(&mut self, robot_id: u8) {
        self.other_robots.retain(|r| r.robot_id != robot_id);
    }
}

impl RobotInfo {
    /// Get the robot's position as a tuple (x, y)
    pub fn get_position(&self) -> (f32, f32) {
        (self.distance_x, self.distance_y)
    }

    /// Get the robot's velocity as a tuple (vel_x, vel_y)
    pub fn get_velocity(&self) -> (f32, f32) {
        (self.vel_x, self.vel_y)
    }

    /// Get the robot's angle
    pub fn get_angle(&self) -> f32 {
        self.angle
    }

    /// Get the robot's ID
    pub fn get_id(&self) -> u8 {
        self.robot_id
    }

    /// Get complete state information
    pub fn get_state(&self) -> ((f32, f32), (f32, f32), f32) {
        (
            (self.distance_x, self.distance_y),
            (self.vel_x, self.vel_y),
            self.angle,
        )
    }
}
