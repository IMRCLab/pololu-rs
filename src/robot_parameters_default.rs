// Robot constants for diffdrive control
#[cfg(feature = "zumo")]
pub mod robot_constants {
    pub const JOYSTICK_CONTROL_DT: u64 = 20;
    pub const TRAJ_FOLLOWING_DT_S: f32 = 0.1; // Default to 100ms time step
    pub const WHEEL_RADIUS: f32 = 0.02;
    pub const WHEEL_BASE: f32 = 0.099;
    pub const MOTOR_DIRECTION_LEFT: f32 = -1.0;
    pub const MOTOR_DIRECTION_RIGHT: f32 = -1.0;
    pub const K_CLIP: f32 = 1.14;
    pub const KP_INNER: f32 = 0.2;
    pub const KI_INNER: f32 = 0.7;
    pub const KD_INNER: f32 = 0.0;
    pub const KX_TRAJ: f32 = 0.1;
    pub const KY_TRAJ: f32 = 0.8;
    pub const KTHETA_TRAJ: f32 = 0.1;
    pub const GEAR_RATIO: f32 = 100.31;
    pub const ENCODER_CPR: f32 = GEAR_RATIO * 12.0; // Made positive - motor direction handles sign
    pub const MAX_SPEED: f32 = 0.65; // 65 cm/s, according to datasheet 75:1 Gear Ratio
    pub const MAX_OMEGA: f32 = 1.0;
    //find out the actual maximum speed of Zumo
    pub const WHEEL_MAX: f32 = MAX_SPEED / WHEEL_RADIUS; //angular velocity in [rad/s]
}

#[cfg(feature = "three-pi")]
pub mod robot_constants {
    pub const JOYSTICK_CONTROL_DT: u64 = 20;
    pub const TRAJ_FOLLOWING_DT_S: f32 = 0.1;
    pub const WHEEL_RADIUS: f32 = 0.016; // 16mm wheel radius 
    pub const WHEEL_BASE: f32 = 0.0842; // 84.2mm wheelbase
    pub const MOTOR_DIRECTION_LEFT: f32 = 1.0;
    pub const MOTOR_DIRECTION_RIGHT: f32 = 1.0;
    pub const K_CLIP: f32 = 1.0;
    pub const KP_INNER: f32 = 0.01;
    pub const KI_INNER: f32 = 0.02;
    pub const KD_INNER: f32 = 0.0;
    pub const KX_TRAJ: f32 = 1.0;
    pub const KY_TRAJ: f32 = 1.0;
    pub const KTHETA_TRAJ: f32 = 2.0;
    pub const GEAR_RATIO: f32 = 15.25;
    pub const ENCODER_CPR: f32 = -GEAR_RATIO * 12.0; // Made positive - motor direction handles sign
    pub const MAX_SPEED: f32 = 4.0; // 4.0 m/s, according to datasheet  Gear Ratio
    pub const MAX_OMEGA: f32 = 1.0;
    //find out the actual maximum speed of Zumo
    pub const WHEEL_MAX: f32 = MAX_SPEED / WHEEL_RADIUS; //angular velocity in [rad/s]
}

#[cfg(not(any(feature = "zumo", feature = "three-pi")))]
pub mod robot_constants {
    pub const JOYSTICK_CONTROL_DT: u64 = 20;
    pub const TRAJ_FOLLOWING_DT_S: f32 = 0.1;
    pub const DT_S: f32 = 0.1; // Default to 100ms time step
    pub const WHEEL_RADIUS: f32 = 0.02;
    pub const WHEEL_BASE: f32 = 0.099;
    pub const MOTOR_DIRECTION_LEFT: f32 = -1.0;
    pub const MOTOR_DIRECTION_RIGHT: f32 = -1.0;
    pub const K_CLIP: f32 = 1.14;
    pub const KP_INNER: f32 = 0.2;
    pub const KI_INNER: f32 = 0.7;
    pub const KD_INNER: f32 = 0.0;
    pub const KX_TRAJ: f32 = 10.0;
    pub const KY_TRAJ: f32 = 10.0;
    pub const KTHETA_TRAJ: f32 = 5.0;
    pub const GEAR_RATIO: f32 = 100.31;
    pub const ENCODER_CPR: f32 = -GEAR_RATIO * 12.0; // Made positive - motor direction handles sign
    pub const MAX_SPEED: f32 = 0.85; // 65 cm/s, according to datasheet 75:1 Gear Ratio
    pub const MAX_OMEGA: f32 = 1.0;
    //find out the actual maximum speed of Zumo
    pub const WHEEL_MAX: f32 = MAX_SPEED / WHEEL_RADIUS; // Maximum wheel speed in m/s scaling is experimentally derived

    // pub const JOYSTICK_CONTROL_DT: u64 = 20;
    // pub const TRAJ_FOLLOWING_DT_S: f32 = 0.1;
    // pub const WHEEL_RADIUS: f32 = 0.016; // 16mm wheel radius
    // pub const WHEEL_BASE: f32 = 0.0842; // 84.2mm wheelbase
    // pub const MOTOR_DIRECTION_LEFT: f32 = 1.0;
    // pub const MOTOR_DIRECTION_RIGHT: f32 = 1.0;
    // pub const K_CLIP: f32 = 1.10;
    // pub const KP_INNER: f32 = 0.01;
    // pub const KI_INNER: f32 = 0.02;
    // pub const KD_INNER: f32 = 0.0;
    // pub const KX_TRAJ: f32 = 1.0;
    // pub const KY_TRAJ: f32 = 1.0;
    // pub const KTHETA_TRAJ: f32 = 2.0;
    // pub const GEAR_RATIO: f32 = 15.25;
    // pub const ENCODER_CPR: f32 = -GEAR_RATIO * 12.0;
    // pub const MAX_SPEED: f32 = 4.0; // 4.0 m/s, according to datasheet Gear Ratio
    // pub const MAX_OMEGA: f32 = 1.0;
    // //find out the actual maximum speed of Zumo
    // pub const WHEEL_MAX: f32 = MAX_SPEED / WHEEL_RADIUS; //angular velocity in [rad/s]
}
