use super::madgwick::normalize;
use core::f32::consts::PI;
use libm::{atan2f, cosf, sinf};

pub struct ComplementaryFilter {
    pitch: f32,
    roll: f32,
    yaw: f32,
    alpha: f32,
}

impl ComplementaryFilter {
    pub fn new(alpha: f32) -> Self {
        Self {
            pitch: 0.0,
            roll: 0.0,
            yaw: 0.0,
            alpha,
        }
    }

    pub fn update(&mut self, gyro: [f32; 3], accel: [f32; 3], mag: [f32; 3], dt: f32) {
        let (gx, gy, gz) = (
            gyro[0] * PI / 180.0,
            gyro[1] * PI / 180.0,
            gyro[2] * PI / 180.0,
        );

        let accel_normalized = normalize(accel);
        let mag = normalize(mag);

        self.pitch += gx * dt;
        self.roll += gy * dt;

        let acc_pitch = libm::atan2f(
            accel_normalized[0],
            libm::sqrtf(
                accel_normalized[1] * accel_normalized[1]
                    + accel_normalized[2] * accel_normalized[2],
            ),
        );
        let acc_roll = libm::atan2f(
            -accel_normalized[1],
            libm::sqrtf(
                accel_normalized[0] * accel_normalized[0]
                    + accel_normalized[2] * accel_normalized[2],
            ),
        );

        self.pitch = self.alpha * self.pitch + (1.0 - self.alpha) * acc_pitch;
        self.roll = self.alpha * self.roll + (1.0 - self.alpha) * acc_roll;

        let xh = mag[0] * cosf(self.pitch) + mag[2] * sinf(self.pitch);
        let yh = mag[0] * sinf(self.roll) * sinf(self.pitch) + mag[1] * cosf(self.roll)
            - mag[2] * sinf(self.roll) * cosf(self.pitch);

        let mag_yaw = atan2f(yh, xh);

        self.yaw += gz * dt;
        self.yaw = self.alpha * self.yaw + (1.0 - self.alpha) * mag_yaw;
    }

    pub fn get_angles_deg(&self) -> (f32, f32, f32) {
        let deg = 180.0 / PI;
        (self.pitch * deg, self.roll * deg, self.yaw * deg)
    }
}
