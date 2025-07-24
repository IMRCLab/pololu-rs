use libm::{asinf, atan2f, sqrtf};

#[derive(Clone, Copy)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub struct Madgwick {
    q: Quaternion,
    beta: f32,
}

impl Madgwick {
    /// suggest setting beta to 0.05~0.1
    pub const fn new(beta: f32) -> Self {
        Self {
            q: Quaternion {
                w: 1.0,
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            beta,
        }
    }

    pub fn update(&mut self, gyro: [f32; 3], accel: [f32; 3], mag: [f32; 3], dt: f32) {
        let gyro_rps = [
            gyro[0] * 3.1415 / 180.0,
            gyro[1] * 3.1415 / 180.0,
            gyro[2] * 3.1415 / 180.0,
        ];

        let [gx, gy, gz] = gyro_rps;
        let [ax, ay, az] = normalize(accel);
        let [mx, my, mz] = normalize(mag);

        let q = self.q;
        let (q1, q2, q3, q4) = (q.w, q.x, q.y, q.z);

        // Reference Direction
        let hx = 2.0
            * (mx * (0.5 - q3 * q3 - q4 * q4)
                + my * (q2 * q3 - q1 * q4)
                + mz * (q2 * q4 + q1 * q3));
        let hy = 2.0
            * (mx * (q2 * q3 + q1 * q4)
                + my * (0.5 - q2 * q2 - q4 * q4)
                + mz * (q3 * q4 - q1 * q2));
        let bx = sqrtf(hx * hx + hy * hy);
        let bz = 2.0
            * (mx * (q2 * q4 - q1 * q3)
                + my * (q3 * q4 + q1 * q2)
                + mz * (0.5 - q2 * q2 - q3 * q3));

        // gradient descent
        let f1 = 2.0 * (q2 * q4 - q1 * q3) - ax;
        let f2 = 2.0 * (q1 * q2 + q3 * q4) - ay;
        let f3 = 2.0 * (0.5 - q2 * q2 - q3 * q3) - az;
        let f4 = 2.0 * bx * (0.5 - q3 * q3 - q4 * q4) + 2.0 * bz * (q2 * q4 - q1 * q3) - mx;
        let f5 = 2.0 * bx * (q2 * q3 - q1 * q4) + 2.0 * bz * (q1 * q2 + q3 * q4) - my;
        let f6 = 2.0 * bx * (q1 * q3 + q2 * q4) + 2.0 * bz * (0.5 - q2 * q2 - q3 * q3) - mz;

        let s1 = -2.0 * (q3 * f1 - q2 * f2) - 2.0 * bz * q3 * f4
            + (-2.0 * bx * q4 + 2.0 * bz * q2) * f5
            + 2.0 * bx * q3 * f6;
        let s2 = 2.0 * (q4 * f1 + q1 * f2 - 2.0 * q2 * f3)
            + 2.0 * bz * q4 * f4
            + (2.0 * bx * q3 + 2.0 * bz * q1) * f5
            + (2.0 * bx * q4 - 4.0 * bz * q2) * f6;
        let s3 = -2.0 * (q1 * f1 - q4 * f2 - 2.0 * q3 * f3)
            + (-4.0 * bx * q3 - 2.0 * bz * q1) * f4
            + (2.0 * bx * q2 + 2.0 * bz * q4) * f5
            + (2.0 * bx * q1 - 4.0 * bz * q3) * f6;
        let s4 = 2.0 * (q2 * f1 + q3 * f2)
            + (-2.0 * bx * q4 + 2.0 * bz * q2) * f4
            + (-2.0 * bx * q1 + 2.0 * bz * q3) * f5;

        let norm_s = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
        let (s1, s2, s3, s4) = (s1 / norm_s, s2 / norm_s, s3 / norm_s, s4 / norm_s);

        // quaternion gradient
        let q_dot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1;
        let q_dot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2;
        let q_dot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3;
        let q_dot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4;

        // Integration
        self.q.w += q_dot1 * dt;
        self.q.x += q_dot2 * dt;
        self.q.y += q_dot3 * dt;
        self.q.z += q_dot4 * dt;

        self.q = normalize_quaternion(self.q);
    }

    /// get euler angles from quaternion
    pub fn get_euler(&self) -> [f32; 3] {
        let q = self.q;

        let roll = atan2f(
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y),
        ) * (180.0 / 3.1415);
        let pitch = asinf(2.0 * (q.w * q.y - q.z * q.x)) * (180.0 / 3.1415);
        let yaw = atan2f(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        ) * (180.0 / 3.1415);

        [roll, pitch, yaw]
    }

    /// get quaternion of the madgwick struct
    pub fn get_quaternion(&self) -> Quaternion {
        self.q
    }
}

/// vector normalization
pub fn normalize(v: [f32; 3]) -> [f32; 3] {
    let norm = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if norm == 0.0 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0] / norm, v[1] / norm, v[2] / norm]
    }
}

/// quaternion normalization
pub fn normalize_quaternion(q: Quaternion) -> Quaternion {
    let norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    Quaternion {
        w: q.w / norm,
        x: q.x / norm,
        y: q.y / norm,
        z: q.z / norm,
    }
}
