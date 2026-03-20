//! 3-state Extended Kalman Filter for differential-drive robot pose fusion.
//!
//! State vector: x = [x, y, θ]ᵀ
//! Prediction: unicycle kinematics using body-frame (v, ω)
//! Update: absolute pose measurement (e.g. motion capture) with H = I₃

use crate::math::{wrap_angle, Mat3, Vec3};
use libm::{cosf, sinf};

// ================================== EKF ======================================

/// Extended Kalman Filter for a 2D robot.
///
/// State: `[x, y, θ]`
/// Input: `(v, ω)` — body-frame linear and angular velocity
/// Measurement: `[x, y, θ]` — absolute pose (e.g. from motion capture)
#[derive(Debug, Clone)]
pub struct Ekf {
    /// State estimate [x, y, θ]
    pub x: Vec3,
    /// Error covariance (3×3)
    pub p: Mat3,
    /// Process noise covariance (3×3, diagonal)
    pub q: Mat3,
    /// Measurement noise covariance (3×3, diagonal)
    pub r: Mat3,
}

impl Ekf {
    /// Create a new EKF with the given initial state and noise matrices.
    pub fn new(x0: Vec3, p0: Mat3, q: Mat3, r: Mat3) -> Self {
        Self {
            x: x0,
            p: p0,
            q,
            r,
        }
    }

    /// Create a default EKF initialized at the origin 
    
    pub fn default_at_origin() -> Self {
        Self {
            x: Vec3::zero(),
            p: Mat3::diag(1.0, 1.0, 1.0),           // large initial uncertainty
            q: Mat3::diag(0.001, 0.001, 0.01),       // process noise (encoders), theta is worse
            r: Mat3::diag(0.0001, 0.0001, 0.001),    // measurement noise (mocap)
        }
    }

    /// Create a default EKF initialized at a given pose.
    pub fn default_at(x: f32, y: f32, theta: f32) -> Self {
        let mut ekf = Self::default_at_origin();
        ekf.x = Vec3::new(x, y, theta);
        ekf
    }

    /// Prediction step: propagate state using unicycle kinematics.
    ///
    /// # Arguments
    /// * `v` — linear velocity [m/s]
    /// * `w` — angular velocity [rad/s]
    /// * `dt` — time step [s]
    pub fn predict(&mut self, v: f32, w: f32, dt: f32) {
        let theta = self.x.data[2];

        // State prediction: x⁻ = f(x, u)
        self.x.data[0] += v * cosf(theta) * dt;
        self.x.data[1] += v * sinf(theta) * dt;
        self.x.data[2] = wrap_angle(self.x.data[2] + w * dt);

        // Jacobian F = ∂f/∂x
        //   [ 1  0  -v·sin(θ)·dt ]
        //   [ 0  1   v·cos(θ)·dt ]
        //   [ 0  0   1           ]
        let f = Mat3 {
            data: [
                [1.0, 0.0, -v * sinf(theta) * dt],
                [0.0, 1.0, v * cosf(theta) * dt],
                [0.0, 0.0, 1.0],
            ],
        };

        // Covariance prediction: P⁻ = F P Fᵀ + Q
        let ft = f.transpose();
        self.p = f.mul(&self.p).mul(&ft).add(&self.q);
    }

    /// Update step: correct state using an absolute pose measurement.
    ///
    /// Since H = I₃ (mocap measures the full state directly), this simplifies to:
    ///   K = P⁻ (P⁻ + R)⁻¹
    ///   x⁺ = x⁻ + K (z − x⁻)
    ///   P⁺ = (I − K) P⁻
    ///
    /// # Arguments
    /// * `z` — measurement [x_meas, y_meas, θ_meas]
    ///
    /// # Returns
    /// `true` if the update succeeded, `false` if the innovation matrix was singular.
    pub fn update(&mut self, z: &Vec3) -> bool {
        // Innovation: y = z - x⁻   (with θ wrapped)
        let mut y = z.sub(&self.x);
        y.data[2] = wrap_angle(y.data[2]); // wrap angular innovation

        // Innovation covariance: S = P + R   (since H = I)
        let s = self.p.add(&self.r);

        // Kalman gain: K = P S⁻¹
        let s_inv = match s.invert() {
            Some(inv) => inv,
            None => return false,
        };
        let k = self.p.mul(&s_inv);

        // State update: x⁺ = x⁻ + K y
        let correction = k.mul_vec(&y);
        self.x = self.x.add(&correction);
        self.x.data[2] = wrap_angle(self.x.data[2]);

        // Covariance update: P⁺ = (I - K) P
        let i_minus_k = Mat3::identity().sub(&k);
        self.p = i_minus_k.mul(&self.p);

        true
    }

    /// Returns the current state estimate as (x, y, θ).
    pub fn state(&self) -> (f32, f32, f32) {
        (self.x.data[0], self.x.data[1], self.x.data[2])
    }
}
