use std::time::Instant;

use crate::{ControllerReport, ControllerState, controller_report::ControllerStateFilterTrait};

/// Default gyro scale: 2000 dps full-scale, 16-bit signed → 0.061 deg/LSB = 0.001065 rad/LSB.
pub const DEFAULT_GYRO_SCALE: f32 = 0.001065;

/// Mahony-style complementary filter combining accel (gravity) and gyro integration.
///
/// World frame: Y-up, left-handed (X right, Z toward user from base station).
/// Quaternion: [w, x, y, z] representing body-to-world rotation.
pub struct ComplementaryFilter {
    pub q: [f32; 4],
    pub gyro_scale: f32,
    alpha: f32,
    last_ts: Option<Instant>,
}

impl ComplementaryFilter {
    pub fn new(gyro_scale: f32) -> Self {
        Self {
            q: [1.0, 0.0, 0.0, 0.0],
            gyro_scale,
            alpha: 0.98,
            last_ts: None,
        }
    }

    /// Feed one sample of raw accel and gyro i16 readings; returns updated quaternion [w, x, y, z].
    ///
    /// `accel`: sensor_raw[3..6] (AX, AY, AZ body-fixed)
    /// `gyro`:  sensor_raw[6..9] (RX, RY, RZ body-fixed, same axes as accel)
    pub fn update(&mut self, accel: [i16; 3], gyro: [i16; 3]) -> [f32; 4] {
        let now = Instant::now();
        let dt = match self.last_ts.replace(now) {
            Some(last) => now.duration_since(last).as_secs_f32().min(0.1),
            None => return self.q,
        };

        let [qw, qx, qy, qz] = self.q;

        let mut gx = gyro[0] as f32 * self.gyro_scale;
        let mut gy = gyro[1] as f32 * self.gyro_scale;
        let mut gz = gyro[2] as f32 * self.gyro_scale;

        let ax = accel[0] as f32;
        let ay = accel[1] as f32;
        let az = accel[2] as f32;
        let am = (ax * ax + ay * ay + az * az).sqrt();
        if am > 100.0 {
            let ax = ax / am;
            let ay = ay / am;
            let az = az / am;

            // Expected gravity direction in body frame (world +Y rotated to body via q^-1)
            let vx = 2.0 * (qx * qy + qw * qz);
            let vy = 1.0 - 2.0 * (qx * qx + qz * qz);
            let vz = 2.0 * (qy * qz - qw * qx);

            // Error = measured × expected (cross product)
            let ex = ay * vz - az * vy;
            let ey = az * vx - ax * vz;
            let ez = ax * vy - ay * vx;

            let kp = 2.0 * (1.0 - self.alpha);
            gx += kp * ex;
            gy += kp * ey;
            gz += kp * ez;
        }

        // Integrate: q += 0.5 * q ⊗ [0, gx, gy, gz] * dt
        let h = 0.5 * dt;
        let nqw = qw + (-qx * gx - qy * gy - qz * gz) * h;
        let nqx = qx + (qw * gx + qy * gz - qz * gy) * h;
        let nqy = qy + (qw * gy - qx * gz + qz * gx) * h;
        let nqz = qz + (qw * gz + qx * gy - qy * gx) * h;

        let mag = (nqw * nqw + nqx * nqx + nqy * nqy + nqz * nqz).sqrt();
        if mag > 1e-6 {
            self.q = [nqw / mag, nqx / mag, nqy / mag, nqz / mag];
        }
        self.q
    }
}

impl ControllerStateFilterTrait for ComplementaryFilter {
    fn filter(&self, report: &ControllerReport) -> ControllerState {
        ControllerState {
            device: report.side.clone(),
            position: report.position,
            orientation: self.q,
            timestamp_ms: report.timestamp_ms,
            touch_x: report.touch_x,
            touch_y: report.touch_y,
            battery: report.battery,
            buttons: report.buttons,
            velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
            state: 0,
        }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_on_zero_gyro_and_downward_accel() {
        // If controller is still with +Y up, accel ≈ [0, G, 0].
        // Filter should converge toward identity over many updates.
        let mut f = ComplementaryFilter::new(DEFAULT_GYRO_SCALE);
        let accel = [0i16, 8192, 0]; // ~1g on +Y
        let gyro  = [0i16, 0, 0];

        // First call initialises timestamp, returns identity.
        let q0 = f.update(accel, gyro);
        assert_eq!(q0, [1.0, 0.0, 0.0, 0.0]);

        // Feed ~100 samples; error should shrink (still approximately identity since we start at identity).
        for _ in 0..100 {
            std::thread::sleep(std::time::Duration::from_millis(1));
            f.update(accel, gyro);
        }
        let q = f.q;
        // w should remain close to 1, others near 0
        assert!((q[0] - 1.0).abs() < 0.01, "w={}", q[0]);
        assert!(q[1].abs() < 0.01, "x={}", q[1]);
        assert!(q[2].abs() < 0.01, "y={}", q[2]);
        assert!(q[3].abs() < 0.01, "z={}", q[3]);
    }

    #[test]
    fn quaternion_stays_normalised() {
        let mut f = ComplementaryFilter::new(DEFAULT_GYRO_SCALE);
        let accel = [100i16, 8000, -200];
        let gyro  = [500i16, -300, 800];
        for _ in 0..200 {
            std::thread::sleep(std::time::Duration::from_millis(1));
            let q = f.update(accel, gyro);
            let mag = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
            assert!((mag - 1.0).abs() < 1e-5, "mag={mag}");
        }
    }
}
