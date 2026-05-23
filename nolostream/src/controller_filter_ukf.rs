use nalgebra::{Matrix3, SMatrix, UnitQuaternion, Vector3, Unit};
use crate::controller_report::{ControllerReport, ControllerSide};
use crate::controller_state::{ControllerState, DeviceId};

/// Multiplicative UKF for a single Nolo controller.
///
/// Error-state: [δθ(3), gyro_bias(3), velocity(3)] — 9-dim.
/// q_hat convention: body-to-world rotation.
/// Accel units: pscale=0.0001 raw → ~0.8192 = 1g.
/// Gyro units:  rad/s (DEFAULT_GYRO_SCALE applied in ControllerReport).
pub struct ControllerFilterUkf {
    q_hat:        UnitQuaternion<f32>,
    bias_hat:     Vector3<f32>,
    vel_hat:      Vector3<f32>,
    p:            SMatrix<f32, 9, 9>,
    last_pos:     Option<Vector3<f32>>,
    last_ts:      Option<u64>,
    cal_gyro:     Vec<Vector3<f32>>,
    calibrated:   bool,
    total_frames: usize,
}

// UKF scaling — ALPHA=1 gives all-positive weights and sigma spread = 3*sqrt(P),
// consistent with the covariance (avoids the huge negative w0c that occurs for ALPHA<<1).
const ALPHA: f32 = 1.0;
const BETA:  f32 = 2.0;
const KAPPA: f32 = 0.0;
const N: usize = 9;
const N_SIG: usize = 2 * N + 1;
const LAMBDA: f32 = ALPHA * ALPHA * (N as f32 + KAPPA) - N as f32; // = 0 for ALPHA=1, KAPPA=0

// Process noise (per second)
const Q_GYRO: f32 = 1e-4;  // orientation random walk (rad²/s)
const Q_BIAS: f32 = 1e-6;  // gyro bias drift ((rad/s)²/s)
const Q_VEL:  f32 = 0.1;   // velocity noise (m²/s³)

// Measurement noise
const R_ACCEL: f32 = 0.01;  // accelerometer (G_units²) — less weight than noise
const R_VEL:   f32 = 0.02;  // optical velocity (m/s)²

// Gravity: in report units, 1g = 8192 LSB * pscale(0.0001) = 0.8192.
// gravity_world() is the specific force direction (what a stationary accelerometer reads):
// +Y in Nolo's Y-up world frame.
const G: f32 = 0.8192;
fn gravity_world() -> Vector3<f32> { Vector3::new(0.0, G, 0.0) }

// Startup calibration: collect still frames to estimate gyro bias.
const CAL_FRAMES:   usize = 60;  // ~2s at 30fps per controller
const CAL_MAX_WAIT: usize = 300; // give up after ~10s total frames

impl ControllerFilterUkf {
    pub fn new() -> Self {
        // Small initial P so sigma spread (3*sqrt(P)) stays within quaternion linearization range.
        let p = SMatrix::<f32, 9, 9>::identity() * 0.01;
        Self {
            q_hat:        UnitQuaternion::identity(),
            bias_hat:     Vector3::zeros(),
            vel_hat:      Vector3::zeros(),
            p,
            last_pos:     None,
            last_ts:      None,
            cal_gyro:     Vec::new(),
            calibrated:   false,
            total_frames: 0,
        }
    }

    pub fn filter(&mut self, report: &ControllerReport) -> ControllerState {
        let ts = report.timestamp_ms;
        let dt = match self.last_ts {
            Some(prev) if ts > prev => ((ts - prev) as f32).min(100.0) * 1e-3,
            _ => 0.01_f32,
        };
        self.last_ts = Some(ts);

        let gyro  = Vector3::new(report.angular_velocity[0], report.angular_velocity[1], report.angular_velocity[2]);
        let accel = Vector3::new(report.acceleration[0],     report.acceleration[1],     report.acceleration[2]);
        let pos   = Vector3::new(report.position[0],         report.position[1],         report.position[2]);

        // First frame: set initial orientation from gravity so we start close to correct.
        if self.total_frames == 0 {
            self.q_hat = init_from_gravity(&accel);
        }

        // Startup bias calibration: accumulate still frames, then refine bias and orientation.
        if !self.calibrated {
            self.total_frames += 1;
            let accel_ok = (accel.norm() - G).abs() < G * 0.25;
            let gyro_ok  = gyro.norm() < 0.15; // rad/s
            if accel_ok && gyro_ok {
                self.cal_gyro.push(gyro);
            }
            if self.cal_gyro.len() >= CAL_FRAMES || self.total_frames >= CAL_MAX_WAIT {
                if !self.cal_gyro.is_empty() {
                    let n = self.cal_gyro.len() as f32;
                    self.bias_hat = self.cal_gyro.iter().fold(Vector3::zeros(), |a, v| a + v) / n;
                }
                self.q_hat = init_from_gravity(&accel);
                self.calibrated = true;
                // Reset P to post-calibration uncertainty.
                let mut p = SMatrix::<f32, 9, 9>::zeros();
                for k in 0..3 { p[(k,k)] = 0.01; }  // ~6° orientation uncertainty
                for k in 3..6 { p[(k,k)] = 1e-4; }  // calibrated bias
                for k in 6..9 { p[(k,k)] = 0.01; }  // 0.1 m/s velocity
                self.p = p;
            }
        }

        self.predict(gyro, accel, dt);
        self.update_accel(accel);

        if let Some(last_pos) = self.last_pos {
            if dt > 1e-4 {
                let vel_measured = (pos - last_pos) / dt;
                self.update_velocity(vel_measured);
            }
        }
        self.last_pos = Some(pos);

        let q = self.q_hat;
        ControllerState {
            device: match report.side {
                ControllerSide::Left  => DeviceId::LeftController,
                ControllerSide::Right => DeviceId::RightController,
            },
            position:         report.position,
            orientation:      [q.w, q.i, q.j, q.k],
            timestamp_ms:     ts,
            touch_x:          report.touch_x,
            touch_y:          report.touch_y,
            battery:          report.battery,
            buttons:          report.buttons,
            velocity:         [self.vel_hat.x, self.vel_hat.y, self.vel_hat.z],
            angular_velocity: [gyro.x - self.bias_hat.x, gyro.y - self.bias_hat.y, gyro.z - self.bias_hat.z],
            state: 0,
        }
    }

    fn predict(&mut self, gyro: Vector3<f32>, accel: Vector3<f32>, dt: f32) {
        let sp = self.sigma_points();
        let (sq, sb, sv) = self.propagate_sigma_points(sp, gyro, accel, dt);

        let w0m = LAMBDA / (N as f32 + LAMBDA);
        let wim = 0.5 / (N as f32 + LAMBDA);
        let w0c = w0m + (1.0 - ALPHA * ALPHA + BETA);
        let wic = wim;

        let q_mean = quat_mean(&sq, w0m, wim);
        let b_mean = weighted_mean_vec3(&sb, w0m, wim);
        let v_mean = weighted_mean_vec3(&sv, w0m, wim);

        let mut p_pred = SMatrix::<f32, 9, 9>::zeros();
        for i in 0..N_SIG {
            let wc = if i == 0 { w0c } else { wic };
            let e = error_vec(&q_mean, &b_mean, &v_mean, &sq[i], &sb[i], &sv[i]);
            p_pred += wc * e * e.transpose();
        }

        for k in 0..3 { p_pred[(k, k)] += Q_GYRO * dt; }
        for k in 3..6 { p_pred[(k, k)] += Q_BIAS * dt; }
        for k in 6..9 { p_pred[(k, k)] += Q_VEL  * dt; }

        self.q_hat    = q_mean;
        self.bias_hat = b_mean;
        self.vel_hat  = v_mean;
        self.p        = p_pred;
    }

    fn update_accel(&mut self, accel: Vector3<f32>) {
        // Skip during high-acceleration (device moving fast); gravity not trustworthy.
        let mag = accel.norm();
        if (mag - G).abs() > G * 0.3 {
            return;
        }

        let sp = self.sigma_points();
        let (sq, sb, sv) = self.propagate_sigma_points(sp, Vector3::zeros(), Vector3::zeros(), 0.0);

        let w0m = LAMBDA / (N as f32 + LAMBDA);
        let wim = 0.5 / (N as f32 + LAMBDA);
        let w0c = w0m + (1.0 - ALPHA * ALPHA + BETA);
        let wic = wim;

        // Predicted gravity in body frame for each sigma point.
        let z_sig: Vec<Vector3<f32>> = sq.iter()
            .map(|q| q.inverse_transform_vector(&gravity_world()))
            .collect();

        let z_mean = z_sig.iter().enumerate()
            .fold(Vector3::zeros(), |acc, (i, z)| acc + (if i == 0 { w0m } else { wim }) * z);

        let mut s = Matrix3::zeros();
        let mut t = SMatrix::<f32, 9, 3>::zeros();
        for i in 0..N_SIG {
            let wc = if i == 0 { w0c } else { wic };
            let dz = z_sig[i] - z_mean;
            let de = error_vec(&self.q_hat, &self.bias_hat, &self.vel_hat, &sq[i], &sb[i], &sv[i]);
            s += wc * dz * dz.transpose();
            t += wc * de * dz.transpose();
        }
        s += R_ACCEL * Matrix3::identity();

        if let Some(s_inv) = s.try_inverse() {
            let k = t * s_inv;
            let dx = k * (accel - z_mean);
            self.apply_correction(&dx);
            self.p -= k * s * k.transpose();
            self.symmetrize_p();
        }
    }

    fn update_velocity(&mut self, vel_measured: Vector3<f32>) {
        let sp = self.sigma_points();
        let (sq, sb, sv) = self.propagate_sigma_points(sp, Vector3::zeros(), Vector3::zeros(), 0.0);

        let w0m = LAMBDA / (N as f32 + LAMBDA);
        let wim = 0.5 / (N as f32 + LAMBDA);
        let w0c = w0m + (1.0 - ALPHA * ALPHA + BETA);
        let wic = wim;

        let z_mean = weighted_mean_vec3(&sv, w0m, wim);

        let mut s = Matrix3::zeros();
        let mut t = SMatrix::<f32, 9, 3>::zeros();
        for i in 0..N_SIG {
            let wc = if i == 0 { w0c } else { wic };
            let dz = sv[i] - z_mean;
            let de = error_vec(&self.q_hat, &self.bias_hat, &self.vel_hat, &sq[i], &sb[i], &sv[i]);
            s += wc * dz * dz.transpose();
            t += wc * de * dz.transpose();
        }
        s += R_VEL * Matrix3::identity();

        if let Some(s_inv) = s.try_inverse() {
            let k = t * s_inv;
            let dx = k * (vel_measured - z_mean);
            self.apply_correction(&dx);
            self.p -= k * s * k.transpose();
            self.symmetrize_p();
        }
    }

    fn sigma_points(&self) -> SMatrix<f32, 9, N_SIG> {
        let scale = (N as f32 + LAMBDA).sqrt();
        let p_reg = self.p + SMatrix::<f32, 9, 9>::identity() * 1e-9;
        let l = cholesky_lower(&p_reg);

        let mut sp = SMatrix::<f32, 9, N_SIG>::zeros();
        for i in 0..N {
            let col = l.column(i) * scale;
            sp.column_mut(1 + i).copy_from(&col);
            sp.column_mut(1 + N + i).copy_from(&(-col));
        }
        sp
    }

    fn propagate_sigma_points(
        &self,
        sp: SMatrix<f32, 9, N_SIG>,
        gyro: Vector3<f32>,
        accel: Vector3<f32>,
        dt: f32,
    ) -> ([UnitQuaternion<f32>; N_SIG], [Vector3<f32>; N_SIG], [Vector3<f32>; N_SIG]) {
        let mut sq = [UnitQuaternion::identity(); N_SIG];
        let mut sb = [Vector3::<f32>::zeros(); N_SIG];
        let mut sv = [Vector3::<f32>::zeros(); N_SIG];

        for i in 0..N_SIG {
            let col = sp.column(i);
            let dtheta = Vector3::new(col[0], col[1], col[2]);
            let dbias  = Vector3::new(col[3], col[4], col[5]);
            let dvel   = Vector3::new(col[6], col[7], col[8]);

            let q_i    = self.q_hat * quat_from_rotvec(dtheta);
            let bias_i = self.bias_hat + dbias;
            let vel_i  = self.vel_hat  + dvel;

            if dt > 1e-6 {
                sq[i] = q_i * quat_from_rotvec((gyro - bias_i) * dt);
                // Rotate specific force to world, subtract to get true acceleration, integrate.
                sv[i] = vel_i + (q_i * accel - gravity_world()) * dt;
            } else {
                sq[i] = q_i;
                sv[i] = vel_i;
            }
            sb[i] = bias_i;
        }
        (sq, sb, sv)
    }

    fn apply_correction(&mut self, dx: &SMatrix<f32, 9, 1>) {
        self.q_hat    = self.q_hat * quat_from_rotvec(Vector3::new(dx[0], dx[1], dx[2]));
        self.bias_hat += Vector3::new(dx[3], dx[4], dx[5]);
        self.vel_hat  += Vector3::new(dx[6], dx[7], dx[8]);
    }

    fn symmetrize_p(&mut self) {
        let pt = self.p.transpose();
        self.p = (self.p + pt) * 0.5;
        for i in 0..9 { self.p[(i, i)] = self.p[(i, i)].max(1e-9); }
    }
}

// ── Math helpers ────────────────────────────────────────────────────────────

fn quat_from_rotvec(v: Vector3<f32>) -> UnitQuaternion<f32> {
    let angle = v.norm();
    if angle < 1e-8 {
        UnitQuaternion::identity()
    } else {
        UnitQuaternion::from_axis_angle(&Unit::new_normalize(v), angle)
    }
}

fn quat_mean(qs: &[UnitQuaternion<f32>; N_SIG], w0: f32, wi: f32) -> UnitQuaternion<f32> {
    let mut mean = qs[0];
    for _ in 0..5 {
        let mut delta = Vector3::zeros();
        for (i, q) in qs.iter().enumerate() {
            let w = if i == 0 { w0 } else { wi };
            delta += w * (mean.inverse() * q).scaled_axis();
        }
        if delta.norm() < 1e-7 { break; }
        mean = mean * quat_from_rotvec(delta);
    }
    mean
}

fn weighted_mean_vec3(vs: &[Vector3<f32>; N_SIG], w0: f32, wi: f32) -> Vector3<f32> {
    vs.iter().enumerate().fold(Vector3::zeros(), |acc, (i, v)| {
        acc + (if i == 0 { w0 } else { wi }) * v
    })
}

fn error_vec(
    q_mean: &UnitQuaternion<f32>, b_mean: &Vector3<f32>, v_mean: &Vector3<f32>,
    q_i: &UnitQuaternion<f32>,    b_i:  &Vector3<f32>,   v_i:   &Vector3<f32>,
) -> SMatrix<f32, 9, 1> {
    let dtheta = (q_mean.inverse() * q_i).scaled_axis();
    let db = b_i - b_mean;
    let dv = v_i - v_mean;
    SMatrix::<f32, 9, 1>::from_column_slice(&[
        dtheta.x, dtheta.y, dtheta.z,
        db.x, db.y, db.z,
        dv.x, dv.y, dv.z,
    ])
}

/// Initialize orientation from accelerometer. Returns body-to-world rotation.
/// Sets pitch and roll from gravity; yaw is left at zero.
fn init_from_gravity(accel: &Vector3<f32>) -> UnitQuaternion<f32> {
    let mag = accel.norm();
    if mag < 0.1 {
        return UnitQuaternion::identity();
    }
    let g_body  = accel / mag;              // specific-force direction in body frame
    let g_world = Vector3::new(0.0, 1.0, 0.0); // +Y in world frame (stationary reads +Y)
    // rotation_between(a, b) = shortest arc from a to b.
    // We want q s.t. q * g_body = g_world (body-to-world rotation).
    UnitQuaternion::rotation_between(&g_body, &g_world)
        .unwrap_or(UnitQuaternion::identity())
}

fn cholesky_lower(m: &SMatrix<f32, 9, 9>) -> SMatrix<f32, 9, 9> {
    let mut l = SMatrix::<f32, 9, 9>::zeros();
    for i in 0..9 {
        for j in 0..=i {
            let mut sum = m[(i, j)];
            for k in 0..j { sum -= l[(i, k)] * l[(j, k)]; }
            if i == j {
                l[(i, j)] = sum.max(0.0).sqrt();
            } else if l[(j, j)] > 1e-12 {
                l[(i, j)] = sum / l[(j, j)];
            }
        }
    }
    l
}
