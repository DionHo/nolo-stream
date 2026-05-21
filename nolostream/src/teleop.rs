use serde::Serialize;
use crate::controller_state::{DeviceId, ControllerState};

const BUTTON_MENU: u8 = 0x04;
const BUTTON_PAD: u8 = 0x01;

// Quaternion for R_x(90°): the Y-up → Z-up right-handed coordinate transform.
// cos(π/4) = sin(π/4) = 1/√2
const Q_T: [f32; 4] = [0.70710678_f32, 0.70710678_f32, 0.0, 0.0];

/// Frame-to-frame pose delta for robotic teleop.
///
/// Coordinates use the **robotics convention**: Z up, right-handed.
/// The X axis is calibrated to point from left to right controller at the time of the last
/// menu-button press. If no calibration has been performed, X aligns with the raw tracker X.
#[derive(Debug, Clone, Serialize)]
pub struct TeleopFrame {
    pub device: DeviceId,
    /// Frame-to-frame position delta in robotics coordinates (Z up, right-handed), metres.
    pub delta_position: [f32; 3],
    /// Frame-to-frame rotation delta as unit quaternion [w,x,y,z] in robotics coordinates.
    pub delta_orientation: [f32; 4],
    pub timestamp_ms: u64,
}

/// Maintains calibration state and computes frame-to-frame teleop deltas.
///
/// - **Yaw calibration**: on a menu-button rising edge (either controller), the horizontal
///   vector from left to right controller becomes the +X axis.
/// - **Delta streaming**: while the touchpad click is held, emits a [`TeleopFrame`] per poll
///   with the frame-to-frame position and orientation delta in robotics (Z-up) coordinates.
pub struct TeleopState {
    /// Combined transform quaternion: Q_T * q_yaw.
    /// Converts from tracker Y-up world frame to Z-up right-handed robotics frame.
    q_total: [f32; 4],
    calibrated: bool,
    prev_left: Option<ControllerState>,
    prev_right: Option<ControllerState>,
    left_pad_held: bool,
    right_pad_held: bool,
    left_menu_prev: bool,
    right_menu_prev: bool,
}

impl TeleopState {
    pub fn new() -> Self {
        Self {
            q_total: Q_T, // yaw = 0 → q_yaw = identity → q_total = Q_T
            calibrated: false,
            prev_left: None,
            prev_right: None,
            left_pad_held: false,
            right_pad_held: false,
            left_menu_prev: false,
            right_menu_prev: false,
        }
    }

    pub fn is_calibrated(&self) -> bool {
        self.calibrated
    }

    /// Process a batch of poses from one poll cycle and return any teleop frames.
    pub fn update(&mut self, poses: &[ControllerState]) -> Vec<TeleopFrame> {
        let left = poses.iter().find(|p| p.device == DeviceId::LeftController);
        let right = poses.iter().find(|p| p.device == DeviceId::RightController);

        // Reset state for controllers that dropped out.
        if left.is_none() {
            self.prev_left = None;
            self.left_pad_held = false;
            self.left_menu_prev = false;
        }
        if right.is_none() {
            self.prev_right = None;
            self.right_pad_held = false;
            self.right_menu_prev = false;
        }

        // Yaw calibration on menu button rising edge (either controller triggers it).
        let left_menu = left.map_or(false, |p| p.buttons & BUTTON_MENU != 0);
        let right_menu = right.map_or(false, |p| p.buttons & BUTTON_MENU != 0);

        if (left_menu && !self.left_menu_prev) || (right_menu && !self.right_menu_prev) {
            if let (Some(l), Some(r)) = (left, right) {
                self.calibrate_yaw(l, r);
            }
        }
        self.left_menu_prev = left_menu;
        self.right_menu_prev = right_menu;

        // Emit deltas while touchpad click held.
        let mut frames = Vec::new();

        if let Some(l) = left {
            let pad_held = l.buttons & BUTTON_PAD != 0;
            if pad_held {
                if self.left_pad_held {
                    if let Some(prev) = &self.prev_left {
                        frames.push(compute_delta(l, prev, self.q_total));
                    }
                }
                self.prev_left = Some(l.clone());
            } else {
                self.prev_left = None;
            }
            self.left_pad_held = pad_held;
        }

        if let Some(r) = right {
            let pad_held = r.buttons & BUTTON_PAD != 0;
            if pad_held {
                if self.right_pad_held {
                    if let Some(prev) = &self.prev_right {
                        frames.push(compute_delta(r, prev, self.q_total));
                    }
                }
                self.prev_right = Some(r.clone());
            } else {
                self.prev_right = None;
            }
            self.right_pad_held = pad_held;
        }

        frames
    }

    fn calibrate_yaw(&mut self, left: &ControllerState, right: &ControllerState) {
        let dx = right.position[0] - left.position[0];
        let dz = right.position[2] - left.position[2];
        let len = (dx * dx + dz * dz).sqrt();
        if len < 1e-4 {
            return; // controllers too close together
        }
        // yaw_angle: rotation around Y that aligns the L→R horizontal vector with +X.
        let yaw_angle = dz.atan2(dx);
        let q_yaw: [f32; 4] = [
            (yaw_angle * 0.5).cos(),
            0.0,
            (yaw_angle * 0.5).sin(),
            0.0,
        ];
        self.q_total = quat_mul(Q_T, q_yaw);
        self.calibrated = true;
    }
}

impl Default for TeleopState {
    fn default() -> Self {
        Self::new()
    }
}

fn compute_delta(current: &ControllerState, prev: &ControllerState, q_total: [f32; 4]) -> TeleopFrame {
    // Position delta: rotate from tracker Y-up into robotics Z-up frame.
    let dp = [
        current.position[0] - prev.position[0],
        current.position[1] - prev.position[1],
        current.position[2] - prev.position[2],
    ];
    let delta_position = quat_rotate_vec(dp, q_total);

    // Orientation delta: relative rotation, then re-expressed in robotics frame.
    let q_rel = quat_mul(current.orientation, quat_conj(prev.orientation));
    let delta_orientation = quat_normalize(quat_sandwich(q_total, q_rel));

    TeleopFrame {
        device: current.device.clone(),
        delta_position,
        delta_orientation,
        timestamp_ms: current.timestamp_ms,
    }
}

// ── Quaternion helpers (w,x,y,z) ─────────────────────────────────────────────

fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    let [aw, ax, ay, az] = a;
    let [bw, bx, by, bz] = b;
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

fn quat_conj(q: [f32; 4]) -> [f32; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// Active rotation of vector v by unit quaternion q: q ⊗ (0,v) ⊗ q*.
fn quat_rotate_vec(v: [f32; 3], q: [f32; 4]) -> [f32; 3] {
    let pv = [0.0_f32, v[0], v[1], v[2]];
    let tmp = quat_mul(q, pv);
    let res = quat_mul(tmp, quat_conj(q));
    [res[1], res[2], res[3]]
}

/// Conjugation q ⊗ v ⊗ q* (re-expresses rotation v in frame q).
fn quat_sandwich(q: [f32; 4], v: [f32; 4]) -> [f32; 4] {
    quat_mul(quat_mul(q, v), quat_conj(q))
}

fn quat_normalize(q: [f32; 4]) -> [f32; 4] {
    let len = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if len < 1e-9 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    [q[0] / len, q[1] / len, q[2] / len, q[3] / len]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_pose(device: DeviceId, pos: [f32; 3], buttons: u8) -> ControllerState {
        ControllerState {
            device,
            position: pos,
            orientation: [1.0, 0.0, 0.0, 0.0],
            timestamp_ms: 0,
            touch_x: 255,
            touch_y: 255,
            battery: 0,
            buttons,
            velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
            state: 0,
        }
    }

    #[test]
    fn no_frames_without_pad_held() {
        let mut state = TeleopState::new();
        let poses = vec![make_pose(DeviceId::LeftController, [0.0, 1.0, 0.0], 0)];
        assert!(state.update(&poses).is_empty());
        assert!(state.update(&poses).is_empty());
    }

    #[test]
    fn no_frame_on_first_pad_press() {
        let mut state = TeleopState::new();
        let pose = make_pose(DeviceId::LeftController, [0.0, 1.0, 0.0], BUTTON_PAD);
        // First frame with pad held: no prev → no delta.
        assert!(state.update(&[pose]).is_empty());
    }

    #[test]
    fn emits_frame_on_sustained_pad() {
        let mut state = TeleopState::new();
        let p1 = make_pose(DeviceId::LeftController, [0.0, 1.0, 0.0], BUTTON_PAD);
        let p2 = make_pose(DeviceId::LeftController, [0.1, 1.0, 0.0], BUTTON_PAD);
        state.update(&[p1]);
        let frames = state.update(&[p2]);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].device, DeviceId::LeftController);
        // Without calibration: q_total = Q_T, so position maps Y-up to Z-up.
        // delta_pos Y-up (0.1, 0, 0) → Z-up: rotate by Q_T = R_x(90°)
        // R_x(90°) * (0.1, 0, 0) = (0.1, 0, 0) (X unchanged)
        assert!((frames[0].delta_position[0] - 0.1).abs() < 1e-5);
        assert!(frames[0].delta_position[1].abs() < 1e-5);
        assert!(frames[0].delta_position[2].abs() < 1e-5);
    }

    #[test]
    fn yaw_calibration_aligns_x_axis() {
        let mut state = TeleopState::new();
        // Left at -Z, right at +Z, with menu pressed → L→R = +Z direction.
        // After calibration, the +Z direction should become +X in robot frame.
        let left = make_pose(DeviceId::LeftController, [0.0, 1.0, -1.0], BUTTON_MENU);
        let right = make_pose(DeviceId::RightController, [0.0, 1.0, 1.0], BUTTON_MENU);
        state.update(&[left.clone(), right.clone()]);
        assert!(state.is_calibrated());

        // Now a delta in +Z (old world) should map to +X in robot frame.
        let pad = BUTTON_PAD;
        let mut p1 = make_pose(DeviceId::LeftController, [0.0, 1.0, 0.0], pad);
        let mut p2 = make_pose(DeviceId::LeftController, [0.0, 1.0, 1.0], pad); // move +Z
        p1.buttons = pad;
        p2.buttons = pad;
        state.update(&[p1]);
        let frames = state.update(&[p2]);
        assert_eq!(frames.len(), 1);
        // +Z movement after yaw calibration (L→R was +Z) → +X in robot frame
        assert!(frames[0].delta_position[0] > 0.5, "X should be ~1.0, got {:?}", frames[0].delta_position);
        assert!(frames[0].delta_position[1].abs() < 1e-4);
        assert!(frames[0].delta_position[2].abs() < 1e-4);
    }
}
