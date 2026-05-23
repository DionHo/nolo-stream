use serde::{Deserialize, Serialize};
use crate::controller_state::{DeviceId, ControllerState};

const BUTTON_MENU: u8 = 0x04;
const BUTTON_TRIGGER: u8 = 0x02;
const BUTTON_SYS: u8 = 0x08;

// Quaternion for R_x(90°): the Y-up → Z-up right-handed coordinate transform.
// cos(π/4) = sin(π/4) = 1/√2
const Q_T: [f32; 4] = [0.70710678_f32, 0.70710678_f32, 0.0, 0.0];

/// Incoming message from the TeleopTarget / robot, received via any transport.
#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum TeleopTargetMsg {
    /// Handshake control. `state` = `"active"` starts the handover sequence.
    Handover { state: String },
    /// TeleopTarget's current absolute pose sent at ≥50 Hz after `handover:active`.
    /// `pose_mm-deg` = [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg],
    /// Z-up right-handed, ZYX extrinsic Euler (KUKA A/B/C convention).
    Relative {
        #[serde(rename = "pose_mm-deg")]
        pose_mm_deg: [f32; 6],
    },
}

/// Handover notification broadcast to all connected clients.
#[derive(Debug, Clone, Serialize)]
pub struct HandoverMsg {
    #[serde(rename = "type")]
    msg_type: &'static str, // always "handover"
    pub state: &'static str, // "active" | "completed"
    /// Reference pose included in `state = "active"` broadcasts.
    #[serde(rename = "pose_mm-deg", skip_serializing_if = "Option::is_none")]
    pub pose_mm_deg: Option<[f32; 6]>,
}

/// Delta frame sent to the TeleopTarget while teleop is active.
///
/// `pose_mm-deg` = [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg],
/// Z-up right-handed, ZYX extrinsic Euler (KUKA A/B/C convention).
#[derive(Debug, Clone, Serialize)]
pub struct TeleopFrame {
    #[serde(rename = "type")]
    msg_type: &'static str, // always "relative"
    pub device: DeviceId,
    #[serde(rename = "pose_mm-deg")]
    pub pose_mm_deg: [f32; 6],
    pub timestamp_ms: u64,
}

/// Return value of [`TeleopState::update`].
pub struct TeleopUpdate {
    /// Delta frames to dispatch to all transports.
    pub frames: Vec<TeleopFrame>,
    /// Optional handover message to broadcast to all clients.
    pub handover_out: Option<HandoverMsg>,
}

enum HandoverPhase {
    Idle,
    WaitingForReferencePose,
    Active,
}

/// Maintains calibration state and computes frame-to-frame teleop deltas.
///
/// **Handover lifecycle:**
/// 1. Receive `TeleopTargetMsg::Handover { state: "active" }` → enter WaitingForReferencePose.
/// 2. Receive first `TeleopTargetMsg::Relative { pose_mm_deg }` → transition to Active, emit
///    `HandoverMsg { state: "active", pose_mm_deg }` for broadcast as reference pose.
/// 3. While Active + trigger held → emit delta frames each poll cycle.
/// 4. SYS button rising edge → emit `HandoverMsg { state: "completed" }`, return to Idle.
///
/// **Yaw calibration**: menu-button rising edge on either controller.
pub struct TeleopState {
    /// Combined transform: Q_T * q_yaw. Converts tracker Y-up to robotics Z-up.
    q_total: [f32; 4],
    calibrated: bool,
    prev_left: Option<ControllerState>,
    prev_right: Option<ControllerState>,
    last_left: Option<ControllerState>,
    last_right: Option<ControllerState>,
    left_trigger_held: bool,
    right_trigger_held: bool,
    left_menu_prev: bool,
    right_menu_prev: bool,
    left_sys_prev: bool,
    right_sys_prev: bool,
    handover: HandoverPhase,
}

impl TeleopState {
    pub fn new() -> Self {
        Self {
            q_total: Q_T,
            calibrated: false,
            prev_left: None,
            prev_right: None,
            last_left: None,
            last_right: None,
            left_trigger_held: false,
            right_trigger_held: false,
            left_menu_prev: false,
            right_menu_prev: false,
            left_sys_prev: false,
            right_sys_prev: false,
            handover: HandoverPhase::Idle,
        }
    }

    pub fn is_calibrated(&self) -> bool {
        self.calibrated
    }

    /// Process a batch of poses and incoming manipulator messages from one poll cycle.
    /// Returns delta frames to dispatch plus any handover broadcast to send to all clients.
    pub fn update(&mut self, poses: &[ControllerState], msgs: &[TeleopTargetMsg]) -> TeleopUpdate {
        let left  = poses.iter().find(|p| p.device == DeviceId::LeftController);
        let right = poses.iter().find(|p| p.device == DeviceId::RightController);

        if let Some(l) = left  { self.last_left  = Some(l.clone()); }
        if let Some(r) = right { self.last_right = Some(r.clone()); }

        // ── Yaw calibration on menu button rising edge ────────────────────────
        let left_menu  = left.map_or(false,  |p| p.buttons & BUTTON_MENU != 0);
        let right_menu = right.map_or(false, |p| p.buttons & BUTTON_MENU != 0);
        if (left_menu && !self.left_menu_prev) || (right_menu && !self.right_menu_prev) {
            let l_pos = self.last_left.as_ref().map(|s| s.position);
            let r_pos = self.last_right.as_ref().map(|s| s.position);
            if let (Some(lp), Some(rp)) = (l_pos, r_pos) {
                self.calibrate_yaw(lp, rp);
            }
        }
        self.left_menu_prev  = left_menu;
        self.right_menu_prev = right_menu;

        // ── Incoming TeleopTarget messages ─────────────────────────────────────
        let mut handover_out: Option<HandoverMsg> = None;
        for msg in msgs {
            match msg {
                TeleopTargetMsg::Handover { state } if state == "active" => {
                    if matches!(self.handover, HandoverPhase::Idle) {
                        self.handover = HandoverPhase::WaitingForReferencePose;
                    }
                }
                TeleopTargetMsg::Relative { pose_mm_deg } => {
                    if matches!(self.handover, HandoverPhase::WaitingForReferencePose) {
                        self.handover = HandoverPhase::Active;
                        handover_out = Some(HandoverMsg {
                            msg_type: "handover",
                            state: "active",
                            pose_mm_deg: Some(*pose_mm_deg),
                        });
                    }
                }
                _ => {}
            }
        }

        // ── SYS button rising edge → end handover ─────────────────────────────
        let left_sys  = left.map_or(false,  |p| p.buttons & BUTTON_SYS != 0);
        let right_sys = right.map_or(false, |p| p.buttons & BUTTON_SYS != 0);
        if (left_sys && !self.left_sys_prev) || (right_sys && !self.right_sys_prev) {
            if matches!(self.handover, HandoverPhase::Active) {
                self.handover = HandoverPhase::Idle;
                self.prev_left  = None;
                self.prev_right = None;
                handover_out = Some(HandoverMsg {
                    msg_type: "handover",
                    state: "completed",
                    pose_mm_deg: None,
                });
            }
        }
        self.left_sys_prev  = left_sys;
        self.right_sys_prev = right_sys;

        // ── Delta frames (only while handover is Active) ──────────────────────
        let mut frames = Vec::new();
        if matches!(self.handover, HandoverPhase::Active) {
            if let Some(l) = left {
                let trigger_held = l.buttons & BUTTON_TRIGGER != 0;
                if trigger_held {
                    if self.left_trigger_held {
                        if let Some(prev) = &self.prev_left {
                            frames.push(compute_delta(l, prev, self.q_total));
                        }
                    }
                    self.prev_left = Some(l.clone());
                } else {
                    self.prev_left = None;
                }
                self.left_trigger_held = trigger_held;
            }

            if let Some(r) = right {
                let trigger_held = r.buttons & BUTTON_TRIGGER != 0;
                if trigger_held {
                    if self.right_trigger_held {
                        if let Some(prev) = &self.prev_right {
                            frames.push(compute_delta(r, prev, self.q_total));
                        }
                    }
                    self.prev_right = Some(r.clone());
                } else {
                    self.prev_right = None;
                }
                self.right_trigger_held = trigger_held;
            }
        }

        TeleopUpdate { frames, handover_out }
    }

    fn calibrate_yaw(&mut self, left_pos: [f32; 3], right_pos: [f32; 3]) {
        let dx = right_pos[0] - left_pos[0];
        let dz = right_pos[2] - left_pos[2];
        let len = (dx * dx + dz * dz).sqrt();
        if len < 1e-4 {
            return; // controllers too close together
        }
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
    let dp = [
        current.position[0] - prev.position[0],
        current.position[1] - prev.position[1],
        current.position[2] - prev.position[2],
    ];
    let dp_robot = quat_rotate_vec(dp, q_total);

    let q_rel = quat_mul(current.orientation, quat_conj(prev.orientation));
    let q_robot = quat_normalize(quat_sandwich(q_total, q_rel));
    let rpy = quat_to_rpy_deg(q_robot);

    TeleopFrame {
        msg_type: "relative",
        device: current.device.clone(),
        pose_mm_deg: [
            dp_robot[0] * 1000.0,
            dp_robot[1] * 1000.0,
            dp_robot[2] * 1000.0,
            rpy[0],
            rpy[1],
            rpy[2],
        ],
        timestamp_ms: current.timestamp_ms,
    }
}

/// Convert unit quaternion [w, x, y, z] to ZYX extrinsic Euler angles in degrees.
/// Convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)  — KUKA A/B/C (Z-Y'-X'').
fn quat_to_rpy_deg(q: [f32; 4]) -> [f32; 3] {
    let [w, x, y, z] = q;
    let sin_pitch = 2.0 * (w * y - z * x);
    let pitch = sin_pitch.clamp(-1.0, 1.0).asin();
    let roll  = f32::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    let yaw   = f32::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    const DEG: f32 = 180.0 / std::f32::consts::PI;
    [roll * DEG, pitch * DEG, yaw * DEG]
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

    fn activate(state: &mut TeleopState) {
        state.update(&[], &[
            TeleopTargetMsg::Handover { state: "active".into() },
            TeleopTargetMsg::Relative { pose_mm_deg: [0.0; 6] },
        ]);
    }

    #[test]
    fn no_frames_without_handover() {
        let mut state = TeleopState::new();
        let poses = vec![make_pose(DeviceId::LeftController, [0.0, 1.0, 0.0], BUTTON_TRIGGER)];
        assert!(state.update(&poses, &[]).frames.is_empty());
        assert!(state.update(&poses, &[]).frames.is_empty());
    }

    #[test]
    fn no_frame_on_first_trigger_press() {
        let mut state = TeleopState::new();
        activate(&mut state);
        let pose = make_pose(DeviceId::LeftController, [0.0, 1.0, 0.0], BUTTON_TRIGGER);
        assert!(state.update(&[pose], &[]).frames.is_empty());
    }

    #[test]
    fn emits_frame_on_sustained_trigger() {
        let mut state = TeleopState::new();
        activate(&mut state);
        let p1 = make_pose(DeviceId::LeftController, [0.0, 1.0, 0.0], BUTTON_TRIGGER);
        let p2 = make_pose(DeviceId::LeftController, [0.1, 1.0, 0.0], BUTTON_TRIGGER);
        state.update(&[p1], &[]);
        let frames = state.update(&[p2], &[]).frames;
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].device, DeviceId::LeftController);
        // Without calibration: q_total = Q_T = R_x(90°).
        // delta_pos Y-up (0.1, 0, 0) → Z-up: R_x(90°) * (0.1, 0, 0) = (0.1, 0, 0) (X unchanged).
        // Converted to mm: 100.0
        assert!((frames[0].pose_mm_deg[0] - 100.0).abs() < 1e-2);
        assert!(frames[0].pose_mm_deg[1].abs() < 1e-2);
        assert!(frames[0].pose_mm_deg[2].abs() < 1e-2);
    }

    #[test]
    fn yaw_calibration_aligns_x_axis() {
        let mut state = TeleopState::new();
        activate(&mut state);
        // Left at -Z, right at +Z, with menu pressed → L→R = +Z direction.
        // After calibration, the +Z direction should become +X in robot frame.
        let left  = make_pose(DeviceId::LeftController,  [0.0, 1.0, -1.0], BUTTON_MENU);
        let right = make_pose(DeviceId::RightController, [0.0, 1.0,  1.0], BUTTON_MENU);
        state.update(&[left, right], &[]);
        assert!(state.is_calibrated());

        let p1 = make_pose(DeviceId::LeftController, [0.0, 1.0, 0.0], BUTTON_TRIGGER);
        let p2 = make_pose(DeviceId::LeftController, [0.0, 1.0, 1.0], BUTTON_TRIGGER);
        state.update(&[p1], &[]);
        let frames = state.update(&[p2], &[]).frames;
        assert_eq!(frames.len(), 1);
        // +Z movement after yaw calibration (L→R was +Z) → +X in robot frame → ~1000 mm
        assert!(frames[0].pose_mm_deg[0] > 500.0, "X should be ~1000mm, got {:?}", frames[0].pose_mm_deg);
        assert!(frames[0].pose_mm_deg[1].abs() < 0.1);
        assert!(frames[0].pose_mm_deg[2].abs() < 0.1);
    }
}
