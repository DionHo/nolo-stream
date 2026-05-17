use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceId {
    Headset,
    LeftController,
    RightController,
}

fn touch_default() -> u8 { 255 }

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    pub device: DeviceId,
    pub position: [f32; 3],
    pub orientation: [f32; 4],
    pub timestamp_ms: u64,
    /// Raw i16 sensor block: 32 values from block base+1, step 2.
    #[serde(default)]
    pub sensor_raw: [i16; 32],
    /// Touch pad X. 255 = no touch, 127 = center, 0 = rightmost (confirmed: base+19).
    #[serde(default = "touch_default")]
    pub touch_x: u8,
    /// Touch pad Y. 255 = no touch, 127 = center, 0 = topmost (confirmed: base+20).
    #[serde(default = "touch_default")]
    pub touch_y: u8,
    /// Battery level 0–255 (tentative: base+21, same offset as nolo-osvr reference).
    #[serde(default)]
    pub battery: u8,
    /// Button bitmask (controllers only; 0 for headset).
    /// Bit 0x01=Pad, 0x02=Trigger, 0x04=Menu, 0x08=System, 0x10=Grip, 0x20=PadTouch.
    #[serde(default)]
    pub buttons: u32,
    /// Linear velocity [x, y, z] m/s. Filled by client-api path; zero on HID path.
    #[serde(default)]
    pub velocity: [f32; 3],
    /// Angular velocity [x, y, z] rad/s. Filled by client-api path; zero on HID path.
    #[serde(default)]
    pub angular_velocity: [f32; 3],
    /// Driver tracking state (0 = normal). Filled by client-api path; 0 on HID path.
    #[serde(default)]
    pub state: i32,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_pose(device: DeviceId) -> Pose {
        Pose {
            device,
            position: [1.0, 2.0, 3.0],
            orientation: [1.0, 0.0, 0.0, 0.0],
            timestamp_ms: 12345,
            sensor_raw: [0; 32],
            touch_x: 255,
            touch_y: 255,
            battery: 0,
            buttons: 0,
            velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
            state: 0,
        }
    }

    #[test]
    fn round_trip_headset() {
        let pose = make_pose(DeviceId::Headset);
        let json = serde_json::to_string(&pose).unwrap();
        assert!(json.contains("\"headset\""));
        let decoded: Pose = serde_json::from_str(&json).unwrap();
        assert!(matches!(decoded.device, DeviceId::Headset));
        assert_eq!(decoded.position, pose.position);
        assert_eq!(decoded.orientation, pose.orientation);
        assert_eq!(decoded.timestamp_ms, pose.timestamp_ms);
    }

    #[test]
    fn round_trip_left_controller() {
        let pose = make_pose(DeviceId::LeftController);
        let json = serde_json::to_string(&pose).unwrap();
        assert!(json.contains("\"left_controller\""));
        let decoded: Pose = serde_json::from_str(&json).unwrap();
        assert!(matches!(decoded.device, DeviceId::LeftController));
        assert_eq!(decoded.position, pose.position);
        assert_eq!(decoded.orientation, pose.orientation);
        assert_eq!(decoded.timestamp_ms, pose.timestamp_ms);
    }

    #[test]
    fn round_trip_right_controller() {
        let pose = make_pose(DeviceId::RightController);
        let json = serde_json::to_string(&pose).unwrap();
        assert!(json.contains("\"right_controller\""));
        let decoded: Pose = serde_json::from_str(&json).unwrap();
        assert!(matches!(decoded.device, DeviceId::RightController));
        assert_eq!(decoded.position, pose.position);
        assert_eq!(decoded.orientation, pose.orientation);
        assert_eq!(decoded.timestamp_ms, pose.timestamp_ms);
    }

    #[test]
    fn touch_defaults_to_no_touch_on_missing_json() {
        let json = r#"{"device":"left_controller","position":[0,0,0],"orientation":[1,0,0,0],"timestamp_ms":0}"#;
        let pose: Pose = serde_json::from_str(json).unwrap();
        assert_eq!(pose.touch_x, 255);
        assert_eq!(pose.touch_y, 255);
        assert_eq!(pose.battery, 0);
        assert_eq!(pose.buttons, 0);
        assert_eq!(pose.velocity, [0.0; 3]);
        assert_eq!(pose.state, 0);
    }
}
