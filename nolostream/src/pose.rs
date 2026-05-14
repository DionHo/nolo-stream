use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceId {
    Headset,
    LeftController,
    RightController,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Pose {
    pub device: DeviceId,
    pub position: [f32; 3],
    pub orientation: [f32; 4],
    pub timestamp_ms: u64,
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
}
