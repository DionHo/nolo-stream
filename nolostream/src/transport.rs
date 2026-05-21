use crate::command::Command;
use crate::teleop::TeleopFrame;
use crate::ControllerState;

pub trait Transport: Send {
    /// Called each poll cycle with fresh pose data. Implementations serialize and send.
    fn send(&mut self, poses: &[ControllerState]) -> Result<(), TransportError>;

    /// Send teleop delta frames. Called only when frames are non-empty.
    /// Default implementation is a no-op; override to transmit teleop data.
    fn send_teleop(&mut self, frames: &[TeleopFrame]) -> Result<(), TransportError> {
        let _ = frames;
        Ok(())
    }

    /// Drain any commands received from clients since the last call.
    fn recv_commands(&mut self) -> Vec<Command> { vec![] }
}

#[derive(Debug)]
pub enum TransportError {
    Disconnected,
    Io(String),
}

impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TransportError::Disconnected => write!(f, "transport disconnected"),
            TransportError::Io(msg) => write!(f, "io error: {msg}"),
        }
    }
}

impl std::error::Error for TransportError {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::controller_state::DeviceId;

    struct MockTransport {
        received: Vec<Vec<ControllerState>>,
        error_after: Option<usize>,
    }

    impl Transport for MockTransport {
        fn send(&mut self, poses: &[ControllerState]) -> Result<(), TransportError> {
            if let Some(limit) = self.error_after {
                if self.received.len() >= limit {
                    return Err(TransportError::Disconnected);
                }
            }
            self.received.push(poses.to_vec());
            Ok(())
        }
    }

    fn make_pose() -> ControllerState {
        ControllerState {
            device: DeviceId::Headset,
            position: [0.0, 1.0, 2.0],
            orientation: [1.0, 0.0, 0.0, 0.0],
            timestamp_ms: 0,
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
    fn mock_transport_records_sends() {
        let mut t = MockTransport { received: vec![], error_after: None };
        let poses = vec![make_pose()];

        t.send(&poses).unwrap();
        t.send(&poses).unwrap();

        assert_eq!(t.received.len(), 2);
        assert_eq!(t.received[0].len(), 1);
    }

    #[test]
    fn mock_transport_returns_disconnected_after_limit() {
        let mut t = MockTransport { received: vec![], error_after: Some(1) };
        let poses = vec![make_pose()];

        t.send(&poses).unwrap();
        let err = t.send(&poses).unwrap_err();
        assert!(matches!(err, TransportError::Disconnected));
    }

    #[test]
    fn transport_error_display() {
        assert_eq!(TransportError::Disconnected.to_string(), "transport disconnected");
        assert_eq!(TransportError::Io("oops".into()).to_string(), "io error: oops");
    }
}
