use crate::Pose;

pub trait Transport: Send {
    /// Called each poll cycle with fresh pose data. Implementations serialize and send.
    fn send(&mut self, poses: &[Pose]) -> Result<(), TransportError>;
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
    use crate::pose::DeviceId;

    struct MockTransport {
        received: Vec<Vec<Pose>>,
        error_after: Option<usize>,
    }

    impl Transport for MockTransport {
        fn send(&mut self, poses: &[Pose]) -> Result<(), TransportError> {
            if let Some(limit) = self.error_after {
                if self.received.len() >= limit {
                    return Err(TransportError::Disconnected);
                }
            }
            self.received.push(poses.to_vec());
            Ok(())
        }
    }

    fn make_pose() -> Pose {
        Pose {
            device: DeviceId::Headset,
            position: [0.0, 1.0, 2.0],
            orientation: [1.0, 0.0, 0.0, 0.0],
            timestamp_ms: 0,
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
