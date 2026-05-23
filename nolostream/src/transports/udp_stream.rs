use std::io;
use std::net::{SocketAddr, UdpSocket};

use crate::teleop::{HandoverMsg, TeleopFrame};
use crate::transport::{Transport, TransportError};
use crate::ControllerState;

pub struct UdpStreamTransport {
    socket: UdpSocket,
    target: SocketAddr,
}

impl UdpStreamTransport {
    /// Bind to an ephemeral local port and record the target address.
    pub fn new(target: SocketAddr) -> io::Result<Self> {
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        Ok(Self { socket, target })
    }
}

impl Transport for UdpStreamTransport {
    fn send(&mut self, poses: &[ControllerState]) -> Result<(), TransportError> {
        let mut data = serde_json::to_vec(poses).unwrap();
        data.push(b'\n');
        let _ = self.socket.send_to(&data, self.target);
        Ok(())
    }

    fn send_teleop(&mut self, frames: &[TeleopFrame]) -> Result<(), TransportError> {
        for frame in frames {
            let mut data = serde_json::to_vec(frame).unwrap();
            data.push(b'\n');
            let _ = self.socket.send_to(&data, self.target);
        }
        Ok(())
    }

    fn send_handover(&mut self, msg: &HandoverMsg) -> Result<(), TransportError> {
        let mut data = serde_json::to_vec(msg).unwrap();
        data.push(b'\n');
        let _ = self.socket.send_to(&data, self.target);
        Ok(())
    }

    // UDP is stateless; recv_teleop_target_msgs returns empty (no bidirectional channel).
}

