use std::io;
use std::net::{SocketAddr, UdpSocket};

use crate::teleop::TeleopFrame;
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
        let inner = serde_json::to_string(frames).unwrap();
        let mut data = format!("{{\"teleop\":{inner}}}").into_bytes();
        data.push(b'\n');
        let _ = self.socket.send_to(&data, self.target);
        Ok(())
    }
}

