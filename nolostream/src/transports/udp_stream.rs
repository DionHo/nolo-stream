use std::io;
use std::net::{SocketAddr, UdpSocket};

use crate::transport::{Transport, TransportError};
use crate::Pose;

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
    fn send(&mut self, poses: &[Pose]) -> Result<(), TransportError> {
        let mut data = serde_json::to_vec(poses).unwrap();
        data.push(b'\n');
        let _ = self.socket.send_to(&data, self.target); // fire-and-forget
        Ok(())
    }
}
