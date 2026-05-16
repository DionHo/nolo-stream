use std::io::Write;
use std::net::{SocketAddr, TcpStream};
use std::time::Duration;

use crate::teleop::TeleopFrame;
use crate::transport::{Transport, TransportError};
use crate::Pose;

pub struct TcpStreamTransport {
    addr: SocketAddr,
    stream: Option<TcpStream>,
}

impl TcpStreamTransport {
    /// Lazy constructor — no connection is made until the first `send()`.
    pub fn connect(addr: SocketAddr) -> Self {
        Self { addr, stream: None }
    }

    fn ensure_connected(&mut self) -> bool {
        if self.stream.is_some() {
            return true;
        }
        match TcpStream::connect_timeout(&self.addr, Duration::from_secs(1)) {
            Ok(s) => {
                let _ = s.set_write_timeout(Some(Duration::from_millis(500)));
                self.stream = Some(s);
                true
            }
            Err(_) => false,
        }
    }
}

impl Transport for TcpStreamTransport {
    fn send(&mut self, poses: &[Pose]) -> Result<(), TransportError> {
        if !self.ensure_connected() {
            return Ok(()); // silently swallow; retry next call
        }

        let mut data = serde_json::to_vec(poses).unwrap();
        data.push(b'\n');

        if let Some(stream) = &mut self.stream {
            if stream.write_all(&data).is_err() {
                self.stream = None;
            }
        }

        Ok(())
    }

    fn send_teleop(&mut self, frames: &[TeleopFrame]) -> Result<(), TransportError> {
        if !self.ensure_connected() {
            return Ok(());
        }
        let inner = serde_json::to_string(frames).unwrap();
        let mut data = format!("{{\"teleop\":{inner}}}").into_bytes();
        data.push(b'\n');
        if let Some(stream) = &mut self.stream {
            if stream.write_all(&data).is_err() {
                self.stream = None;
            }
        }
        Ok(())
    }
}

