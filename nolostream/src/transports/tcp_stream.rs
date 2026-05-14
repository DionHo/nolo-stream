use std::io::Write;
use std::net::{SocketAddr, TcpStream};
use std::time::Duration;

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
}

impl Transport for TcpStreamTransport {
    fn send(&mut self, poses: &[Pose]) -> Result<(), TransportError> {
        // Reconnect if we have no stream.
        if self.stream.is_none() {
            match TcpStream::connect_timeout(&self.addr, Duration::from_secs(1)) {
                Ok(s) => {
                    let _ = s.set_write_timeout(Some(Duration::from_millis(500)));
                    self.stream = Some(s);
                }
                Err(_) => return Ok(()), // silently swallow; retry next call
            }
        }

        let mut data = serde_json::to_vec(poses).unwrap();
        data.push(b'\n');

        if let Some(stream) = &mut self.stream {
            if stream.write_all(&data).is_err() {
                self.stream = None; // will reconnect on next call
            }
        }

        Ok(())
    }
}
