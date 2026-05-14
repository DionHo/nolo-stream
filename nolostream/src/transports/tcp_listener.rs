use std::io::{self, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};

use crate::transport::{Transport, TransportError};
use crate::Pose;

pub struct TcpListenerTransport {
    listener: TcpListener,
    clients: Vec<TcpStream>,
}

impl TcpListenerTransport {
    pub fn bind(port: u16) -> io::Result<Self> {
        let listener = TcpListener::bind(("0.0.0.0", port))?;
        listener.set_nonblocking(true)?;
        Ok(Self { listener, clients: Vec::new() })
    }

    /// Returns the local address the listener is bound to (useful for ephemeral port discovery).
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.listener.local_addr()
    }
}

impl Transport for TcpListenerTransport {
    fn send(&mut self, poses: &[Pose]) -> Result<(), TransportError> {
        // Accept any pending new connections (non-blocking).
        loop {
            match self.listener.accept() {
                Ok((stream, _)) => self.clients.push(stream),
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }

        if self.clients.is_empty() {
            return Ok(());
        }

        let mut data = serde_json::to_vec(poses).unwrap();
        data.push(b'\n');

        self.clients.retain_mut(|client| client.write_all(&data).is_ok());

        Ok(())
    }
}
