use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};

use crate::teleop::{HandoverMsg, TeleopTargetMsg, TeleopFrame};
use crate::transport::{Transport, TransportError};
use crate::ControllerState;

struct TcpClient {
    stream: TcpStream,
    read_buf: Vec<u8>,
}

pub struct TcpListenerTransport {
    listener: TcpListener,
    clients: Vec<TcpClient>,
    pending_teleop_target_msgs: VecDeque<TeleopTargetMsg>,
}

impl TcpListenerTransport {
    pub fn bind(port: u16) -> io::Result<Self> {
        let listener = TcpListener::bind(("0.0.0.0", port))?;
        listener.set_nonblocking(true)?;
        Ok(Self { listener, clients: Vec::new(), pending_teleop_target_msgs: VecDeque::new() })
    }

    /// Returns the local address the listener is bound to (useful for ephemeral port discovery).
    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.listener.local_addr()
    }

    fn accept_new_clients(&mut self) {
        loop {
            match self.listener.accept() {
                Ok((stream, _)) => {
                    // 1 ms read timeout makes reads effectively non-blocking.
                    let _ = stream.set_read_timeout(Some(std::time::Duration::from_millis(1)));
                    self.clients.push(TcpClient { stream, read_buf: Vec::new() });
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
    }

    /// Read available bytes from all clients, parse complete newline-delimited JSON lines.
    fn drain_incoming(&mut self) {
        let mut buf = [0u8; 1024];
        for client in &mut self.clients {
            loop {
                match client.stream.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => client.read_buf.extend_from_slice(&buf[..n]),
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock
                           || e.kind() == io::ErrorKind::TimedOut => break,
                    Err(_) => break,
                }
            }
            // Parse all complete lines.
            while let Some(pos) = client.read_buf.iter().position(|&b| b == b'\n') {
                let line: Vec<u8> = client.read_buf.drain(..=pos).collect();
                if let Ok(txt) = std::str::from_utf8(&line) {
                    if let Ok(tmsg) = serde_json::from_str::<TeleopTargetMsg>(txt.trim()) {
                        self.pending_teleop_target_msgs.push_back(tmsg);
                    }
                }
            }
        }
    }
}

impl Transport for TcpListenerTransport {
    fn send(&mut self, poses: &[ControllerState]) -> Result<(), TransportError> {
        self.accept_new_clients();
        self.drain_incoming();

        if self.clients.is_empty() {
            return Ok(());
        }

        let mut data = serde_json::to_vec(poses).unwrap();
        data.push(b'\n');

        self.clients.retain_mut(|client| client.stream.write_all(&data).is_ok());

        Ok(())
    }

    fn send_teleop(&mut self, frames: &[TeleopFrame]) -> Result<(), TransportError> {
        if self.clients.is_empty() {
            return Ok(());
        }
        for frame in frames {
            let mut data = serde_json::to_vec(frame).unwrap();
            data.push(b'\n');
            self.clients.retain_mut(|client| client.stream.write_all(&data).is_ok());
        }
        Ok(())
    }

    fn send_handover(&mut self, msg: &HandoverMsg) -> Result<(), TransportError> {
        if self.clients.is_empty() {
            return Ok(());
        }
        let mut data = serde_json::to_vec(msg).unwrap();
        data.push(b'\n');
        self.clients.retain_mut(|client| client.stream.write_all(&data).is_ok());
        Ok(())
    }

    fn recv_teleop_target_msgs(&mut self) -> Vec<TeleopTargetMsg> {
        self.pending_teleop_target_msgs.drain(..).collect()
    }
}

