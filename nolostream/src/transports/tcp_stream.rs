use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::time::Duration;

use crate::teleop::{HandoverMsg, TeleopTargetMsg, TeleopFrame};
use crate::transport::{Transport, TransportError};
use crate::ControllerState;

pub struct TcpStreamTransport {
    addr: SocketAddr,
    stream: Option<TcpStream>,
    read_buf: Vec<u8>,
    pending_teleop_target_msgs: VecDeque<TeleopTargetMsg>,
}

impl TcpStreamTransport {
    /// Lazy constructor — no connection is made until the first `send()`.
    pub fn connect(addr: SocketAddr) -> Self {
        Self { addr, stream: None, read_buf: Vec::new(), pending_teleop_target_msgs: VecDeque::new() }
    }

    fn ensure_connected(&mut self) -> bool {
        if self.stream.is_some() {
            return true;
        }
        match TcpStream::connect_timeout(&self.addr, Duration::from_secs(1)) {
            Ok(s) => {
                let _ = s.set_write_timeout(Some(Duration::from_millis(500)));
                let _ = s.set_read_timeout(Some(Duration::from_millis(1)));
                self.stream = Some(s);
                true
            }
            Err(_) => false,
        }
    }

    fn drain_incoming(&mut self) {
        let stream = match &mut self.stream {
            Some(s) => s,
            None => return,
        };
        let mut buf = [0u8; 1024];
        loop {
            match stream.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => self.read_buf.extend_from_slice(&buf[..n]),
                Err(e) if e.kind() == io::ErrorKind::WouldBlock
                       || e.kind() == io::ErrorKind::TimedOut => break,
                Err(_) => break,
            }
        }
        while let Some(pos) = self.read_buf.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = self.read_buf.drain(..=pos).collect();
            if let Ok(txt) = std::str::from_utf8(&line) {
                if let Ok(tmsg) = serde_json::from_str::<TeleopTargetMsg>(txt.trim()) {
                    self.pending_teleop_target_msgs.push_back(tmsg);
                }
            }
        }
    }
}

impl Transport for TcpStreamTransport {
    fn send(&mut self, poses: &[ControllerState]) -> Result<(), TransportError> {
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
        self.drain_incoming();
        for frame in frames {
            let mut data = serde_json::to_vec(frame).unwrap();
            data.push(b'\n');
            if let Some(stream) = &mut self.stream {
                if stream.write_all(&data).is_err() {
                    self.stream = None;
                    break;
                }
            }
        }
        Ok(())
    }

    fn send_handover(&mut self, msg: &HandoverMsg) -> Result<(), TransportError> {
        if !self.ensure_connected() {
            return Ok(());
        }
        let mut data = serde_json::to_vec(msg).unwrap();
        data.push(b'\n');
        if let Some(stream) = &mut self.stream {
            if stream.write_all(&data).is_err() {
                self.stream = None;
            }
        }
        Ok(())
    }

    fn recv_teleop_target_msgs(&mut self) -> Vec<TeleopTargetMsg> {
        self.drain_incoming();
        self.pending_teleop_target_msgs.drain(..).collect()
    }
}

