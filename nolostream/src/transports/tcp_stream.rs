use std::io::{self, Read, Write};
use std::net::{SocketAddr, TcpStream};
use std::time::Duration;

use crate::controller_state::DeviceId;
use crate::teleop::{HandoverMsg, TeleopTargetMsg, TeleopFrame};
use crate::transport::{Transport, TransportError};
use crate::ControllerState;

/// TCP transport dedicated to one controller's teleop stream.
///
/// Manages per-controller handover state:
/// - Receives `{"type":"handover_active"}` from the robot → activates, sends echo confirmation.
/// - Forwards delta frames for its controller when active.
/// - On `send_handover(Release)` → sends `{"type":"release"}` and deactivates.
pub struct TcpTeleopTransport {
    device: DeviceId,
    addr: SocketAddr,
    stream: Option<TcpStream>,
    read_buf: Vec<u8>,
    is_active: bool,
}

impl TcpTeleopTransport {
    /// Lazy constructor — no connection is made until the first `send()`.
    pub fn connect(addr: SocketAddr, device: DeviceId) -> Self {
        Self { device, addr, stream: None, read_buf: Vec::new(), is_active: false }
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

    /// Drain incoming bytes, detect HandoverActive, send echo confirmation.
    fn drain_incoming(&mut self) {
        // Read available bytes into buffer.
        let mut disconnected = false;
        if let Some(stream) = &mut self.stream {
            let mut buf = [0u8; 1024];
            loop {
                match stream.read(&mut buf) {
                    Ok(0) => { disconnected = true; break; }   // remote closed
                    Ok(n) => self.read_buf.extend_from_slice(&buf[..n]),
                    Err(e) if e.kind() == io::ErrorKind::WouldBlock
                           || e.kind() == io::ErrorKind::TimedOut => break,
                    Err(_) => { disconnected = true; break; }  // broken pipe
                }
            }
        }
        if disconnected {
            self.stream = None;
            self.is_active = false;
            self.read_buf.clear();
            return;
        }

        // Parse complete newline-delimited messages.
        let mut activation_received = false;
        while let Some(pos) = self.read_buf.iter().position(|&b| b == b'\n') {
            let line: Vec<u8> = self.read_buf.drain(..=pos).collect();
            if let Ok(txt) = std::str::from_utf8(&line) {
                if let Ok(TeleopTargetMsg::HandoverActive) =
                    serde_json::from_str::<TeleopTargetMsg>(txt.trim())
                {
                    activation_received = true;
                }
            }
        }

        // Activate and send confirmation echo.
        if activation_received {
            self.is_active = true;
            let mut data = serde_json::to_vec(&HandoverMsg::Active).unwrap();
            data.push(b'\n');
            if let Some(stream) = &mut self.stream {
                if stream.write_all(&data).is_err() {
                    self.stream = None;
                }
            }
        }
    }
}

impl Transport for TcpTeleopTransport {
    /// Drain incoming data each poll cycle (enables handover detection when trigger not held).
    fn send(&mut self, _poses: &[ControllerState]) -> Result<(), TransportError> {
        if self.ensure_connected() {
            self.drain_incoming();
        }
        Ok(())
    }

    /// Forward delta frames for this controller when handover is active.
    fn send_teleop(&mut self, frames: &[TeleopFrame]) -> Result<(), TransportError> {
        if !self.is_active {
            return Ok(());
        }
        if !self.ensure_connected() {
            return Ok(());
        }
        for frame in frames {
            if frame.device != self.device {
                continue;
            }
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

    /// Send release notification and deactivate when SYS button ends handover.
    fn send_handover(&mut self, msg: &HandoverMsg) -> Result<(), TransportError> {
        if matches!(msg, HandoverMsg::Release) && self.is_active {
            self.is_active = false;
            if self.ensure_connected() {
                let mut data = serde_json::to_vec(msg).unwrap();
                data.push(b'\n');
                if let Some(stream) = &mut self.stream {
                    if stream.write_all(&data).is_err() {
                        self.stream = None;
                    }
                }
            }
        }
        Ok(())
    }
}

