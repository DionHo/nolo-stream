use std::io::{self};
use std::net::{SocketAddr, TcpListener, TcpStream};

use tungstenite::WebSocket;

use crate::transport::{Transport, TransportError};
use crate::Pose;

pub struct WsListenerTransport {
    listener: TcpListener,
    clients: Vec<WebSocket<TcpStream>>,
}

impl WsListenerTransport {
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

impl Transport for WsListenerTransport {
    fn send(&mut self, poses: &[Pose]) -> Result<(), TransportError> {
        // Accept any pending new TCP connections (non-blocking).
        loop {
            match self.listener.accept() {
                Ok((stream, _)) => {
                    // Ensure the stream is blocking for the WS handshake.
                    let _ = stream.set_nonblocking(false);
                    match tungstenite::accept(stream) {
                        Ok(ws) => {
                            // Switch to non-blocking for subsequent sends.
                            let _ = ws.get_ref().set_nonblocking(true);
                            self.clients.push(ws);
                        }
                        Err(_) => {} // bad handshake — skip
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }

        let json_str = serde_json::to_string(poses).unwrap();
        let msg = tungstenite::Message::Text(json_str.into());

        self.clients.retain_mut(|client| client.send(msg.clone()).is_ok());

        Ok(())
    }
}
