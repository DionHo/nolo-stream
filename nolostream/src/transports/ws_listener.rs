use std::collections::VecDeque;
use std::io;
use std::net::{SocketAddr, TcpListener, TcpStream};

use tungstenite::WebSocket;

use crate::command::Command;
use crate::teleop::TeleopFrame;
use crate::transport::{Transport, TransportError};
use crate::ControllerState;

pub struct WsListenerTransport {
    listener: TcpListener,
    clients: Vec<WebSocket<TcpStream>>,
    pending_commands: VecDeque<Command>,
}

impl WsListenerTransport {
    pub fn bind(port: u16) -> io::Result<Self> {
        let listener = TcpListener::bind(("0.0.0.0", port))?;
        listener.set_nonblocking(true)?;
        Ok(Self { listener, clients: Vec::new(), pending_commands: VecDeque::new() })
    }

    pub fn local_addr(&self) -> io::Result<SocketAddr> {
        self.listener.local_addr()
    }

    fn accept_new_clients(&mut self) {
        loop {
            match self.listener.accept() {
                Ok((stream, addr)) => {
                    let _ = stream.set_read_timeout(Some(std::time::Duration::from_millis(50)));
                    if let Ok(ws) = tungstenite::accept(stream) {
                        let _ = ws.get_ref().set_nonblocking(true);
                        let _ = ws.get_ref().set_read_timeout(None);
                        eprintln!("ws client connected from {addr}");
                        self.clients.push(ws);
                    }
                }
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
    }
}

impl Transport for WsListenerTransport {
    fn send(&mut self, poses: &[ControllerState]) -> Result<(), TransportError> {
        self.accept_new_clients();

        let json_str = serde_json::to_string(poses).unwrap();
        let msg = tungstenite::Message::Text(json_str);
        let cmds = &mut self.pending_commands;

        let before = self.clients.len();
        self.clients.retain_mut(|client| {
            loop {
                match client.read() {
                    Ok(tungstenite::Message::Text(txt)) => {
                        if let Ok(cmd) = serde_json::from_str::<Command>(&txt) {
                            cmds.push_back(cmd);
                        }
                    }
                    Ok(tungstenite::Message::Close(_)) => return false,
                    Ok(_) => {}
                    Err(tungstenite::Error::Io(e)) if e.kind() == io::ErrorKind::WouldBlock => break,
                    Err(_) => return false,
                }
            }
            client.send(msg.clone()).is_ok()
        });

        let dropped = before - self.clients.len();
        if dropped > 0 {
            eprintln!("ws: {dropped} client(s) disconnected ({} remaining)", self.clients.len());
        }

        Ok(())
    }

    fn send_teleop(&mut self, frames: &[TeleopFrame]) -> Result<(), TransportError> {
        if self.clients.is_empty() {
            return Ok(());
        }
        let inner = serde_json::to_string(frames).unwrap();
        let msg = tungstenite::Message::Text(format!("{{\"teleop\":{inner}}}"));
        for client in &mut self.clients {
            let _ = client.send(msg.clone());
        }
        Ok(())
    }

    fn recv_commands(&mut self) -> Vec<Command> {
        self.pending_commands.drain(..).collect()
    }
}

