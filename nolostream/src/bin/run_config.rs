use std::net::SocketAddr;
use std::path::PathBuf;

use nolostream::{
    transport::Transport, TcpListenerTransport, TcpStreamTransport, UdpStreamTransport,
    WsListenerTransport, DEFAULT_GYRO_SCALE,
};

/// Runtime streaming configuration, shared between CLI and GUI.
#[derive(Clone, Debug)]
pub struct RunConfig {
    pub tcp_listen_port: Option<u16>,
    pub ws_listen_port:  Option<u16>,
    pub tcp_stream_to:   Option<SocketAddr>,
    pub udp_stream_to:   Option<SocketAddr>,
    pub gyro_scale:      f32,
    pub debug:           bool,
    pub csv_log:         Option<PathBuf>,
}

impl Default for RunConfig {
    fn default() -> Self {
        RunConfig {
            tcp_listen_port: None,
            ws_listen_port:  None,
            tcp_stream_to:   None,
            udp_stream_to:   None,
            gyro_scale:      DEFAULT_GYRO_SCALE,
            debug:           false,
            csv_log:         None,
        }
    }
}

/// Build transports from a `RunConfig`.
/// Returns `(transports, errors)`. In headless mode, callers may exit on errors;
/// in GUI mode, callers push errors to the log and continue.
pub fn build_transports(config: &RunConfig) -> (Vec<Box<dyn Transport>>, Vec<String>) {
    let mut transports: Vec<Box<dyn Transport>> = Vec::new();
    let mut errors: Vec<String> = Vec::new();

    if let Some(port) = config.tcp_listen_port {
        match TcpListenerTransport::bind(port) {
            Ok(t) => {
                eprintln!("TCP listener on :{port}");
                transports.push(Box::new(t));
            }
            Err(e) => errors.push(format!("TCP listen :{port}: {e}")),
        }
    }
    if let Some(port) = config.ws_listen_port {
        match WsListenerTransport::bind(port) {
            Ok(t) => {
                eprintln!("WebSocket listener on :{port}");
                transports.push(Box::new(t));
            }
            Err(e) => errors.push(format!("WS listen :{port}: {e}")),
        }
    }
    if let Some(addr) = config.tcp_stream_to {
        eprintln!("TCP streaming to {addr}");
        transports.push(Box::new(TcpStreamTransport::connect(addr)));
    }
    if let Some(addr) = config.udp_stream_to {
        match UdpStreamTransport::new(addr) {
            Ok(t) => {
                eprintln!("UDP streaming to {addr}");
                transports.push(Box::new(t));
            }
            Err(e) => errors.push(format!("UDP push {addr}: {e}")),
        }
    }

    (transports, errors)
}
