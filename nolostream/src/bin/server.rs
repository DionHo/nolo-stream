use std::net::SocketAddr;
use clap::Parser;
use nolostream::{NoloStream, TcpListenerTransport, WsListenerTransport, TcpStreamTransport, UdpStreamTransport};

#[derive(Parser)]
#[command(name = "nolostream_server", version, about = "Stream NoloVR pose data over TCP/UDP/WebSocket")]
struct Args {
    #[arg(long)]
    tcp_listen_at: Option<u16>,

    #[arg(long)]
    ws_listen_at: Option<u16>,

    #[arg(long)]
    tcp_stream_to: Option<SocketAddr>,

    #[arg(long)]
    udp_stream_to: Option<SocketAddr>,
}

fn main() {
    let args = Args::parse();

    // Validate at least one transport is specified
    if args.tcp_listen_at.is_none() && args.ws_listen_at.is_none()
        && args.tcp_stream_to.is_none() && args.udp_stream_to.is_none()
    {
        eprintln!("error: at least one of --tcp-listen-at, --ws-listen-at, --tcp-stream-to, --udp-stream-to must be specified");
        std::process::exit(1);
    }

    // Open NoloVR device
    let mut stream = NoloStream::new().unwrap_or_else(|e| {
        eprintln!("error: failed to open NoloVR device: {e:?}");
        std::process::exit(1);
    });

    // Register transports
    if let Some(port) = args.tcp_listen_at {
        let t = TcpListenerTransport::bind(port).unwrap_or_else(|e| {
            eprintln!("error: failed to bind TCP listener on :{port}: {e}");
            std::process::exit(1);
        });
        stream.add_transport(Box::new(t));
        eprintln!("TCP listener on :{port}");
    }
    if let Some(port) = args.ws_listen_at {
        let t = WsListenerTransport::bind(port).unwrap_or_else(|e| {
            eprintln!("error: failed to bind WebSocket listener on :{port}: {e}");
            std::process::exit(1);
        });
        stream.add_transport(Box::new(t));
        eprintln!("WebSocket listener on :{port}");
    }
    if let Some(addr) = args.tcp_stream_to {
        stream.add_transport(Box::new(TcpStreamTransport::connect(addr)));
        eprintln!("TCP streaming to {addr}");
    }
    if let Some(addr) = args.udp_stream_to {
        let t = UdpStreamTransport::new(addr).unwrap_or_else(|e| {
            eprintln!("error: failed to create UDP socket for {addr}: {e}");
            std::process::exit(1);
        });
        stream.add_transport(Box::new(t));
        eprintln!("UDP streaming to {addr}");
    }

    // Polling loop — Ctrl-C exits
    eprintln!("streaming... (Ctrl-C to stop)");
    let mut total: u64 = 0;
    let mut headset: u64 = 0;
    let mut left: u64 = 0;
    let mut right: u64 = 0;
    let mut last_log = std::time::Instant::now();
    loop {
        match stream.poll_once() {
            Ok(poses) => {
                if !poses.is_empty() {
                    total += poses.len() as u64;
                    for p in &poses {
                        match p.device {
                            nolostream::DeviceId::Headset => headset += 1,
                            nolostream::DeviceId::LeftController => left += 1,
                            nolostream::DeviceId::RightController => right += 1,
                        }
                    }
                    if last_log.elapsed() >= std::time::Duration::from_secs(5) {
                        eprintln!("poses total={total} headset={headset} left={left} right={right}");
                        last_log = std::time::Instant::now();
                    }
                }
            }
            Err(e) => {
                eprintln!("poll error: {e:?}");
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
        }
    }
}
