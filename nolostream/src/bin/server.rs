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
        stream.add_transport(Box::new(TcpListenerTransport::bind(port).unwrap()));
        eprintln!("TCP listener on :{port}");
    }
    if let Some(port) = args.ws_listen_at {
        stream.add_transport(Box::new(WsListenerTransport::bind(port).unwrap()));
        eprintln!("WebSocket listener on :{port}");
    }
    if let Some(addr) = args.tcp_stream_to {
        stream.add_transport(Box::new(TcpStreamTransport::connect(addr)));
        eprintln!("TCP streaming to {addr}");
    }
    if let Some(addr) = args.udp_stream_to {
        stream.add_transport(Box::new(UdpStreamTransport::new(addr).unwrap()));
        eprintln!("UDP streaming to {addr}");
    }

    // Polling loop — Ctrl-C exits
    eprintln!("streaming... (Ctrl-C to stop)");
    loop {
        match stream.poll_once() {
            Ok(_) => {},
            Err(e) => {
                eprintln!("poll error: {e:?}");
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
        }
    }
}
