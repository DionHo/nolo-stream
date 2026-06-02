use std::io::{BufRead, BufReader, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use clap::Parser;
use tungstenite::{accept, Message};

const INDEX_HTML: &str = include_str!("../web/index.html");

#[derive(Parser)]
#[command(name = "miniviz", version, about = "Visualize NoloVR pose data")]
struct Args {
    /// WebSocket URL to receive pose data from NoloStream
    #[arg(long)]
    connect: String,

    /// TCP port for left-controller teleop (NoloStream connects here)
    #[arg(long, default_value = "9001")]
    teleop_left_port: u16,

    /// TCP port for right-controller teleop (NoloStream connects here)
    #[arg(long, default_value = "9002")]
    teleop_right_port: u16,
}

/// Shared writable TCP connection slot.
type SharedConn = Arc<Mutex<Option<TcpStream>>>;

/// Insert `"device":"<name>"` at the start of a JSON object string.
fn enrich_json(json: &str, device: &str) -> String {
    let t = json.trim();
    if t.starts_with('{') && t.ends_with('}') {
        let inner = &t[1..t.len() - 1];
        if inner.trim().is_empty() {
            format!(r#"{{"device":"{}"}}"#, device)
        } else {
            format!(r#"{{"device":"{}",{}}}"#, device, inner)
        }
    } else {
        json.to_owned()
    }
}

/// Write a newline-terminated message to a SharedConn. Clears the slot on error.
fn write_conn(conn: &SharedConn, msg: &str) {
    let mut guard = conn.lock().unwrap();
    if let Some(stream) = guard.as_mut() {
        let mut data = msg.to_owned();
        data.push('\n');
        if stream.write_all(data.as_bytes()).is_err() {
            *guard = None;
        }
    }
}

/// Listen for a TCP connection from NoloStream, read JSON lines, enrich with device tag,
/// and forward to `to_browser`. Stores the write-half in `conn`.
fn spawn_tcp_listener(
    port: u16,
    device: &'static str,
    to_browser: std::sync::mpsc::SyncSender<String>,
    conn: SharedConn,
) {
    thread::Builder::new()
        .name(format!("tcp-{device}"))
        .spawn(move || {
            let listener = TcpListener::bind(("127.0.0.1", port))
                .unwrap_or_else(|e| panic!("cannot bind teleop {device} port {port}: {e}"));
            eprintln!("[teleop] {device}: listening on :{port}");
            for incoming in listener.incoming() {
                let Ok(stream) = incoming else { continue };
                eprintln!("[teleop] {device}: NoloStream connected");
                let write_half = match stream.try_clone() {
                    Ok(s) => s,
                    Err(_) => continue,
                };
                *conn.lock().unwrap() = Some(write_half);
                let reader = BufReader::new(&stream);
                for line in reader.lines() {
                    match line {
                        Ok(text) if !text.trim().is_empty() => {
                            let enriched = enrich_json(&text, device);
                            let _ = to_browser.try_send(enriched);
                        }
                        Ok(_) => {}
                        Err(_) => break,
                    }
                }
                *conn.lock().unwrap() = None;
                eprintln!("[teleop] {device}: NoloStream disconnected");
            }
        })
        .unwrap();
}

/// Parse a browser WebSocket message and forward `handover_active` to the right TCP connection.
/// Expected: {"type":"handover_active","device":"left_controller"|"right_controller"}
fn handle_browser_msg(text: &str, left: &SharedConn, right: &SharedConn) {
    if !text.contains(r#""handover_active""#) {
        return;
    }
    let conn = if text.contains("left_controller") {
        left
    } else if text.contains("right_controller") {
        right
    } else {
        return;
    };
    write_conn(conn, r#"{"type":"handover_active"}"#);
}

/// Run the WebSocket server the browser connects to for teleop data.
/// Accepts one browser at a time; relays frames from `rx` and handles browser commands.
fn run_teleop_ws_server(
    listener: TcpListener,
    rx: std::sync::mpsc::Receiver<String>,
    left: SharedConn,
    right: SharedConn,
) {
    for incoming in listener.incoming() {
        let Ok(tcp) = incoming else { continue };
        // Short timeout for handshake, then switch to non-blocking.
        let _ = tcp.set_read_timeout(Some(Duration::from_millis(100)));
        let Ok(mut ws) = accept(tcp) else { continue };
        let _ = ws.get_ref().set_nonblocking(true);
        let _ = ws.get_ref().set_read_timeout(None);
        // Discard frames buffered while no browser was connected.
        while rx.try_recv().is_ok() {}
        loop {
            // Read browser commands (non-blocking).
            match ws.read() {
                Ok(Message::Text(text)) => handle_browser_msg(&text, &left, &right),
                Ok(Message::Close(_)) => break,
                Ok(Message::Ping(data)) => {
                    let _ = ws.send(Message::Pong(data));
                }
                Ok(_) => {}
                Err(tungstenite::Error::Io(ref e))
                    if e.kind() == std::io::ErrorKind::WouldBlock => {}
                Err(_) => break,
            }
            // Forward pending teleop frames to browser.
            let mut send_err = false;
            while let Ok(msg) = rx.try_recv() {
                if ws.send(Message::Text(msg)).is_err() {
                    send_err = true;
                    break;
                }
            }
            if send_err {
                break;
            }
            thread::sleep(Duration::from_millis(5));
        }
    }
}

fn main() {
    let args = Args::parse();

    let left_conn: SharedConn = Arc::new(Mutex::new(None));
    let right_conn: SharedConn = Arc::new(Mutex::new(None));

    // Channel: TCP threads → browser WS thread.
    let (tx, rx) = std::sync::mpsc::sync_channel::<String>(512);

    spawn_tcp_listener(args.teleop_left_port, "left_controller", tx.clone(), left_conn.clone());
    spawn_tcp_listener(args.teleop_right_port, "right_controller", tx, right_conn.clone());

    // WebSocket server for browser (OS-assigned port).
    let teleop_ws_listener =
        TcpListener::bind("127.0.0.1:0").expect("cannot bind teleop WS server");
    let teleop_ws_port = teleop_ws_listener.local_addr().unwrap().port();
    eprintln!("[miniviz] teleop WS for browser on :{teleop_ws_port}");
    {
        let left = left_conn.clone();
        let right = right_conn.clone();
        thread::Builder::new()
            .name("teleop-ws".to_owned())
            .spawn(move || run_teleop_ws_server(teleop_ws_listener, rx, left, right))
            .unwrap();
    }

    // HTTP server (OS-assigned port).
    let http_server =
        tiny_http::Server::http("127.0.0.1:0").expect("cannot bind HTTP server");
    let http_port = http_server
        .server_addr()
        .to_ip()
        .expect("server address is not IP")
        .port();

    let teleop_ws_url = format!("ws://127.0.0.1:{teleop_ws_port}");
    let url = format!(
        "http://127.0.0.1:{http_port}/?ws={}&teleop_ws={}",
        urlencoding::encode(&args.connect),
        urlencoding::encode(&teleop_ws_url),
    );

    eprintln!("[miniviz] serving at http://127.0.0.1:{http_port}/");
    open::that(&url).expect("cannot open browser");

    let content_type =
        tiny_http::Header::from_bytes(b"Content-Type", b"text/html; charset=utf-8").unwrap();
    for request in http_server.incoming_requests() {
        let response =
            tiny_http::Response::from_string(INDEX_HTML).with_header(content_type.clone());
        let _ = request.respond(response);
    }
}
