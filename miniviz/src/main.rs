use clap::Parser;
use tiny_http::Server;

const INDEX_HTML: &str = include_str!("../web/index.html");

#[derive(Parser)]
#[command(name = "miniviz", version, about = "Visualize NoloVR pose data")]
struct Args {
    #[arg(long)]
    connect: String, // WebSocket URL, e.g. ws://127.0.0.1:12345
}

fn main() {
    let args = Args::parse();

    let server = Server::http("127.0.0.1:0").expect("Failed to bind HTTP server");
    let port = server
        .server_addr()
        .to_ip()
        .expect("server address is not IP")
        .port();

    let encoded_ws = urlencoding::encode(&args.connect);
    let url = format!("http://127.0.0.1:{port}/?ws={encoded_ws}");

    eprintln!("miniviz serving at http://127.0.0.1:{port}/");

    open::that(&url).expect("Failed to open browser");

    let content_type =
        tiny_http::Header::from_bytes(b"Content-Type", b"text/html; charset=utf-8").unwrap();

    for request in server.incoming_requests() {
        let response = tiny_http::Response::from_string(INDEX_HTML).with_header(content_type.clone());
        let _ = request.respond(response);
    }
}
