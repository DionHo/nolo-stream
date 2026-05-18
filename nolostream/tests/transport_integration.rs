use std::io::BufRead;
use std::net::{TcpStream, UdpSocket};

use nolostream::{DeviceId, Pose, Transport, TcpListenerTransport, UdpStreamTransport};

fn headset_pose() -> Pose {
    Pose {
        device: DeviceId::Headset,
        position: [1.0, 2.0, 3.0],
        orientation: [1.0, 0.0, 0.0, 0.0],
        sensor_raw: [0; 32],
        timestamp_ms: 12345,
        touch_x: 255,
        touch_y: 255,
        battery: 0,
        buttons: 0,
        velocity: [0.0; 3],
        angular_velocity: [0.0; 3],
        state: 0,
    }
}

#[test]
fn tcp_listener_accepts_and_broadcasts() {
    let mut server = TcpListenerTransport::bind(0).unwrap();
    let port = server.local_addr().unwrap().port();

    // Connect to 127.0.0.1 — 0.0.0.0 is not a valid connect target on Windows.
    let client = TcpStream::connect(("127.0.0.1", port)).unwrap();

    let poses = vec![headset_pose()];
    server.send(&poses).unwrap();

    // Read one newline-delimited JSON line from the client.
    let mut reader = std::io::BufReader::new(client);
    let mut line = String::new();
    reader.read_line(&mut line).unwrap();

    let decoded: Vec<Pose> = serde_json::from_str(line.trim_end()).unwrap();
    assert_eq!(decoded.len(), 1);
    assert!(matches!(decoded[0].device, DeviceId::Headset));
    assert_eq!(decoded[0].timestamp_ms, 12345);
    assert_eq!(decoded[0].position, [1.0, 2.0, 3.0]);
}

#[test]
fn udp_stream_sends_datagrams() {
    let receiver = UdpSocket::bind("127.0.0.1:0").unwrap();
    let addr = receiver.local_addr().unwrap();
    receiver.set_read_timeout(Some(std::time::Duration::from_millis(500))).unwrap();

    let mut transport = UdpStreamTransport::new(addr).unwrap();

    let poses = vec![Pose {
        device: DeviceId::LeftController,
        position: [0.1, 0.2, 0.3],
        orientation: [1.0, 0.0, 0.0, 0.0],
        sensor_raw: [0; 32],
        timestamp_ms: 42,
        touch_x: 255,
        touch_y: 255,
        battery: 0,
        buttons: 0,
        velocity: [0.0; 3],
        angular_velocity: [0.0; 3],
        state: 0,
    }];
    transport.send(&poses).unwrap();

    let mut buf = vec![0u8; 4096];
    let (n, _) = receiver.recv_from(&mut buf).unwrap();
    let json_str = std::str::from_utf8(&buf[..n]).unwrap().trim_end();

    let decoded: Vec<Pose> = serde_json::from_str(json_str).unwrap();
    assert_eq!(decoded.len(), 1);
    assert!(matches!(decoded[0].device, DeviceId::LeftController));
    assert_eq!(decoded[0].timestamp_ms, 42);
    assert_eq!(decoded[0].position, [0.1_f32, 0.2, 0.3]);
}
