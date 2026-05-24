mod run_config;
#[cfg(feature = "client-api")] mod client_api_runner;
#[cfg(feature = "gui")] mod gui;

use std::collections::HashMap;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::time::{Duration, Instant};
use clap::Parser;
use nolostream::{CsvLogger, DeviceId, NoloStream};
use nolostream::DEFAULT_GYRO_SCALE;
use run_config::RunConfig;
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

    /// Print latest pose values for each device once per second.
    #[arg(long)]
    debug: bool,

    /// Use NoloClientLib.dll (requires NoloServer.exe running) instead of direct HID access.
    #[cfg(feature = "client-api")]
    #[arg(long)]
    client_api: bool,

    /// Open GUI even if transport flags are present (default: GUI when no flags given).
    #[arg(long)]
    no_ui: bool,

    /// Dump raw HID bytes to stderr (first 8 bytes per report) and exit after 5 s.
    #[arg(long)]
    raw_dump: bool,

    /// Enumerate all HID interfaces for the NoloVR VID/PID and try reading from each.
    #[arg(long)]
    enumerate: bool,

    /// Dump full decrypted frames to stderr (one 0xa5 and one 0xa6 sample), then exit.
    #[arg(long)]
    dump_decrypted: bool,

    /// Calibrate gyro scale: rotate a controller 360° around its Y-axis (sensor bubble axis)
    /// in approximately 5 s when prompted, then a suggested --gyro-scale value is printed.
    #[arg(long)]
    gyro_cal: bool,

    /// Gyro scale in rad/LSB (overrides default 0.001065). Use the value printed by --gyro-cal.
    #[arg(long, default_value_t = DEFAULT_GYRO_SCALE)]
    gyro_scale: f32,

    /// Write all incoming data to a CSV file (clears on start). Used by the API comparison session.
    #[arg(long)]
    csv_log: Option<PathBuf>,
}

fn main() {
    let args = Args::parse();

    // Enumerate mode: list all HID interfaces for the NoloVR VID/PID.
    if args.enumerate {
        let api = hidapi::HidApi::new().unwrap_or_else(|e| {
            eprintln!("hid api error: {e}");
            std::process::exit(1);
        });
        let devices: Vec<_> = api.device_list()
            .filter(|d| d.vendor_id() == nolostream::NOLO_VID && d.product_id() == nolostream::NOLO_PID)
            .collect();
        if devices.is_empty() {
            eprintln!("No NoloVR HID interfaces found");
        } else {
            eprintln!("{} NoloVR HID interface(s) — reading 1 s each:", devices.len());
            for (i, d) in devices.iter().enumerate() {
                let path = d.path().to_string_lossy();
                eprintln!("  [{i}] {path}  (iface={} usage_page={:#06x} usage={:#06x})",
                    d.interface_number(), d.usage_page(), d.usage());
                // Try reading from this interface for 1 s
                if let Ok(dev) = api.open_path(d.path()) {
                    let deadline = Instant::now() + Duration::from_secs(1);
                    let mut count = 0u32;
                    let mut sample = Vec::new();
                    while Instant::now() < deadline {
                        let mut buf = [0u8; 64];
                        if let Ok(n) = dev.read_timeout(&mut buf, 50) {
                            if n > 0 {
                                count += 1;
                                if sample.is_empty() {
                                    sample = buf[..n.min(8)].to_vec();
                                }
                            }
                        }
                    }
                    if count > 0 {
                        let hex: Vec<String> = sample.iter().map(|b| format!("{b:02x}")).collect();
                        eprintln!("       → {count} reports (first: {})", hex.join(" "));
                    } else {
                        eprintln!("       → 0 reports (no data)");
                    }
                } else {
                    eprintln!("       → could not open");
                }
            }
        }
        return;
    }

    // Dump-decrypted mode: capture 5 samples of each frame type, show which bytes change.
    if args.dump_decrypted {
        let device = nolostream::NoloDevice::open().unwrap_or_else(|e| {
            eprintln!("error: {e}");
            std::process::exit(1);
        });
        eprintln!("dump-decrypted: capturing 5 samples of each frame type (10 s max)");
        let deadline = Instant::now() + Duration::from_secs(10);
        let mut samples_a5: Vec<[u8; 64]> = Vec::new();
        let mut samples_a6: Vec<[u8; 64]> = Vec::new();
        while Instant::now() < deadline && (samples_a5.len() < 5 || samples_a6.len() < 5) {
            if let Ok(raw) = device.read_report() {
                if raw.len() == 64 {
                    if let Some(dec) = nolostream::decrypt_report(&raw) {
                        match dec[0] {
                            0xa5 | 0x10 if samples_a5.len() < 5 => { samples_a5.push(dec); }
                            0xa6 | 0x11 if samples_a6.len() < 5 => { samples_a6.push(dec); }
                            _ => {}
                        }
                    }
                }
            }
        }
        for (label, samples) in [("0xa5 (ctrl)", &samples_a5), ("0xa6 (hmd)", &samples_a6)] {
            if samples.is_empty() { continue; }
            eprintln!("--- {label}: {} samples ---", samples.len());
            eprintln!("     offset: 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f");
            for row in 0..4usize {
                let off = row * 16;
                // Show min..max range of each byte across samples
                let mut line = format!("  [{off:02x}]  ");
                for col in 0..16usize {
                    let vals: Vec<u8> = samples.iter().map(|s| s[off + col]).collect();
                    let min = *vals.iter().min().unwrap();
                    let max = *vals.iter().max().unwrap();
                    if min == max {
                        line.push_str(&format!("{min:02x} "));
                    } else {
                        line.push_str("** ");
                    }
                }
                eprintln!("{line}(** = changing)");
            }
            eprintln!("  first sample:");
            for row in 0..4usize {
                let off = row * 16;
                let hex: Vec<String> = samples[0][off..off+16].iter().map(|b| format!("{b:02x}")).collect();
                eprintln!("  [{off:02x}] {}", hex.join(" "));
            }
        }
        return;
    }

    // Raw dump mode: open device, print first 8 bytes of each report for 5 s, exit.
    if args.raw_dump {
        let device = nolostream::NoloDevice::open().unwrap_or_else(|e| {
            eprintln!("error: {e}");
            std::process::exit(1);
        });
        eprintln!("raw-dump: printing raw HID reports for 5 s (Ctrl-C to stop)");
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut count = 0u64;
        while Instant::now() < deadline {
            match device.read_report() {
                Ok(buf) if !buf.is_empty() => {
                    count += 1;
                    let preview: Vec<String> = buf.iter().take(8).map(|b| format!("{b:02x}")).collect();
                    eprintln!("[raw#{count:04}] n={} bytes: {}", buf.len(), preview.join(" "));
                }
                Ok(_) => {} // timeout / no data
                Err(e) => eprintln!("[raw] read error: {e}"),
            }
        }
        eprintln!("[raw] done — {count} reports received in 5 s");
        return;
    }

    // Gyro calibration mode: rotate controller 360° around Y-axis (bubble axis) in 5 s.
    if args.gyro_cal {
        let device = nolostream::NoloDevice::open().unwrap_or_else(|e| {
            eprintln!("error: {e}");
            std::process::exit(1);
        });
        eprintln!("gyro-cal: reading left controller (0xa5 frame), capturing sensor_raw[7] (RY = bubble axis)");
        eprintln!("  Hold the controller still...");
        std::thread::sleep(Duration::from_secs(2));
        eprintln!("  START rotating 360° around the sensor bubble axis NOW — you have 5 seconds.");
        let cal_duration = Duration::from_secs(5);
        let deadline = Instant::now() + cal_duration;
        let mut sum: i64 = 0;
        let mut count: u64 = 0;
        let mut prev_time = Instant::now();
        let mut total_dt_secs: f64 = 0.0;
        while Instant::now() < deadline {
            if let Ok(raw) = device.read_report() {
                if raw.len() >= 64 {
                    if let Some(dec) = nolostream::decrypt_report(&raw) {
                        if matches!(dec[0], 0xa5 | 0x10) {
                            // sensor_raw[7] = RY: i16 BE at base+1+7*2 = base+15 = buf[16..17]
                            let ry = i16::from_be_bytes([dec[16], dec[17]]) as i64;
                            let now = Instant::now();
                            total_dt_secs += now.duration_since(prev_time).as_secs_f64();
                            prev_time = now;
                            sum += ry.abs();
                            count += 1;
                        }
                    }
                }
            }
        }
        eprintln!("  STOP.");
        if count < 10 {
            eprintln!("  Too few samples ({count}). Is the left controller on and close to the base station?");
            std::process::exit(1);
        }
        let dt_avg = total_dt_secs / count as f64;
        let scale = std::f64::consts::TAU / (sum as f64 * dt_avg);
        eprintln!("  samples={count}  dt_avg={dt_avg:.4}s  |sum(RY)|={sum}");
        eprintln!("  Suggested: --gyro-scale {scale:.6}");
        eprintln!("  (default was {DEFAULT_GYRO_SCALE:.6})");
        return;
    }

    // ── Build RunConfig from CLI args ─────────────────────────────────────────
    let config = RunConfig {
        tcp_listen_port: args.tcp_listen_at,
        ws_listen_port:  args.ws_listen_at,
        tcp_stream_to:   args.tcp_stream_to,
        udp_stream_to:   args.udp_stream_to,
        gyro_scale:      args.gyro_scale,
        debug:           args.debug,
        csv_log:         args.csv_log.clone(),
    };
    let has_transports = config.tcp_listen_port.is_some()
        || config.ws_listen_port.is_some()
        || config.tcp_stream_to.is_some()
        || config.udp_stream_to.is_some()
        || config.csv_log.is_some();

    // ── GUI mode (default when no transport flags given) ──────────────────────
    #[cfg(feature = "gui")]
    if !has_transports && !args.no_ui {
        gui::run_gui(config);
        return;
    }

    if !has_transports {
        eprintln!("error: at least one of --tcp-listen-at, --ws-listen-at, --tcp-stream-to, --udp-stream-to must be specified");
        #[cfg(feature = "gui")]
        eprintln!("       (or run without transport flags to open the GUI)");
        std::process::exit(1);
    }

    // ── Client-API path ───────────────────────────────────────────────────────
    #[cfg(feature = "client-api")]
    if args.client_api {
        let (transports, errors) = run_config::build_transports(&config);
        for e in &errors {
            eprintln!("error: {e}");
        }
        if !errors.is_empty() {
            std::process::exit(1);
        }
        client_api_runner::run(transports, config.debug, config.csv_log);
        return;
    }

    run_headless(config);
}

// ── Headless HID streaming loop with reconnect ────────────────────────────────

fn run_headless(config: RunConfig) {
    let mut stream = NoloStream::new();
    if config.gyro_scale != DEFAULT_GYRO_SCALE {
        stream.set_gyro_scale(config.gyro_scale);
        eprintln!("gyro_scale = {:.6} rad/LSB", config.gyro_scale);
    }
    if let Some(ref csv_path) = config.csv_log {
        match CsvLogger::create(csv_path) {
            Ok(logger) => stream.set_csv_log(logger),
            Err(e) => {
                eprintln!("error: cannot open csv-log {csv_path:?}: {e}");
                std::process::exit(1);
            }
        }
    }

    let (transports, errors) = run_config::build_transports(&config);
    for e in &errors {
        eprintln!("error: {e}");
    }
    if !errors.is_empty() {
        std::process::exit(1);
    }
    for t in transports {
        stream.add_transport(t);
    }

    if config.debug {
        eprintln!("debug mode: printing latest poses every 1s");
    }
    if !stream.is_device_connected() {
        eprintln!("NoloVR device not found at startup, retrying every 1 s…");
    }
    eprintln!("streaming… (Ctrl-C to stop)");

    let mut total: u64 = 0;
    let mut counts: HashMap<DeviceId, u64> = HashMap::new();
    let mut latest: HashMap<DeviceId, nolostream::ControllerState> = HashMap::new();
    let mut last_log = Instant::now();
    let mut last_reconnect = Instant::now() - Duration::from_secs(2);
    let mut was_connected = stream.is_device_connected();

    loop {
        // Retry HID connection at most once per second
        if !stream.is_device_connected() && last_reconnect.elapsed() >= Duration::from_secs(1) {
            last_reconnect = Instant::now();
            stream.try_reconnect();
        }
        let now_connected = stream.is_device_connected();
        if now_connected != was_connected {
            if now_connected {
                eprintln!("HID device reconnected");
            } else {
                eprintln!("HID device disconnected, retrying…");
            }
            was_connected = now_connected;
        }

        match stream.poll_once() {
            Ok((poses, _)) => {
                if !poses.is_empty() {
                    total += poses.len() as u64;
                    for p in &poses {
                        *counts.entry(p.device.clone()).or_insert(0) += 1;
                        if config.debug {
                            latest.insert(p.device.clone(), p.clone());
                        }
                    }
                }
                let interval = if config.debug { Duration::from_secs(1) } else { Duration::from_secs(5) };
                if last_log.elapsed() >= interval {
                    let hmd   = counts.get(&DeviceId::Headset).copied().unwrap_or(0);
                    let left  = counts.get(&DeviceId::LeftController).copied().unwrap_or(0);
                    let right = counts.get(&DeviceId::RightController).copied().unwrap_or(0);
                    eprintln!("--- poses total={total} hmd={hmd} left={left} right={right}");
                    if config.debug {
                        for (dev, p) in &latest {
                            let tag = match dev {
                                DeviceId::Headset         => "HMD",
                                DeviceId::LeftController  => "L  ",
                                DeviceId::RightController => "R  ",
                            };
                            eprintln!(
                                "  [{tag}] pos=[{:+.4}, {:+.4}, {:+.4}]  q=[{:+.5}, {:+.5}, {:+.5}, {:+.5}]",
                                p.position[0], p.position[1], p.position[2],
                                p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3]
                            );
                        }
                    }
                    last_log = Instant::now();
                }
            }
            Err(e) => {
                eprintln!("poll error: {e:?}");
                std::thread::sleep(Duration::from_millis(10));
            }
        }
    }
}

