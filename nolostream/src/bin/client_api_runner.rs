// This module is only compiled when the `client-api` feature is active.
// Declared in server.rs as: #[cfg(feature = "client-api")] mod client_api_runner;
//
// Removing NoloClientLib support in the future requires only:
//   - Delete this file
//   - Remove the `#[cfg(feature = "client-api")] mod client_api_runner;` line from server.rs
//   - Remove the one `if args.client_api { … }` branch from main()

use std::collections::HashMap;
use std::path::PathBuf;
use std::time::{Duration, Instant};

use nolostream::{
    transport::{Transport, TransportError},
    teleop::TeleopState,
    Command, ControllerState, CsvLogger, DeviceId, NoloClientApi,
};

pub fn run(
    mut transports: Vec<Box<dyn Transport>>,
    debug: bool,
    csv_log: Option<PathBuf>,
) {
    use nolostream::nolo_data_to_poses;

    let api = NoloClientApi::open().unwrap_or_else(|e| {
        eprintln!("error: {e}");
        std::process::exit(1);
    });

    let mut csv_logger: Option<CsvLogger> = csv_log.as_deref().map(|p| {
        CsvLogger::create(p).unwrap_or_else(|e| {
            eprintln!("error: cannot open csv-log {p:?}: {e}");
            std::process::exit(1);
        })
    });

    if debug {
        eprintln!("debug mode: printing latest poses every 1s");
    }
    eprintln!("streaming via NoloClientLib... (Ctrl-C to stop)");

    let mut total: u64 = 0;
    let mut counts: HashMap<DeviceId, u64> = HashMap::new();
    let mut latest: HashMap<DeviceId, ControllerState> = HashMap::new();
    let mut last_log = Instant::now();
    let mut teleop = TeleopState::new();

    loop {
        if let Some(raw) = api.get_data() {
            let poses = nolo_data_to_poses(&raw);
            total += poses.len() as u64;
            for p in &poses {
                *counts.entry(p.device.clone()).or_insert(0) += 1;
                if debug {
                    latest.insert(p.device.clone(), p.clone());
                }
            }
            if let Some(ref mut logger) = csv_logger {
                for p in &poses {
                    if let Err(e) = logger.write_pose("client_api", p, None) {
                        eprintln!("csv-log write error: {e}");
                    }
                }
            }
            dispatch(&mut transports, &poses);

            // Drain incoming handover messages from transports.
            for t in transports.iter_mut() {
                for msg in t.recv_teleop_target_msgs() {
                    match msg {
                        nolostream::teleop::TeleopTargetMsg::HandoverActive => {
                            teleop.on_handover_active();
                        }
                    }
                }
            }

            let update = teleop.update(&poses);
            if !update.frames.is_empty() {
                for t in transports.iter_mut() {
                    if let Err(e) = t.send_teleop(&update.frames) {
                        eprintln!("teleop dispatch error: {e}");
                    }
                }
            }
            if let Some(ref handover) = update.handover_out {
                for t in transports.iter_mut() {
                    let _ = t.send_handover(handover);
                }
            }
        }

        for t in transports.iter_mut() {
            for cmd in t.recv_commands() {
                match cmd {
                    Command::Haptic { device, intensity } => {
                        api.haptic_pulse(&device, intensity);
                    }
                    Command::SetHmdCenter { x, y, z } => {
                        api.set_hmd_center(x, y, z);
                    }
                    Command::CeilingMode { enabled } => {
                        api.ceiling_mode(enabled);
                    }
                    Command::UiCommand { content } => {
                        api.send_ui_command(&content);
                    }
                }
            }
        }

        let interval = if debug { Duration::from_secs(1) } else { Duration::from_secs(5) };
        if last_log.elapsed() >= interval {
            let hmd   = counts.get(&DeviceId::Headset).copied().unwrap_or(0);
            let left  = counts.get(&DeviceId::LeftController).copied().unwrap_or(0);
            let right = counts.get(&DeviceId::RightController).copied().unwrap_or(0);
            eprintln!("--- poses total={total} hmd={hmd} left={left} right={right}");
            if debug {
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

        std::thread::sleep(Duration::from_millis(16));
    }
}

fn dispatch(transports: &mut Vec<Box<dyn Transport>>, poses: &[ControllerState]) {
    transports.retain_mut(|t| match t.send(poses) {
        Ok(()) => true,
        Err(TransportError::Disconnected) => false,
        Err(TransportError::Io(msg)) => {
            eprintln!("transport io error: {msg}");
            true
        }
    });
}
