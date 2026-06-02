use std::collections::VecDeque;
use std::net::SocketAddr;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use eframe::egui;

use nolostream::{DeviceId, NoloStream};

use crate::run_config::{build_transports, RunConfig};

// ── Shared state between GUI and worker thread ────────────────────────────────

pub struct AppState {
    pub config:           RunConfig,
    /// Bumped by GUI when config changes; worker rebuilds transports on mismatch.
    pub config_version:   u32,
    pub device_connected: bool,
    pub last_pose_time:   Option<Instant>,
    pub battery:          [u8; 2],  // [left, right]
    pub raw_left:         Option<[u8; 64]>,
    pub raw_right:        Option<[u8; 64]>,
    /// UKF P diagonal: [[left p_diag], [right p_diag]], indices 0-2=orient, 3-5=bias, 6-8=pos, 9-11=vel
    pub ukf_p:            [[f32; 12]; 2],
    pub log:              VecDeque<String>,
}

impl AppState {
    pub fn new(initial_config: RunConfig) -> Self {
        AppState {
            config:           initial_config,
            config_version:   1, // start at 1 so worker builds transports immediately
            device_connected: false,
            last_pose_time:   None,
            battery:          [0; 2],
            raw_left:         None,
            raw_right:        None,
            ukf_p:            [[0.01f32; 12]; 2],
            log:              VecDeque::with_capacity(128),
        }
    }

    pub fn push_log(&mut self, msg: impl Into<String>) {
        let secs = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();
        let h = (secs / 3600) % 24;
        let m = (secs / 60) % 60;
        let s = secs % 60;
        let entry = format!("[{h:02}:{m:02}:{s:02}] {}", msg.into());
        if self.log.len() >= 128 {
            self.log.pop_front();
        }
        self.log.push_back(entry);
    }
}

// ── Worker thread ─────────────────────────────────────────────────────────────

pub fn spawn_worker(state: Arc<Mutex<AppState>>) {
    thread::Builder::new()
        .name("nolo-worker".into())
        .spawn(move || worker_main(state))
        .expect("worker thread spawn failed");
}

fn worker_main(state: Arc<Mutex<AppState>>) {
    let mut stream = NoloStream::new();
    let mut current_version: u32 = 0; // will differ from config_version=1 on first iter
    let mut last_reconnect = Instant::now() - Duration::from_secs(2);
    let mut was_connected = stream.is_device_connected();
    // Tracks last pose received in the worker; used to detect silent disconnection.
    let mut last_worker_pose_at = Instant::now();

    {
        let mut s = state.lock().unwrap();
        if was_connected {
            s.push_log("HID device found at startup");
        } else {
            s.push_log("HID device not found, searching…");
        }
        s.device_connected = was_connected;
    }

    loop {
        // ── Read current config ────────────────────────────────────────────
        let (config, version) = {
            let s = state.lock().unwrap();
            (s.config.clone(), s.config_version)
        };

        // ── Rebuild transports on config change ────────────────────────────
        if version != current_version {
            current_version = version;
            let (transports, errors) = build_transports(&config);
            stream.replace_transports(transports);
            stream.set_gyro_scale(config.gyro_scale);
            let mut s = state.lock().unwrap();
            for e in errors {
                s.push_log(format!("transport error: {e}"));
            }
            s.push_log("transports reconfigured");
        }

        // ── HID reconnect (at most once per second) ────────────────────────
        // If device handle is open but no poses in 2 s, force-disconnect so
        // the reconnect loop below will reopen it (handles silent USB unplug).
        if stream.is_device_connected() && last_worker_pose_at.elapsed() > Duration::from_secs(2) {
            stream.force_disconnect();
        }

        // Propagate connect/disconnect to AppState BEFORE attempting reconnect,
        // so the GUI sees "Searching" for at least one repaint cycle.
        let now_connected = stream.is_device_connected();
        if now_connected != was_connected {
            let mut s = state.lock().unwrap();
            if now_connected {
                s.push_log("HID device connected");
            } else {
                s.push_log("HID device disconnected");
            }
            s.device_connected = now_connected;
            was_connected = now_connected;
        }

        if !stream.is_device_connected() && last_reconnect.elapsed() >= Duration::from_secs(1) {
            last_reconnect = Instant::now();
            stream.try_reconnect();
        }

        // ── Poll HID ──────────────────────────────────────────────────────
        if stream.is_device_connected() {
            match stream.poll_once() {
                Ok((poses, _)) if !poses.is_empty() => {
                    let mut s = state.lock().unwrap();
                    s.last_pose_time = Some(Instant::now());
                    for p in &poses {
                        match p.device {
                            DeviceId::LeftController  => s.battery[0] = p.battery,
                            DeviceId::RightController => s.battery[1] = p.battery,
                            DeviceId::Headset         => {}
                        }
                    }
                    // Bucket by controller side (byte 0: 0xa5/0x10=Left, 0xa6/0x11=Right).
                    if let Some(r) = stream.last_raw_report() {
                        match r[0] {
                            0xa5 | 0x10 => s.raw_left  = Some(r),
                            0xa6 | 0x11 => s.raw_right = Some(r),
                            _ => {}
                        }
                    }
                    s.ukf_p[0] = stream.ukf_p_left();
                    s.ukf_p[1] = stream.ukf_p_right();
                    drop(s);
                    last_worker_pose_at = Instant::now();
                }
                Ok(_) => {}
                Err(e) => {
                    state.lock().unwrap().push_log(format!("poll error: {e}"));
                }
            }
        } else {
            thread::sleep(Duration::from_millis(100));
        }
    }
}

// ── GUI ───────────────────────────────────────────────────────────────────────

struct NoloApp {
    state: Arc<Mutex<AppState>>,

    // Local editing buffers — parsed only when "Apply" is clicked
    tcp_listen_enabled:    bool,
    tcp_listen_port:       String,
    ws_listen_enabled:     bool,
    ws_listen_port:        String,
    teleop_left_enabled:   bool,
    teleop_left_addr:      String,
    teleop_right_enabled:  bool,
    teleop_right_addr:     String,
    udp_push_enabled:      bool,
    udp_push_addr:         String,
    gyro_scale_str:        String,

    apply_error: Option<String>,
}

impl NoloApp {
    fn new(state: Arc<Mutex<AppState>>) -> Self {
        let config = state.lock().unwrap().config.clone();
        NoloApp {
            tcp_listen_enabled:    config.tcp_listen_port.is_some(),
            tcp_listen_port:       config.tcp_listen_port.map_or("8123".into(), |p| p.to_string()),
            ws_listen_enabled:     config.ws_listen_port.is_some(),
            ws_listen_port:        config.ws_listen_port.map_or("8765".into(), |p| p.to_string()),
            teleop_left_enabled:   config.teleop_left_to.is_some(),
            teleop_left_addr:      config.teleop_left_to.map_or(String::new(), |a| a.to_string()),
            teleop_right_enabled:  config.teleop_right_to.is_some(),
            teleop_right_addr:     config.teleop_right_to.map_or(String::new(), |a| a.to_string()),
            udp_push_enabled:      config.udp_stream_to.is_some(),
            udp_push_addr:         config.udp_stream_to.map_or(String::new(), |a| a.to_string()),
            gyro_scale_str:        format!("{:.6}", config.gyro_scale),
            apply_error:           None,
            state,
        }
    }

    fn apply_config(&mut self) {
        let mut config = RunConfig::default();
        let mut errors: Vec<String> = Vec::new();

        if self.tcp_listen_enabled {
            match self.tcp_listen_port.trim().parse::<u16>() {
                Ok(port) => config.tcp_listen_port = Some(port),
                Err(_) => errors.push(format!("invalid TCP listen port: {}", self.tcp_listen_port)),
            }
        }
        if self.ws_listen_enabled {
            match self.ws_listen_port.trim().parse::<u16>() {
                Ok(port) => config.ws_listen_port = Some(port),
                Err(_) => errors.push(format!("invalid WS listen port: {}", self.ws_listen_port)),
            }
        }
        if self.teleop_left_enabled {
            match self.teleop_left_addr.trim().parse::<SocketAddr>() {
                Ok(addr) => config.teleop_left_to = Some(addr),
                Err(_) => errors.push(format!("invalid Teleop L addr: {}", self.teleop_left_addr)),
            }
        }
        if self.teleop_right_enabled {
            match self.teleop_right_addr.trim().parse::<SocketAddr>() {
                Ok(addr) => config.teleop_right_to = Some(addr),
                Err(_) => errors.push(format!("invalid Teleop R addr: {}", self.teleop_right_addr)),
            }
        }
        if self.udp_push_enabled {
            match self.udp_push_addr.trim().parse::<SocketAddr>() {
                Ok(addr) => config.udp_stream_to = Some(addr),
                Err(_) => errors.push(format!("invalid UDP push addr: {}", self.udp_push_addr)),
            }
        }
        match self.gyro_scale_str.trim().parse::<f32>() {
            Ok(s) if s > 0.0 => config.gyro_scale = s,
            _ => errors.push(format!("invalid gyro scale: {}", self.gyro_scale_str)),
        }

        if errors.is_empty() {
            let mut s = self.state.lock().unwrap();
            s.config = config;
            s.config_version = s.config_version.wrapping_add(1);
            s.push_log("config applied");
            self.apply_error = None;
        } else {
            self.apply_error = Some(errors.join("; "));
        }
    }
}

impl eframe::App for NoloApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        ui.ctx().clone().request_repaint_after(Duration::from_millis(50));

        let (device_connected, last_pose_time, battery, raw_left, raw_right, ukf_p, log_lines) = {
            let s = self.state.lock().unwrap();
            (
                s.device_connected,
                s.last_pose_time,
                s.battery,
                s.raw_left,
                s.raw_right,
                s.ukf_p,
                s.log.iter().cloned().collect::<Vec<_>>(),
            )
        };

        // ── Top status bar ────────────────────────────────────────────────
        let is_streaming = last_pose_time
            .map(|t| t.elapsed() < Duration::from_millis(200))
            .unwrap_or(false);
        egui::Panel::top("status_bar").show_inside(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading("NoloStream");
                ui.separator();
                let (dot, color, label) = if !device_connected {
                    ("○", egui::Color32::from_rgb(220, 80, 80), "Searching…")
                } else if is_streaming {
                    ("●", egui::Color32::from_rgb(80, 200, 80), "Streaming")
                } else {
                    ("●", egui::Color32::from_rgb(80, 120, 220), "Connected")
                };
                ui.colored_label(color, format!("{dot} {label}"));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let fmt_bat = |v: u8| if v == 255 { "—".to_string() } else { format!("{}%", v) };
                    let base_bat = raw_left.or(raw_right).map(|r| fmt_bat(r[58])).unwrap_or_else(|| "—".to_string());
                    ui.label(format!("L {}  R {}  Base {}", fmt_bat(battery[0]), fmt_bat(battery[1]), base_bat));
                });
            });
        });

        // ── Config side panel ─────────────────────────────────────────────
        egui::Panel::left("config_panel")
            .min_size(310.0)
            .resizable(true)
            .show_inside(ui, |ui| {
                ui.add_space(4.0);
                ui.heading("Transports");
                ui.separator();

                egui::Grid::new("transport_grid")
                    .num_columns(3)
                    .spacing([6.0, 4.0])
                    .show(ui, |ui| {
                        // TCP listen
                        ui.checkbox(&mut self.tcp_listen_enabled, "TCP listen");
                        ui.label("port:");
                        ui.add_enabled(
                            self.tcp_listen_enabled,
                            egui::TextEdit::singleline(&mut self.tcp_listen_port).desired_width(60.0),
                        );
                        ui.end_row();

                        // WS listen
                        ui.checkbox(&mut self.ws_listen_enabled, "WS listen");
                        ui.label("port:");
                        ui.add_enabled(
                            self.ws_listen_enabled,
                            egui::TextEdit::singleline(&mut self.ws_listen_port).desired_width(60.0),
                        );
                        ui.end_row();

                        // Teleop L target
                        ui.checkbox(&mut self.teleop_left_enabled, "Teleop L");
                        ui.label("addr:");
                        ui.add_enabled(
                            self.teleop_left_enabled,
                            egui::TextEdit::singleline(&mut self.teleop_left_addr).desired_width(140.0),
                        );
                        ui.end_row();

                        // Teleop R target
                        ui.checkbox(&mut self.teleop_right_enabled, "Teleop R");
                        ui.label("addr:");
                        ui.add_enabled(
                            self.teleop_right_enabled,
                            egui::TextEdit::singleline(&mut self.teleop_right_addr).desired_width(140.0),
                        );
                        ui.end_row();

                        // UDP push
                        ui.checkbox(&mut self.udp_push_enabled, "UDP push");
                        ui.label("addr:");
                        ui.add_enabled(
                            self.udp_push_enabled,
                            egui::TextEdit::singleline(&mut self.udp_push_addr).desired_width(140.0),
                        );
                        ui.end_row();
                    });

                ui.add_space(8.0);
                ui.heading("Sensor");
                ui.separator();
                ui.horizontal(|ui| {
                    ui.label("Gyro scale:");
                    ui.text_edit_singleline(&mut self.gyro_scale_str);
                });

                ui.add_space(12.0);
                ui.separator();
                if ui.button("  Apply  ").clicked() {
                    self.apply_config();
                }
                if let Some(ref err) = self.apply_error {
                    ui.colored_label(egui::Color32::from_rgb(220, 80, 80), err);
                }

                // ── UKF Uncertainties ────────────────────────────────────────
                ui.add_space(8.0);
                egui::CollapsingHeader::new("UKF Uncertainties")
                    .default_open(true)
                    .show(ui, |ui| {
                        // Helper: colour-code a value by lo/hi thresholds.
                        let sigma_color = |v: f32, lo: f32, hi: f32| -> egui::Color32 {
                            if v >= hi { egui::Color32::from_rgb(220, 80, 80) }
                            else if v >= lo { egui::Color32::from_rgb(220, 180, 60) }
                            else { egui::Color32::from_rgb(80, 200, 80) }
                        };
                        let sigma3 = |p: &[f32; 12], a: usize| -> f32 {
                            ((p[a] + p[a+1] + p[a+2]) / 3.0).sqrt()
                        };
                        let labels = ["L", "R"];
                        egui::Grid::new("ukf_grid")
                            .num_columns(5)
                            .spacing([6.0, 3.0])
                            .show(ui, |ui| {
                                ui.label("");
                                ui.label(egui::RichText::new("σ_orient°").small());
                                ui.label(egui::RichText::new("σ_bias mr/s").small());
                                ui.label(egui::RichText::new("σ_pos mm").small());
                                ui.label(egui::RichText::new("σ_vel m/s").small());
                                ui.end_row();
                                for (i, lbl) in labels.iter().enumerate() {
                                    let p = &ukf_p[i];
                                    let so = sigma3(p, 0) * (180.0 / std::f32::consts::PI);
                                    let sb = sigma3(p, 3) * 1000.0;
                                    let sp = sigma3(p, 6) * 1000.0;
                                    let sv = sigma3(p, 9);
                                    ui.label(*lbl);
                                    ui.colored_label(sigma_color(so, 3.0, 10.0),
                                        egui::RichText::new(format!("{so:5.1}")).monospace().size(11.0));
                                    ui.colored_label(sigma_color(sb, 20.0, 50.0),
                                        egui::RichText::new(format!("{sb:6.1}")).monospace().size(11.0));
                                    ui.colored_label(sigma_color(sp, 5.0, 15.0),
                                        egui::RichText::new(format!("{sp:6.1}")).monospace().size(11.0));
                                    ui.colored_label(sigma_color(sv, 0.2, 0.5),
                                        egui::RichText::new(format!("{sv:5.3}")).monospace().size(11.0));
                                    ui.end_row();
                                }
                            });
                    });

                // ── Unmapped bytes ───────────────────────────────────────────
                ui.add_space(4.0);
                egui::CollapsingHeader::new("Unmapped bytes")
                    .default_open(false)
                    .show(ui, |ui| {
                let any_raw = raw_left.or(raw_right);
                egui::Grid::new("raw_grid")
                    .num_columns(2)
                    .spacing([6.0, 2.0])
                    .show(ui, |ui| {
                        let fmt_bytes = |r: &[u8], range: std::ops::RangeInclusive<usize>| -> String {
                            range.map(|i| format!("{:3}", r[i])).collect::<Vec<_>>().join(" ")
                        };

                        ui.label("L 23–24:");
                        ui.label(egui::RichText::new(
                            raw_left.map(|r| fmt_bytes(&r, 23..=24)).unwrap_or_else(|| "  —".into())
                        ).monospace().size(11.0));
                        ui.end_row();

                        ui.label("R 23–24:");
                        ui.label(egui::RichText::new(
                            raw_right.map(|r| fmt_bytes(&r, 23..=24)).unwrap_or_else(|| "  —".into())
                        ).monospace().size(11.0));
                        ui.end_row();

                        ui.label("31–36:");
                        ui.label(egui::RichText::new(
                            any_raw.map(|r| fmt_bytes(&r, 31..=36)).unwrap_or_else(|| "—".into())
                        ).monospace().size(11.0));
                        ui.end_row();

                        ui.label("43–48:");
                        ui.label(egui::RichText::new(
                            any_raw.map(|r| fmt_bytes(&r, 43..=48)).unwrap_or_else(|| "—".into())
                        ).monospace().size(11.0));
                        ui.end_row();

                        ui.label("57–62:");
                        ui.label(egui::RichText::new(
                            any_raw.map(|r| fmt_bytes(&r, 57..=62)).unwrap_or_else(|| "—".into())
                        ).monospace().size(11.0));
                        ui.end_row();
                    });
                    });
            });

        // ── Log panel (central) ───────────────────────────────────────────
        ui.heading("Log");
        ui.separator();
        egui::ScrollArea::vertical()
            .stick_to_bottom(true)
            .show(ui, |ui| {
                for line in &log_lines {
                    ui.label(egui::RichText::new(line).monospace().size(11.0));
                }
            });
    }
}

// ── Entry point ───────────────────────────────────────────────────────────────

pub fn run_gui(initial_config: RunConfig) {
    let state = Arc::new(Mutex::new(AppState::new(initial_config)));
    spawn_worker(Arc::clone(&state));

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("NoloStream")
            .with_inner_size([750.0, 520.0]),
        ..Default::default()
    };

    let state_for_app = Arc::clone(&state);
    if let Err(e) = eframe::run_native(
        "NoloStream",
        options,
        Box::new(move |_cc| Ok(Box::new(NoloApp::new(state_for_app)))),
    ) {
        eprintln!("GUI error: {e}");
    }
}
