use std::ffi::{c_char, c_void, CString};
use std::ptr;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::time::{Duration, Instant, SystemTime};

use libloading::Library;

use crate::{DeviceId, Pose};

// ── FFI structs ───────────────────────────────────────────────────────────────

#[repr(C)]
#[derive(Copy, Clone)]
struct NVector2 { x: f32, y: f32 }

#[repr(C)]
#[derive(Copy, Clone)]
struct NVector3 { x: f32, y: f32, z: f32 }

#[repr(C)]
#[derive(Copy, Clone)]
struct NQuaternion { x: f32, y: f32, z: f32, w: f32 }

#[repr(C)]
#[derive(Copy, Clone)]
struct ControllerData {
    version_id: i32,
    position:   NVector3,
    rotation:   NQuaternion,
    buttons:    u32,
    touched:    i32,
    touch_axis: NVector2,
    battery:    i32,
    state:      i32,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct HmdData {
    hmd_version_id:            i32,
    hmd_position:              NVector3,
    hmd_init_position:         NVector3,
    hmd_two_point_drift_angle: u32,
    hmd_rotation:              NQuaternion,
    hmd_state:                 i32,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct BaseStationData { version_id: i32, power: i32 }

#[repr(C)]
#[derive(Copy, Clone)]
struct SensorData {
    l_velocity: NVector3, l_angular_velocity: NVector3,
    r_velocity: NVector3, r_angular_velocity: NVector3,
    h_velocity: NVector3, h_angular_velocity: NVector3,
}

// Full NOLOData with #pragma pack(push,1).
// packed required: two u8 fields at offsets 308/309 precede NVector3 (align 4)
// → without pack, 2 bytes of padding would make the struct 324 bytes instead of 322.
#[repr(C, packed)]
#[derive(Copy, Clone)]
pub struct NoloDataRaw {
    left_data:          ControllerData,
    right_data:         ControllerData,
    hmd_data:           HmdData,
    bs_data:            BaseStationData,
    expand_data:        [u8; 64],
    sensor_data:        SensorData,
    left_pack_number:   u8,
    right_pack_number:  u8,
    fixed_eye_position: NVector3,
}

const _: () = assert!(std::mem::size_of::<ControllerData>() == 56);
const _: () = assert!(std::mem::size_of::<HmdData>()        == 52);
const _: () = assert!(std::mem::size_of::<SensorData>()     == 72);
const _: () = assert!(std::mem::size_of::<NoloDataRaw>()    == 322);

// ── Global callback state ─────────────────────────────────────────────────────

static NOLO_BYTES: Mutex<[u8; 322]> = Mutex::new([0u8; 322]);
static DATA_READY: AtomicBool        = AtomicBool::new(false);
static CONNECTED:  AtomicBool        = AtomicBool::new(false);

unsafe extern "C" fn on_zmq_connected() {
    eprintln!("[client-api] ZMQ connected");
    CONNECTED.store(true, Ordering::Release);
}

unsafe extern "C" fn on_zmq_disconnected() {
    eprintln!("[client-api] ZMQ disconnected");
    CONNECTED.store(false, Ordering::Release);
    DATA_READY.store(false, Ordering::Release);
}

// pfnDataCallBack: void(__cdecl*)(const NOLOData&) — C++ reference = pointer in ABI.
unsafe extern "C" fn on_new_data(data: *const NoloDataRaw) {
    if data.is_null() { return; }
    let src = std::slice::from_raw_parts(data as *const u8, 322);
    if let Ok(mut guard) = NOLO_BYTES.lock() {
        guard.copy_from_slice(src);
        DATA_READY.store(true, Ordering::Release);
    }
}

// ── Function pointer types ────────────────────────────────────────────────────

type FnOpenZmq            = unsafe extern "C" fn() -> bool;
type FnCloseZmq           = unsafe extern "C" fn();
type FnRegisterCallback   = unsafe extern "C" fn(callback_type: i32, fn_ptr: *mut c_void);
type FnTriggerHapticPulse = unsafe extern "C" fn(device_type: i32, intensity: i32);
type FnSetHmdCenter       = unsafe extern "C" fn(center: *const NVector3);
type FnSetBCellingMode    = unsafe extern "C" fn(ceiling_mode: bool);
type FnSendUICommand      = unsafe extern "C" fn(cmd: *const c_char);

const CB_ZMQ_CONNECTED:    i32 = 0;
const CB_ZMQ_DISCONNECTED: i32 = 1;
const CB_NEW_DATA:         i32 = 5;

// ENoloDeviceType values
const DEV_LEFT:  i32 = 1; // eLeftController
const DEV_RIGHT: i32 = 2; // eRightController

// ── Public API ────────────────────────────────────────────────────────────────

pub struct NoloClientApi {
    // All fn ptrs have no destructors; _lib must be last so the DLL stays loaded in Drop.
    close_zmq:        FnCloseZmq,
    haptic_pulse:     FnTriggerHapticPulse,
    set_hmd_center:   FnSetHmdCenter,
    set_ceiling_mode: FnSetBCellingMode,
    send_ui_command:  FnSendUICommand,
    _lib: Library,
}

impl NoloClientApi {
    pub fn open() -> Result<Self, Box<dyn std::error::Error>> {
        let dll_path = std::env::current_exe()?
            .parent()
            .ok_or("no parent dir")?
            .join("NoloClientLib.dll");

        let lib = unsafe { Library::new(&dll_path) }
            .map_err(|e| format!("failed to load {:?}: {}", dll_path, e))?;

        macro_rules! sym {
            ($name:literal, $ty:ty) => {
                unsafe {
                    *lib.get::<$ty>($name)
                        .map_err(|e| format!("{}: {}", stringify!($name), e))?
                }
            };
        }

        let open_zmq:         FnOpenZmq            = sym!(b"OpenNoloZeroMQ\0",      FnOpenZmq);
        let close_zmq:        FnCloseZmq           = sym!(b"CloseNoloZeroMQ\0",     FnCloseZmq);
        let register_callback:FnRegisterCallback   = sym!(b"RegisterCallBack\0",    FnRegisterCallback);
        let haptic_pulse:     FnTriggerHapticPulse = sym!(b"TriggerHapticPulse\0",  FnTriggerHapticPulse);
        let set_hmd_center:   FnSetHmdCenter       = sym!(b"SetHmdCenter\0",         FnSetHmdCenter);
        let set_ceiling_mode: FnSetBCellingMode    = sym!(b"SetBCellingMode\0",      FnSetBCellingMode);
        let send_ui_command:  FnSendUICommand      = sym!(b"SendUIComand\0",         FnSendUICommand); // note: SDK typo

        CONNECTED.store(false, Ordering::SeqCst);
        DATA_READY.store(false, Ordering::SeqCst);

        // Register callbacks BEFORE opening ZMQ (mirrors driver sample order).
        unsafe {
            register_callback(CB_ZMQ_CONNECTED,    on_zmq_connected    as *mut c_void);
            register_callback(CB_ZMQ_DISCONNECTED, on_zmq_disconnected as *mut c_void);
            register_callback(CB_NEW_DATA,         on_new_data         as *mut c_void);
        }

        let ok = unsafe { open_zmq() };
        if !ok {
            return Err("OpenNoloZeroMQ returned false -- is NoloServer running?".into());
        }

        // Wait up to 3 s for the async on_zmq_connected callback.
        let deadline = Instant::now() + Duration::from_secs(3);
        while Instant::now() < deadline {
            if CONNECTED.load(Ordering::Acquire) { break; }
            std::thread::sleep(Duration::from_millis(50));
        }
        if !CONNECTED.load(Ordering::Acquire) {
            eprintln!("[client-api] warning: on_zmq_connected did not fire within 3 s");
        }

        Ok(NoloClientApi { close_zmq, haptic_pulse, set_hmd_center, set_ceiling_mode, send_ui_command, _lib: lib })
    }

    /// Returns the latest snapshot if at least one `on_new_data` callback has fired.
    pub fn get_data(&self) -> Option<NoloDataRaw> {
        if !DATA_READY.load(Ordering::Acquire) {
            return None;
        }
        let guard = NOLO_BYTES.lock().ok()?;
        Some(unsafe { std::mem::transmute_copy::<[u8; 322], NoloDataRaw>(&*guard) })
    }

    /// Trigger a haptic pulse on a controller. `device`: "left_controller" or "right_controller".
    /// `intensity`: 50–100 (clamped).
    pub fn haptic_pulse(&self, device: &str, intensity: u8) {
        let dev_type = match device {
            "left_controller"  => DEV_LEFT,
            "right_controller" => DEV_RIGHT,
            _ => return,
        };
        let intensity = (intensity as i32).clamp(50, 100);
        unsafe { (self.haptic_pulse)(dev_type, intensity) };
    }

    /// Set the HMD tracking centre offset (metres).
    pub fn set_hmd_center(&self, x: f32, y: f32, z: f32) {
        let v = NVector3 { x, y, z };
        unsafe { (self.set_hmd_center)(&v as *const NVector3) };
    }

    /// Toggle ceiling-mount mode.
    pub fn ceiling_mode(&self, enabled: bool) {
        unsafe { (self.set_ceiling_mode)(enabled) };
    }

    /// Send a raw JSON UI command string to NoloServer.
    pub fn send_ui_command(&self, content: &str) {
        if let Ok(cs) = CString::new(content) {
            unsafe { (self.send_ui_command)(cs.as_ptr()) };
        }
    }
}

impl Drop for NoloClientApi {
    fn drop(&mut self) {
        unsafe { (self.close_zmq)() };
    }
}

// ── Data conversion ───────────────────────────────────────────────────────────

fn nv3(v: NVector3) -> [f32; 3] { [v.x, v.y, v.z] }

pub fn nolo_data_to_poses(data: &NoloDataRaw) -> Vec<Pose> {
    let ts = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);

    let left:   ControllerData = unsafe { ptr::read_unaligned(ptr::addr_of!(data.left_data)) };
    let right:  ControllerData = unsafe { ptr::read_unaligned(ptr::addr_of!(data.right_data)) };
    let hmd:    HmdData        = unsafe { ptr::read_unaligned(ptr::addr_of!(data.hmd_data)) };
    let sensor: SensorData     = unsafe { ptr::read_unaligned(ptr::addr_of!(data.sensor_data)) };

    let mut left_pose  = controller_to_pose(&left,  DeviceId::LeftController,  ts);
    let mut right_pose = controller_to_pose(&right, DeviceId::RightController, ts);
    let mut hmd_pose   = hmd_to_pose(&hmd, ts);

    left_pose.velocity         = nv3(sensor.l_velocity);
    left_pose.angular_velocity = nv3(sensor.l_angular_velocity);
    left_pose.state            = left.state;

    right_pose.velocity         = nv3(sensor.r_velocity);
    right_pose.angular_velocity = nv3(sensor.r_angular_velocity);
    right_pose.state            = right.state;

    hmd_pose.velocity         = nv3(sensor.h_velocity);
    hmd_pose.angular_velocity = nv3(sensor.h_angular_velocity);
    hmd_pose.state            = hmd.hmd_state;

    vec![left_pose, right_pose, hmd_pose]
}

fn controller_to_pose(c: &ControllerData, device: DeviceId, ts: u64) -> Pose {
    let has_touch = (c.buttons & 0x20) != 0; // ePadTouch
    let (touch_x, touch_y) = if has_touch {
        (
            (c.touch_axis.x * 127.0 + 127.0).clamp(0.0, 254.0) as u8,
            (c.touch_axis.y * 127.0 + 127.0).clamp(0.0, 254.0) as u8,
        )
    } else {
        (255, 255)
    };
    Pose {
        device,
        position:     [c.position.x, c.position.y, c.position.z],
        orientation:  [c.rotation.w, c.rotation.x, c.rotation.y, c.rotation.z],
        timestamp_ms: ts,
        sensor_raw:   [0; 31],
        touch_x,
        touch_y,
        battery:      c.battery.clamp(0, 255) as u8,
        buttons:      c.buttons,
        velocity:         [0.0; 3],
        angular_velocity: [0.0; 3],
        state:            0,
    }
}

fn hmd_to_pose(h: &HmdData, ts: u64) -> Pose {
    Pose {
        device:           DeviceId::Headset,
        position:         [h.hmd_position.x, h.hmd_position.y, h.hmd_position.z],
        orientation:      [h.hmd_rotation.w, h.hmd_rotation.x, h.hmd_rotation.y, h.hmd_rotation.z],
        timestamp_ms:     ts,
        sensor_raw:       [0; 31],
        touch_x:          255,
        touch_y:          255,
        battery:          0,
        buttons:          0,
        velocity:         [0.0; 3],
        angular_velocity: [0.0; 3],
        state:            0,
    }
}
