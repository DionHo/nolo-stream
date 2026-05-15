use std::ffi::c_void;
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
// packed is required because the two u8 fields at offsets 308/309 are followed by
// NVector3 (align 4) — without pack natural alignment would add 2 padding bytes.
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
// The DLL calls on_new_data from its ZMQ receive thread.
// We store raw bytes in a Mutex so the polling thread can read them safely.

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

type FnOpenZmq          = unsafe extern "C" fn() -> bool;
type FnCloseZmq         = unsafe extern "C" fn();
type FnRegisterCallback = unsafe extern "C" fn(callback_type: i32, fn_ptr: *mut c_void);

const CB_ZMQ_CONNECTED:    i32 = 0; // eOnZMQConnected
const CB_ZMQ_DISCONNECTED: i32 = 1; // eOnZMQDisConnected
const CB_NEW_DATA:         i32 = 5; // eOnNewData

// ── Public API ────────────────────────────────────────────────────────────────

pub struct NoloClientApi {
    // close_zmq is a raw fn ptr (Copy, no destructor) — must be listed before _lib
    // so Drop can call it while _lib is still alive.
    close_zmq: FnCloseZmq,
    _lib: Library,
}

impl NoloClientApi {
    /// Load `NoloClientLib.dll` from the directory of the running executable,
    /// register data callbacks, and open the ZMQ connection to NoloServer.
    pub fn open() -> Result<Self, Box<dyn std::error::Error>> {
        let dll_path = std::env::current_exe()?
            .parent()
            .ok_or("no parent dir")?
            .join("NoloClientLib.dll");

        let lib = unsafe { Library::new(&dll_path) }
            .map_err(|e| format!("failed to load {:?}: {}", dll_path, e))?;

        // Load symbols before moving `lib` into the struct.
        let open_zmq: FnOpenZmq = unsafe {
            *lib.get::<FnOpenZmq>(b"OpenNoloZeroMQ\0")
                .map_err(|e| format!("OpenNoloZeroMQ: {}", e))?
        };
        let close_zmq: FnCloseZmq = unsafe {
            *lib.get::<FnCloseZmq>(b"CloseNoloZeroMQ\0")
                .map_err(|e| format!("CloseNoloZeroMQ: {}", e))?
        };
        let register_callback: FnRegisterCallback = unsafe {
            *lib.get::<FnRegisterCallback>(b"RegisterCallBack\0")
                .map_err(|e| format!("RegisterCallBack: {}", e))?
        };

        // Reset stale state from a previous run.
        CONNECTED.store(false, Ordering::SeqCst);
        DATA_READY.store(false, Ordering::SeqCst);

        // Register callbacks BEFORE opening ZMQ (mirrors the driver sample order).
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

        Ok(NoloClientApi { close_zmq, _lib: lib })
    }

    /// Returns the latest snapshot if at least one `on_new_data` callback has fired.
    pub fn get_data(&self) -> Option<NoloDataRaw> {
        if !DATA_READY.load(Ordering::Acquire) {
            return None;
        }
        let guard = NOLO_BYTES.lock().ok()?;
        // Safety: NoloDataRaw is repr(C, packed) and has the same size as [u8; 322].
        Some(unsafe { std::mem::transmute_copy::<[u8; 322], NoloDataRaw>(&*guard) })
    }
}

impl Drop for NoloClientApi {
    fn drop(&mut self) {
        // _lib is still loaded at this point (Drop runs before fields are dropped).
        unsafe { (self.close_zmq)() };
    }
}

// ── Data conversion ───────────────────────────────────────────────────────────

pub fn nolo_data_to_poses(data: &NoloDataRaw) -> Vec<Pose> {
    let ts = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .map(|d| d.as_millis() as u64)
        .unwrap_or(0);

    // Use read_unaligned to safely copy sub-structs from the packed parent.
    let left:  ControllerData = unsafe { ptr::read_unaligned(ptr::addr_of!(data.left_data)) };
    let right: ControllerData = unsafe { ptr::read_unaligned(ptr::addr_of!(data.right_data)) };
    let hmd:   HmdData        = unsafe { ptr::read_unaligned(ptr::addr_of!(data.hmd_data)) };

    vec![
        controller_to_pose(&left,  DeviceId::LeftController,  ts),
        controller_to_pose(&right, DeviceId::RightController, ts),
        hmd_to_pose(&hmd, ts),
    ]
}

fn controller_to_pose(c: &ControllerData, device: DeviceId, ts: u64) -> Pose {
    let has_touch = (c.buttons & 0x20) != 0; // ePadTouch = 0x20
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
        sensor_raw:   [0; 19],
        touch_x,
        touch_y,
        battery:      c.battery.clamp(0, 255) as u8,
    }
}

fn hmd_to_pose(h: &HmdData, ts: u64) -> Pose {
    Pose {
        device:       DeviceId::Headset,
        position:     [h.hmd_position.x, h.hmd_position.y, h.hmd_position.z],
        orientation:  [h.hmd_rotation.w, h.hmd_rotation.x, h.hmd_rotation.y, h.hmd_rotation.z],
        timestamp_ms: ts,
        sensor_raw:   [0; 19],
        touch_x:      255,
        touch_y:      255,
        battery:      0,
    }
}
