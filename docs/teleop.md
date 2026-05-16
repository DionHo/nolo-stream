# Teleop API

Teleop turns NoloStream into a **robot remote-control input device**.  
The server processes controller pose data and emits compact, frame-to-frame **delta frames** over any transport.  
A robot receives these deltas and applies them to its end-effector pose.

---

## Coordinate system

Teleop data uses the **robotics convention**:

| Axis | Direction |
|------|-----------|
| X    | Calibrated left-to-right (see yaw calibration below) |
| Y    | Horizontal, orthogonal to X (forward/backward after calibration) |
| Z    | Vertical, up is positive |

This is a **right-handed, Z-up** coordinate system.

Raw tracker coordinates are Y-up (Y = up, Z = forward/backward, X = right).  
The teleop module performs the conversion automatically:

```
robot_x = tracker_x
robot_y = −tracker_z
robot_z =  tracker_y
```

For orientation quaternions the axis vector is transformed the same way:
`q_robot = [w, qx, −qz, qy]` from the tracker quaternion `[w, qx, qy, qz]`.

---

## Yaw calibration

**Trigger**: press the **Menu button** (bit `0x04`) on either controller.  
Both controllers must be visible at the moment of the press.

**Effect**: the horizontal vector from the **left controller to the right controller** becomes the **+X axis** in all subsequent teleop output.

This lets you calibrate the robot's forward/right directions to your physical setup without modifying the tracker position.  
If calibration has never been performed, X aligns with the raw tracker X axis.

---

## Delta streaming

**Trigger**: hold the **Touchpad click** (bit `0x01`) on a controller.

While held, every poll cycle produces a `TeleopFrame` for that controller.  
On the first frame when the click begins, no delta is emitted (no prior reference exists).  
Frames resume immediately from zero on the next click.

### `TeleopFrame` JSON fields

```json
{
  "device": "left_controller",
  "delta_position":    [0.001, 0.000, -0.002],
  "delta_orientation": [0.9999, 0.001, 0.000, -0.001],
  "timestamp_ms": 1715702400123
}
```

| Field | Type | Description |
|-------|------|-------------|
| `device` | string | `"left_controller"` or `"right_controller"` |
| `delta_position` | `[x,y,z]` f32, metres | Frame-to-frame position change in robotics Z-up coordinates |
| `delta_orientation` | `[w,x,y,z]` f32, unit quaternion | Frame-to-frame rotation in robotics Z-up coordinates (Hamilton convention) |
| `timestamp_ms` | u64 | Host time of the **current** frame, ms since UNIX epoch |

Position deltas are typically very small (sub-millimetre at controller tracking rates of ~60–120 Hz).

---

## Wire format

Teleop frames are sent as a separate JSON message on **every transport** (TCP, UDP, WebSocket):

```
{"teleop":[{...frame...},{...frame...}]}\n
```

A single poll cycle can produce at most two frames (one per controller).

Receivers can distinguish teleop messages from pose messages by the top-level JSON shape:
- **Array** → pose batch `[{pose}, ...]`
- **Object with `"teleop"` key** → teleop delta batch

---

## Robot-side application

The robot should maintain an end-effector pose `(position, orientation)` and apply each delta as a **global** left-multiplication:

```python
# position: simply add the delta
ee_pos += delta_position

# orientation (global rotation, left-multiply convention):
ee_orientation = delta_orientation * ee_orientation
ee_orientation = normalize(ee_orientation)
```

The delta quaternion represents a rotation expressed in the **robot's world frame** (Z-up, calibrated X).  
Left-multiplying keeps the rotation axes in the world frame, which is the typical teleop convention.

---

## Rust library API

```rust
use nolostream::{TeleopFrame, TeleopState};

let mut state = TeleopState::new();

// In your poll loop:
let (poses, teleop_frames) = stream.poll_once()?;

// teleop_frames are also automatically dispatched to all transports.
// You can inspect them locally:
for frame in &teleop_frames {
    println!("{:?}", frame);
}

// Check calibration status:
if state.is_calibrated() { ... }
```

`TeleopState` is embedded inside `NoloStream` and updated automatically.  
`poll_once` returns the frames so callers can inspect or log them; the dispatch to transports has already happened.

For the **client-api** path (`--client-api` flag), a standalone `TeleopState` is maintained in the server binary and dispatched the same way.

---

## Miniviz

Two wireframe target boxes appear in the 3D scene:

- **Green (L)** — left controller teleop target
- **Yellow (R)** — right controller teleop target

When you hold the touchpad and move the controller, the corresponding target box moves and rotates accordingly.  
The coordinate conversion from Z-up (teleop) back to Y-up (Babylon.js scene) is:

```
viz_x =  robot_x
viz_y =  robot_z   (robot Z-up → viz Y-up)
viz_z = −robot_y
```

Use the **RESET** button in the control panel to return both targets to the origin.

The overlay shows `TELEOP: calibrating...` when the Menu button is held, and `TELEOP: active (device)` while a touchpad is clicked.
