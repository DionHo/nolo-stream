# Teleop API

Teleop turns NoloStream into a **robot remote-control input device**.  
The server processes controller pose data and emits compact, frame-to-frame **delta frames** over any transport.  
A *TeleopTarget* (a robot or miniviz acting as one) initiates the handover, provides a reference pose, and then follows the controller deltas.

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

**Trigger**: hold the **Trigger button** (bit `0x02`) on a controller **while handover is active**.

While held, every poll cycle produces a `TeleopFrame` for that controller.  
On the first frame when the trigger begins, no delta is emitted (no prior reference exists).  
Frames resume immediately from zero on the next trigger press.

### `TeleopFrame` JSON fields

```json
{
  "type": "relative",
  "device": "left_controller",
  "pose_mm-deg": [1.0, 0.0, -2.0, 0.1, 0.0, -0.2],
  "timestamp_ms": 1715702400123
}
```

| Field | Type | Description |
|-------|------|-------------|
| `type` | string | Always `"relative"` |
| `device` | string | `"left_controller"` or `"right_controller"` |
| `pose_mm-deg` | `[x, y, z, roll, pitch, yaw]` f32 | Frame-to-frame position delta in **mm** (Z-up) + orientation delta as **ZYX extrinsic Euler angles** in degrees |
| `timestamp_ms` | u64 | Host time of the **current** frame, ms since UNIX epoch |

**Euler convention**: ZYX extrinsic (KUKA A/B/C order) — `R = Rz(yaw) × Ry(pitch) × Rx(roll)`.

Position deltas are in millimetres; at 60–120 Hz a typical delta is ≪ 1 mm.

## Handshake protocol

```mermaid
sequenceDiagram
    participant T as TeleopTarget
    participant N as NoloStream

    T->>N: {type:"handover",state:"active"}
    Note over N: WaitingForReferencePose
    T->>N: {type:"relative","pose_mm-deg":[x,y,z,r,p,y]} (current pose)
    Note over N: captures first message as reference pose
    N-->>T: {type:"handover",state:"active","pose_mm-deg":[x,y,z,r,p,y]}
    Note over N: Active

    loop While trigger held (≥50 Hz)
        N-->>T: {type:"relative","device":"...","pose_mm-deg":[dx,dy,dz,dr,dp,dy]}
    end

    Note over N: SYS button (0x08) pressed
    N->>T: {type:"handover",state:"completed"}
    Note over N: Idle
```

### State machine

| State | Description |
|-------|-------------|
| `Idle` | Default. No teleop output. Waiting for `{type:"handover",state:"active"}` from TeleopTarget. |
| `WaitingForReferencePose` | Active signal received. Waiting for first `{type:"relative",...}` message from TeleopTarget to capture the reference pose. |
| `Active` | Reference pose captured. Forwards controller delta frames when trigger is held. SYS button transitions to Idle. |

### Messages

**TeleopTarget → NoloStream**

| Message | Description |
|---------|-------------|
| `{"type":"handover","state":"active"}` | Start handover; NoloStream expects a reference pose next |
| `{"type":"relative","pose_mm-deg":[x,y,z,r,p,y]}` | TeleopTarget's current pose; first one is captured as reference |

**NoloStream → TeleopTarget**

| Message | Description |
|---------|-------------|
| `{"type":"handover","state":"active","pose_mm-deg":[...]}` | Confirms handover active; echoes reference pose back to TeleopTarget |
| `{"type":"relative","device":"...","pose_mm-deg":[dx,dy,dz,dr,dp,dy]}` | Delta frame while trigger held |
| `{"type":"handover","state":"completed"}` | Handover ended (SYS button pressed) |

---

## Wire format

All teleop and handover messages are sent as individual JSON objects on **every transport** (TCP, UDP, WebSocket), one per line (TCP) or per datagram (UDP):

```
{"type":"relative","device":"left_controller","pose_mm-deg":[1.0,0.0,-2.0,0.1,0.0,-0.2],"timestamp_ms":123456}\n
{"type":"handover","state":"active","pose_mm-deg":[0,0,0,0,0,0]}\n
```

Receivers distinguish message types by the `type` field:
- **`"relative"`** → delta frame
- **`"handover"`** → handover state change
- **Array** → pose batch `[{pose}, ...]`

---

## Robot-side application

The robot should maintain an end-effector pose `(position_mm, [roll, pitch, yaw])` and accumulate each delta:

```python
# position (mm): add deltas directly
ee_pos_mm[0] += pose_mm_deg[0]  # x
ee_pos_mm[1] += pose_mm_deg[1]  # y
ee_pos_mm[2] += pose_mm_deg[2]  # z

# orientation: accumulate Euler angles (ZYX extrinsic)
ee_rpy_deg[0] += pose_mm_deg[3]  # roll
ee_rpy_deg[1] += pose_mm_deg[4]  # pitch
ee_rpy_deg[2] += pose_mm_deg[5]  # yaw
```

Alternatively convert to a quaternion and apply as left-multiplication in the world frame.

---

## Rust library API

```rust
use nolostream::{TeleopFrame, TeleopState, TeleopTargetMsg, TeleopUpdate};

// In your poll loop:
let (poses, teleop_frames) = stream.poll_once()?;
// Teleop frames and handover messages are automatically dispatched to all transports.
```

For a custom integration, construct the state machine directly:

```rust
let mut teleop = TeleopState::new();
let teleop_target_msgs: Vec<TeleopTargetMsg> = /* from transport.recv_teleop_target_msgs() */;
let update: TeleopUpdate = teleop.update(&poses, &teleop_target_msgs);
// update.frames  — delta frames to forward
// update.handover_out — optional handover notification to broadcast
```

`TeleopState` is embedded inside `NoloStream` and updated automatically.  
`poll_once` returns the frames so callers can inspect or log them; the dispatch to transports has already happened.

For the **client-api** path (`--client-api` flag), a standalone `TeleopState` is maintained in the server binary and dispatched the same way.

---

## Miniviz

Two wireframe target boxes appear in the 3D scene:

- **Green (L)** — left controller teleop target
- **Yellow (R)** — right controller teleop target

When you hold the trigger and move the controller (handover active), the corresponding target box moves and rotates accordingly.  
The coordinate conversion from Z-up (teleop) back to Y-up (Babylon.js scene) is:

```
viz_x =  robot_x
viz_y =  robot_z   (robot Z-up → viz Y-up)
viz_z = −robot_y
```

Euler angles are converted to a quaternion via `rpyDegToQuat(roll, pitch, yaw)` (ZYX extrinsic) before applying to the mesh.

Use the **RESET** button to return both targets to the origin.  
Use the **START HANDOVER** button (sends `{type:"handover",state:"active"}` followed by the current target mesh pose) to initiate the handover flow from the browser.

The overlay shows:
- `TELEOP: handover active` — handover confirmed by server
- `TELEOP: active (device)` — delta frames received
- `TELEOP: handover completed` — SYS button pressed, handover ended
