use serde::Deserialize;

/// Commands sent from a client (e.g. miniviz) back to the server over WebSocket.
#[derive(Debug, Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
pub enum Command {
    /// Trigger a haptic pulse on a controller.
    /// `device`: "left_controller" or "right_controller"
    /// `intensity`: 50–100
    Haptic { device: String, intensity: u8 },

    /// Set the HMD tracking centre offset (metres).
    SetHmdCenter { x: f32, y: f32, z: f32 },

    /// Toggle ceiling-mount mode.
    CeilingMode { enabled: bool },

    /// Forward a raw JSON UI command to NoloServer.
    UiCommand { content: String },
}
