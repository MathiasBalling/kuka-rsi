//! Commands callers push to the real-time thread. Fixed-size, no heap.

use arrayvec::{ArrayString, ArrayVec};

use crate::config::Limits;

/// Points per `Stream` command; the API splits longer chunks.
pub const MAX_CHUNK: usize = 64;

#[derive(Clone, Debug, PartialEq)]
pub enum Command {
    /// Absolute joint targets (rad), each `cycles_per_point` cycles after the previous.
    Stream { points: ArrayVec<[f64; 6], MAX_CHUNK>, cycles_per_point: u32, replace: bool },
    /// Quintic profile from the current command to `q` (rad) over `cycles`.
    MoveTo { q: [f64; 6], cycles: u32 },
    /// Drop the queue and stay where the command is now.
    Hold,
    /// Send `Error: <msg>` to the robot (stops it) and freeze the command.
    Fault(ArrayString<56>),
    /// Send a notification message to the smartHMI.
    Notify(ArrayString<56>),
    /// Pulse the stop channel so `RSI_MOVECORR()` ends.
    StopProgram,
    /// Playback speed factor, 0.0 to 1.0.
    SetSpeedScale(f64),
    SetLimits(Limits),
}
