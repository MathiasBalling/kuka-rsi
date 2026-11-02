//! Commands callers push to the real-time thread. Fixed-size, no heap.

use arrayvec::{ArrayString, ArrayVec};

use crate::config::Limits;

/// Points per `Stream` command; the API splits longer chunks.
pub const MAX_CHUNK: usize = 64;

#[derive(Clone, Debug, PartialEq)]
pub enum Command {
    /// Absolute joint targets (rad), each `cycles_per_point` cycles (may be fractional) after the
    /// previous. `batch_remaining` is the number of points in the later chunks of the same
    /// `Driver::stream` call (0 for the last chunk), so the call is accepted or rejected whole.
    /// `first` marks the first chunk of a call: it clears a rejection left by an earlier call
    /// whose later chunks never arrived.
    Stream { points: ArrayVec<[f64; 6], MAX_CHUNK>, cycles_per_point: f64, replace: bool, batch_remaining: u32, first: bool },
    /// Quintic profile from the current command to `q` (rad) over `cycles` (may be fractional).
    MoveTo { q: [f64; 6], cycles: f64 },
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
