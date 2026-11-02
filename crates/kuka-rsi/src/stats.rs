//! Counters published by the real-time thread and events it emits.

use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Stats {
    pub sessions: u64,
    pub cycles: u64,
    pub replies: u64,
    /// Replies sent later than one cycle after the telegram arrived.
    pub late_replies: u64,
    pub reply_latency_max_us: u64,
    pub reply_latency_mean_us: f64,
    pub interarrival_max_us: u64,
    /// IPOC steps that differed from the step seen between the first two telegrams.
    pub ipoc_gaps: u64,
    /// Older datagrams discarded because a newer one was already queued.
    pub skipped_datagrams: u64,
    pub parse_errors: u64,
    /// Socket errors on the real-time thread (receive or reply).
    pub io_errors: u64,
    pub sentype_mismatches: u64,
    pub position_clamps: u64,
    pub velocity_clamps: u64,
    pub acceleration_clamps: u64,
    pub correction_clamps: u64,
    /// Cycles in which the target queue was empty while playback was running.
    pub starved_cycles: u64,
    /// Every-cycle log entries dropped because the reader did not keep up.
    pub log_drops: u64,
    /// Segments waiting in the interpolator.
    pub queue_len: u32,
    /// Late packets as counted by the robot (`DEF_Delay`).
    pub robot_delay: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ClampKind {
    Position,
    Velocity,
    Acceleration,
    Correction,
}

/// Something worth logging, emitted by the real-time thread without logging itself.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum Event {
    Listening,
    RobotConnected { ipoc: u64 },
    LinkLost,
    LateReply { latency_us: u64 },
    ParseError,
    IoError,
    SentypeMismatch,
    Clamped(ClampKind),
    Starved,
    TrackingError { error_rad: f64 },
    CommandRejected,
    Fault,
    Notified,
    StopPulsed,
    RtPriority { ok: bool },
    Panic,
    LogDropped,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stats_and_events_round_trip_through_json() {
        let s = Stats { cycles: 12, reply_latency_mean_us: 37.5, queue_len: 3, robot_delay: 2, ..Default::default() };
        let json = serde_json::to_string(&s).unwrap();
        assert!(json.contains("\"cycles\":12"), "{json}");
        let back: Stats = serde_json::from_str(&json).unwrap();
        assert_eq!(back, s);

        for e in [
            Event::RobotConnected { ipoc: 4004 },
            Event::LateReply { latency_us: 4500 },
            Event::TrackingError { error_rad: 0.05 },
            Event::RtPriority { ok: false },
            Event::Clamped(ClampKind::Velocity),
            Event::LinkLost,
        ] {
            let json = serde_json::to_string(&e).unwrap();
            let back: Event = serde_json::from_str(&json).unwrap();
            assert_eq!(back, e, "{json}");
        }
    }
}
