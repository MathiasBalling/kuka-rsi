//! Turning caller targets into per-cycle joint commands: interpolation, limits, session.

pub mod limiter;

pub use limiter::{ClampCounts, Limiter};
