//! Turning caller targets into per-cycle joint commands: interpolation, limits, session.

pub mod interpolator;
pub mod limiter;

pub use interpolator::{Blend, Interpolator, Segment, MAX_QUEUE};
pub use limiter::{ClampCounts, Limiter};
