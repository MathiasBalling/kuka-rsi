//! KUKA.RobotSensorInterface 5.0 client.
//!
//! See `docs/design.md`.

pub mod config;
pub mod error;
pub mod telegram;

#[cfg(test)]
pub(crate) mod test_alloc;

pub use error::{Error, Result};
