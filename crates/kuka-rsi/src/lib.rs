//! KUKA.RobotSensorInterface 5.0 client.
//!
//! See `docs/design.md`.

pub mod config;
pub mod control;
pub mod error;
pub mod link;
pub mod state;
pub mod stats;
pub mod telegram;

pub use error::{Error, Result};
