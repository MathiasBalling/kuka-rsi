//! KUKA.RobotSensorInterface 5.0 client.
//!
//! See `docs/design.md`.

pub mod config;
pub mod control;
pub mod controller;
pub mod driver;
pub mod error;
pub mod link;
pub mod sim;
pub mod state;
pub mod stats;
pub mod telegram;

pub use driver::Driver;
pub use error::{Error, Result};
