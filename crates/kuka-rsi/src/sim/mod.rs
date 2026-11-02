//! A fake KR C5 controller that speaks the RSI Ethernet protocol, for tests and demos.

pub mod controller;
pub mod wire;

pub use controller::{ControllerSim, SimConfig, SimExit, SimHandle, SimStatus};
pub use wire::{parse_sen, write_rob};
