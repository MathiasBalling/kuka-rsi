//! A fake KR C5 controller that speaks the RSI Ethernet protocol, for tests and demos.

pub mod wire;

pub use wire::{parse_sen, write_rob};
