//! `maturin` adds the macOS `-undefined dynamic_lookup` linker args itself when it builds
//! the wheel, but a plain `cargo build`/`cargo test` of this crate (e.g. for CI's
//! warning check) does not go through maturin, so pyo3's own build script no longer
//! adds them either. Add them here so `cargo build -p kuka-rsi-py` links on its own.
fn main() {
    pyo3_build_config::add_extension_module_link_args();
}
