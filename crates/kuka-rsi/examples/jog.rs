//! Move A6 by +2 degrees over two seconds and back. Usage:
//!   cargo run --example jog -- controller/kuka_rsi_ethernet.xml [bind-addr]

use std::time::Duration;

use kuka_rsi::config::{DriverConfig, Limits};
use kuka_rsi::driver::Driver;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let mut args = std::env::args().skip(1);
    let config = args.next().ok_or("usage: jog <ethernet.xml> [bind]")?;
    let mut cfg = DriverConfig::from_ethernet_xml(&config)?
        .limits(Limits::kr10_defaults().with_max_velocity_all(10f64.to_radians()))
        .stop_channel("MoveStop");
    if let Some(b) = args.next() {
        cfg = cfg.bind(b.parse()?);
    }
    let drv = Driver::spawn(cfg)?;
    drv.wait_for_robot(Duration::from_secs(60))?;
    let q0 = drv.state().q_setpoint;
    let mut q1 = q0;
    q1[5] += 2f64.to_radians();
    drv.move_to(q1, Duration::from_secs(2))?;
    drv.wait_idle(Duration::from_secs(5))?;
    drv.move_to(q0, Duration::from_secs(2))?;
    drv.wait_idle(Duration::from_secs(5))?;
    println!("{:?}", drv.stats());
    drv.close()?;
    Ok(())
}
