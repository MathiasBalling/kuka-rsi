//! Stream a 10 Hz sine on A6 for five seconds, the way a policy would. Usage:
//!   cargo run --example stream_trajectory -- controller/kuka_rsi_ethernet.xml [bind-addr]

use std::time::Duration;

use kuka_rsi::config::{DriverConfig, Limits};
use kuka_rsi::driver::Driver;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let mut args = std::env::args().skip(1);
    let config = args.next().ok_or("usage: stream_trajectory <ethernet.xml> [bind]")?;
    let mut cfg = DriverConfig::from_ethernet_xml(&config)?
        .limits(Limits::kr10_defaults().with_max_velocity_all(20f64.to_radians()))
        .stop_channel("MoveStop");
    if let Some(b) = args.next() {
        cfg = cfg.bind(b.parse()?);
    }
    let drv = Driver::spawn(cfg)?;
    drv.wait_for_robot(Duration::from_secs(60))?;
    let q0 = drv.state().q_setpoint;
    let dt = Duration::from_millis(100);
    for chunk in 0..10u32 {
        let points: Vec<[f64; 6]> = (1..=5)
            .map(|k| {
                let t = (chunk * 5 + k) as f64 * 0.1;
                let mut q = q0;
                q[5] += 3f64.to_radians() * (t * 2.0).sin();
                q
            })
            .collect();
        drv.stream(&points, dt, false)?;
        std::thread::sleep(Duration::from_millis(500));
    }
    drv.wait_idle(Duration::from_secs(5))?;
    println!("{:?}", drv.stats());
    drv.close()?;
    Ok(())
}
