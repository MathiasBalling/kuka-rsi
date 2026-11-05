//! Move A6 by +DEG degrees over SECS seconds and back, printing the measured A6 angle at the
//! start, at the far end and back home. Usage:
//!   cargo run --example jog -- controller/kuka_rsi_ethernet.xml [--deg 2] [--secs 2] [--bind ADDR]

use std::time::Duration;

use kuka_rsi::config::{DriverConfig, Limits};
use kuka_rsi::driver::Driver;

const USAGE: &str = "usage: jog <ethernet.xml> [--deg 2] [--secs 2] [--bind ADDR]";

/// Set by the Ctrl-C handler; the handler then closes the driver and exits with 130.
static INTERRUPTED: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let result = run();
    if INTERRUPTED.load(std::sync::atomic::Ordering::SeqCst) {
        // The session ending under Ctrl-C makes `run` fail (link lost); let the handler finish
        // the clean shutdown and exit instead of reporting that as an error.
        loop {
            std::thread::park();
        }
    }
    result
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let mut args = std::env::args().skip(1);
    let config = args.next().ok_or(USAGE)?;
    let (mut deg, mut secs, mut bind) = (2.0f64, 2.0f64, None);
    while let Some(flag) = args.next() {
        let value = args.next().ok_or(USAGE)?;
        match flag.as_str() {
            "--deg" => deg = value.parse()?,
            "--secs" => secs = value.parse()?,
            "--bind" => bind = Some(value.parse()?),
            _ => return Err(USAGE.into()),
        }
    }
    if !(deg.abs() <= 45.0 && secs >= 1.0) {
        return Err("keep --deg within ±45 and --secs at least 1".into());
    }

    let mut cfg = DriverConfig::from_ethernet_xml(&config)?
        .limits(Limits::kr10_defaults().with_max_velocity_all(10f64.to_radians()))
        .stop_channel("MoveStop");
    if let Some(b) = bind {
        cfg = cfg.bind(b);
    }
    let drv = std::sync::Arc::new(Driver::spawn(cfg)?);
    // Ctrl-C ends the robot program cleanly: close() pulses MoveStop and waits for the session
    // to end. Without this the robot stops on its own after the ETHERNET timeout, with a message.
    let on_ctrl_c = std::sync::Arc::clone(&drv);
    ctrlc::set_handler(move || {
        INTERRUPTED.store(true, std::sync::atomic::Ordering::SeqCst);
        eprintln!("Ctrl-C: ending the robot program (MoveStop) and closing");
        let _ = on_ctrl_c.close();
        std::process::exit(130);
    })?;
    drv.wait_for_robot(Duration::from_secs(60))?;
    // Playback follows the pendant override (10 % stretches 2 s to 20 s), so wait generously.
    let wait = Duration::from_secs_f64(secs * 10.0 + 30.0);
    let a6 = |d: &Driver| d.state().q[5].to_degrees();

    let q0 = drv.state().q_setpoint;
    println!("A6 start {:9.3} deg", a6(&drv));
    let mut q1 = q0;
    q1[5] += deg.to_radians();
    drv.move_to(q1, Duration::from_secs_f64(secs))?;
    drv.wait_idle(wait)?;
    std::thread::sleep(Duration::from_millis(300));
    println!("A6 far   {:9.3} deg (target {:+.1})", a6(&drv), deg);
    drv.move_to(q0, Duration::from_secs_f64(secs))?;
    drv.wait_idle(wait)?;
    std::thread::sleep(Duration::from_millis(300));
    println!("A6 back  {:9.3} deg", a6(&drv));
    println!("{:?}", drv.stats());
    drv.close()?;
    Ok(())
}
