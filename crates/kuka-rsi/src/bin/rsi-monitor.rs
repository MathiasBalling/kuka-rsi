//! Print robot state and driver statistics. Holds position unless the config is ONLYSEND.

use std::net::SocketAddr;
use std::time::{Duration, Instant};

use clap::Parser;
use kuka_rsi::config::DriverConfig;
use kuka_rsi::driver::Driver;
use kuka_rsi::state::rad2deg6;

#[derive(Parser)]
#[command(name = "rsi-monitor", about = "Connect to the robot (or rsi-sim) and print state and stats")]
struct Args {
    /// RSI Ethernet config XML
    #[arg(long)]
    config: std::path::PathBuf,
    /// Bind address override; default IP_NUMBER:PORT from the config
    #[arg(long)]
    bind: Option<SocketAddr>,
    /// Seconds to run, 0 = until the link is lost
    #[arg(long, default_value_t = 0.0)]
    seconds: f64,
    #[arg(long, default_value_t = 60.0)]
    wait: f64,
    #[arg(long, default_value = "MoveStop")]
    stop_channel: String,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let a = Args::parse();
    let mut cfg = DriverConfig::from_ethernet_xml(&a.config)?;
    if let Some(b) = a.bind {
        cfg = cfg.bind(b);
    }
    if cfg.ethernet.receive.iter().any(|e| e.tag == a.stop_channel) {
        cfg = cfg.stop_channel(&a.stop_channel);
    }
    let drv = Driver::spawn(cfg)?;
    println!("listening on {} for the first telegram (up to {} s)", drv.local_addr(), a.wait);
    drv.wait_for_robot(Duration::from_secs_f64(a.wait))?;
    println!("robot connected, start pose {:?} deg", rad2deg6(&drv.state().q_setpoint));
    let t0 = Instant::now();
    let mut last_stats = Instant::now();
    loop {
        std::thread::sleep(Duration::from_millis(200));
        let st = drv.state();
        println!(
            "t={:8.3} ipoc={} q(deg)={:?} delay={} ov={:.0}% running={}",
            st.t,
            st.ipoc,
            rad2deg6(&st.q).map(|v| (v * 1000.0).round() / 1000.0),
            st.delay,
            st.speed_override * 100.0,
            st.program_running
        );
        if last_stats.elapsed() >= Duration::from_secs(1) {
            println!("{:?}", drv.stats());
            for e in drv.events() {
                println!("event: {e:?}");
            }
            last_stats = Instant::now();
        }
        if drv.session() != kuka_rsi::state::SessionState::Active {
            println!("link lost");
            break;
        }
        if a.seconds > 0.0 && t0.elapsed().as_secs_f64() >= a.seconds {
            break;
        }
    }
    drv.close()?;
    Ok(())
}
