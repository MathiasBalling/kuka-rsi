//! Fake KR C5 controller speaking RSI over UDP.

use std::net::SocketAddr;
use std::time::Duration;

use clap::Parser;
use kuka_rsi::config::EthernetConfig;
use kuka_rsi::sim::{ControllerSim, SimConfig};

#[derive(Parser)]
#[command(name = "rsi-sim", about = "Fake KUKA controller: sends <Rob> telegrams, expects <Sen> replies")]
struct Args {
    /// RSI Ethernet config XML (same file the driver reads)
    #[arg(long)]
    config: std::path::PathBuf,
    /// Where to send telegrams; default IP_NUMBER:PORT from the config
    #[arg(long)]
    target: Option<SocketAddr>,
    #[arg(long, default_value_t = 4)]
    cycle_ms: u64,
    /// Late packets tolerated before RSIBad
    #[arg(long, default_value_t = 100)]
    timeout: u32,
    /// First-order lag of the actual position, in cycles
    #[arg(long, default_value_t = 0.0)]
    lag: f64,
    #[arg(long, default_value_t = 0)]
    jitter_us: u64,
    #[arg(long, default_value_t = 0.0)]
    drop: f64,
    /// Start pose A1..A6 in degrees, comma separated
    #[arg(long, default_value = "0,-90,90,0,90,0", value_delimiter = ',')]
    start: Vec<f64>,
    #[arg(long)]
    max_cycles: Option<u64>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let a = Args::parse();
    let eth = EthernetConfig::from_path(&a.config)?;
    let target = match a.target {
        Some(t) => t,
        None => SocketAddr::new(eth.ip_number.parse()?, eth.port),
    };
    let mut cfg = SimConfig::new(eth, target);
    cfg.cycle = Duration::from_millis(a.cycle_ms);
    cfg.timeout = a.timeout;
    cfg.lag_cycles = a.lag;
    cfg.jitter = Duration::from_micros(a.jitter_us);
    cfg.drop_probability = a.drop;
    cfg.max_cycles = a.max_cycles;
    if a.start.len() != 6 {
        return Err("--start needs six values".into());
    }
    cfg.q_start_deg.copy_from_slice(&a.start);
    println!("rsi-sim: sending to {target} every {} ms", a.cycle_ms);
    let exit = ControllerSim::new(cfg)?.run()?;
    println!("rsi-sim: exit {exit:?}");
    Ok(())
}
