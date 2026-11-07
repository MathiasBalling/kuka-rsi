//! Print robot state and driver statistics. Holds position unless the config is ONLYSEND.

use std::net::SocketAddr;
use std::time::{Duration, Instant};

use clap::Parser;
use kuka_rsi::config::{DriverConfig, EthernetConfig};
use kuka_rsi::driver::Driver;
use kuka_rsi::state::rad2deg6;

#[derive(Parser)]
#[command(name = "rsi-monitor", about = "Connect to the robot (or rsi-sim) and print state and stats")]
struct Args {
    /// RSI Ethernet config XML; default: the built-in kuka_rsi_ethernet.xml (use
    /// kuka_rsi_monitor.xml with the kuka_rsi_monitor program)
    #[arg(long)]
    config: Option<std::path::PathBuf>,
    /// Bind address override; default IP_NUMBER:PORT from the config
    #[arg(long)]
    bind: Option<SocketAddr>,
    /// Seconds to run, 0 = until the link is lost (or Ctrl-C with --keep)
    #[arg(long, default_value_t = 0.0)]
    seconds: f64,
    /// Keep running after the link is lost and answer the next session (a restarted program)
    #[arg(long)]
    keep: bool,
    #[arg(long, default_value_t = 60.0)]
    wait: f64,
    #[arg(long, default_value = "MoveStop")]
    stop_channel: String,
    /// Write every cycle to this CSV file (see kuka_rsi::record)
    #[arg(long)]
    log: Option<std::path::PathBuf>,
}

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
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let a = Args::parse();
    let mut cfg = DriverConfig::new(EthernetConfig::from_path_or_shipped(a.config.as_deref())?);
    if let Some(b) = a.bind {
        cfg = cfg.bind(b);
    }
    // An ONLYSEND config never answers, so a stop pulse would never reach the robot and
    // `close()` would wait a second for a session end that does not come.
    if !cfg.ethernet.onlysend && cfg.ethernet.receive.iter().any(|e| e.tag == a.stop_channel) {
        cfg = cfg.stop_channel(&a.stop_channel);
    }
    println!(
        "config: {}{}",
        a.config.as_ref().map_or("built-in kuka_rsi_ethernet.xml".into(), |p| p.display().to_string()),
        if cfg.ethernet.onlysend { " (ONLYSEND: the robot only sends)" } else { "" }
    );
    let mut csv = a.log.as_ref().map(kuka_rsi::record::CsvLog::create).transpose()?;
    let mut states = Vec::new();
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
    println!("listening on {} for the first telegram (up to {} s)", drv.local_addr(), a.wait);
    drv.wait_for_robot(Duration::from_secs_f64(a.wait))?;
    println!("robot connected, start pose {:?} deg", rad2deg6(&drv.state().q_setpoint));
    let t0 = Instant::now();
    let mut last_stats = Instant::now();
    loop {
        std::thread::sleep(Duration::from_millis(200));
        if let Some(c) = &mut csv {
            states.clear();
            drv.drain_log(&mut states);
            c.write(&states)?;
        }
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
            if !a.keep {
                println!("link lost");
                break;
            }
            println!("link lost; waiting for the next session (restart the program on the pendant)");
            while drv.session() != kuka_rsi::state::SessionState::Active {
                std::thread::sleep(Duration::from_millis(100));
                if a.seconds > 0.0 && t0.elapsed().as_secs_f64() >= a.seconds {
                    break;
                }
            }
            if drv.session() == kuka_rsi::state::SessionState::Active {
                println!(
                    "session {} connected, start pose {:?} deg",
                    drv.stats().sessions,
                    rad2deg6(&drv.state().q_setpoint)
                );
            }
        }
        if a.seconds > 0.0 && t0.elapsed().as_secs_f64() >= a.seconds {
            break;
        }
    }
    if let Some(c) = &mut csv {
        states.clear();
        drv.drain_log(&mut states);
        c.write(&states)?;
        println!(
            "logged {} cycles to {}",
            c.rows(),
            a.log.as_ref().map(|p| p.display().to_string()).unwrap_or_default()
        );
    }
    drv.close()?;
    Ok(())
}
