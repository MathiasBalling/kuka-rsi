//! Check the PC side of an RSI setup, and with `--wait` the live link to the robot.

use std::net::{IpAddr, SocketAddr};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use clap::Parser;
use kuka_rsi::doctor::{self, Options, Progress, Status};

#[derive(Parser)]
#[command(
    name = "rsi-doctor",
    about = "Check the config, network, priority and (with --wait) the live link to the robot",
    after_help = "With --wait, start kuka_rsi (or kuka_rsi_monitor) on the pendant when asked. The doctor \
                  holds the robot where it stands for --hold seconds, then ends the program (MoveStop)."
)]
struct Args {
    /// RSI Ethernet config XML, the same file as on the robot; default: the built-in
    /// kuka_rsi_ethernet.xml
    #[arg(long)]
    config: Option<std::path::PathBuf>,
    /// The robot's RSI address (KLI sensor interface), for the route and ping checks
    #[arg(long)]
    robot: Option<IpAddr>,
    /// Bind address override; default IP_NUMBER:PORT from the config
    #[arg(long)]
    bind: Option<SocketAddr>,
    /// Seconds to wait for the robot's first telegram; 0 skips the live checks
    #[arg(long, default_value_t = 0.0)]
    wait: f64,
    /// Seconds to hold the robot while measuring the timing
    #[arg(long, default_value_t = 3.0)]
    hold: f64,
    #[arg(long, default_value = "MoveStop")]
    stop_channel: String,
    /// The robot's RSI cycle in ms: 4 (#IPO_FAST) or 12 (#IPO, RSI_ON(#ABSOLUTE, #IPO, #BASE))
    #[arg(long, default_value_t = 4, value_parser = kuka_rsi::config::parse_cycle_ms)]
    cycle_ms: u64,
}

fn main() -> std::process::ExitCode {
    let a = Args::parse();
    let cancel = Arc::new(AtomicBool::new(false));
    let on_ctrl_c = cancel.clone();
    // Ctrl-C ends the live check early; the doctor still ends the robot program cleanly.
    if let Err(e) = ctrlc::set_handler(move || on_ctrl_c.store(true, Ordering::SeqCst)) {
        eprintln!("cannot install the Ctrl-C handler: {e}");
    }
    let mut opts = Options { config: a.config, ..Options::default() };
    opts.bind = a.bind;
    opts.robot = a.robot;
    opts.wait = Duration::from_secs_f64(a.wait.max(0.0));
    opts.hold = Duration::from_secs_f64(a.hold.max(0.5));
    opts.stop_channel = a.stop_channel;
    opts.cycle = Duration::from_millis(a.cycle_ms);
    opts.cancel = Some(cancel.clone());

    let checks = doctor::run(&opts, |p| match p {
        Progress::Listening { addr, wait } => {
            println!(
                "        listening on {addr}: start the program on the pendant now (waiting {} s)",
                wait.as_secs()
            );
        }
        Progress::Check(c) => {
            println!("  {}  {:<10}  {}", c.status.label(), c.name, c.detail);
            if let Some(h) = &c.hint {
                print_wrapped(h, "        ", 100);
            }
        }
    });
    let count = |s| checks.iter().filter(|c| c.status == s).count();
    let (fail, warn) = (count(Status::Fail), count(Status::Warn));
    println!("\n{fail} failed, {warn} warnings");
    if cancel.load(Ordering::SeqCst) {
        std::process::ExitCode::from(130)
    } else if fail > 0 {
        std::process::ExitCode::FAILURE
    } else {
        std::process::ExitCode::SUCCESS
    }
}

fn print_wrapped(text: &str, indent: &str, width: usize) {
    let mut line = String::new();
    for word in text.split_whitespace() {
        if !line.is_empty() && indent.len() + line.len() + 1 + word.len() > width {
            println!("{indent}{line}");
            line.clear();
        }
        if !line.is_empty() {
            line.push(' ');
        }
        line.push_str(word);
    }
    if !line.is_empty() {
        println!("{indent}{line}");
    }
}
