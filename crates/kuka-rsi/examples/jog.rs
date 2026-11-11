//! Move one or more axes from where the robot stands and back, and report how well it followed.
//! Usage:
//!   cargo run --example jog -- [controller/kuka_rsi_ethernet.xml] [--axis 6] [--deg 2]
//!       [--move A1=30,A3=-15] [--speed 10] [--secs S] [--repeat 1] [--log run.csv] [--bind ADDR]
//!       [--cycle-ms 4]
//!
//! `--axis`/`--deg` move one axis (default A6 by 2°); `--move` moves several at once. Offsets
//! are capped at 30° (A1 to A5) and 45° (A6), the joint speed at 30°/s. Each leg (out and back)
//! prints the measured angle against the target, the largest lag between command and actual
//! position per moving axis, late replies, clamps and the robot's `Delay`. `--log` writes every
//! cycle to a CSV file (see `kuka_rsi::record`). `--cycle-ms 12` is for a robot program that runs
//! the 12 ms sensor mode, `RSI_ON(#ABSOLUTE, #IPO, #BASE)`; the default 4 ms is `#IPO_FAST`.

use std::time::{Duration, Instant};

use kuka_rsi::Error;
use kuka_rsi::config::{DriverConfig, EthernetConfig, Limits};
use kuka_rsi::driver::Driver;
use kuka_rsi::record::CsvLog;
use kuka_rsi::state::RobotState;
use kuka_rsi::stats::Stats;

const USAGE: &str = "usage: jog [ethernet.xml] [--axis N] [--deg D] [--move A1=30,A3=-15] [--speed 10] [--secs S] \
                     [--repeat 1] [--log FILE] [--bind ADDR] [--cycle-ms 4]";

/// Largest offset per axis, degrees: what the cell allows around the usual pose.
const CAP_DEG: [f64; 6] = [30.0, 30.0, 30.0, 30.0, 30.0, 45.0];
const MAX_SPEED_DEG_S: f64 = 30.0;

#[derive(Debug, PartialEq)]
struct Plan {
    offsets_deg: [f64; 6],
    speed_deg_s: f64,
    secs: f64,
    repeat: u32,
    log: Option<String>,
    bind: Option<String>,
    cycle_ms: u64,
}

/// Parse `A1=30,A3=-15` into per-axis offsets in degrees.
fn parse_move(spec: &str) -> Result<[f64; 6], String> {
    let mut out = [0.0; 6];
    for part in spec.split(',').map(str::trim).filter(|p| !p.is_empty()) {
        let (axis, deg) = part.split_once('=').ok_or(format!("expected AXIS=DEG, got {part:?}"))?;
        let i = axis_index(axis.trim().trim_start_matches(['A', 'a']))?;
        out[i] = deg.trim().parse().map_err(|_| format!("bad angle in {part:?}"))?;
    }
    Ok(out)
}

fn axis_index(s: &str) -> Result<usize, String> {
    match s.parse::<usize>() {
        Ok(n @ 1..=6) => Ok(n - 1),
        _ => Err(format!("axis must be 1 to 6, got {s:?}")),
    }
}

/// Duration whose quintic profile peaks at `speed` (peak = 1.875 · distance / duration).
fn default_secs(offsets_deg: &[f64; 6], speed_deg_s: f64) -> f64 {
    let largest = offsets_deg.iter().fold(0.0f64, |m, d| m.max(d.abs()));
    (1.875 * largest / speed_deg_s).max(1.0)
}

fn parse_args(args: &[String]) -> Result<(Option<String>, Plan), String> {
    let mut it = args.iter().peekable();
    // The config file is optional (default: the built-in kuka_rsi_ethernet.xml).
    let config = it.next_if(|a| !a.starts_with("--")).cloned();
    let (mut axis, mut deg, mut moves) = (None, None, None);
    let (mut speed, mut secs, mut repeat, mut log, mut bind) = (10.0, None, 1u32, None, None);
    let mut cycle_ms = 4;
    while let Some(flag) = it.next() {
        let value = it.next().ok_or(USAGE)?;
        let num = |v: &str| v.parse::<f64>().map_err(|_| format!("{flag}: not a number: {v}"));
        match flag.as_str() {
            "--axis" => axis = Some(axis_index(value)?),
            "--deg" => deg = Some(num(value)?),
            "--move" => moves = Some(parse_move(value)?),
            "--speed" => speed = num(value)?,
            "--secs" => secs = Some(num(value)?),
            "--repeat" => repeat = value.parse().map_err(|_| format!("--repeat: not a count: {value}"))?,
            "--log" => log = Some(value.clone()),
            "--bind" => bind = Some(value.clone()),
            "--cycle-ms" => cycle_ms = kuka_rsi::config::parse_cycle_ms(value)?,
            _ => return Err(USAGE.into()),
        }
    }
    let offsets_deg = match (moves, axis, deg) {
        (Some(_), Some(_), _) | (Some(_), _, Some(_)) => return Err("use either --move or --axis/--deg".into()),
        (Some(m), None, None) => m,
        (None, axis, deg) => {
            let mut o = [0.0; 6];
            o[axis.unwrap_or(5)] = deg.unwrap_or(2.0);
            o
        }
    };
    for (i, (d, cap)) in offsets_deg.iter().zip(CAP_DEG).enumerate() {
        if !d.is_finite() || d.abs() > cap {
            return Err(format!("A{}: {d}° is beyond the ±{cap}° cap", i + 1));
        }
    }
    if offsets_deg.iter().all(|d| *d == 0.0) {
        return Err("nothing to move".into());
    }
    if !(1.0..=MAX_SPEED_DEG_S).contains(&speed) {
        return Err(format!("--speed must be 1 to {MAX_SPEED_DEG_S} °/s"));
    }
    let secs = secs.unwrap_or_else(|| default_secs(&offsets_deg, speed));
    if !secs.is_finite() || secs < 1.0 {
        return Err("--secs must be at least 1".into());
    }
    if repeat == 0 {
        return Err("--repeat must be at least 1".into());
    }
    Ok((config, Plan { offsets_deg, speed_deg_s: speed, secs, repeat, log, bind, cycle_ms }))
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

/// Move the per-cycle log into `leg` and append it to the CSV.
fn drain(drv: &Driver, leg: &mut Vec<RobotState>, csv: &mut Option<CsvLog>) -> Result<(), Error> {
    let start = leg.len();
    drv.drain_log(leg);
    if let Some(c) = csv {
        c.write(&leg[start..])?;
    }
    Ok(())
}

/// Wait for the move to finish, draining the log meanwhile, then let the robot settle.
fn follow(drv: &Driver, timeout: Duration, leg: &mut Vec<RobotState>, csv: &mut Option<CsvLog>) -> Result<(), Error> {
    let deadline = Instant::now() + timeout;
    loop {
        match drv.wait_idle(Duration::from_millis(100)) {
            Ok(()) => break,
            Err(Error::IdleTimeout) if Instant::now() < deadline => drain(drv, leg, csv)?,
            Err(e) => return Err(e),
        }
    }
    std::thread::sleep(Duration::from_millis(300));
    drain(drv, leg, csv)
}

fn clamps(before: &Stats, after: &Stats) -> String {
    format!(
        "clamps: {} position, {} velocity, {} acceleration, {} correction",
        after.position_clamps - before.position_clamps,
        after.velocity_clamps - before.velocity_clamps,
        after.acceleration_clamps - before.acceleration_clamps,
        after.correction_clamps - before.correction_clamps
    )
}

fn report(name: &str, target: &[f64; 6], moving: &[usize], leg: &[RobotState], before: &Stats, drv: &Driver) {
    let q = drv.state().q;
    let after = drv.stats();
    let delay_max = leg.iter().map(|s| s.delay).max().unwrap_or(0);
    let secs = match (leg.first(), leg.last()) {
        (Some(a), Some(b)) => b.t - a.t,
        _ => 0.0,
    };
    println!("{name} ({secs:.1} s):");
    for &i in moving {
        let lag = leg.iter().map(|s| (s.q_command[i] - s.q[i]).abs()).fold(0.0, f64::max);
        println!(
            "  A{}  target {:9.3}  measured {:9.3}  error {:+.3}  max lag {:.3} deg",
            i + 1,
            target[i].to_degrees(),
            q[i].to_degrees(),
            (q[i] - target[i]).to_degrees(),
            lag.to_degrees()
        );
    }
    println!(
        "  {} cycles, {} late replies, {}, robot Delay max {}",
        leg.len(),
        after.late_replies - before.late_replies,
        clamps(before, &after),
        delay_max
    );
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (config, plan) = parse_args(&args)?;

    let eth = EthernetConfig::from_path_or_shipped(config.as_deref().map(std::path::Path::new))?;
    println!("config: {}", config.as_deref().unwrap_or("built-in kuka_rsi_ethernet.xml"));
    if plan.cycle_ms != 4 {
        println!("cycle: {} ms (the robot program must run the same sensor mode)", plan.cycle_ms);
    }
    let mut cfg = DriverConfig::new(eth)
        .limits(Limits::kr10_defaults().with_max_velocity_all(plan.speed_deg_s.to_radians()))
        .stop_channel("MoveStop")
        .cycle(Duration::from_millis(plan.cycle_ms));
    if let Some(b) = &plan.bind {
        cfg = cfg.bind(b.parse()?);
    }
    let mut csv = plan.log.as_ref().map(CsvLog::create).transpose()?;
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
    println!("listening on {}: start the program on the pendant", drv.local_addr());
    drv.wait_for_robot(Duration::from_secs(60))?;

    let q0 = drv.state().q_setpoint;
    let mut q1 = q0;
    for (q, d) in q1.iter_mut().zip(plan.offsets_deg) {
        *q += d.to_radians();
    }
    drv.limits().check_position(&q1)?;
    let moving: Vec<usize> = (0..6).filter(|&i| plan.offsets_deg[i] != 0.0).collect();
    let start: Vec<String> =
        moving.iter().map(|&i| format!("A{} {:.3}", i + 1, drv.state().q[i].to_degrees())).collect();
    println!("start (deg): {}", start.join("  "));
    // Playback follows the pendant override (10 % stretches 2 s to 20 s), so wait generously.
    let wait = Duration::from_secs_f64(plan.secs * 10.0 + 30.0);
    let mut leg = Vec::new();
    drain(&drv, &mut leg, &mut csv)?;
    for round in 1..=plan.repeat {
        for (name, target) in [("out", q1), ("back", q0)] {
            leg.clear();
            let before = drv.stats();
            drv.move_to(target, Duration::from_secs_f64(plan.secs))?;
            follow(&drv, wait, &mut leg, &mut csv)?;
            report(&format!("{round}/{} {name}", plan.repeat), &target, &moving, &leg, &before, &drv);
        }
    }
    println!("{:?}", drv.stats());
    if let (Some(c), Some(path)) = (&csv, &plan.log) {
        println!("logged {} cycles to {path}", c.rows());
    }
    drv.close()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(s: &str) -> Vec<String> {
        std::iter::once("cfg.xml").chain(s.split_whitespace()).map(String::from).collect()
    }

    #[test]
    fn defaults_move_a6_by_two_degrees_at_ten_degrees_per_second() {
        let (config, p) = parse_args(&args("")).unwrap();
        assert_eq!(config.as_deref(), Some("cfg.xml"));
        assert_eq!(p.offsets_deg, [0.0, 0.0, 0.0, 0.0, 0.0, 2.0]);
        assert_eq!((p.speed_deg_s, p.secs, p.repeat), (10.0, 1.0, 1));
    }

    #[test]
    fn the_config_file_is_optional() {
        let flags: Vec<String> = ["--axis", "1", "--deg", "30"].map(String::from).into();
        let (config, p) = parse_args(&flags).unwrap();
        assert_eq!(config, None);
        assert_eq!(p.offsets_deg[0], 30.0);
        assert_eq!(parse_args(&[]).unwrap().0, None);
    }

    #[test]
    fn the_cycle_defaults_to_4_ms() {
        assert_eq!(parse_args(&args("")).unwrap().1.cycle_ms, 4);
        assert_eq!(parse_args(&args("--cycle-ms 12")).unwrap().1.cycle_ms, 12);
        assert!(parse_args(&args("--cycle-ms 0")).is_err());
        assert!(parse_args(&args("--cycle-ms 4.5")).is_err());
    }

    #[test]
    fn deg_alone_keeps_meaning_a6() {
        let (_, p) = parse_args(&args("--deg 45 --secs 10")).unwrap();
        assert_eq!(p.offsets_deg[5], 45.0);
        assert_eq!(p.secs, 10.0);
    }

    #[test]
    fn move_spec_sets_several_axes() {
        let (_, p) = parse_args(&args("--move A1=30,a3=-15,6=45 --speed 20 --repeat 3 --log x.csv")).unwrap();
        assert_eq!(p.offsets_deg, [30.0, 0.0, -15.0, 0.0, 0.0, 45.0]);
        // The largest offset sets the duration so the quintic profile peaks at the speed limit.
        assert!((p.secs - 1.875 * 45.0 / 20.0).abs() < 1e-12);
        assert_eq!((p.repeat, p.log.as_deref()), (3, Some("x.csv")));
    }

    #[test]
    fn caps_hold_per_axis() {
        assert!(parse_args(&args("--axis 1 --deg 30")).is_ok());
        assert!(parse_args(&args("--axis 1 --deg -30.5")).unwrap_err().contains("A1"));
        assert!(parse_args(&args("--move A2=31")).unwrap_err().contains("A2"));
        assert!(parse_args(&args("--deg 45")).is_ok());
        assert!(parse_args(&args("--deg 46")).is_err());
        assert!(parse_args(&args("--deg NaN")).is_err());
        assert!(parse_args(&args("--speed 31")).is_err());
        assert!(parse_args(&args("--speed 0.5")).is_err());
        assert!(parse_args(&args("--secs 0.5")).is_err());
        assert!(parse_args(&args("--repeat 0")).is_err());
    }

    #[test]
    fn bad_specs_are_rejected() {
        assert!(parse_args(&args("--axis 7")).is_err());
        assert!(parse_args(&args("--move A1:30")).is_err());
        assert!(parse_args(&args("--move A1=x")).is_err());
        assert!(parse_args(&args("--move A1=0")).unwrap_err().contains("nothing"));
        assert!(parse_args(&args("--move A1=5 --axis 2")).is_err());
        assert!(parse_args(&args("--bogus 1")).is_err());
    }
}
