//! Stream a multi-axis sine the way a policy would: every 100 ms the next 0.5 s of the path as
//! five points, replacing what was queued. Usage:
//!   cargo run --example stream_trajectory -- [controller/kuka_rsi_ethernet.xml] [--axes 6] [--amp 3]
//!       [--period 4] [--secs 8] [--stop-after S] [--speed 20] [--log run.csv] [--bind ADDR]
//!
//! Each axis in `--axes` (comma-separated, e.g. `1,3,6`) swings ±`--amp` degrees around the
//! start pose with period `--period`, phases spread across the axes. The swing fades in over the
//! first period and out over the last, so the path starts and ends at rest. `--stop-after` stops
//! sending mid-path, the way a crashed policy would: the robot plays out the last chunk, holds,
//! and the report shows how far it moved after the stop. Then it returns to the start pose.
//! Amplitudes are capped like `jog` (30°, A6 45°), and the path's peak speed must stay within
//! `--speed` (at most 30°/s).

use std::f64::consts::TAU;
use std::time::{Duration, Instant};

use kuka_rsi::Error;
use kuka_rsi::config::{DriverConfig, EthernetConfig, Limits};
use kuka_rsi::driver::Driver;
use kuka_rsi::record::CsvLog;
use kuka_rsi::state::RobotState;
use kuka_rsi::stats::Stats;

const USAGE: &str = "usage: stream_trajectory [ethernet.xml] [--axes 6] [--amp 3] [--period 4] [--secs 8] \
                     [--stop-after S] [--speed 20] [--log FILE] [--bind ADDR]";
const CAP_DEG: [f64; 6] = [30.0, 30.0, 30.0, 30.0, 30.0, 45.0];
const MAX_SPEED_DEG_S: f64 = 30.0;
/// Chunk period and horizon of the emulated policy.
const CHUNK_EVERY: Duration = Duration::from_millis(100);
const POINTS: usize = 5;

#[derive(Debug, PartialEq)]
struct Plan {
    axes: Vec<usize>,
    amp_deg: f64,
    period: f64,
    secs: f64,
    stop_after: Option<f64>,
    speed_deg_s: f64,
    log: Option<String>,
    bind: Option<String>,
}

impl Plan {
    /// Offset from the start pose at time `t`, degrees.
    fn offset_deg(&self, t: f64) -> [f64; 6] {
        let mut o = [0.0; 6];
        let fade = smoothstep(t.min(self.secs - t) / self.period);
        for (k, &i) in self.axes.iter().enumerate() {
            let phase = TAU * k as f64 / self.axes.len() as f64;
            o[i] = self.amp_deg * fade * (TAU * t / self.period + phase).sin();
        }
        o
    }
}

/// 0 below 0, 1 above 1, smooth in between with zero slope at both ends (peak slope 1.5).
fn smoothstep(x: f64) -> f64 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

fn parse_args(args: &[String]) -> Result<(Option<String>, Plan), String> {
    let mut it = args.iter().peekable();
    // The config file is optional (default: the built-in kuka_rsi_ethernet.xml).
    let config = it.next_if(|a| !a.starts_with("--")).cloned();
    let mut p = Plan {
        axes: vec![5],
        amp_deg: 3.0,
        period: 4.0,
        secs: 8.0,
        stop_after: None,
        speed_deg_s: 20.0,
        log: None,
        bind: None,
    };
    while let Some(flag) = it.next() {
        let value = it.next().ok_or(USAGE)?;
        let num = |v: &str| v.parse::<f64>().map_err(|_| format!("{flag}: not a number: {v}"));
        match flag.as_str() {
            "--axes" => {
                p.axes = value
                    .split(',')
                    .map(|a| match a.trim().trim_start_matches(['A', 'a']).parse::<usize>() {
                        Ok(n @ 1..=6) => Ok(n - 1),
                        _ => Err(format!("axis must be 1 to 6, got {a:?}")),
                    })
                    .collect::<Result<_, _>>()?;
                p.axes.sort_unstable();
                p.axes.dedup();
            }
            "--amp" => p.amp_deg = num(value)?,
            "--period" => p.period = num(value)?,
            "--secs" => p.secs = num(value)?,
            "--stop-after" => p.stop_after = Some(num(value)?),
            "--speed" => p.speed_deg_s = num(value)?,
            "--log" => p.log = Some(value.clone()),
            "--bind" => p.bind = Some(value.clone()),
            _ => return Err(USAGE.into()),
        }
    }
    for &i in &p.axes {
        if !p.amp_deg.is_finite() || p.amp_deg <= 0.0 || p.amp_deg > CAP_DEG[i] {
            return Err(format!("--amp must be above 0 and at most {}° for A{}", CAP_DEG[i], i + 1));
        }
    }
    if !(1.0..=MAX_SPEED_DEG_S).contains(&p.speed_deg_s) {
        return Err(format!("--speed must be 1 to {MAX_SPEED_DEG_S} °/s"));
    }
    if !(p.period.is_finite() && p.secs.is_finite()) || p.period < 1.0 || p.secs < 2.0 * p.period || p.secs > 600.0 {
        return Err("need --period of at least 1 s and --secs of at least two periods, at most 600 s".into());
    }
    // Peak speed of amp · fade · sin: amp · (2π + 1.5) / period at most.
    let peak = p.amp_deg * (TAU + 1.5) / p.period;
    if peak > p.speed_deg_s {
        return Err(format!(
            "the path peaks at {peak:.1}°/s, above --speed {}: lower --amp or raise --period",
            p.speed_deg_s
        ));
    }
    if let Some(s) = p.stop_after {
        if !(s > 0.0 && s < p.secs) {
            return Err("--stop-after must be within --secs".into());
        }
    }
    Ok((config, p))
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

fn drain(drv: &Driver, log: &mut Vec<RobotState>, csv: &mut Option<CsvLog>) -> Result<(), Error> {
    let start = log.len();
    drv.drain_log(log);
    if let Some(c) = csv {
        c.write(&log[start..])?;
    }
    Ok(())
}

/// Clamp counts since `before`, by kind. Acceleration clamps are expected while streaming: the
/// points are joined by straight segments, and the limiter rounds the corners between them.
fn clamps(before: &Stats, after: &Stats) -> String {
    format!(
        "clamps: {} position, {} velocity, {} acceleration, {} correction",
        after.position_clamps - before.position_clamps,
        after.velocity_clamps - before.velocity_clamps,
        after.acceleration_clamps - before.acceleration_clamps,
        after.correction_clamps - before.correction_clamps
    )
}

fn run() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (config, plan) = parse_args(&args)?;
    let eth = EthernetConfig::from_path_or_shipped(config.as_deref().map(std::path::Path::new))?;
    println!("config: {}", config.as_deref().unwrap_or("built-in kuka_rsi_ethernet.xml"));
    let mut cfg = DriverConfig::new(eth)
        .limits(Limits::kr10_defaults().with_max_velocity_all(plan.speed_deg_s.to_radians()))
        .stop_channel("MoveStop");
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
    let at = |t: f64| {
        let mut q = q0;
        for (q, o) in q.iter_mut().zip(plan.offset_deg(t)) {
            *q += o.to_radians();
        }
        q
    };
    let mut log = Vec::new();
    drain(&drv, &mut log, &mut csv)?;
    log.clear();
    let before = drv.stats();
    let end = plan.stop_after.unwrap_or(plan.secs);
    println!("streaming A{:?} ±{}° for {end:.1} s", plan.axes.iter().map(|i| i + 1).collect::<Vec<_>>(), plan.amp_deg);
    let t0 = Instant::now();
    let mut next = t0;
    let mut last_target = q0;
    while t0.elapsed().as_secs_f64() < end {
        let t = t0.elapsed().as_secs_f64();
        // Points `CHUNK_EVERY` apart: the first one is due when the next chunk replaces this one.
        let dt = CHUNK_EVERY.as_secs_f64();
        let points: Vec<[f64; 6]> = (1..=POINTS).map(|k| at((t + k as f64 * dt).min(plan.secs))).collect();
        drv.stream(&points, CHUNK_EVERY, true)?;
        last_target = points[POINTS - 1];
        drain(&drv, &mut log, &mut csv)?;
        next += CHUNK_EVERY;
        std::thread::sleep(next.saturating_duration_since(Instant::now()));
    }
    let streamed = log.len();
    // The robot plays out the last chunk (0.5 s of path) and holds; watch it for a second.
    std::thread::sleep(Duration::from_secs(1));
    drain(&drv, &mut log, &mut csv)?;
    let after = drv.stats();
    let q_held = drv.state().q;

    println!("stream ({} cycles):", streamed);
    for &i in &plan.axes {
        let lag = log[..streamed].iter().map(|s| (s.q_command[i] - s.q[i]).abs()).fold(0.0, f64::max);
        println!(
            "  A{}  max lag {:.3} deg, held {:+.3} deg from the last target sent",
            i + 1,
            lag.to_degrees(),
            (q_held[i] - last_target[i]).to_degrees()
        );
    }
    println!(
        "  {} late replies, {}, {} starved cycles (holding after the stream), robot Delay max {}",
        after.late_replies - before.late_replies,
        clamps(&before, &after),
        after.starved_cycles - before.starved_cycles,
        log.iter().map(|s| s.delay).max().unwrap_or(0)
    );

    let back_secs = (1.875 * plan.amp_deg / plan.speed_deg_s).max(2.0);
    drv.move_to(q0, Duration::from_secs_f64(back_secs))?;
    drv.wait_idle(Duration::from_secs_f64(back_secs * 10.0 + 30.0))?;
    drain(&drv, &mut log, &mut csv)?;
    let q = drv.state().q;
    let err = plan.axes.iter().map(|&i| (q[i] - q0[i]).to_degrees().abs()).fold(0.0, f64::max);
    println!("back at the start pose, largest error {err:.3} deg");
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
    fn the_config_file_is_optional() {
        assert_eq!(parse_args(&args("")).unwrap().0.as_deref(), Some("cfg.xml"));
        let flags: Vec<String> = ["--axes", "1", "--amp", "5"].map(String::from).into();
        let (config, p) = parse_args(&flags).unwrap();
        assert_eq!((config, p.axes, p.amp_deg), (None, vec![0], 5.0));
    }

    #[test]
    fn defaults_are_the_small_a6_sine() {
        let (_, p) = parse_args(&args("")).unwrap();
        assert_eq!((p.axes.clone(), p.amp_deg, p.period, p.secs), (vec![5], 3.0, 4.0, 8.0));
    }

    #[test]
    fn path_starts_and_ends_at_rest_and_stays_within_the_amplitude() {
        let (_, p) = parse_args(&args("--axes 1,3,6 --amp 10 --period 4 --secs 12 --speed 20")).unwrap();
        assert_eq!(p.axes, vec![0, 2, 5]);
        assert_eq!(p.offset_deg(0.0), [0.0; 6]);
        assert!(p.offset_deg(12.0).iter().all(|o| o.abs() < 1e-9));
        let dt = 0.001;
        let mut peak_speed: f64 = 0.0;
        for k in 0..12_000 {
            let t = k as f64 * dt;
            let (a, b) = (p.offset_deg(t), p.offset_deg(t + dt));
            for i in 0..6 {
                assert!(a[i].abs() <= 10.0 + 1e-9);
                peak_speed = peak_speed.max((b[i] - a[i]).abs() / dt);
            }
        }
        assert!(peak_speed <= 10.0 * (TAU + 1.5) / 4.0, "{peak_speed}");
        assert!(peak_speed > 10.0 * TAU / 4.0 * 0.9, "the full swing is reached: {peak_speed}");
    }

    #[test]
    fn caps_and_speed_bound_hold() {
        assert!(parse_args(&args("--axes 1 --amp 30 --period 12 --secs 24 --speed 20")).is_ok());
        assert!(parse_args(&args("--axes 1 --amp 31 --period 20 --secs 40")).unwrap_err().contains("A1"));
        assert!(parse_args(&args("--axes 6 --amp 45 --period 20 --secs 40")).is_ok());
        assert!(parse_args(&args("--amp 10 --period 2 --secs 4")).unwrap_err().contains("peaks"));
        assert!(parse_args(&args("--amp NaN")).is_err());
        assert!(parse_args(&args("--amp 0")).is_err());
        assert!(parse_args(&args("--speed 40")).is_err());
        assert!(parse_args(&args("--secs 5")).is_err(), "less than two periods");
        assert!(parse_args(&args("--stop-after 9")).is_err());
        assert!(parse_args(&args("--stop-after 3")).is_ok());
        assert!(parse_args(&args("--axes 0")).is_err());
        assert!(parse_args(&args("--bogus 1")).is_err());
    }
}
