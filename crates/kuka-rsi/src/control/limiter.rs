//! Position, velocity, acceleration and correction clamps applied every cycle.

use std::time::Duration;

use crate::config::Limits;

/// A clamp that moves a value by no more than this (rad) is floating-point rounding of motion
/// exactly at a limit, not a real clamp: the clamped value is still applied, but not counted.
const CLAMP_TOLERANCE: f64 = 1e-12;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ClampCounts {
    pub position: u64,
    pub velocity: u64,
    pub acceleration: u64,
    pub correction: u64,
}

/// Per-cycle position, velocity, acceleration and correction clamps. With an acceleration limit,
/// the closing speed toward the target is bounded by the braking distance relative to the
/// target's own velocity, so a feasible moving target is tracked without lag while a stopped,
/// drained or jumped-behind target is approached without overshoot ringing; each joint limit is
/// braked toward the same way as a stationary target, so the command never carries a stream's
/// overshoot past it. The command never crosses a position limit; one that starts outside the
/// limits returns to them under the velocity and acceleration clamps.
pub struct Limiter {
    limits: Limits,
    dt: f64,
    prev: Option<[f64; 6]>,
    prev_delta: [f64; 6],
    prev_target: Option<[f64; 6]>,
    counts: ClampCounts,
}

/// `v` clamped to `[lo, hi]`. Sets `hit` only when the clamp moves `v` by more than
/// `CLAMP_TOLERANCE`, so motion exactly at a limit is not counted through rounding.
fn clamp_counted(v: f64, lo: f64, hi: f64, hit: &mut u64) -> f64 {
    let c = v.clamp(lo, hi);
    if (c - v).abs() > CLAMP_TOLERANCE {
        *hit = 1;
    }
    c
}

/// Closing speed (rad/s) toward a point `d` rad away from which it can still be matched with
/// deceleration <= `a`, minus one cycle of deceleration so the discrete step never needs more
/// than `a`; never slower than closing the remaining gap in one accelerating step.
fn closing_speed(d: f64, a: f64, dt: f64) -> f64 {
    ((2.0 * a * d).sqrt() - a * dt).max((d / dt).min(a * dt))
}

impl Limiter {
    pub fn new(limits: Limits, cycle: Duration) -> Limiter {
        Limiter {
            limits,
            dt: cycle.as_secs_f64(),
            prev: None,
            prev_delta: [0.0; 6],
            prev_target: None,
            counts: ClampCounts::default(),
        }
    }

    /// Start tracking from `q` with zero velocity.
    pub fn reset(&mut self, q: [f64; 6]) {
        self.prev = Some(q);
        self.prev_delta = [0.0; 6];
        self.prev_target = None;
    }

    pub fn set_limits(&mut self, limits: Limits) {
        self.limits = limits;
    }

    /// The last command returned by `step`.
    pub fn last(&self) -> Option<[f64; 6]> {
        self.prev
    }

    pub fn counts(&self) -> ClampCounts {
        self.counts
    }

    /// Clamp `target` and return the command for this cycle, per axis in order:
    /// 1. clamp the target to the position limits;
    /// 2. clamp the delta from the last command to the velocity limit;
    /// 3. with an acceleration limit: bound the closing speed toward the target by the braking
    ///    distance relative to the target's own velocity (so a feasible moving target is tracked
    ///    without lag while a stopped, drained or jumped-behind target is approached without
    ///    overshoot ringing); bound the motion toward each position limit by the braking distance
    ///    to it as a stationary target (so a stream ending at speed near a limit stops at the
    ///    limit); then clamp the change of delta to `max_acceleration × dt²`;
    /// 4. clamp `q − q_start` to `±max_correction`;
    /// 5. clamp the command to the position limits as a hard guarantee (a command already outside
    ///    them may only move back toward them).
    ///
    /// The two braking bounds are not counted as clamps; every other kind counts at most once
    /// per step, and only when it moves a value by more than `CLAMP_TOLERANCE`.
    pub fn step(&mut self, target: [f64; 6], q_start: [f64; 6]) -> [f64; 6] {
        let l = &self.limits;
        let dt = self.dt;
        let mut q = target;
        let mut hit = ClampCounts::default();
        for i in 0..6 {
            q[i] = clamp_counted(q[i], l.position_min[i], l.position_max[i], &mut hit.position);
        }
        let target_clamped = q;
        let Some(prev) = self.prev else {
            self.counts.position += hit.position;
            self.prev = Some(q);
            self.prev_delta = [0.0; 6];
            self.prev_target = Some(target_clamped);
            return q;
        };
        let mut delta = [0.0; 6];
        for i in 0..6 {
            let vmax = l.max_velocity[i] * dt;
            delta[i] = clamp_counted(q[i] - prev[i], -vmax, vmax, &mut hit.velocity);
            if let Some(a) = l.max_acceleration {
                let a = a[i];
                let vmax = l.max_velocity[i];
                // The target's own per-cycle velocity (rad/s), bounded by the velocity limit so a
                // replaced chunk that jumps cannot fake a huge speed. Zero on the first step after reset.
                let vt = match self.prev_target {
                    Some(pt) => ((q[i] - pt[i]) / dt).clamp(-vmax, vmax),
                    None => 0.0,
                };
                // The gap relative to the target's motion: excluding the target's own step this
                // cycle, so catching up to a moving target does not oscillate around it.
                let gap = q[i] - prev[i] - vt * dt;
                let v_close = closing_speed(gap.abs(), a, dt);
                // Bound only the component of the motion that closes the gap; moving away or moving
                // with the target is left to the velocity and change-of-delta clamps.
                if gap > 0.0 {
                    delta[i] = delta[i].min(vt * dt + v_close * dt);
                } else if gap < 0.0 {
                    delta[i] = delta[i].max(vt * dt - v_close * dt);
                }
                // Each position limit is a stationary target: bound the motion toward it by the
                // braking distance to it, so the command stops at the limit instead of carrying
                // a stream's overshoot past it.
                if delta[i] > 0.0 {
                    let gap_up = (l.position_max[i] - prev[i]).max(0.0);
                    delta[i] = delta[i].min(closing_speed(gap_up, a, dt) * dt);
                } else if delta[i] < 0.0 {
                    let gap_down = (prev[i] - l.position_min[i]).max(0.0);
                    delta[i] = delta[i].max(-closing_speed(gap_down, a, dt) * dt);
                }
                // Change of delta (acceleration) — this one counts as a clamp.
                let amax = a * dt * dt;
                let pd = self.prev_delta[i];
                delta[i] = clamp_counted(delta[i], pd - amax, pd + amax, &mut hit.acceleration);
            }
            q[i] = prev[i] + delta[i];
            let corr = q[i] - q_start[i];
            let c = clamp_counted(corr, -l.max_correction[i], l.max_correction[i], &mut hit.correction);
            if c != corr {
                q[i] = q_start[i] + c;
                delta[i] = q[i] - prev[i];
            }
            // Hard guarantee: the command never crosses a position limit. A command already outside
            // them (after a `reset` or `set_limits` outside the range) is not jumped back in one
            // cycle, since the jump would also land in `prev_delta` and the change-of-delta clamp
            // would keep it going; it may only move back toward them, under the clamps above.
            let lo = l.position_min[i].min(prev[i]);
            let hi = l.position_max[i].max(prev[i]);
            let c = clamp_counted(q[i], lo, hi, &mut hit.position);
            if c != q[i] {
                q[i] = c;
                delta[i] = q[i] - prev[i];
            }
        }
        self.counts.position += hit.position;
        self.counts.velocity += hit.velocity;
        self.counts.acceleration += hit.acceleration;
        self.counts.correction += hit.correction;
        self.prev_delta = delta;
        self.prev = Some(q);
        self.prev_target = Some(target_clamped);
        q
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Limits;
    use std::time::Duration;

    fn limiter() -> Limiter {
        let l = Limits::kr10_defaults().with_max_velocity_all(1.0).with_max_acceleration_all(None);
        let mut lim = Limiter::new(l, Duration::from_millis(4));
        lim.reset([0.0; 6]);
        lim
    }

    fn with_accel() -> Limiter {
        let l = Limits::kr10_defaults().with_max_velocity_all(1.0).with_max_acceleration_all(Some(10.0));
        let mut lim = Limiter::new(l, Duration::from_millis(4));
        lim.reset([0.0; 6]);
        lim
    }

    #[test]
    fn passes_reachable_targets_untouched() {
        let mut lim = limiter();
        let q = lim.step([0.001, -0.002, 0.003, 0.0, 0.0, 0.004], [0.0; 6]);
        assert_eq!(q, [0.001, -0.002, 0.003, 0.0, 0.0, 0.004]);
        assert_eq!(lim.counts(), ClampCounts::default());
        assert_eq!(lim.last(), Some(q));
    }

    #[test]
    fn velocity_clamp_spreads_a_step_over_cycles() {
        let mut lim = limiter();
        let target = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0];
        let q1 = lim.step(target, [0.0; 6]);
        assert!((q1[0] - 0.004).abs() < 1e-12, "1 rad/s * 4 ms");
        let q2 = lim.step(target, [0.0; 6]);
        assert!((q2[0] - 0.008).abs() < 1e-12);
        let q3 = lim.step(target, [0.0; 6]);
        assert!((q3[0] - 0.01).abs() < 1e-12, "reached");
        assert_eq!(lim.counts().velocity, 2);
    }

    #[test]
    fn position_clamp_counts_once_per_step() {
        let mut lim = limiter();
        let l = Limits::kr10_defaults();
        let q = lim.step([l.position_max[0] + 1.0, 0.0, 0.0, 0.0, 0.0, l.position_min[5] - 1.0], [0.0; 6]);
        assert!(q[0] <= l.position_max[0]);
        assert!(q[5] >= l.position_min[5]);
        assert_eq!(lim.counts().position, 1);
    }

    #[test]
    fn acceleration_clamp_limits_change_of_delta() {
        let l = Limits::kr10_defaults().with_max_velocity_all(10.0).with_max_acceleration_all(Some(100.0));
        let mut lim = Limiter::new(l, Duration::from_millis(4));
        lim.reset([0.0; 6]);
        let a_dt2 = 100.0 * 0.004 * 0.004; // 0.0016 rad per cycle per cycle
        let q1 = lim.step([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6]);
        assert!((q1[0] - a_dt2).abs() < 1e-12, "first delta limited to a*dt²");
        let q2 = lim.step([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6]);
        assert!((q2[0] - 3.0 * a_dt2).abs() < 1e-12, "delta grows by a*dt² each cycle");
        assert_eq!(lim.counts().acceleration, 2);
    }

    #[test]
    fn correction_clamp_bounds_distance_from_start() {
        let mut l = Limits::kr10_defaults().with_max_velocity_all(100.0).with_max_acceleration_all(None);
        l.max_correction = [0.05; 6];
        let mut lim = Limiter::new(l, Duration::from_millis(4));
        lim.reset([0.1; 6]);
        let q = lim.step([0.3; 6], [0.1; 6]);
        assert!(q.iter().all(|v| (*v - 0.15).abs() < 1e-12), "{q:?}");
        assert_eq!(lim.counts().correction, 1);
    }

    #[test]
    fn first_step_without_reset_adopts_target() {
        let l = Limits::kr10_defaults().with_max_velocity_all(0.001);
        let mut lim = Limiter::new(l, Duration::from_millis(4));
        let q = lim.step([0.5; 6], [0.0; 6]);
        assert_eq!(q, [0.5; 6]);
        assert_eq!(lim.counts().velocity, 0);
    }

    #[test]
    fn reset_restarts_tracking() {
        let mut lim = limiter();
        lim.step([0.004, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6]);
        lim.reset([0.5; 6]);
        let q = lim.step([0.5; 6], [0.5; 6]);
        assert_eq!(q, [0.5; 6]);
    }

    #[test]
    fn approach_from_rest_never_overshoots_and_settles() {
        let mut lim = with_accel();
        let target = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0];
        let mut last = 0.0;
        for _ in 0..600 {
            let q = lim.step(target, [0.0; 6]);
            assert!(q[0] <= 0.3 + 1e-9, "overshoot: {}", q[0]);
            assert!(q[0] >= last - 1e-12, "moved backwards: {} after {}", q[0], last);
            last = q[0];
        }
        assert!((last - 0.3).abs() < 1e-6, "did not settle: {last}");
    }

    #[test]
    fn hold_at_speed_decelerates_without_ringing() {
        let mut lim = with_accel();
        let far = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        for _ in 0..100 {
            lim.step(far, [0.0; 6]); // accelerate to full speed (0.004 rad/cycle after 25 cycles)
        }
        let hold = lim.last().unwrap();
        let mut prev = hold[0];
        let mut reversals = 0;
        let mut direction = 1.0f64;
        let mut max_overshoot = 0.0f64;
        for _ in 0..1000 {
            let q = lim.step(hold, [0.0; 6]);
            max_overshoot = max_overshoot.max(q[0] - hold[0]);
            let step = q[0] - prev;
            if step.abs() > 1e-12 && step.signum() != direction {
                reversals += 1;
                direction = step.signum();
            }
            prev = q[0];
        }
        // braking distance v²/(2a) = 1/(20) = 0.05 rad, plus one cycle of slack
        assert!(max_overshoot <= 0.05 + 0.004 + 1e-9, "overshoot {max_overshoot}");
        assert!(reversals <= 1, "ringing: {reversals} reversals");
        assert!((prev - hold[0]).abs() < 1e-6, "did not settle: {prev} vs {}", hold[0]);
    }

    /// Drives `lim` toward a target that advances `step_per_cycle` each cycle from `start`, returning
    /// (positions, targets) for `n` cycles.
    fn track(lim: &mut Limiter, start: f64, step_per_cycle: f64, n: usize) -> (Vec<f64>, Vec<f64>) {
        let mut qs = Vec::with_capacity(n);
        let mut ts = Vec::with_capacity(n);
        for k in 1..=n {
            let t = start + step_per_cycle * k as f64;
            let q = lim.step([t, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6]);
            qs.push(q[0]);
            ts.push(t);
        }
        (qs, ts)
    }

    fn reversals(qs: &[f64]) -> usize {
        let mut count = 0;
        let mut dir = 0.0f64;
        for w in qs.windows(2) {
            let s = w[1] - w[0];
            if s.abs() > 1e-12 {
                if dir != 0.0 && s.signum() != dir {
                    count += 1;
                }
                dir = s.signum();
            }
        }
        count
    }

    #[test]
    fn feasible_steady_stream_is_tracked_without_lag() {
        let mut lim = with_accel();
        let (qs, ts) = track(&mut lim, 0.0, 0.002, 400); // 0.5 rad/s, half the velocity limit
        assert_eq!(reversals(&qs), 0, "qs[..40] = {:?}", &qs[..40.min(qs.len())]);
        let lag = (ts[399] - qs[399]).abs();
        assert!(lag < 1e-9, "steady-state lag {lag}; qs[..40] = {:?}, ts[..40] = {:?}", &qs[..40], &ts[..40]);
        let before = lim.counts().acceleration;
        track(&mut lim, ts[399], 0.002, 100);
        assert_eq!(lim.counts().acceleration, before, "no acceleration clamps while tracking a feasible stream");
    }

    #[test]
    fn catching_up_to_a_slow_target_does_not_ring() {
        let mut lim = with_accel();
        let (qs, ts) = track(&mut lim, 0.1, 0.0005, 400); // target starts 0.1 rad ahead, moves at 0.125 rad/s
        assert!(
            reversals(&qs) <= 1,
            "{} reversals; qs[..40] = {:?}, ts[..40] = {:?}",
            reversals(&qs),
            &qs[..40],
            &ts[..40]
        );
        assert!(
            (ts[399] - qs[399]).abs() < 1e-6,
            "not tracking: {} vs {}; qs[..40] = {:?}, ts[..40] = {:?}",
            qs[399],
            ts[399],
            &qs[..40],
            &ts[..40]
        );
    }

    #[test]
    fn drained_stream_stops_without_ringing() {
        let mut lim = with_accel();
        let (_, ts) = track(&mut lim, 0.0, 0.004, 100); // full speed
        let stop = ts[99];
        let mut qs = Vec::new();
        for _ in 0..600 {
            qs.push(lim.step([stop, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6])[0]);
        }
        assert!(reversals(&qs) <= 1, "{} reversals; qs[..40] = {:?}", reversals(&qs), &qs[..40]);
        assert!((qs[599] - stop).abs() < 1e-6, "qs[..40] = {:?}", &qs[..40]);
        let overshoot = qs.iter().cloned().fold(f64::MIN, f64::max) - stop;
        assert!(overshoot <= 0.05 + 0.004 + 1e-9, "overshoot {overshoot}; qs[..40] = {:?}", &qs[..40]); // v²/(2a) + one cycle
    }

    #[test]
    fn target_jumping_behind_while_moving_reverses_once() {
        let mut lim = with_accel();
        let (qs0, _) = track(&mut lim, 0.0, 0.004, 100);
        let back = qs0[99] - 0.2;
        let mut qs = Vec::new();
        for _ in 0..800 {
            qs.push(lim.step([back, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6])[0]);
        }
        assert!(reversals(&qs) <= 1, "{} reversals; qs[..40] = {:?}", reversals(&qs), &qs[..40]);
        assert!((qs[799] - back).abs() < 1e-6, "did not settle: {}; qs[..40] = {:?}", qs[799], &qs[..40]);
    }

    #[test]
    fn drained_stream_near_a_joint_limit_never_crosses_it() {
        let mut l = Limits::kr10_defaults().with_max_velocity_all(1.0).with_max_acceleration_all(Some(10.0));
        l.position_max[0] = 0.2;
        let mut lim = Limiter::new(l, Duration::from_millis(4));
        lim.reset([0.0; 6]);
        // full-speed stream whose targets run into the limit and stop there
        let mut qs = Vec::new();
        for k in 1..=700 {
            let t = (0.004 * k as f64).min(0.2);
            qs.push(lim.step([t, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6])[0]);
        }
        let max = qs.iter().cloned().fold(f64::MIN, f64::max);
        assert!(max <= 0.2 + 1e-9, "crossed the joint limit: {max}");
        assert!(reversals(&qs) <= 1, "{} reversals", reversals(&qs));
        assert!((qs[699] - 0.2).abs() < 1e-6, "did not settle at the limit: {}", qs[699]);
        assert_eq!(lim.counts().position, 0, "braking toward the limit should make the hard output clamp unnecessary");
    }

    #[test]
    fn catch_up_tracking_error_does_not_oscillate() {
        let mut lim = with_accel();
        let (qs, ts) = track(&mut lim, 0.0175, 0.0014, 600); // 1 degree behind a 20 deg/s target
        let mut sign_changes = 0;
        let mut last = 0.0f64;
        for (q, t) in qs.iter().zip(&ts) {
            let e = t - q;
            if e.abs() > 1e-9 {
                if last != 0.0 && e.signum() != last {
                    sign_changes += 1;
                }
                last = e.signum();
            }
        }
        assert!(sign_changes <= 1, "tracking error changed sign {sign_changes} times");
        assert!((ts[599] - qs[599]).abs() < 1e-6);
    }

    #[test]
    fn motion_exactly_at_the_velocity_limit_is_not_counted_as_a_clamp() {
        let mut lim = with_accel();
        // ramp at exactly the acceleration limit up to exactly vmax, then cruise at exactly vmax;
        // 700 cycles end at 2.75 rad, short of A1's 170° (2.967 rad) limit, where braking for the
        // limit would lag the stream and be a real velocity clamp
        let mut t = 0.0;
        let mut v = 0.0f64;
        for _ in 0..700 {
            v = (v + 10.0 * 0.004).min(1.0);
            t += v * 0.004;
            lim.step([t, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6]);
        }
        let c = lim.counts();
        assert_eq!(c.velocity, 0, "{c:?}");
        assert_eq!(c.acceleration, 0, "{c:?}");
    }

    #[test]
    fn tightening_limits_past_the_command_does_not_jump_or_run_away() {
        let mut lim = with_accel();
        let (qs0, _) = track(&mut lim, 0.0, 0.004, 100); // moving up at full speed
        let before = qs0[99];
        let mut l = Limits::kr10_defaults().with_max_velocity_all(1.0).with_max_acceleration_all(Some(10.0));
        l.position_max[0] = 0.25; // below the current command
        lim.set_limits(l);
        let mut qs = vec![before];
        for k in 101..=900 {
            qs.push(lim.step([0.004 * k as f64, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6])[0]);
        }
        for w in qs.windows(2) {
            assert!((w[1] - w[0]).abs() <= 0.004 + 1e-12, "jumped {} -> {}; qs[..40] = {:?}", w[0], w[1], &qs[..40]);
        }
        let max = qs.iter().cloned().fold(f64::MIN, f64::max);
        assert!(max <= before + 1e-12, "moved further outside the limits: {max}; qs[..40] = {:?}", &qs[..40]);
        assert!((qs[800] - 0.25).abs() < 1e-6, "did not return to the limit: {}; qs[..40] = {:?}", qs[800], &qs[..40]);
    }

    #[test]
    fn feasible_stream_ending_at_a_joint_limit_stops_at_it() {
        let mut l = Limits::kr10_defaults().with_max_velocity_all(1.0).with_max_acceleration_all(Some(10.0));
        l.position_max[0] = 0.2;
        let mut lim = Limiter::new(l, Duration::from_millis(4));
        lim.reset([0.0; 6]);
        // ramp at the acceleration limit to full speed (tracked without lag), then cruise straight
        // into the limit: tracking alone would carry the v²/(2a) = 0.05 rad braking overshoot past it
        let (mut t, mut v) = (0.0f64, 0.0f64);
        let mut qs = Vec::new();
        for _ in 0..700 {
            v = (v + 10.0 * 0.004).min(1.0);
            t = (t + v * 0.004).min(0.2);
            qs.push(lim.step([t, 0.0, 0.0, 0.0, 0.0, 0.0], [0.0; 6])[0]);
        }
        let max = qs.iter().cloned().fold(f64::MIN, f64::max);
        assert!(max <= 0.2 + 1e-9, "crossed the joint limit: {max}; qs[..40] = {:?}", &qs[..40]);
        assert_eq!(reversals(&qs), 0, "qs[..40] = {:?}", &qs[..40]);
        assert!((qs[699] - 0.2).abs() < 1e-6, "did not settle at the limit: {}", qs[699]);
        assert_eq!(lim.counts().position, 0, "braking toward the limit should make the hard output clamp unnecessary");
    }
}
