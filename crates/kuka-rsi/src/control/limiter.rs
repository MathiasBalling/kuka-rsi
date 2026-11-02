//! Position, velocity, acceleration and correction clamps applied every cycle.

use std::time::Duration;

use crate::config::Limits;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ClampCounts {
    pub position: u64,
    pub velocity: u64,
    pub acceleration: u64,
    pub correction: u64,
}

pub struct Limiter {
    limits: Limits,
    dt: f64,
    prev: Option<[f64; 6]>,
    prev_delta: [f64; 6],
    counts: ClampCounts,
}

impl Limiter {
    pub fn new(limits: Limits, cycle: Duration) -> Limiter {
        Limiter { limits, dt: cycle.as_secs_f64(), prev: None, prev_delta: [0.0; 6], counts: ClampCounts::default() }
    }

    /// Start tracking from `q` with zero velocity.
    pub fn reset(&mut self, q: [f64; 6]) {
        self.prev = Some(q);
        self.prev_delta = [0.0; 6];
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

    /// Clamp `target` and return the command for this cycle: position, then velocity, then a
    /// braking-aware acceleration bound (so decelerating toward a stopped or reversed target
    /// never overshoots and rings) followed by the existing change-of-delta clamp, then correction.
    pub fn step(&mut self, target: [f64; 6], q_start: [f64; 6]) -> [f64; 6] {
        let l = &self.limits;
        let mut q = target;
        let mut hit = ClampCounts::default();
        for i in 0..6 {
            let c = q[i].clamp(l.position_min[i], l.position_max[i]);
            if c != q[i] {
                hit.position = 1;
                q[i] = c;
            }
        }
        let Some(prev) = self.prev else {
            self.counts.position += hit.position;
            self.prev = Some(q);
            self.prev_delta = [0.0; 6];
            return q;
        };
        let mut delta = [0.0; 6];
        for i in 0..6 {
            delta[i] = q[i] - prev[i];
            let vmax = l.max_velocity[i] * self.dt;
            let c = delta[i].clamp(-vmax, vmax);
            if c != delta[i] {
                hit.velocity = 1;
                delta[i] = c;
            }
            if let Some(a) = l.max_acceleration {
                let a = a[i];
                let d = (q[i] - prev[i]).abs();
                // Fastest speed from which the target can still be reached with deceleration <= a,
                // minus one cycle of deceleration so the discrete step never needs more than a;
                // but never slower than closing the remaining gap in one accelerating step.
                let v_brake = ((2.0 * a * d).sqrt() - a * self.dt).max((d / self.dt).min(a * self.dt));
                let dmax = v_brake * self.dt;
                let c = delta[i].clamp(-dmax, dmax);
                if c != delta[i] {
                    hit.acceleration = 1;
                    delta[i] = c;
                }
                let amax = a * self.dt * self.dt;
                let c = delta[i].clamp(self.prev_delta[i] - amax, self.prev_delta[i] + amax);
                if c != delta[i] {
                    hit.acceleration = 1;
                    delta[i] = c;
                }
            }
            q[i] = prev[i] + delta[i];
            let corr = q[i] - q_start[i];
            let c = corr.clamp(-l.max_correction[i], l.max_correction[i]);
            if c != corr {
                hit.correction = 1;
                q[i] = q_start[i] + c;
                delta[i] = q[i] - prev[i];
            }
        }
        self.counts.position += hit.position;
        self.counts.velocity += hit.velocity;
        self.counts.acceleration += hit.acceleration;
        self.counts.correction += hit.correction;
        self.prev_delta = delta;
        self.prev = Some(q);
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
}
