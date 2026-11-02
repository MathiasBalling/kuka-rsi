//! Queue of segments played back on the robot's cycle clock.

pub const MAX_QUEUE: usize = 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Blend {
    Linear,
    Quintic,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    pub q_end: [f64; 6],
    pub cycles: u32,
    pub blend: Blend,
}

pub struct Interpolator {
    ring: Vec<Segment>,
    head: usize,
    len: usize,
    seg_start: [f64; 6],
    current: [f64; 6],
    progress: f64,
    starved: u64,
}

impl Interpolator {
    pub fn new(q0: [f64; 6]) -> Interpolator {
        let filler = Segment { q_end: q0, cycles: 1, blend: Blend::Linear };
        Interpolator { ring: vec![filler; MAX_QUEUE], head: 0, len: 0, seg_start: q0, current: q0, progress: 0.0, starved: 0 }
    }

    pub fn reset(&mut self, q0: [f64; 6]) {
        self.head = 0;
        self.len = 0;
        self.seg_start = q0;
        self.current = q0;
        self.progress = 0.0;
    }

    pub fn push(&mut self, seg: Segment) -> bool {
        if self.len == MAX_QUEUE {
            return false;
        }
        let idx = (self.head + self.len) % MAX_QUEUE;
        self.ring[idx] = seg;
        self.len += 1;
        true
    }

    fn pop_front(&mut self) {
        self.head = (self.head + 1) % MAX_QUEUE;
        self.len -= 1;
    }

    /// Drop every queued segment; playback continues from the current position.
    pub fn replace(&mut self) {
        self.len = 0;
        self.seg_start = self.current;
        self.progress = 0.0;
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_idle(&self) -> bool {
        self.len == 0
    }

    pub fn current(&self) -> [f64; 6] {
        self.current
    }

    pub fn starved(&self) -> u64 {
        self.starved
    }

    /// Advance by `rate` cycles and return the position on the path.
    pub fn step(&mut self, rate: f64) -> [f64; 6] {
        if self.len == 0 {
            if rate > 0.0 {
                self.starved += 1;
            }
            return self.current;
        }
        self.progress += rate.max(0.0);
        loop {
            let seg = self.ring[self.head];
            let cycles = seg.cycles.max(1) as f64;
            if self.progress >= cycles {
                self.seg_start = seg.q_end;
                self.current = seg.q_end;
                self.progress -= cycles;
                self.pop_front();
                if self.len == 0 {
                    self.progress = 0.0;
                    return self.current;
                }
                continue;
            }
            let s = self.progress / cycles;
            let s = match seg.blend {
                Blend::Linear => s,
                Blend::Quintic => s * s * s * (10.0 + s * (-15.0 + 6.0 * s)),
            };
            for i in 0..6 {
                self.current[i] = self.seg_start[i] + s * (seg.q_end[i] - self.seg_start[i]);
            }
            return self.current;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seg(v: f64, cycles: u32, blend: Blend) -> Segment {
        Segment { q_end: [v; 6], cycles, blend }
    }

    #[test]
    fn linear_segment_advances_one_cycle_per_step() {
        let mut it = Interpolator::new([0.0; 6]);
        assert!(it.push(seg(1.0, 4, Blend::Linear)));
        assert_eq!(it.len(), 1);
        assert!(!it.is_idle());
        assert_eq!(it.step(1.0)[0], 0.25);
        assert_eq!(it.step(1.0)[0], 0.5);
        assert_eq!(it.step(1.0)[0], 0.75);
        assert_eq!(it.step(1.0)[0], 1.0);
        assert!(it.is_idle());
        assert_eq!(it.step(1.0)[0], 1.0, "holds");
        assert_eq!(it.starved(), 1);
    }

    #[test]
    fn rate_scales_and_zero_pauses() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 4, Blend::Linear));
        assert_eq!(it.step(0.5)[0], 0.125);
        assert_eq!(it.step(0.0)[0], 0.125);
        assert_eq!(it.starved(), 0);
        for _ in 0..7 {
            it.step(0.5);
        }
        assert_eq!(it.current()[0], 1.0);
        assert_eq!(it.step(0.0)[0], 1.0);
        assert_eq!(it.starved(), 0, "paused, not starved");
    }

    #[test]
    fn quintic_is_smooth_at_the_ends_and_symmetric() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 10, Blend::Quintic));
        let first = it.step(1.0)[0];
        assert!(first < 0.01, "slow start: {first}");
        for _ in 0..3 {
            it.step(1.0);
        }
        let mid = it.step(1.0)[0];
        assert!((mid - 0.5).abs() < 1e-12, "midpoint {mid}");
        for _ in 0..4 {
            it.step(1.0);
        }
        let last = it.step(1.0)[0];
        assert_eq!(last, 1.0);
    }

    #[test]
    fn segments_chain_and_leftover_progress_carries_over() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 2, Blend::Linear));
        it.push(seg(3.0, 2, Blend::Linear));
        assert_eq!(it.step(1.5)[0], 0.75);
        assert_eq!(it.step(1.5)[0], 2.0, "1.0 reached, then half of the second segment");
        assert_eq!(it.step(1.0)[0], 3.0);
        assert!(it.is_idle());
    }

    #[test]
    fn replace_drops_the_queue_and_continues_from_current() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 4, Blend::Linear));
        it.push(seg(2.0, 4, Blend::Linear));
        it.step(1.0);
        assert_eq!(it.current()[0], 0.25);
        it.replace();
        assert_eq!(it.len(), 0);
        assert_eq!(it.step(1.0)[0], 0.25, "holds at the replaced position");
        it.push(seg(0.75, 2, Blend::Linear));
        assert_eq!(it.step(1.0)[0], 0.5);
        assert_eq!(it.step(1.0)[0], 0.75);
    }

    #[test]
    fn push_fails_when_full_and_zero_cycles_counts_as_one() {
        let mut it = Interpolator::new([0.0; 6]);
        for _ in 0..MAX_QUEUE {
            assert!(it.push(seg(1.0, 0, Blend::Linear)));
        }
        assert!(!it.push(seg(1.0, 1, Blend::Linear)));
        assert_eq!(it.step(1.0)[0], 1.0);
        assert_eq!(it.len(), MAX_QUEUE - 1);
    }

    #[test]
    fn reset_clears_everything() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 4, Blend::Linear));
        it.step(1.0);
        it.reset([5.0; 6]);
        assert!(it.is_idle());
        assert_eq!(it.current(), [5.0; 6]);
        assert_eq!(it.step(1.0), [5.0; 6]);
    }
}
