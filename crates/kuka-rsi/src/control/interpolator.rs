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
    /// Playback length in cycles; fractional values play exactly (the leftover carries into the
    /// next segment). Zero, negative or non-finite counts as one cycle.
    pub cycles: f64,
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
    /// Set once a segment has been pushed; cleared by `reset`/`replace`. Guards `starved`
    /// against counting an empty queue that was never given anything to play.
    armed: bool,
    /// Played segments still in the ring behind `head`, for `rewind`: every one played since the
    /// last `reset`/`replace` that a later `push` has not overwritten.
    behind: usize,
    /// Where the oldest of the `behind` segments starts; `seg_start` when there is none.
    behind_start: [f64; 6],
}

fn cycles_of(seg: &Segment) -> f64 {
    if seg.cycles > 0.0 && seg.cycles.is_finite() { seg.cycles } else { 1.0 }
}

/// The position `progress` cycles into `seg`, which starts at `start`.
fn position_in(start: &[f64; 6], seg: &Segment, progress: f64) -> [f64; 6] {
    let s = progress / cycles_of(seg);
    let s = match seg.blend {
        Blend::Linear => s,
        Blend::Quintic => s * s * s * (10.0 + s * (-15.0 + 6.0 * s)),
    };
    let mut q = [0.0; 6];
    for i in 0..6 {
        q[i] = start[i] + s * (seg.q_end[i] - start[i]);
    }
    q
}

impl Interpolator {
    pub fn new(q0: [f64; 6]) -> Interpolator {
        let filler = Segment { q_end: q0, cycles: 1.0, blend: Blend::Linear };
        Interpolator {
            ring: vec![filler; MAX_QUEUE],
            head: 0,
            len: 0,
            seg_start: q0,
            current: q0,
            progress: 0.0,
            starved: 0,
            armed: false,
            behind: 0,
            behind_start: q0,
        }
    }

    pub fn reset(&mut self, q0: [f64; 6]) {
        self.head = 0;
        self.len = 0;
        self.seg_start = q0;
        self.current = q0;
        self.progress = 0.0;
        self.armed = false;
        self.behind = 0;
        self.behind_start = q0;
    }

    pub fn push(&mut self, seg: Segment) -> bool {
        if self.len == MAX_QUEUE {
            return false;
        }
        if self.len + self.behind == MAX_QUEUE {
            // The ring is full of queued and played segments: this one overwrites the oldest
            // played one.
            let oldest = (self.head + MAX_QUEUE - self.behind) % MAX_QUEUE;
            self.behind_start = self.ring[oldest].q_end;
            self.behind -= 1;
        }
        let idx = (self.head + self.len) % MAX_QUEUE;
        self.ring[idx] = seg;
        self.len += 1;
        self.armed = true;
        true
    }

    fn pop_front(&mut self) {
        self.head = (self.head + 1) % MAX_QUEUE;
        self.len -= 1;
        self.behind += 1;
    }

    /// The segment `back` places behind the head, and where it starts.
    fn behind_segment(&self, back: usize) -> (&Segment, [f64; 6]) {
        let seg = &self.ring[(self.head + MAX_QUEUE - back) % MAX_QUEUE];
        let start = if back < self.behind {
            self.ring[(self.head + MAX_QUEUE - back - 1) % MAX_QUEUE].q_end
        } else {
            self.behind_start
        };
        (seg, start)
    }

    /// Where the playback stood `cycles` ago: how many segments behind the head, how far into
    /// that segment, and how many cycles back that really is (less than `cycles` at the start
    /// of what the ring still holds).
    fn locate(&self, cycles: f64) -> (usize, f64, f64) {
        let mut back = 0;
        let mut progress = self.progress - cycles.max(0.0);
        let mut rewound = self.progress;
        while progress < 0.0 && back < self.behind {
            back += 1;
            let c = cycles_of(self.behind_segment(back).0);
            progress += c;
            rewound += c;
        }
        let progress = progress.max(0.0);
        (back, progress, rewound - progress)
    }

    /// The position on the path `cycles` of playback ago, or the oldest one still known.
    pub fn position_behind(&self, cycles: f64) -> [f64; 6] {
        let (back, progress, _) = self.locate(cycles);
        if back == 0 && self.len == 0 {
            return self.current;
        }
        let (seg, start) = self.behind_segment(back);
        position_in(&start, seg, progress)
    }

    /// Take the playback back by `cycles`: what was played since is queued again. Returns the
    /// cycles taken back, which is less at the start of what the ring still holds (a `reset`
    /// or `replace`, or the oldest played segment that later pushes have not overwritten).
    pub fn rewind(&mut self, cycles: f64) -> f64 {
        let (back, progress, rewound) = self.locate(cycles);
        if rewound <= 0.0 {
            return 0.0;
        }
        let (seg, start) = self.behind_segment(back);
        self.current = position_in(&start, seg, progress);
        self.seg_start = start;
        self.head = (self.head + MAX_QUEUE - back) % MAX_QUEUE;
        self.len += back;
        self.behind -= back;
        self.progress = progress;
        self.armed = true;
        rewound
    }

    /// Drop every queued segment; playback continues from the current position.
    pub fn replace(&mut self) {
        self.len = 0;
        self.seg_start = self.current;
        self.progress = 0.0;
        self.armed = false;
        self.behind = 0;
        self.behind_start = self.current;
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
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
            if self.armed && rate > 0.0 {
                self.starved += 1;
            }
            return self.current;
        }
        self.progress += rate.max(0.0);
        loop {
            let seg = self.ring[self.head];
            let cycles = cycles_of(&seg);
            if self.progress >= cycles {
                if self.behind == 0 {
                    self.behind_start = self.seg_start;
                }
                self.seg_start = seg.q_end;
                self.current = seg.q_end;
                self.progress -= cycles;
                self.pop_front();
                if self.len == 0 {
                    self.progress = 0.0;
                    if seg.blend == Blend::Quintic {
                        // A completed point-to-point move is deliberate idleness, not starvation.
                        self.armed = false;
                    }
                    return self.current;
                }
                continue;
            }
            self.current = position_in(&self.seg_start, &seg, self.progress);
            return self.current;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seg(v: f64, cycles: f64, blend: Blend) -> Segment {
        Segment { q_end: [v; 6], cycles, blend }
    }

    #[test]
    fn linear_segment_advances_one_cycle_per_step() {
        let mut it = Interpolator::new([0.0; 6]);
        assert!(it.push(seg(1.0, 4.0, Blend::Linear)));
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
        it.push(seg(1.0, 4.0, Blend::Linear));
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
        it.push(seg(1.0, 10.0, Blend::Quintic));
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
        it.push(seg(1.0, 2.0, Blend::Linear));
        it.push(seg(3.0, 2.0, Blend::Linear));
        assert_eq!(it.step(1.5)[0], 0.75);
        assert_eq!(it.step(1.5)[0], 2.0, "1.0 reached, then half of the second segment");
        assert_eq!(it.step(1.0)[0], 3.0);
        assert!(it.is_idle());
    }

    #[test]
    fn fractional_cycles_play_exactly() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 2.5, Blend::Linear));
        for want in [0.4, 0.8, 1.0] {
            let got = it.step(1.0)[0];
            assert!((got - want).abs() < 1e-12, "{got} vs {want}");
        }
        assert!(it.is_idle());

        // The leftover half cycle carries into the next segment: two 2.5-cycle segments take
        // exactly five cycles.
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 2.5, Blend::Linear));
        it.push(seg(2.0, 2.5, Blend::Linear));
        for want in [0.4, 0.8, 1.2, 1.6, 2.0] {
            let got = it.step(1.0)[0];
            assert!((got - want).abs() < 1e-12, "{got} vs {want}");
        }
        assert!(it.is_idle());
    }

    #[test]
    fn replace_drops_the_queue_and_continues_from_current() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 4.0, Blend::Linear));
        it.push(seg(2.0, 4.0, Blend::Linear));
        it.step(1.0);
        assert_eq!(it.current()[0], 0.25);
        it.replace();
        assert_eq!(it.len(), 0);
        assert_eq!(it.step(1.0)[0], 0.25, "holds at the replaced position");
        it.push(seg(0.75, 2.0, Blend::Linear));
        assert_eq!(it.step(1.0)[0], 0.5);
        assert_eq!(it.step(1.0)[0], 0.75);
    }

    #[test]
    fn push_fails_when_full_and_zero_cycles_counts_as_one() {
        let mut it = Interpolator::new([0.0; 6]);
        for _ in 0..MAX_QUEUE {
            assert!(it.push(seg(1.0, 0.0, Blend::Linear)));
        }
        assert!(!it.push(seg(1.0, 1.0, Blend::Linear)));
        assert_eq!(it.step(1.0)[0], 1.0);
        assert_eq!(it.len(), MAX_QUEUE - 1);
    }

    #[test]
    fn completed_quintic_move_does_not_starve() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 4.0, Blend::Quintic));
        for _ in 0..4 {
            it.step(1.0);
        }
        assert!(it.is_idle());
        for _ in 0..3 {
            it.step(1.0);
        }
        assert_eq!(it.starved(), 0);

        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 4.0, Blend::Linear));
        for _ in 0..4 {
            it.step(1.0);
        }
        assert!(it.is_idle());
        for _ in 0..3 {
            it.step(1.0);
        }
        assert_eq!(it.starved(), 3);
    }

    #[test]
    fn rewind_goes_back_within_a_segment() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 4.0, Blend::Linear));
        for _ in 0..3 {
            it.step(1.0);
        }
        assert_eq!(it.position_behind(2.0)[0], 0.25);
        assert_eq!(it.current()[0], 0.75, "looking back changes nothing");
        assert_eq!(it.rewind(2.0), 2.0);
        assert_eq!(it.current()[0], 0.25);
        assert_eq!(it.step(1.0)[0], 0.5);
    }

    #[test]
    fn rewind_goes_back_across_segments() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 2.0, Blend::Linear));
        it.push(seg(3.0, 2.0, Blend::Linear));
        for _ in 0..3 {
            it.step(1.0);
        }
        assert_eq!(it.current()[0], 2.0);
        assert_eq!(it.len(), 1);
        assert_eq!(it.position_behind(2.0)[0], 0.5);
        assert_eq!(it.rewind(2.0), 2.0);
        assert_eq!(it.current()[0], 0.5);
        assert_eq!(it.len(), 2);
        for want in [1.0, 2.0, 3.0] {
            assert_eq!(it.step(1.0)[0], want);
        }
        assert!(it.is_idle());
    }

    #[test]
    fn rewind_queues_a_finished_move_again() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 4.0, Blend::Quintic));
        for _ in 0..4 {
            it.step(1.0);
        }
        assert!(it.is_idle());
        assert_eq!(it.rewind(2.0), 2.0);
        assert!(!it.is_idle());
        assert_eq!(it.current()[0], 0.5, "the middle of the move");
        it.step(1.0);
        assert_eq!(it.step(1.0)[0], 1.0);
        assert!(it.is_idle());
        it.step(1.0);
        assert_eq!(it.starved(), 0, "a point-to-point move that ended again");
    }

    #[test]
    fn rewind_stops_at_the_start_of_the_queue() {
        let mut it = Interpolator::new([0.5; 6]);
        it.push(seg(1.5, 4.0, Blend::Linear));
        it.step(1.0);
        it.step(1.0);
        assert_eq!(it.position_behind(10.0)[0], 0.5);
        assert_eq!(it.rewind(10.0), 2.0);
        assert_eq!(it.current()[0], 0.5);
        assert_eq!(it.rewind(1.0), 0.0, "nothing left to take back");

        // Nothing played before a reset comes back.
        for _ in 0..4 {
            it.step(1.0);
        }
        it.reset([5.0; 6]);
        assert_eq!(it.position_behind(3.0), [5.0; 6]);
        assert_eq!(it.rewind(3.0), 0.0);
        assert_eq!(it.current(), [5.0; 6]);
        assert!(it.is_idle());
    }

    #[test]
    fn rewind_stops_at_segments_overwritten_by_later_pushes() {
        let mut it = Interpolator::new([0.0; 6]);
        for i in 0..MAX_QUEUE {
            assert!(it.push(seg((i + 1) as f64, 1.0, Blend::Linear)));
        }
        for _ in 0..MAX_QUEUE {
            it.step(1.0);
        }
        assert!(it.is_idle());
        // All but the last two played segments are overwritten.
        for i in 0..MAX_QUEUE - 2 {
            assert!(it.push(seg(2000.0 + i as f64, 1.0, Blend::Linear)));
        }
        let end = MAX_QUEUE as f64;
        assert_eq!(it.position_behind(1.0)[0], end - 1.0);
        assert_eq!(it.position_behind(10.0)[0], end - 2.0, "the start of the oldest segment left");
        assert_eq!(it.rewind(10.0), 2.0);
        assert_eq!(it.current()[0], end - 2.0);
        assert_eq!(it.len(), MAX_QUEUE);
        assert!(!it.push(seg(0.0, 1.0, Blend::Linear)), "the queue is full again");
        assert_eq!(it.step(1.0)[0], end - 1.0);
        assert_eq!(it.step(1.0)[0], end);
        assert_eq!(it.step(1.0)[0], 2000.0);
    }

    #[test]
    fn reset_clears_everything() {
        let mut it = Interpolator::new([0.0; 6]);
        it.push(seg(1.0, 4.0, Blend::Linear));
        it.step(1.0);
        it.reset([5.0; 6]);
        assert!(it.is_idle());
        assert_eq!(it.current(), [5.0; 6]);
        assert_eq!(it.step(1.0), [5.0; 6]);
    }
}
