//! One CSV row per robot cycle, from [`Driver::drain_log`](crate::driver::Driver::drain_log), for
//! looking at a session afterwards: tracking lag, smoothness, `Delay`. Angles in degrees, TCP
//! position in millimetres, as on the pendant.

use std::fmt::Write as _;
use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;

use crate::state::{RobotState, SessionState};

/// The CSV header, newline-terminated.
pub const CSV_HEADER: &str = concat!(
    "t_s,cycle,ipoc,session,program_running,override,delay,",
    "q_a1_deg,q_a2_deg,q_a3_deg,q_a4_deg,q_a5_deg,q_a6_deg,",
    "cmd_a1_deg,cmd_a2_deg,cmd_a3_deg,cmd_a4_deg,cmd_a5_deg,cmd_a6_deg,",
    "lag_a1_deg,lag_a2_deg,lag_a3_deg,lag_a4_deg,lag_a5_deg,lag_a6_deg,",
    "ak_a1_deg,ak_a2_deg,ak_a3_deg,ak_a4_deg,ak_a5_deg,ak_a6_deg,",
    "tcp_x_mm,tcp_y_mm,tcp_z_mm,tcp_a_deg,tcp_b_deg,tcp_c_deg,",
    "torque_a1,torque_a2,torque_a3,torque_a4,torque_a5,torque_a6\n"
);

/// One state as a newline-terminated CSV row matching [`CSV_HEADER`]. `lag` is the command sent
/// this cycle minus the actual position.
pub fn csv_row(s: &RobotState) -> String {
    let mut r = String::with_capacity(512);
    let session = match s.session {
        SessionState::Listening => "Listening",
        SessionState::Active => "Active",
        SessionState::Lost => "Lost",
    };
    let _ = write!(
        r,
        "{:.6},{},{},{},{},{:.3},{}",
        s.t,
        s.cycle,
        s.ipoc,
        session,
        u8::from(s.program_running),
        s.speed_override,
        s.delay
    );
    for v in s.q {
        let _ = write!(r, ",{:.4}", v.to_degrees());
    }
    for v in s.q_command {
        let _ = write!(r, ",{:.4}", v.to_degrees());
    }
    for (c, q) in s.q_command.iter().zip(&s.q) {
        let _ = write!(r, ",{:.4}", (c - q).to_degrees());
    }
    for v in s.ak {
        let _ = write!(r, ",{v:.4}");
    }
    for v in &s.tcp[..3] {
        let _ = write!(r, ",{:.3}", v * 1000.0);
    }
    for v in &s.tcp[3..] {
        let _ = write!(r, ",{:.4}", v.to_degrees());
    }
    for v in s.torque {
        let _ = write!(r, ",{v:.3}");
    }
    r.push('\n');
    r
}

/// A CSV file of robot states. Every [`write`](CsvLog::write) is flushed, so a program killed
/// by Ctrl-C keeps what it logged.
pub struct CsvLog {
    out: BufWriter<File>,
    rows: u64,
}

impl CsvLog {
    /// Create (or truncate) the file, and its folder if missing, and write the header.
    pub fn create(path: impl AsRef<Path>) -> io::Result<CsvLog> {
        if let Some(dir) = path.as_ref().parent().filter(|d| !d.as_os_str().is_empty()) {
            std::fs::create_dir_all(dir)?;
        }
        let mut out = BufWriter::new(File::create(path)?);
        out.write_all(CSV_HEADER.as_bytes())?;
        out.flush()?;
        Ok(CsvLog { out, rows: 0 })
    }

    pub fn write(&mut self, states: &[RobotState]) -> io::Result<()> {
        for s in states {
            self.out.write_all(csv_row(s).as_bytes())?;
        }
        self.rows += states.len() as u64;
        self.out.flush()
    }

    /// Rows written so far.
    pub fn rows(&self) -> u64 {
        self.rows
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn header_names_every_axis() {
        const AXES: [&str; 6] = ["a1", "a2", "a3", "a4", "a5", "a6"];
        for prefix in ["q_", "cmd_", "lag_", "ak_", "torque_"] {
            for a in AXES {
                assert!(CSV_HEADER.contains(&format!("{prefix}{a}")), "{prefix}{a}");
            }
        }
    }
}
