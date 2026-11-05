//! Robot state in SI units, converted from the wire telegram.

use serde::{Deserialize, Serialize};

use crate::telegram::{RobotTelegram, Schema};

pub fn deg2rad(d: f64) -> f64 {
    d.to_radians()
}

pub fn rad2deg(r: f64) -> f64 {
    r.to_degrees()
}

pub fn deg2rad6(v: &[f64; 6]) -> [f64; 6] {
    v.map(f64::to_radians)
}

pub fn rad2deg6(v: &[f64; 6]) -> [f64; 6] {
    v.map(f64::to_degrees)
}

/// Where the real-time thread is in the connection life cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum SessionState {
    #[default]
    Listening,
    Active,
    Lost,
}

/// Snapshot published after every cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RobotState {
    /// Seconds since the driver started, taken when the telegram arrived.
    pub t: f64,
    /// Cycle count within the current session.
    pub cycle: u64,
    pub ipoc: u64,
    pub session: SessionState,
    /// Actual joint positions, rad.
    pub q: [f64; 6],
    /// Programmed joint setpoint (constant under `RSI_MOVECORR`), rad.
    pub q_setpoint: [f64; 6],
    /// Actual TCP pose: x, y, z in m; a, b, c in rad (KUKA Z-Y'-X'').
    pub tcp: [f64; 6],
    pub tcp_setpoint: [f64; 6],
    /// Gear torques as reported by the robot.
    pub torque: [f64; 6],
    /// Late packets counted by the robot.
    pub delay: u32,
    /// Program override, 0.0 to 1.0.
    pub speed_override: f64,
    pub program_running: bool,
    /// Joint target the driver sent this cycle, rad.
    pub q_command: [f64; 6],
    /// Correction the driver sent this cycle, deg.
    pub ak: [f64; 6],
}

/// Slots of the user elements the state layer reads.
#[derive(Clone, Copy, Debug, Default)]
pub struct StateMap {
    pub torque: [Option<usize>; 6],
    pub prog_status: Option<usize>,
    pub ov_pro: Option<usize>,
}

impl StateMap {
    pub fn from_schema(s: &Schema) -> StateMap {
        let axes = ["A1", "A2", "A3", "A4", "A5", "A6"];
        let mut torque = [None; 6];
        for i in 0..6 {
            torque[i] = s.rob_slot("GearTorque", Some(axes[i]));
        }
        StateMap { torque, prog_status: s.rob_slot("ProgStatus", Some("R")), ov_pro: s.rob_slot("OvPro", Some("R")) }
    }
}

fn cart_si(v: &[f64; 6]) -> [f64; 6] {
    [v[0] / 1000.0, v[1] / 1000.0, v[2] / 1000.0, v[3].to_radians(), v[4].to_radians(), v[5].to_radians()]
}

impl RobotState {
    /// Copy the telegram's fields in, converting to SI. Bookkeeping fields are left alone.
    pub fn update_from(&mut self, tele: &RobotTelegram, map: &StateMap) {
        self.ipoc = tele.ipoc;
        self.q = deg2rad6(&tele.aipos);
        self.q_setpoint = deg2rad6(&tele.aspos);
        self.tcp = cart_si(&tele.rist);
        self.tcp_setpoint = cart_si(&tele.rsol);
        for i in 0..6 {
            self.torque[i] = map.torque[i].map_or(0.0, |s| tele.slots[s]);
        }
        self.delay = tele.delay;
        self.speed_override = map.ov_pro.map_or(1.0, |s| (tele.slots[s] / 100.0).clamp(0.0, 1.0));
        self.program_running = map.prog_status.map_or(true, |s| tele.slots[s].round() as i64 == 3);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::EthernetConfig;
    use crate::telegram::{Parser, RobotTelegram, Schema};
    use std::f64::consts::PI;

    const SHIPPED: &str = include_str!("../controller/kuka_rsi_ethernet.xml");
    const TELEGRAM: &[u8] = br#"<Rob Type="KUKA"><RIst X="600.1234" Y="-10.5" Z="780.0" A="180.0" B="70.0" C="-180.0" /><RSol X="600.0" Y="-10.0" Z="780.0" A="180.0" B="70.0" C="-180.0" /><AIPos A1="1.5" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.25" /><ASPos A1="1.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0" /><Delay D="2" /><GearTorque A1="0.1" A2="0.2" A3="0.3" A4="0.4" A5="0.5" A6="0.6" /><ProgStatus R="3" /><OvPro R="100.0" /><IPOC>4004</IPOC></Rob>"#;

    fn shipped() -> (Schema, RobotTelegram) {
        let s = Schema::compile(&EthernetConfig::from_str(SHIPPED).unwrap()).unwrap();
        let mut t = RobotTelegram::default();
        Parser::new(&s).parse(TELEGRAM, &mut t).unwrap();
        (s, t)
    }

    #[test]
    fn unit_conversions() {
        assert!((deg2rad(180.0) - PI).abs() < 1e-12);
        assert!((rad2deg(PI / 2.0) - 90.0).abs() < 1e-12);
        assert_eq!(deg2rad6(&[0.0, 180.0, 0.0, 0.0, 0.0, 0.0])[1], PI);
        assert_eq!(rad2deg6(&[PI; 6])[5], 180.0);
    }

    #[test]
    fn update_from_converts_and_maps() {
        let (s, t) = shipped();
        let map = StateMap::from_schema(&s);
        assert_eq!(map.torque, [Some(0), Some(1), Some(2), Some(3), Some(4), Some(5)]);
        assert_eq!(map.prog_status, Some(6));
        assert_eq!(map.ov_pro, Some(7));
        let mut st = RobotState { t: 1.5, cycle: 9, session: SessionState::Active, ..Default::default() };
        st.update_from(&t, &map);
        assert_eq!(st.ipoc, 4004);
        assert!((st.q[1] + PI / 2.0).abs() < 1e-12);
        assert!((st.q[0] - deg2rad(1.5)).abs() < 1e-12);
        assert!((st.q_setpoint[0] - deg2rad(1.0)).abs() < 1e-12);
        assert!((st.tcp[0] - 0.6001234).abs() < 1e-12);
        assert!((st.tcp[3] - PI).abs() < 1e-12);
        assert!((st.tcp_setpoint[1] + 0.010).abs() < 1e-12);
        assert_eq!(st.torque, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]);
        assert_eq!(st.delay, 2);
        assert_eq!(st.speed_override, 1.0);
        assert!(st.program_running);
        assert_eq!((st.t, st.cycle, st.session), (1.5, 9, SessionState::Active), "untouched");
    }

    #[test]
    fn program_stopped_and_override_scaled() {
        let (s, mut t) = shipped();
        let map = StateMap::from_schema(&s);
        t.slots[6] = 4.0;
        t.slots[7] = 50.0;
        let mut st = RobotState::default();
        st.update_from(&t, &map);
        assert!(!st.program_running);
        assert_eq!(st.speed_override, 0.5);
    }

    #[test]
    fn unmapped_schema_defaults_to_running_full_speed() {
        let (_, t) = shipped();
        let mut st = RobotState::default();
        st.update_from(&t, &StateMap::default());
        assert!(st.program_running);
        assert_eq!(st.speed_override, 1.0);
        assert_eq!(st.torque, [0.0; 6]);
    }

    #[test]
    fn robot_state_round_trips_through_json() {
        let st = RobotState {
            t: 1.5,
            cycle: 9,
            ipoc: 4004,
            session: SessionState::Active,
            q: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            program_running: true,
            speed_override: 0.5,
            ..Default::default()
        };
        let json = serde_json::to_string(&st).unwrap();
        let back: RobotState = serde_json::from_str(&json).unwrap();
        assert_eq!(back, st);
    }
}
