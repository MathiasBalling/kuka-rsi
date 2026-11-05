//! Configuration: the controller's `RSI_EthernetConfig` XML, driver limits and settings.

use std::net::SocketAddr;
use std::path::Path;
use std::time::Duration;

use serde::{Deserialize, Serialize};

use crate::{Error, Result};

/// Data type of one telegram element, as declared in the `TYPE` attribute.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ElementType {
    Bool,
    Double,
    Long,
    String,
}

/// `INDX` attribute: a reserved keyword element or a numbered ETHERNET channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum Index {
    Internal,
    Channel(u8),
}

/// One `<ELEMENT>` of the `SEND` or `RECEIVE` section.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Element {
    pub tag: String,
    pub ty: ElementType,
    pub indx: Index,
    pub holdon: Option<bool>,
}

/// The controller's Ethernet configuration file, validated.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EthernetConfig {
    pub ip_number: String,
    pub port: u16,
    pub sentype: String,
    pub onlysend: bool,
    pub send: Vec<Element>,
    pub receive: Vec<Element>,
}

/// Keywords the driver understands on the robot-to-PC side (RSI manual 7.5.5).
pub const SEND_KEYWORDS: &[&str] = &[
    "DEF_RIst", "DEF_RSol", "DEF_AIPos", "DEF_ASPos", "DEF_EIPos", "DEF_ESPos", "DEF_MACur", "DEF_MECur", "DEF_Delay",
];
/// Keywords the driver understands on the PC-to-robot side (RSI manual 7.5.6).
pub const RECEIVE_KEYWORDS: &[&str] = &["DEF_EStr"];

#[derive(Deserialize)]
#[serde(rename = "ROOT")]
struct RawRoot {
    #[serde(rename = "CONFIG")]
    config: RawConfig,
    #[serde(rename = "SEND")]
    send: RawSection,
    #[serde(rename = "RECEIVE")]
    receive: RawSection,
}

#[derive(Deserialize)]
struct RawConfig {
    #[serde(rename = "IP_NUMBER")]
    ip_number: String,
    #[serde(rename = "PORT")]
    port: String,
    #[serde(rename = "SENTYPE")]
    sentype: String,
    #[serde(rename = "ONLYSEND", default)]
    onlysend: Option<String>,
}

#[derive(Deserialize)]
struct RawSection {
    #[serde(rename = "ELEMENTS")]
    elements: RawElements,
}

#[derive(Deserialize)]
struct RawElements {
    #[serde(rename = "ELEMENT", default)]
    element: Vec<RawElement>,
}

#[derive(Deserialize)]
struct RawElement {
    #[serde(rename = "@TAG")]
    tag: String,
    #[serde(rename = "@TYPE")]
    ty: String,
    #[serde(rename = "@INDX")]
    indx: String,
    #[serde(rename = "@HOLDON")]
    holdon: Option<String>,
}

fn cfg_err(msg: impl Into<String>) -> Error {
    Error::Config(msg.into())
}

impl RawElement {
    fn convert(self, section: &str) -> Result<Element> {
        let ty = match self.ty.trim().to_ascii_uppercase().as_str() {
            "BOOL" => ElementType::Bool,
            "DOUBLE" => ElementType::Double,
            "LONG" => ElementType::Long,
            "STRING" => ElementType::String,
            other => return Err(cfg_err(format!("{section} element {:?}: unknown TYPE {other:?}", self.tag))),
        };
        let indx = if self.indx.trim().eq_ignore_ascii_case("INTERNAL") {
            Index::Internal
        } else {
            let n: u8 = self.indx.trim().parse().map_err(|_| cfg_err(format!("{section} element {:?}: bad INDX {:?}", self.tag, self.indx)))?;
            if !(1..=64).contains(&n) {
                return Err(cfg_err(format!("{section} element {:?}: INDX {n} outside 1..=64", self.tag)));
            }
            Index::Channel(n)
        };
        let holdon = match self.holdon.as_deref().map(str::trim) {
            None => None,
            Some("0") => Some(false),
            Some("1") => Some(true),
            Some(other) => return Err(cfg_err(format!("{section} element {:?}: HOLDON must be 0 or 1, got {other:?}", self.tag))),
        };
        Ok(Element { tag: self.tag.trim().to_string(), ty, indx, holdon })
    }
}

impl EthernetConfig {
    /// Parse and validate the XML text of an `RSI_EthernetConfig` file.
    pub fn from_str(xml: &str) -> Result<Self> {
        let raw: RawRoot = quick_xml::de::from_str(xml).map_err(|e| cfg_err(format!("cannot parse Ethernet config XML: {e}")))?;
        let port: u16 = raw.config.port.trim().parse().map_err(|_| cfg_err(format!("PORT {:?} is not a number", raw.config.port)))?;
        if !(1..=65534).contains(&port) {
            return Err(cfg_err("PORT must be 1..=65534"));
        }
        let onlysend = match raw.config.onlysend.as_deref().map(|s| s.trim().to_ascii_uppercase()) {
            None => false,
            Some(s) if s == "TRUE" => true,
            Some(s) if s == "FALSE" => false,
            Some(s) => return Err(cfg_err(format!("ONLYSEND must be TRUE or FALSE, got {s:?}"))),
        };
        let send = raw.send.elements.element.into_iter().map(|e| e.convert("SEND")).collect::<Result<Vec<_>>>()?;
        let receive = raw.receive.elements.element.into_iter().map(|e| e.convert("RECEIVE")).collect::<Result<Vec<_>>>()?;
        let cfg = EthernetConfig {
            ip_number: raw.config.ip_number.trim().to_string(),
            port,
            sentype: raw.config.sentype.trim().to_string(),
            onlysend,
            send,
            receive,
        };
        cfg.validate()?;
        Ok(cfg)
    }

    /// Read and validate an `RSI_EthernetConfig` file.
    pub fn from_path(path: impl AsRef<Path>) -> Result<Self> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path).map_err(|e| cfg_err(format!("cannot read {}: {e}", path.display())))?;
        Self::from_str(&text)
    }

    /// Number of numbered channels (elements whose `INDX` is not `INTERNAL`).
    pub fn channels(elements: &[Element]) -> usize {
        elements.iter().filter(|e| matches!(e.indx, Index::Channel(_))).count()
    }

    fn validate(&self) -> Result<()> {
        if self.sentype.is_empty() {
            return Err(cfg_err("SENTYPE is empty"));
        }
        Self::validate_section("SEND", &self.send, SEND_KEYWORDS)?;
        Self::validate_section("RECEIVE", &self.receive, RECEIVE_KEYWORDS)?;
        if !self.onlysend {
            // Absolute corrections: a session takes `q_start` from `ASPos` and holds the robot at
            // `AIPos`. Without them both stay zero and the first `AK` commands a jump.
            for keyword in ["DEF_AIPos", "DEF_ASPos"] {
                if !self.send.iter().any(|e| e.indx == Index::Internal && e.tag.eq_ignore_ascii_case(keyword)) {
                    return Err(cfg_err(format!(
                        "SEND must contain {keyword} unless ONLYSEND is TRUE: absolute corrections need the axis setpoint and actual position"
                    )));
                }
            }
            for i in 1..=6 {
                let tag = format!("AK.A{i}");
                let Some(e) = self.receive.iter().find(|e| e.tag == tag) else {
                    return Err(cfg_err(format!("RECEIVE must contain {tag} unless ONLYSEND is TRUE")));
                };
                // With HOLDON="0" the robot resets a missing correction to zero, which in absolute
                // mode means "back to the start pose".
                if e.holdon != Some(true) {
                    return Err(cfg_err(format!(
                        "RECEIVE element {tag}: AK must have HOLDON=\"1\": a late reply must hold the last correction, not reset it to zero"
                    )));
                }
            }
        }
        Ok(())
    }

    fn validate_section(section: &str, elements: &[Element], keywords: &[&str]) -> Result<()> {
        let mut seen = vec![false; 65];
        for e in elements {
            match e.indx {
                Index::Internal => {
                    if e.tag.to_ascii_uppercase().starts_with("DEF_TECH") {
                        return Err(cfg_err(format!("{section}: DEF_Tech keywords are not supported by this driver ({})", e.tag)));
                    }
                    if !keywords.iter().any(|k| k.eq_ignore_ascii_case(&e.tag)) {
                        return Err(cfg_err(format!("{section}: unknown INTERNAL keyword {:?}", e.tag)));
                    }
                }
                Index::Channel(n) => {
                    if e.ty == ElementType::String {
                        return Err(cfg_err(format!("{section} element {:?}: TYPE STRING is only valid for the DEF_EStr keyword", e.tag)));
                    }
                    if seen[n as usize] {
                        return Err(cfg_err(format!("{section}: INDX {n} used twice")));
                    }
                    seen[n as usize] = true;
                }
            }
        }
        let count = Self::channels(elements);
        if (1..=count).any(|n| !seen[n]) {
            return Err(cfg_err(format!("{section}: channel INDX values must be consecutive from 1 to {count}")));
        }
        Ok(())
    }
}

const AXIS_NAMES: [&str; 6] = ["A1", "A2", "A3", "A4", "A5", "A6"];

/// Driver-side limits in SI units. Enforced by the limiter every cycle.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Limits {
    /// Lower joint bound, rad.
    pub position_min: [f64; 6],
    /// Upper joint bound, rad.
    pub position_max: [f64; 6],
    /// Per-joint speed bound, rad/s.
    pub max_velocity: [f64; 6],
    /// Per-joint acceleration bound, rad/s²; `None` disables the acceleration clamp.
    pub max_acceleration: Option<[f64; 6]>,
    /// Bound on `|q - q_start|`, rad; mirrors the `AxisCorr` limits in the RSI context.
    pub max_correction: [f64; 6],
    /// Warning threshold for `|q_actual - q_command|`, rad.
    pub max_tracking_error: f64,
}

fn all_deg(v: f64) -> [f64; 6] {
    [v.to_radians(); 6]
}

impl Limits {
    /// KR 10 R1100-2 axis ranges from the data sheet (A1 ±170°, A2 −190°/+45°, A3 −120°/+156°,
    /// A4 ±185°, A5 ±120°, A6 ±350°) with bring-up speeds: 30°/s, 300°/s². Verify the ranges
    /// against `$SOFTN_END` / `$SOFTP_END` on the pendant before the first motion.
    pub fn kr10_defaults() -> Limits {
        let position_min = [-170f64, -190.0, -120.0, -185.0, -120.0, -350.0].map(f64::to_radians);
        let position_max = [170f64, 45.0, 156.0, 185.0, 120.0, 350.0].map(f64::to_radians);
        let mut max_correction = [0.0; 6];
        for i in 0..6 {
            max_correction[i] = position_max[i] - position_min[i];
        }
        Limits {
            position_min,
            position_max,
            max_velocity: all_deg(30.0),
            max_acceleration: Some(all_deg(300.0)),
            max_correction,
            max_tracking_error: 2f64.to_radians(),
        }
    }

    pub fn with_max_velocity_all(mut self, v: f64) -> Limits {
        self.max_velocity = [v; 6];
        self
    }

    pub fn with_max_acceleration_all(mut self, a: Option<f64>) -> Limits {
        self.max_acceleration = a.map(|a| [a; 6]);
        self
    }

    pub fn validate(&self) -> Result<()> {
        for i in 0..6 {
            let n = AXIS_NAMES[i];
            if !(self.position_min[i].is_finite() && self.position_max[i].is_finite()) || self.position_min[i] >= self.position_max[i] {
                return Err(cfg_err(format!("limits: position_min must be below position_max for {n}")));
            }
            if !(self.max_velocity[i] > 0.0) || !self.max_velocity[i].is_finite() {
                return Err(cfg_err(format!("limits: max_velocity must be positive and finite for {n}")));
            }
            if let Some(a) = self.max_acceleration {
                if !(a[i] > 0.0) || !a[i].is_finite() {
                    return Err(cfg_err(format!("limits: max_acceleration must be positive and finite for {n}")));
                }
            }
            if !(self.max_correction[i] > 0.0) || !self.max_correction[i].is_finite() {
                return Err(cfg_err(format!("limits: max_correction must be positive and finite for {n}")));
            }
        }
        if !(self.max_tracking_error > 0.0) || !self.max_tracking_error.is_finite() {
            return Err(cfg_err("limits: max_tracking_error must be positive and finite"));
        }
        Ok(())
    }

    /// Reject a target outside the position bounds or with a non-finite component.
    pub fn check_position(&self, q: &[f64; 6]) -> Result<()> {
        for i in 0..6 {
            if !q[i].is_finite() {
                return Err(Error::Limit(format!("{} target is not finite", AXIS_NAMES[i])));
            }
            if q[i] < self.position_min[i] || q[i] > self.position_max[i] {
                return Err(Error::Limit(format!(
                    "{} target {:.4} rad outside [{:.4}, {:.4}]",
                    AXIS_NAMES[i], q[i], self.position_min[i], self.position_max[i]
                )));
            }
        }
        Ok(())
    }
}

/// Everything the driver needs to start.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DriverConfig {
    pub ethernet: EthernetConfig,
    /// Override of the bind address; default is `IP_NUMBER:PORT` from the Ethernet config.
    pub bind: Option<SocketAddr>,
    pub cycle: Duration,
    /// Silence after which an active session is declared lost.
    pub link_timeout: Duration,
    /// Try to raise the real-time thread's scheduling priority.
    pub rt_priority: bool,
    /// Receive element (text, BOOL) wired to the `Stop` object, e.g. `MoveStop`.
    pub stop_channel: Option<String>,
    pub limits: Limits,
}

impl DriverConfig {
    pub fn new(ethernet: EthernetConfig) -> DriverConfig {
        DriverConfig {
            ethernet,
            bind: None,
            cycle: Duration::from_millis(4),
            link_timeout: Duration::from_millis(500),
            rt_priority: true,
            stop_channel: None,
            limits: Limits::kr10_defaults(),
        }
    }

    pub fn from_ethernet_xml(path: impl AsRef<Path>) -> Result<DriverConfig> {
        Ok(DriverConfig::new(EthernetConfig::from_path(path)?))
    }

    pub fn bind(mut self, addr: SocketAddr) -> Self {
        self.bind = Some(addr);
        self
    }

    pub fn limits(mut self, limits: Limits) -> Self {
        self.limits = limits;
        self
    }

    pub fn stop_channel(mut self, tag: &str) -> Self {
        self.stop_channel = Some(tag.to_string());
        self
    }

    pub fn link_timeout(mut self, d: Duration) -> Self {
        self.link_timeout = d;
        self
    }

    pub fn rt_priority(mut self, on: bool) -> Self {
        self.rt_priority = on;
        self
    }

    pub fn cycle(mut self, d: Duration) -> Self {
        self.cycle = d;
        self
    }

    /// The address to bind: the override, or `IP_NUMBER:PORT` from the Ethernet config.
    pub fn bind_addr(&self) -> Result<SocketAddr> {
        if let Some(a) = self.bind {
            return Ok(a);
        }
        let ip: std::net::IpAddr = self
            .ethernet
            .ip_number
            .parse()
            .map_err(|_| cfg_err(format!("IP_NUMBER {:?} is not an IP address", self.ethernet.ip_number)))?;
        Ok(SocketAddr::new(ip, self.ethernet.port))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SHIPPED: &str = include_str!("../controller/kuka_rsi_ethernet.xml");
    const MONITOR: &str = include_str!("../controller/kuka_rsi_monitor.xml");

    #[test]
    fn parses_shipped_config() {
        let c = EthernetConfig::from_str(SHIPPED).unwrap();
        assert_eq!(c.ip_number, "192.168.10.10");
        assert_eq!(c.port, 49152);
        assert_eq!(c.sentype, "KukaRsi");
        assert!(!c.onlysend);
        assert_eq!(c.send.len(), 13);
        assert_eq!(c.receive.len(), 8);
        assert_eq!(c.send[0].tag, "DEF_RIst");
        assert_eq!(c.send[0].indx, Index::Internal);
        assert_eq!(c.send[5], Element { tag: "GearTorque.A1".into(), ty: ElementType::Double, indx: Index::Channel(1), holdon: None });
        assert_eq!(c.receive[0].holdon, Some(true));
        assert_eq!(c.receive[6], Element { tag: "MoveStop".into(), ty: ElementType::Bool, indx: Index::Channel(7), holdon: Some(false) });
        assert_eq!(EthernetConfig::channels(&c.send), 8);
        assert_eq!(EthernetConfig::channels(&c.receive), 7);
    }

    #[test]
    fn monitor_config_is_onlysend() {
        let c = EthernetConfig::from_str(MONITOR).unwrap();
        assert!(c.onlysend);
        assert_eq!(c.receive.len(), 8);
    }

    fn with_receive(receive: &str) -> String {
        format!(
            "<ROOT><CONFIG><IP_NUMBER>127.0.0.1</IP_NUMBER><PORT>49152</PORT><SENTYPE>T</SENTYPE><ONLYSEND>FALSE</ONLYSEND></CONFIG>\
             <SEND><ELEMENTS>{POSITIONS}</ELEMENTS></SEND>\
             <RECEIVE><ELEMENTS>{receive}</ELEMENTS></RECEIVE></ROOT>"
        )
    }

    const POSITIONS: &str = "<ELEMENT TAG=\"DEF_AIPos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" /><ELEMENT TAG=\"DEF_ASPos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" />";

    fn config(onlysend: bool, send: &str, receive: &str) -> String {
        let onlysend = if onlysend { "TRUE" } else { "FALSE" };
        format!(
            "<ROOT><CONFIG><IP_NUMBER>127.0.0.1</IP_NUMBER><PORT>49152</PORT><SENTYPE>T</SENTYPE><ONLYSEND>{onlysend}</ONLYSEND></CONFIG>\
             <SEND><ELEMENTS>{send}</ELEMENTS></SEND><RECEIVE><ELEMENTS>{receive}</ELEMENTS></RECEIVE></ROOT>"
        )
    }

    const AK: &str = "<ELEMENT TAG=\"AK.A1\" TYPE=\"DOUBLE\" INDX=\"1\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A2\" TYPE=\"DOUBLE\" INDX=\"2\" HOLDON=\"1\" />\
        <ELEMENT TAG=\"AK.A3\" TYPE=\"DOUBLE\" INDX=\"3\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A4\" TYPE=\"DOUBLE\" INDX=\"4\" HOLDON=\"1\" />\
        <ELEMENT TAG=\"AK.A5\" TYPE=\"DOUBLE\" INDX=\"5\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A6\" TYPE=\"DOUBLE\" INDX=\"6\" HOLDON=\"1\" />";

    #[test]
    fn absolute_mode_requires_axis_positions() {
        EthernetConfig::from_str(&config(false, POSITIONS, AK)).unwrap();
        // Keywords match case-insensitively, like the keyword lists.
        let lower = "<ELEMENT TAG=\"def_aipos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" /><ELEMENT TAG=\"DEF_ASPOS\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" />";
        EthernetConfig::from_str(&config(false, lower, AK)).unwrap();
        for (send, missing) in [
            ("<ELEMENT TAG=\"DEF_AIPos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" />", "DEF_ASPos"),
            ("<ELEMENT TAG=\"DEF_ASPos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" />", "DEF_AIPos"),
            ("<ELEMENT TAG=\"DEF_RIst\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" />", "DEF_AIPos"),
        ] {
            let err = EthernetConfig::from_str(&config(false, send, AK)).unwrap_err().to_string();
            assert!(err.contains(missing), "{err}");
            assert!(err.contains("absolute corrections need the axis setpoint and actual position"), "{err}");
        }
    }

    #[test]
    fn ak_must_hold_on_late_replies() {
        for holdon in [" HOLDON=\"0\"", ""] {
            let ak = AK.replace("INDX=\"3\" HOLDON=\"1\"", &format!("INDX=\"3\"{holdon}"));
            let err = EthernetConfig::from_str(&config(false, POSITIONS, &ak)).unwrap_err().to_string();
            assert!(err.contains("AK.A3"), "{err}");
            assert!(err.contains("AK must have HOLDON=\"1\""), "{err}");
        }
    }

    #[test]
    fn onlysend_configs_are_exempt_from_the_absolute_mode_rules() {
        let ak_reset = AK.replace("HOLDON=\"1\"", "HOLDON=\"0\"");
        let c = EthernetConfig::from_str(&config(true, "<ELEMENT TAG=\"DEF_RIst\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" />", &ak_reset)).unwrap();
        assert!(c.onlysend);
        EthernetConfig::from_str(&config(true, "", "")).unwrap();
    }

    #[test]
    fn rejects_non_consecutive_indices() {
        let xml = with_receive(&format!("{AK}<ELEMENT TAG=\"X\" TYPE=\"LONG\" INDX=\"9\" />"));
        let err = EthernetConfig::from_str(&xml).unwrap_err().to_string();
        assert!(err.contains("consecutive"), "{err}");
    }

    #[test]
    fn rejects_unknown_type_and_keyword() {
        let xml = with_receive(&format!("{AK}<ELEMENT TAG=\"X\" TYPE=\"FLOAT\" INDX=\"7\" />"));
        assert!(EthernetConfig::from_str(&xml).unwrap_err().to_string().contains("TYPE"));
        let xml = with_receive(&format!("{AK}<ELEMENT TAG=\"DEF_Tech.T2\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" />"));
        assert!(EthernetConfig::from_str(&xml).unwrap_err().to_string().contains("DEF_Tech"));
    }

    #[test]
    fn rejects_string_typed_channels() {
        let xml = with_receive(&format!("{AK}<ELEMENT TAG=\"Msg\" TYPE=\"STRING\" INDX=\"7\" />"));
        let err = EthernetConfig::from_str(&xml).unwrap_err().to_string();
        assert!(err.contains("STRING"), "{err}");
        let xml = "<ROOT><CONFIG><IP_NUMBER>127.0.0.1</IP_NUMBER><PORT>49152</PORT><SENTYPE>T</SENTYPE><ONLYSEND>TRUE</ONLYSEND></CONFIG>\
                   <SEND><ELEMENTS><ELEMENT TAG=\"Name\" TYPE=\"STRING\" INDX=\"1\" /></ELEMENTS></SEND><RECEIVE><ELEMENTS></ELEMENTS></RECEIVE></ROOT>";
        assert!(EthernetConfig::from_str(xml).unwrap_err().to_string().contains("STRING"));
    }

    #[test]
    fn rejects_bad_port_and_onlysend_values() {
        let xml = "<ROOT><CONFIG><IP_NUMBER>1.2.3.4</IP_NUMBER><PORT>0</PORT><SENTYPE>T</SENTYPE><ONLYSEND>MAYBE</ONLYSEND></CONFIG>\
                   <SEND><ELEMENTS></ELEMENTS></SEND><RECEIVE><ELEMENTS></ELEMENTS></RECEIVE></ROOT>";
        let err = EthernetConfig::from_str(xml).unwrap_err().to_string();
        assert!(err.contains("PORT") || err.contains("ONLYSEND"), "{err}");
    }

    #[test]
    fn rejects_port_above_range() {
        let xml = "<ROOT><CONFIG><IP_NUMBER>1.2.3.4</IP_NUMBER><PORT>65535</PORT><SENTYPE>T</SENTYPE><ONLYSEND>TRUE</ONLYSEND></CONFIG>\
                   <SEND><ELEMENTS></ELEMENTS></SEND><RECEIVE><ELEMENTS></ELEMENTS></RECEIVE></ROOT>";
        let err = EthernetConfig::from_str(xml).unwrap_err().to_string();
        assert!(err.contains("PORT"), "{err}");
    }

    #[test]
    fn from_path_reads_file() {
        let dir = std::env::temp_dir().join(format!("kuka-rsi-cfg-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("c.xml");
        std::fs::write(&p, SHIPPED).unwrap();
        assert_eq!(EthernetConfig::from_path(&p).unwrap().port, 49152);
        assert!(EthernetConfig::from_path(dir.join("missing.xml")).is_err());
    }

    #[test]
    fn kr10_defaults_are_valid_and_conservative() {
        let l = Limits::kr10_defaults();
        l.validate().unwrap();
        assert!((l.position_max[0] - 170f64.to_radians()).abs() < 1e-12);
        assert!((l.position_min[1] + 190f64.to_radians()).abs() < 1e-12);
        assert!(l.max_velocity.iter().all(|v| (*v - 30f64.to_radians()).abs() < 1e-12));
        assert_eq!(l.max_acceleration.map(|a| a[0]), Some(300f64.to_radians()));
        for i in 0..6 {
            assert!((l.max_correction[i] - (l.position_max[i] - l.position_min[i])).abs() < 1e-12);
        }
        let faster = l.with_max_velocity_all(1.0);
        assert_eq!(faster.max_velocity, [1.0; 6]);
        assert_eq!(l.with_max_acceleration_all(None).max_acceleration, None);
    }

    #[test]
    fn limits_validation_rejects_nonsense() {
        let mut l = Limits::kr10_defaults();
        l.position_min[2] = l.position_max[2] + 0.1;
        assert!(l.validate().unwrap_err().to_string().contains("position"));
        let mut l = Limits::kr10_defaults();
        l.max_velocity[0] = 0.0;
        assert!(l.validate().unwrap_err().to_string().contains("velocity"));
        let mut l = Limits::kr10_defaults();
        l.max_correction[5] = f64::NAN;
        assert!(l.validate().is_err());
        let mut l = Limits::kr10_defaults();
        l.max_tracking_error = -1.0;
        assert!(l.validate().is_err());
        let mut l = Limits::kr10_defaults();
        l.max_acceleration = Some([0.0; 6]);
        assert!(l.validate().unwrap_err().to_string().contains("acceleration"));
    }

    #[test]
    fn check_position_bounds_and_finiteness() {
        let l = Limits::kr10_defaults();
        l.check_position(&[0.0; 6]).unwrap();
        let mut q = [0.0; 6];
        q[0] = 4.0;
        assert!(l.check_position(&q).unwrap_err().to_string().contains("A1"));
        q[0] = f64::INFINITY;
        assert!(l.check_position(&q).is_err());
    }

    #[test]
    fn driver_config_defaults_and_bind_addr() {
        let dir = std::env::temp_dir().join(format!("kuka-rsi-drv-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("c.xml");
        std::fs::write(&p, SHIPPED).unwrap();
        let c = DriverConfig::from_ethernet_xml(&p).unwrap();
        assert_eq!(c.cycle, Duration::from_millis(4));
        assert_eq!(c.link_timeout, Duration::from_millis(500));
        assert!(c.rt_priority);
        assert_eq!(c.stop_channel, None);
        assert_eq!(c.bind_addr().unwrap().to_string(), "192.168.10.10:49152");
        let c = c.bind("127.0.0.1:0".parse().unwrap()).stop_channel("MoveStop").link_timeout(Duration::from_millis(250)).rt_priority(false);
        assert_eq!(c.bind_addr().unwrap().to_string(), "127.0.0.1:0");
        assert_eq!(c.stop_channel.as_deref(), Some("MoveStop"));
        assert!(!c.rt_priority);
    }

    #[test]
    fn bind_addr_rejects_bad_ip_in_config() {
        let mut e = EthernetConfig::from_str(SHIPPED).unwrap();
        e.ip_number = "not-an-ip".into();
        let err = DriverConfig::new(e).bind_addr().unwrap_err().to_string();
        assert!(err.contains("IP_NUMBER"), "{err}");
    }
}
