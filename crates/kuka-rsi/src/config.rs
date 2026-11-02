//! Configuration: the controller's `RSI_EthernetConfig` XML, driver limits and settings.

use std::path::Path;

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
        if port == 0 {
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
            for i in 1..=6 {
                let tag = format!("AK.A{i}");
                if !self.receive.iter().any(|e| e.tag == tag) {
                    return Err(cfg_err(format!("RECEIVE must contain {tag} unless ONLYSEND is TRUE")));
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

#[cfg(test)]
mod tests {
    use super::*;

    const SHIPPED: &str = include_str!("../../../controller/kuka_rsi_ethernet.xml");
    const MONITOR: &str = include_str!("../../../controller/kuka_rsi_monitor.xml");

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
             <SEND><ELEMENTS><ELEMENT TAG=\"DEF_AIPos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" /></ELEMENTS></SEND>\
             <RECEIVE><ELEMENTS>{receive}</ELEMENTS></RECEIVE></ROOT>"
        )
    }

    const AK: &str = "<ELEMENT TAG=\"AK.A1\" TYPE=\"DOUBLE\" INDX=\"1\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A2\" TYPE=\"DOUBLE\" INDX=\"2\" HOLDON=\"1\" />\
        <ELEMENT TAG=\"AK.A3\" TYPE=\"DOUBLE\" INDX=\"3\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A4\" TYPE=\"DOUBLE\" INDX=\"4\" HOLDON=\"1\" />\
        <ELEMENT TAG=\"AK.A5\" TYPE=\"DOUBLE\" INDX=\"5\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A6\" TYPE=\"DOUBLE\" INDX=\"6\" HOLDON=\"1\" />";

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
    fn rejects_bad_port_and_onlysend_values() {
        let xml = "<ROOT><CONFIG><IP_NUMBER>1.2.3.4</IP_NUMBER><PORT>0</PORT><SENTYPE>T</SENTYPE><ONLYSEND>MAYBE</ONLYSEND></CONFIG>\
                   <SEND><ELEMENTS></ELEMENTS></SEND><RECEIVE><ELEMENTS></ELEMENTS></RECEIVE></ROOT>";
        let err = EthernetConfig::from_str(xml).unwrap_err().to_string();
        assert!(err.contains("PORT") || err.contains("ONLYSEND"), "{err}");
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
}
