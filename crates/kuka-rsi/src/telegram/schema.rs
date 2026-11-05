use crate::config::{Element, ElementType, EthernetConfig, Index};
use crate::{Error, Result};

/// Maximum ETHERNET channels per direction (RSI manual 2.2.3).
pub const MAX_SLOTS: usize = 64;

const AXES: &[&str] = &["A1", "A2", "A3", "A4", "A5", "A6"];
const EXT: &[&str] = &["E1", "E2", "E3", "E4", "E5", "E6"];
const CART: &[&str] = &["X", "Y", "Z", "A", "B", "C"];

/// Reserved robot-to-PC keywords (config tag `DEF_<name>`), see RSI manual 7.5.5.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Keyword {
    RIst,
    RSol,
    AIPos,
    ASPos,
    EIPos,
    ESPos,
    MACur,
    MECur,
    Delay,
}

impl Keyword {
    fn from_tag(tag: &str) -> Option<Keyword> {
        let t = tag.to_ascii_uppercase();
        Some(match t.as_str() {
            "DEF_RIST" => Keyword::RIst,
            "DEF_RSOL" => Keyword::RSol,
            "DEF_AIPOS" => Keyword::AIPos,
            "DEF_ASPOS" => Keyword::ASPos,
            "DEF_EIPOS" => Keyword::EIPos,
            "DEF_ESPOS" => Keyword::ESPos,
            "DEF_MACUR" => Keyword::MACur,
            "DEF_MECUR" => Keyword::MECur,
            "DEF_DELAY" => Keyword::Delay,
            _ => return None,
        })
    }

    /// Element name the robot uses on the wire.
    pub fn tag(self) -> &'static str {
        match self {
            Keyword::RIst => "RIst",
            Keyword::RSol => "RSol",
            Keyword::AIPos => "AIPos",
            Keyword::ASPos => "ASPos",
            Keyword::EIPos => "EIPos",
            Keyword::ESPos => "ESPos",
            Keyword::MACur => "MACur",
            Keyword::MECur => "MECur",
            Keyword::Delay => "Delay",
        }
    }

    /// Attribute names in wire order.
    pub fn attrs(self) -> &'static [&'static str] {
        match self {
            Keyword::RIst | Keyword::RSol => CART,
            Keyword::AIPos | Keyword::ASPos | Keyword::MACur => AXES,
            Keyword::EIPos | Keyword::ESPos | Keyword::MECur => EXT,
            Keyword::Delay => &["D"],
        }
    }
}

/// One attribute of a grouped user element, bound to a channel slot.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Attr {
    pub name: String,
    pub slot: usize,
    pub ty: ElementType,
}

/// An element of the robot-to-PC telegram.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RobElement {
    Keyword(Keyword),
    Attrs { tag: String, attrs: Vec<Attr> },
    Text { tag: String, slot: usize, ty: ElementType },
}

/// An element of the PC-to-robot telegram.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SenElement {
    Ak,
    RKorr,
    Ek,
    EStr,
    Attrs { tag: String, attrs: Vec<Attr> },
    Text { tag: String, slot: usize, ty: ElementType },
}

/// Telegram layout in both directions, compiled once from the Ethernet config.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Schema {
    pub sentype: String,
    pub onlysend: bool,
    pub rob: Vec<RobElement>,
    pub sen: Vec<SenElement>,
}

fn split_tag(tag: &str) -> (&str, Option<&str>) {
    match tag.split_once('.') {
        Some((name, attr)) => (name, Some(attr)),
        None => (tag, None),
    }
}

/// An element name, its attributes, and the type of a text element.
type Group = (String, Vec<Attr>, Option<ElementType>);

/// Group consecutive `Name.Attr` elements into `(name, attrs)`, keep text elements as `(name, [])`.
fn group(elements: &[Element]) -> Result<Vec<Group>> {
    let mut out: Vec<Group> = Vec::new();
    for e in elements {
        let Index::Channel(n) = e.indx else { continue };
        let slot = n as usize - 1;
        let (name, attr) = split_tag(&e.tag);
        match attr {
            Some(a) => {
                if let Some(last) = out.last_mut().filter(|g| g.0 == name && g.2.is_none()) {
                    last.1.push(Attr { name: a.to_string(), slot, ty: e.ty });
                } else {
                    if out.iter().any(|g| g.0 == name) {
                        return Err(Error::Config(format!("elements of {name} must be consecutive in the config")));
                    }
                    out.push((name.to_string(), vec![Attr { name: a.to_string(), slot, ty: e.ty }], None));
                }
            }
            None => {
                if out.iter().any(|g| g.0 == name) {
                    return Err(Error::Config(format!("tag {name} used both as text and grouped element")));
                }
                out.push((name.to_string(), vec![Attr { name: String::new(), slot, ty: e.ty }], Some(e.ty)));
            }
        }
    }
    Ok(out)
}

fn expect_group(name: &str, attrs: &[Attr], want: &[&str]) -> Result<()> {
    let got: Vec<&str> = attrs.iter().map(|a| a.name.as_str()).collect();
    if got != want {
        return Err(Error::Config(format!("{name} must have attributes {want:?} in order, got {got:?}")));
    }
    Ok(())
}

impl Schema {
    /// Compile the telegram layout. Fails on layouts the driver cannot handle.
    pub fn compile(cfg: &EthernetConfig) -> Result<Schema> {
        let mut rob = Vec::new();
        let mut rob_groups = group(&cfg.send)?.into_iter();
        for e in &cfg.send {
            match e.indx {
                Index::Internal => {
                    let kw = Keyword::from_tag(&e.tag)
                        .ok_or_else(|| Error::Config(format!("unknown SEND keyword {:?}", e.tag)))?;
                    rob.push(RobElement::Keyword(kw));
                }
                Index::Channel(_) => {
                    let (name, attr) = split_tag(&e.tag);
                    // Emit the group once, when its first element is met.
                    let first_of_group = match attr {
                        None => true,
                        Some(_) => !rob.iter().any(|r| matches!(r, RobElement::Attrs { tag, .. } if tag == name)),
                    };
                    if first_of_group {
                        let (tag, attrs, text_ty) = rob_groups.next().expect("group count matches");
                        match text_ty {
                            Some(ty) => rob.push(RobElement::Text { tag, slot: attrs[0].slot, ty }),
                            None => rob.push(RobElement::Attrs { tag, attrs }),
                        }
                    }
                }
            }
        }

        let mut sen = Vec::new();
        let mut sen_groups = group(&cfg.receive)?.into_iter();
        for e in &cfg.receive {
            match e.indx {
                Index::Internal => {
                    if e.tag.eq_ignore_ascii_case("DEF_EStr") {
                        sen.push(SenElement::EStr);
                    } else {
                        return Err(Error::Config(format!("unknown RECEIVE keyword {:?}", e.tag)));
                    }
                }
                Index::Channel(_) => {
                    let (name, attr) = split_tag(&e.tag);
                    let first_of_group = match attr {
                        None => true,
                        Some(_) => !sen.iter().any(|s| {
                            matches!(s, SenElement::Attrs { tag, .. } if tag == name)
                                || matches!(
                                    (s, name),
                                    (SenElement::Ak, "AK") | (SenElement::RKorr, "RKorr") | (SenElement::Ek, "EK")
                                )
                        }),
                    };
                    if first_of_group {
                        let (tag, attrs, text_ty) = sen_groups.next().expect("group count matches");
                        match (tag.as_str(), text_ty) {
                            ("AK", None) => {
                                expect_group("AK", &attrs, AXES)?;
                                sen.push(SenElement::Ak);
                            }
                            ("RKorr", None) => {
                                expect_group("RKorr", &attrs, CART)?;
                                sen.push(SenElement::RKorr);
                            }
                            ("EK", None) => {
                                expect_group("EK", &attrs, EXT)?;
                                sen.push(SenElement::Ek);
                            }
                            (_, Some(ty)) => sen.push(SenElement::Text { tag, slot: attrs[0].slot, ty }),
                            (_, None) => sen.push(SenElement::Attrs { tag, attrs }),
                        }
                    }
                }
            }
        }

        Ok(Schema { sentype: cfg.sentype.clone(), onlysend: cfg.onlysend, rob, sen })
    }

    /// Slot of a robot-to-PC user element (`attr` is `None` for text elements).
    pub fn rob_slot(&self, tag: &str, attr: Option<&str>) -> Option<usize> {
        self.rob.iter().find_map(|e| match (e, attr) {
            (RobElement::Attrs { tag: t, attrs }, Some(a)) if t == tag => {
                attrs.iter().find(|x| x.name == a).map(|x| x.slot)
            }
            (RobElement::Text { tag: t, slot, .. }, None) if t == tag => Some(*slot),
            _ => None,
        })
    }

    /// Slot of a PC-to-robot user element (`attr` is `None` for text elements).
    pub fn sen_slot(&self, tag: &str, attr: Option<&str>) -> Option<usize> {
        self.sen.iter().find_map(|e| match (e, attr) {
            (SenElement::Attrs { tag: t, attrs }, Some(a)) if t == tag => {
                attrs.iter().find(|x| x.name == a).map(|x| x.slot)
            }
            (SenElement::Text { tag: t, slot, .. }, None) if t == tag => Some(*slot),
            _ => None,
        })
    }

    /// True when the reply carries `AK` (joint corrections).
    pub fn has_ak(&self) -> bool {
        self.sen.iter().any(|e| matches!(e, SenElement::Ak))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::EthernetConfig;

    const SHIPPED: &str = include_str!("../../controller/kuka_rsi_ethernet.xml");

    fn cfg(receive: &str, send: &str) -> EthernetConfig {
        let xml = format!(
            "<ROOT><CONFIG><IP_NUMBER>127.0.0.1</IP_NUMBER><PORT>49152</PORT><SENTYPE>T</SENTYPE><ONLYSEND>FALSE</ONLYSEND></CONFIG>\
             <SEND><ELEMENTS>{send}</ELEMENTS></SEND><RECEIVE><ELEMENTS>{receive}</ELEMENTS></RECEIVE></ROOT>"
        );
        EthernetConfig::from_str(&xml).unwrap()
    }

    const POSITIONS: &str = "<ELEMENT TAG=\"DEF_AIPos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" /><ELEMENT TAG=\"DEF_ASPos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" />";

    const AK: &str = "<ELEMENT TAG=\"AK.A1\" TYPE=\"DOUBLE\" INDX=\"1\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A2\" TYPE=\"DOUBLE\" INDX=\"2\" HOLDON=\"1\" />\
        <ELEMENT TAG=\"AK.A3\" TYPE=\"DOUBLE\" INDX=\"3\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A4\" TYPE=\"DOUBLE\" INDX=\"4\" HOLDON=\"1\" />\
        <ELEMENT TAG=\"AK.A5\" TYPE=\"DOUBLE\" INDX=\"5\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A6\" TYPE=\"DOUBLE\" INDX=\"6\" HOLDON=\"1\" />";

    #[test]
    fn compiles_shipped_config() {
        let s = Schema::compile(&EthernetConfig::from_str(SHIPPED).unwrap()).unwrap();
        assert_eq!(s.sentype, "KukaRsi");
        assert!(!s.onlysend);
        assert_eq!(s.rob.len(), 8, "{:?}", s.rob);
        assert!(matches!(s.rob[0], RobElement::Keyword(Keyword::RIst)));
        assert!(matches!(s.rob[4], RobElement::Keyword(Keyword::Delay)));
        match &s.rob[5] {
            RobElement::Attrs { tag, attrs } => {
                assert_eq!(tag, "GearTorque");
                assert_eq!(attrs.len(), 6);
                assert_eq!(attrs[2].name, "A3");
                assert_eq!(attrs[2].slot, 2);
                assert_eq!(attrs[2].ty, ElementType::Double);
            }
            other => panic!("{other:?}"),
        }
        match &s.rob[6] {
            RobElement::Attrs { tag, attrs } => {
                assert_eq!(tag, "ProgStatus");
                assert_eq!(attrs[0].name, "R");
                assert_eq!(attrs[0].slot, 6);
                assert_eq!(attrs[0].ty, ElementType::Long);
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(s.rob_slot("OvPro", Some("R")), Some(7));
        assert_eq!(s.rob_slot("GearTorque", Some("A3")), Some(2));
        assert_eq!(s.rob_slot("Nope", None), None);

        assert_eq!(s.sen.len(), 3, "{:?}", s.sen);
        assert!(matches!(s.sen[0], SenElement::Ak));
        assert!(matches!(&s.sen[1], SenElement::Text { tag, slot: 6, ty: ElementType::Bool } if tag == "MoveStop"));
        assert!(matches!(s.sen[2], SenElement::EStr));
        assert_eq!(s.sen_slot("MoveStop", None), Some(6));
        assert!(s.has_ak());
    }

    #[test]
    fn groups_rkorr_and_ek_and_user_attrs() {
        let receive = format!(
            "{AK}<ELEMENT TAG=\"RKorr.X\" TYPE=\"DOUBLE\" INDX=\"7\" /><ELEMENT TAG=\"RKorr.Y\" TYPE=\"DOUBLE\" INDX=\"8\" />\
             <ELEMENT TAG=\"RKorr.Z\" TYPE=\"DOUBLE\" INDX=\"9\" /><ELEMENT TAG=\"RKorr.A\" TYPE=\"DOUBLE\" INDX=\"10\" />\
             <ELEMENT TAG=\"RKorr.B\" TYPE=\"DOUBLE\" INDX=\"11\" /><ELEMENT TAG=\"RKorr.C\" TYPE=\"DOUBLE\" INDX=\"12\" />\
             <ELEMENT TAG=\"Digout.o1\" TYPE=\"BOOL\" INDX=\"13\" /><ELEMENT TAG=\"Digout.o2\" TYPE=\"BOOL\" INDX=\"14\" />\
             <ELEMENT TAG=\"DiO\" TYPE=\"LONG\" INDX=\"15\" />"
        );
        let s = Schema::compile(&cfg(&receive, POSITIONS)).unwrap();
        assert!(matches!(s.sen[0], SenElement::Ak));
        assert!(matches!(s.sen[1], SenElement::RKorr));
        assert!(
            matches!(&s.sen[2], SenElement::Attrs { tag, attrs } if tag == "Digout" && attrs.len() == 2 && attrs[1].slot == 13)
        );
        assert!(matches!(&s.sen[3], SenElement::Text { tag, slot: 14, ty: ElementType::Long } if tag == "DiO"));
        assert_eq!(s.sen_slot("Digout", Some("o2")), Some(13));
    }

    #[test]
    fn groups_ek_and_rejects_out_of_order_ek() {
        let ek = "<ELEMENT TAG=\"EK.E1\" TYPE=\"DOUBLE\" INDX=\"7\" /><ELEMENT TAG=\"EK.E2\" TYPE=\"DOUBLE\" INDX=\"8\" />\
                  <ELEMENT TAG=\"EK.E3\" TYPE=\"DOUBLE\" INDX=\"9\" /><ELEMENT TAG=\"EK.E4\" TYPE=\"DOUBLE\" INDX=\"10\" />\
                  <ELEMENT TAG=\"EK.E5\" TYPE=\"DOUBLE\" INDX=\"11\" /><ELEMENT TAG=\"EK.E6\" TYPE=\"DOUBLE\" INDX=\"12\" />";
        let s = Schema::compile(&cfg(&format!("{AK}{ek}"), POSITIONS)).unwrap();
        assert!(matches!(s.sen[0], SenElement::Ak));
        assert!(matches!(s.sen[1], SenElement::Ek));
        assert_eq!(s.sen.len(), 2);

        let ek_swapped = "<ELEMENT TAG=\"EK.E2\" TYPE=\"DOUBLE\" INDX=\"7\" /><ELEMENT TAG=\"EK.E1\" TYPE=\"DOUBLE\" INDX=\"8\" />\
                  <ELEMENT TAG=\"EK.E3\" TYPE=\"DOUBLE\" INDX=\"9\" /><ELEMENT TAG=\"EK.E4\" TYPE=\"DOUBLE\" INDX=\"10\" />\
                  <ELEMENT TAG=\"EK.E5\" TYPE=\"DOUBLE\" INDX=\"11\" /><ELEMENT TAG=\"EK.E6\" TYPE=\"DOUBLE\" INDX=\"12\" />";
        let err = Schema::compile(&cfg(&format!("{AK}{ek_swapped}"), POSITIONS)).unwrap_err().to_string();
        assert!(err.contains("EK"), "{err}");
    }

    #[test]
    fn rejects_partial_ak_group() {
        let receive =
            "<ELEMENT TAG=\"AK.A1\" TYPE=\"DOUBLE\" INDX=\"1\" /><ELEMENT TAG=\"AK.A2\" TYPE=\"DOUBLE\" INDX=\"2\" />";
        let xml = format!(
            "<ROOT><CONFIG><IP_NUMBER>127.0.0.1</IP_NUMBER><PORT>49152</PORT><SENTYPE>T</SENTYPE><ONLYSEND>TRUE</ONLYSEND></CONFIG>\
             <SEND><ELEMENTS><ELEMENT TAG=\"DEF_AIPos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" /></ELEMENTS></SEND><RECEIVE><ELEMENTS>{receive}</ELEMENTS></RECEIVE></ROOT>"
        );
        let c = EthernetConfig::from_str(&xml).unwrap();
        let err = Schema::compile(&c).unwrap_err().to_string();
        assert!(err.contains("AK"), "{err}");
    }

    #[test]
    fn keyword_tags_and_attrs() {
        assert_eq!(Keyword::RIst.tag(), "RIst");
        assert_eq!(Keyword::RIst.attrs(), &["X", "Y", "Z", "A", "B", "C"]);
        assert_eq!(Keyword::AIPos.attrs(), &["A1", "A2", "A3", "A4", "A5", "A6"]);
        assert_eq!(Keyword::Delay.attrs(), &["D"]);
    }
}
