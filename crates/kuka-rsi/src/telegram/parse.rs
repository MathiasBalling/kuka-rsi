//! Parser for `<Rob>` telegrams. No allocation after construction: quick-xml reads
//! borrowed events from the caller's buffer and values land in fixed arrays.

use quick_xml::events::{BytesStart, Event};

use super::schema::{Keyword, RobElement, Schema, MAX_SLOTS};

/// One robot-to-PC telegram in wire units (degrees, millimetres).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RobotTelegram {
    pub ipoc: u64,
    pub rist: [f64; 6],
    pub rsol: [f64; 6],
    pub aipos: [f64; 6],
    pub aspos: [f64; 6],
    pub eipos: [f64; 6],
    pub espos: [f64; 6],
    pub macur: [f64; 6],
    pub mecur: [f64; 6],
    pub delay: u32,
    pub slots: [f64; MAX_SLOTS],
}

impl Default for RobotTelegram {
    fn default() -> Self {
        RobotTelegram {
            ipoc: 0,
            rist: [0.0; 6],
            rsol: [0.0; 6],
            aipos: [0.0; 6],
            aspos: [0.0; 6],
            eipos: [0.0; 6],
            espos: [0.0; 6],
            macur: [0.0; 6],
            mecur: [0.0; 6],
            delay: 0,
            slots: [0.0; MAX_SLOTS],
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ParseError {
    #[error("malformed xml: {0}")]
    Xml(String),
    #[error("root element is {0:?}, expected Rob")]
    Root(String),
    #[error("TYPE is {got:?}, expected {want:?}")]
    Sentype { got: String, want: String },
    #[error("telegram has no IPOC")]
    MissingIpoc,
    #[error("bad number: {0}")]
    Number(String),
}

/// Reusable parser bound to one schema.
pub struct Parser {
    schema: Schema,
    seen: Vec<bool>,
    missing: u64,
    unknown: u64,
}

fn parse_f64(s: &str, what: &str) -> Result<f64, ParseError> {
    let s = s.trim();
    if s.is_empty() {
        return Ok(0.0);
    }
    s.parse::<f64>().map_err(|_| ParseError::Number(format!("{what}={s:?}")))
}

impl Parser {
    pub fn new(schema: &Schema) -> Parser {
        Parser { schema: schema.clone(), seen: vec![false; schema.rob.len()], missing: 0, unknown: 0 }
    }

    /// Schema elements absent from telegrams so far (cumulative).
    pub fn missing(&self) -> u64 {
        self.missing
    }

    /// Elements not in the schema so far (cumulative).
    pub fn unknown(&self) -> u64 {
        self.unknown
    }

    fn check_root(&self, e: &BytesStart<'_>) -> Result<(), ParseError> {
        let qname = e.name();
        let name: &str = qname.as_ref();
        if name != "Rob" {
            return Err(ParseError::Root(name.to_string()));
        }
        for a in e.attributes() {
            let a = a.map_err(|err| ParseError::Xml(err.to_string()))?;
            let key: &str = a.key.as_ref();
            if key.eq_ignore_ascii_case("TYPE") {
                let got: &str = a.value.as_ref();
                if got != self.schema.sentype {
                    return Err(ParseError::Sentype { got: got.to_string(), want: self.schema.sentype.clone() });
                }
            }
        }
        Ok(())
    }

    /// Returns the schema index of the element, or None if unknown.
    fn element_index(&self, name: &str) -> Option<usize> {
        self.schema.rob.iter().position(|e| match e {
            RobElement::Keyword(k) => k.tag() == name,
            RobElement::Attrs { tag, .. } | RobElement::Text { tag, .. } => tag == name,
        })
    }

    fn read_attrs(&self, idx: usize, e: &BytesStart<'_>, out: &mut RobotTelegram) -> Result<(), ParseError> {
        match &self.schema.rob[idx] {
            RobElement::Keyword(k) => {
                let names = k.attrs();
                for a in e.attributes() {
                    let a = a.map_err(|err| ParseError::Xml(err.to_string()))?;
                    let key: &str = a.key.as_ref();
                    let Some(i) = names.iter().position(|n| *n == key) else { continue };
                    let v = parse_f64(a.value.as_ref(), key)?;
                    match k {
                        Keyword::RIst => out.rist[i] = v,
                        Keyword::RSol => out.rsol[i] = v,
                        Keyword::AIPos => out.aipos[i] = v,
                        Keyword::ASPos => out.aspos[i] = v,
                        Keyword::EIPos => out.eipos[i] = v,
                        Keyword::ESPos => out.espos[i] = v,
                        Keyword::MACur => out.macur[i] = v,
                        Keyword::MECur => out.mecur[i] = v,
                        Keyword::Delay => out.delay = v as u32,
                    }
                }
            }
            RobElement::Attrs { attrs, .. } => {
                for a in e.attributes() {
                    let a = a.map_err(|err| ParseError::Xml(err.to_string()))?;
                    let key: &str = a.key.as_ref();
                    if let Some(attr) = attrs.iter().find(|x| x.name == key) {
                        out.slots[attr.slot] = parse_f64(a.value.as_ref(), key)?;
                    }
                }
            }
            RobElement::Text { .. } => {}
        }
        Ok(())
    }

    /// Parse one telegram into `out`. Values of absent elements are left untouched.
    pub fn parse(&mut self, bytes: &[u8], out: &mut RobotTelegram) -> Result<(), ParseError> {
        let mut reader = quick_xml::Reader::from_reader(bytes);
        reader.config_mut().trim_text(true);
        let mut buf: Vec<u8> = Vec::new(); // never grows: events borrow from `bytes`
        self.seen.iter_mut().for_each(|s| *s = false);
        let mut depth = 0usize;
        let mut got_ipoc = false;
        let mut text_target: Option<TextTarget> = None;
        loop {
            let ev = reader.read_event_into(&mut buf).map_err(|e| ParseError::Xml(e.to_string()))?;
            match ev {
                Event::Eof => break,
                Event::Start(e) => {
                    if depth == 0 {
                        self.check_root(&e)?;
                    } else if depth == 1 {
                        let qname = e.name();
                        let name: &str = qname.as_ref();
                        if name == "IPOC" {
                            text_target = Some(TextTarget::Ipoc);
                        } else if let Some(idx) = self.element_index(name) {
                            self.seen[idx] = true;
                            match &self.schema.rob[idx] {
                                RobElement::Text { slot, .. } => text_target = Some(TextTarget::Slot(*slot)),
                                _ => self.read_attrs(idx, &e, out)?,
                            }
                        } else {
                            self.unknown += 1;
                        }
                    }
                    depth += 1;
                }
                Event::Empty(e) => {
                    if depth == 0 {
                        self.check_root(&e)?;
                        break;
                    }
                    if depth == 1 {
                        let qname = e.name();
                        let name: &str = qname.as_ref();
                        if let Some(idx) = self.element_index(name) {
                            self.seen[idx] = true;
                            self.read_attrs(idx, &e, out)?;
                        } else {
                            self.unknown += 1;
                        }
                    }
                }
                Event::End(_) => {
                    depth = depth.saturating_sub(1);
                    text_target = None;
                }
                Event::Text(t) => {
                    if depth == 2 {
                        let s: &str = t.as_ref();
                        match text_target {
                            Some(TextTarget::Ipoc) => {
                                out.ipoc = s.trim().parse::<u64>().map_err(|_| ParseError::Number(format!("IPOC={s:?}")))?;
                                got_ipoc = true;
                            }
                            Some(TextTarget::Slot(slot)) => out.slots[slot] = parse_f64(s, "text")?,
                            None => {}
                        }
                    }
                }
                _ => {}
            }
        }
        if depth != 0 {
            return Err(ParseError::Xml("unclosed root element".into()));
        }
        if !got_ipoc {
            return Err(ParseError::MissingIpoc);
        }
        self.missing += self.seen.iter().filter(|s| !**s).count() as u64;
        Ok(())
    }
}

#[derive(Clone, Copy)]
enum TextTarget {
    Ipoc,
    Slot(usize),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::EthernetConfig;

    const SHIPPED: &str = include_str!("../../../../controller/kuka_rsi_ethernet.xml");

    fn schema(xml: &str) -> Schema {
        Schema::compile(&EthernetConfig::from_str(xml).unwrap()).unwrap()
    }

    fn manual_schema() -> Schema {
        schema(
            "<ROOT><CONFIG><IP_NUMBER>127.0.0.1</IP_NUMBER><PORT>49152</PORT><SENTYPE>KUKA</SENTYPE><ONLYSEND>TRUE</ONLYSEND></CONFIG>\
             <SEND><ELEMENTS><ELEMENT TAG=\"DEF_RIst\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" /><ELEMENT TAG=\"DEF_AIPos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" />\
             <ELEMENT TAG=\"DEF_MACur\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" /><ELEMENT TAG=\"DEF_Delay\" TYPE=\"LONG\" INDX=\"INTERNAL\" /></ELEMENTS></SEND>\
             <RECEIVE><ELEMENTS></ELEMENTS></RECEIVE></ROOT>",
        )
    }

    /// The example document from RSI manual section 7.5.5 (Tech element left out, unsupported).
    const MANUAL_EXAMPLE: &[u8] = br#"<Rob TYPE="KUKA">
 <RIst X="0.0" Y="0.0" Z="0.0" A="0.0" B="0.0" C="0.0" />
 <AIPos A1="0.0" A2="0.0" A3="0.0" A4="0.0" A5="0.0" A6="0.0" />
 <MACur A1="1.0" A2="1.0" A3="1.0" A4="1.0" A5="1.0" A6="1.0" />
 <Delay D="" />
 <IPOC>123645634563</IPOC>
</Rob>"#;

    #[test]
    fn parses_manual_example() {
        let mut p = Parser::new(&manual_schema());
        let mut t = RobotTelegram::default();
        p.parse(MANUAL_EXAMPLE, &mut t).unwrap();
        assert_eq!(t.ipoc, 123_645_634_563);
        assert_eq!(t.macur, [1.0; 6]);
        assert_eq!(t.delay, 0);
        assert_eq!(p.missing(), 0);
        assert_eq!(p.unknown(), 0);
    }

    const SHIPPED_TELEGRAM: &[u8] = br#"<Rob TYPE="KukaRsi"><RIst X="600.1234" Y="-10.5" Z="780.0" A="180.0" B="70.0" C="-180.0" /><RSol X="600.0" Y="-10.0" Z="780.0" A="180.0" B="70.0" C="-180.0" /><AIPos A1="1.5" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.25" /><ASPos A1="1.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0" /><Delay D="2" /><GearTorque A1="0.1" A2="0.2" A3="0.3" A4="0.4" A5="0.5" A6="0.6" /><ProgStatus R="3" /><OvPro R="100.0" /><IPOC>4004</IPOC></Rob>"#;

    #[test]
    fn parses_shipped_layout_into_slots() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        p.parse(SHIPPED_TELEGRAM, &mut t).unwrap();
        assert_eq!(t.rist[0], 600.1234);
        assert_eq!(t.aipos, [1.5, -90.0, 90.0, 0.0, 90.0, 0.25]);
        assert_eq!(t.aspos[0], 1.0);
        assert_eq!(t.delay, 2);
        assert_eq!(&t.slots[0..6], &[0.1, 0.2, 0.3, 0.4, 0.5, 0.6]);
        assert_eq!(t.slots[6], 3.0);
        assert_eq!(t.slots[7], 100.0);
        assert_eq!(t.ipoc, 4004);
    }

    #[test]
    fn attribute_order_does_not_matter_and_whitespace_is_tolerated() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        let xml = b"<Rob TYPE=\"KukaRsi\">\n  <AIPos A6=\"6\" A5=\"5\" A4=\"4\" A3=\"3\" A2=\"2\" A1=\"1\"/>\n  <IPOC> 8 </IPOC>\n</Rob>\n";
        p.parse(xml, &mut t).unwrap();
        assert_eq!(t.aipos, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        assert_eq!(t.ipoc, 8);
        assert_eq!(p.missing(), 7, "all other schema elements were absent");
    }

    #[test]
    fn missing_elements_keep_previous_values_and_unknown_are_counted() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        p.parse(SHIPPED_TELEGRAM, &mut t).unwrap();
        let xml = b"<Rob TYPE=\"KukaRsi\"><Bogus Q=\"1\" /><Other>7</Other><IPOC>4008</IPOC></Rob>";
        p.parse(xml, &mut t).unwrap();
        assert_eq!(t.aipos[0], 1.5, "kept");
        assert_eq!(t.slots[6], 3.0, "kept");
        assert_eq!(t.ipoc, 4008);
        assert_eq!(p.unknown(), 2);
        assert_eq!(p.missing(), 8);
    }

    #[test]
    fn rejects_wrong_root_sentype_and_missing_ipoc() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        assert_eq!(p.parse(b"<Sen Type=\"KukaRsi\"><IPOC>1</IPOC></Sen>", &mut t), Err(ParseError::Root("Sen".into())));
        assert_eq!(
            p.parse(b"<Rob TYPE=\"Other\"><IPOC>1</IPOC></Rob>", &mut t),
            Err(ParseError::Sentype { got: "Other".into(), want: "KukaRsi".into() })
        );
        assert_eq!(p.parse(b"<Rob TYPE=\"KukaRsi\"><Delay D=\"0\" /></Rob>", &mut t), Err(ParseError::MissingIpoc));
        assert!(matches!(p.parse(b"<Rob TYPE=\"KukaRsi\"><IPOC>x</IPOC></Rob>", &mut t), Err(ParseError::Number(_))));
        assert!(matches!(p.parse(b"<Rob TYPE=\"KukaRsi\"><IPOC>1</IPOC>", &mut t), Err(ParseError::Xml(_))));
    }

    #[test]
    fn bool_and_long_slots_parse_by_numeric_value() {
        let s = schema(
            "<ROOT><CONFIG><IP_NUMBER>127.0.0.1</IP_NUMBER><PORT>49152</PORT><SENTYPE>T</SENTYPE><ONLYSEND>TRUE</ONLYSEND></CONFIG>\
             <SEND><ELEMENTS><ELEMENT TAG=\"Digout.o1\" TYPE=\"BOOL\" INDX=\"1\" /><ELEMENT TAG=\"Digout.o2\" TYPE=\"BOOL\" INDX=\"2\" />\
             <ELEMENT TAG=\"DiL\" TYPE=\"LONG\" INDX=\"3\" /></ELEMENTS></SEND><RECEIVE><ELEMENTS></ELEMENTS></RECEIVE></ROOT>",
        );
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        p.parse(b"<Rob TYPE=\"T\"><Digout o1=\"1.000000\" o2=\"0.00\" /><DiL>42</DiL><IPOC>12</IPOC></Rob>", &mut t).unwrap();
        assert_eq!(&t.slots[0..3], &[1.0, 0.0, 42.0]);
    }

    #[test]
    fn parser_is_reusable_across_many_telegrams() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        for i in 0..1000u64 {
            let xml = format!("<Rob TYPE=\"KukaRsi\"><AIPos A1=\"{i}\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\" /><IPOC>{}</IPOC></Rob>", i * 4);
            p.parse(xml.as_bytes(), &mut t).unwrap();
            assert_eq!(t.aipos[0], i as f64);
            assert_eq!(t.ipoc, i * 4);
        }
    }
}
