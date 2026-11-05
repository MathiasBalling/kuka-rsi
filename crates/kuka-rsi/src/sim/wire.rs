//! Mirror images of the driver's parser and writer: the robot side of the wire.

use std::io::Write as _;

use quick_xml::escape::unescape;
use quick_xml::events::{BytesStart, Event};

use crate::config::ElementType;
use crate::telegram::{Keyword, ParseError, RobElement, RobotTelegram, Schema, SenElement, SensorTelegram};

fn keyword_values<'a>(k: Keyword, t: &'a RobotTelegram) -> &'a [f64] {
    match k {
        Keyword::RIst => &t.rist,
        Keyword::RSol => &t.rsol,
        Keyword::AIPos => &t.aipos,
        Keyword::ASPos => &t.aspos,
        Keyword::EIPos => &t.eipos,
        Keyword::ESPos => &t.espos,
        Keyword::MACur => &t.macur,
        Keyword::MECur => &t.mecur,
        Keyword::Delay => &[],
    }
}

fn write_value(out: &mut Vec<u8>, ty: ElementType, v: f64, precision: usize) {
    match ty {
        ElementType::Double | ElementType::String => {
            let _ = write!(out, "{v:.precision$}");
        }
        ElementType::Long | ElementType::Bool => {
            let _ = write!(out, "{}", v.round() as i64);
        }
    }
}

/// Serialize a robot telegram the way the controller would.
pub fn write_rob(schema: &Schema, tele: &RobotTelegram, precision: usize, out: &mut Vec<u8>) {
    out.clear();
    let _ = write!(out, "<Rob Type=\"{}\">", crate::telegram::parse::ROB_TYPE);
    for e in &schema.rob {
        match e {
            RobElement::Keyword(Keyword::Delay) => {
                let _ = write!(out, "<Delay D=\"{}\" />", tele.delay);
            }
            RobElement::Keyword(k) => {
                let _ = write!(out, "<{}", k.tag());
                for (n, v) in k.attrs().iter().zip(keyword_values(*k, tele)) {
                    let _ = write!(out, " {n}=\"{v:.precision$}\"");
                }
                out.extend_from_slice(b" />");
            }
            RobElement::Attrs { tag, attrs } => {
                let _ = write!(out, "<{tag}");
                for a in attrs {
                    let _ = write!(out, " {}=\"", a.name);
                    write_value(out, a.ty, tele.slots[a.slot], precision);
                    out.push(b'"');
                }
                out.extend_from_slice(b" />");
            }
            RobElement::Text { tag, slot, ty } => {
                let _ = write!(out, "<{tag}>");
                write_value(out, *ty, tele.slots[*slot], precision);
                let _ = write!(out, "</{tag}>");
            }
        }
    }
    let _ = write!(out, "<IPOC>{}</IPOC></Rob>", tele.ipoc);
}

const AXES: [&str; 6] = ["A1", "A2", "A3", "A4", "A5", "A6"];
const EXT: [&str; 6] = ["E1", "E2", "E3", "E4", "E5", "E6"];
const CART: [&str; 6] = ["X", "Y", "Z", "A", "B", "C"];

fn num(s: &str) -> Result<f64, ParseError> {
    let s = s.trim();
    if s.is_empty() {
        return Ok(0.0);
    }
    // Mirrors the driver's `parse_f64`: `f64::from_str` also accepts NaN, inf and infinity,
    // and overflows to inf ("1e999"); none of those may reach the telegram.
    match s.parse::<f64>() {
        Ok(v) if v.is_finite() => Ok(v),
        _ => Err(ParseError::Number(s.to_string())),
    }
}

enum Target {
    Ipoc,
    EStr,
    Slot(usize),
}

fn on_element(schema: &Schema, e: &BytesStart<'_>, depth: usize, out: &mut SensorTelegram, target: &mut Option<Target>) -> Result<(), ParseError> {
    let qname = e.name();
    let name: &str = qname.as_ref();
    if depth == 0 {
        if name != "Sen" {
            return Err(ParseError::Root(name.to_string()));
        }
        let mut saw_type = false;
        for a in e.attributes() {
            let a = a.map_err(|err| ParseError::Xml(err.to_string()))?;
            let key: &str = a.key.as_ref();
            if key == "Type" {
                saw_type = true;
                let got: &str = a.value.as_ref();
                if got != schema.sentype {
                    return Err(ParseError::Sentype { got: got.to_string(), want: schema.sentype.clone() });
                }
            }
        }
        if !saw_type {
            return Err(ParseError::Sentype { got: String::new(), want: schema.sentype.clone() });
        }
        return Ok(());
    }
    if depth != 1 {
        return Ok(());
    }
    *target = None;
    match name {
        "IPOC" => *target = Some(Target::Ipoc),
        "EStr" => {
            out.estr.clear();
            *target = Some(Target::EStr);
        }
        "AK" | "RKorr" | "EK" => {
            let (names, dest): (&[&str; 6], &mut [f64; 6]) = match name {
                "AK" => (&AXES, &mut out.ak),
                "RKorr" => (&CART, &mut out.rkorr),
                _ => (&EXT, &mut out.ek),
            };
            for a in e.attributes() {
                let a = a.map_err(|err| ParseError::Xml(err.to_string()))?;
                let key: &str = a.key.as_ref();
                if let Some(i) = names.iter().position(|n| *n == key) {
                    dest[i] = num(a.value.as_ref())?;
                }
            }
        }
        _ => {
            for s in &schema.sen {
                match s {
                    SenElement::Attrs { tag, attrs } if tag == name => {
                        for a in e.attributes() {
                            let a = a.map_err(|err| ParseError::Xml(err.to_string()))?;
                            let key: &str = a.key.as_ref();
                            if let Some(at) = attrs.iter().find(|x| x.name == key) {
                                out.slots[at.slot] = num(a.value.as_ref())?;
                            }
                        }
                    }
                    SenElement::Text { tag, slot, .. } if tag == name => *target = Some(Target::Slot(*slot)),
                    _ => {}
                }
            }
        }
    }
    Ok(())
}

/// Appends as much of `s` as fits in `out`'s remaining capacity, without splitting a
/// UTF-8 character.
fn push_truncated(out: &mut arrayvec::ArrayString<64>, s: &str) {
    let room = out.capacity() - out.len();
    let mut end = s.len().min(room);
    while end > 0 && !s.is_char_boundary(end) {
        end -= 1;
    }
    out.push_str(&s[..end]);
}

/// Parse a sensor telegram the way the controller would.
pub fn parse_sen(schema: &Schema, bytes: &[u8], out: &mut SensorTelegram) -> Result<(), ParseError> {
    let mut reader = quick_xml::Reader::from_reader(bytes);
    // `EStr` text can contain XML entities (RSI manual 7.5.5 allows `&lt;`/`&gt;`/`&amp;`
    // in messages): quick-xml reports each as its own `GeneralRef` event, splitting the
    // surrounding text into separate `Text` fragments. Trimming those fragments would eat
    // whitespace that sits right next to an entity, so text is left untrimmed; every other
    // value read here is trimmed by hand (see `num` and the `IPOC` arm below).
    let mut depth = 0usize;
    let mut got_ipoc = false;
    let mut target: Option<Target> = None;
    loop {
        // `Reader<&[u8]>::read_event` borrows from `bytes`; the buffered `read_event_into` copies.
        let ev = reader.read_event().map_err(|e| ParseError::Xml(e.to_string()))?;
        match ev {
            Event::Eof => break,
            Event::Start(e) => {
                on_element(schema, &e, depth, out, &mut target)?;
                depth += 1;
            }
            Event::Empty(e) => {
                on_element(schema, &e, depth, out, &mut target)?;
                if depth == 0 {
                    break;
                }
            }
            Event::End(_) => {
                depth = depth.saturating_sub(1);
                target = None;
            }
            Event::Text(t) => {
                if depth == 2 {
                    match target {
                        Some(Target::Ipoc) => {
                            let s: &str = t.as_ref();
                            out.ipoc = s.trim().parse().map_err(|_| ParseError::Number(s.to_string()))?;
                            got_ipoc = true;
                        }
                        Some(Target::EStr) => {
                            let text = unescape(t.as_ref()).map_err(|e| ParseError::Xml(e.to_string()))?;
                            push_truncated(&mut out.estr, &text);
                        }
                        Some(Target::Slot(slot)) => out.slots[slot] = num(t.as_ref())?,
                        None => {}
                    }
                }
            }
            Event::GeneralRef(r) => {
                if depth == 2 {
                    match target {
                        Some(Target::EStr) => {
                            let resolved = match r.resolve_char_ref().map_err(|e| ParseError::Xml(e.to_string()))? {
                                Some(ch) => ch.to_string(),
                                None => {
                                    let name: &str = r.as_ref();
                                    quick_xml::escape::resolve_predefined_entity(name)
                                        .ok_or_else(|| ParseError::Xml(format!("unknown entity &{name};")))?
                                        .to_string()
                                }
                            };
                            push_truncated(&mut out.estr, &resolved);
                        }
                        // Numeric fields (IPOC, user slots) never legitimately contain an
                        // entity reference; treat one as malformed rather than let the
                        // surrounding text fragments silently overwrite each other.
                        Some(Target::Ipoc) | Some(Target::Slot(_)) => {
                            return Err(ParseError::Xml("entity reference inside a numeric field".into()));
                        }
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
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::EthernetConfig;
    use crate::telegram::{Parser, RobotTelegram, Schema, SensorTelegram, Writer};

    const SHIPPED: &str = include_str!("../../../../controller/kuka_rsi_ethernet.xml");

    fn schema() -> Schema {
        Schema::compile(&EthernetConfig::from_str(SHIPPED).unwrap()).unwrap()
    }

    #[test]
    fn rob_round_trip_through_the_driver_parser() {
        let s = schema();
        let mut t = RobotTelegram { ipoc: 4004, delay: 3, ..Default::default() };
        t.rist = [600.1234, -10.5, 780.0, 180.0, 70.0, -180.0];
        t.aipos = [1.5, -90.0, 90.0, 0.0, 90.0, 0.25];
        t.aspos = [1.0, -90.0, 90.0, 0.0, 90.0, 0.0];
        t.slots[..8].copy_from_slice(&[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 3.0, 100.0]);
        let mut out = Vec::new();
        write_rob(&s, &t, 4, &mut out);
        let text = std::str::from_utf8(&out).unwrap();
        assert!(text.starts_with("<Rob Type=\"KUKA\"><RIst X=\"600.1234\""), "{text}");
        assert!(text.contains("<ProgStatus R=\"3\" />"), "{text}");
        assert!(text.contains("<Delay D=\"3\" />"), "{text}");
        assert!(text.ends_with("<IPOC>4004</IPOC></Rob>"), "{text}");
        let mut back = RobotTelegram::default();
        Parser::new(&s).parse(&out, &mut back).unwrap();
        assert_eq!(back, t);
    }

    #[test]
    fn sen_round_trip_through_the_driver_writer() {
        let s = schema();
        let mut t = SensorTelegram { ipoc: 77, ..Default::default() };
        t.ak = [0.5, -1.25, 2.0, 0.0, 0.125, -0.0625];
        t.slots[6] = 1.0;
        t.estr.push_str("Error: a<b & c>");
        let mut out = Vec::new();
        Writer::new(&s).write(&t, &mut out);
        let mut back = SensorTelegram::default();
        parse_sen(&s, &out, &mut back).unwrap();
        assert_eq!(back, t);
    }

    proptest::proptest! {
        #[test]
        fn sen_round_trip_any_ak(ak in proptest::array::uniform6(-360.0f64..360.0), ipoc in 0u64..u64::MAX) {
            let s = schema();
            let mut t = SensorTelegram { ipoc, ..Default::default() };
            t.ak = ak;
            let mut out = Vec::new();
            Writer::new(&s).write(&t, &mut out);
            let mut back = SensorTelegram::default();
            parse_sen(&s, &out, &mut back).unwrap();
            for i in 0..6 {
                proptest::prop_assert!((back.ak[i] - ak[i]).abs() < 1e-6, "axis {}: {} vs {}", i, back.ak[i], ak[i]);
            }
            proptest::prop_assert_eq!(back.ipoc, ipoc);
        }
    }

    #[test]
    fn sen_parser_rejects_wrong_type_and_missing_ipoc() {
        let s = schema();
        let mut back = SensorTelegram::default();
        assert!(matches!(parse_sen(&s, b"<Sen Type=\"Other\"><IPOC>1</IPOC></Sen>", &mut back), Err(ParseError::Sentype { .. })));
        assert!(matches!(parse_sen(&s, b"<Sen Type=\"KukaRsi\"><EStr/></Sen>", &mut back), Err(ParseError::MissingIpoc)));
        assert!(matches!(parse_sen(&s, b"<Rob Type=\"KUKA\"><IPOC>1</IPOC></Rob>", &mut back), Err(ParseError::Root(_))));
        assert!(matches!(parse_sen(&s, b"<Sen><IPOC>1</IPOC></Sen>", &mut back), Err(ParseError::Sentype { .. })));
    }

    #[test]
    fn sen_parser_rejects_entities_in_numeric_fields() {
        let s = schema();
        let mut back = SensorTelegram::default();
        assert!(matches!(parse_sen(&s, b"<Sen Type=\"KukaRsi\"><IPOC>1&#48;2</IPOC></Sen>", &mut back), Err(ParseError::Xml(_))));
        assert!(matches!(
            parse_sen(&s, b"<Sen Type=\"KukaRsi\"><MoveStop>&amp;1</MoveStop><IPOC>1</IPOC></Sen>", &mut back),
            Err(ParseError::Xml(_))
        ));
        assert!(matches!(
            parse_sen(&s, b"<Sen Type=\"KukaRsi\"><AK A1=\"NaN\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\" /><IPOC>1</IPOC></Sen>", &mut back),
            Err(ParseError::Number(_))
        ));
        parse_sen(&s, b"<Sen Type=\"KukaRsi\"><EStr>a&amp;b</EStr><IPOC>1</IPOC></Sen>", &mut back).unwrap();
        assert_eq!(back.estr.as_str(), "a&b");
    }
}
