//! Writer for `<Sen>` telegrams into a caller-provided buffer.

use std::io::Write as _;

use arrayvec::ArrayString;

use super::schema::{MAX_SLOTS, Schema, SenElement};
use crate::config::ElementType;

/// One PC-to-robot telegram in wire units (degrees, millimetres).
#[derive(Clone, Debug, PartialEq)]
pub struct SensorTelegram {
    pub ipoc: u64,
    pub ak: [f64; 6],
    pub rkorr: [f64; 6],
    pub ek: [f64; 6],
    pub estr: ArrayString<64>,
    pub slots: [f64; MAX_SLOTS],
}

impl Default for SensorTelegram {
    fn default() -> Self {
        SensorTelegram {
            ipoc: 0,
            ak: [0.0; 6],
            rkorr: [0.0; 6],
            ek: [0.0; 6],
            estr: ArrayString::new(),
            slots: [0.0; MAX_SLOTS],
        }
    }
}

/// Reusable writer bound to one schema.
pub struct Writer {
    schema: Schema,
}

const AXES: [&str; 6] = ["A1", "A2", "A3", "A4", "A5", "A6"];
const EXT: [&str; 6] = ["E1", "E2", "E3", "E4", "E5", "E6"];
const CART: [&str; 6] = ["X", "Y", "Z", "A", "B", "C"];

fn write_group(out: &mut Vec<u8>, tag: &str, names: &[&str], values: &[f64]) {
    let _ = write!(out, "<{tag}");
    for (n, v) in names.iter().zip(values) {
        let _ = write!(out, " {n}=\"{v:.6}\"");
    }
    out.extend_from_slice(b" />");
}

fn write_value(out: &mut Vec<u8>, ty: ElementType, v: f64) {
    match ty {
        // STRING channels are rejected by EthernetConfig validation; only DEF_EStr carries text.
        ElementType::Double | ElementType::String => {
            let _ = write!(out, "{v:.6}");
        }
        ElementType::Long | ElementType::Bool => {
            let _ = write!(out, "{}", v.round() as i64);
        }
    }
}

fn write_escaped(out: &mut Vec<u8>, s: &str) {
    for b in s.bytes() {
        match b {
            b'<' => out.extend_from_slice(b"&lt;"),
            b'>' => out.extend_from_slice(b"&gt;"),
            b'&' => out.extend_from_slice(b"&amp;"),
            b'"' => out.extend_from_slice(b"&quot;"),
            _ => out.push(b),
        }
    }
}

impl Writer {
    pub fn new(schema: &Schema) -> Writer {
        Writer { schema: schema.clone() }
    }

    /// Serialize `t` into `out` (cleared first).
    pub fn write(&self, t: &SensorTelegram, out: &mut Vec<u8>) {
        out.clear();
        let _ = write!(out, "<Sen Type=\"{}\">", self.schema.sentype);
        for e in &self.schema.sen {
            match e {
                SenElement::Ak => write_group(out, "AK", &AXES, &t.ak),
                SenElement::RKorr => write_group(out, "RKorr", &CART, &t.rkorr),
                SenElement::Ek => write_group(out, "EK", &EXT, &t.ek),
                SenElement::EStr => {
                    if t.estr.is_empty() {
                        out.extend_from_slice(b"<EStr/>");
                    } else {
                        out.extend_from_slice(b"<EStr>");
                        write_escaped(out, &t.estr);
                        out.extend_from_slice(b"</EStr>");
                    }
                }
                SenElement::Attrs { tag, attrs } => {
                    let _ = write!(out, "<{tag}");
                    for a in attrs {
                        let _ = write!(out, " {}=\"", a.name);
                        write_value(out, a.ty, t.slots[a.slot]);
                        out.push(b'"');
                    }
                    out.extend_from_slice(b" />");
                }
                SenElement::Text { tag, slot, ty } => {
                    let _ = write!(out, "<{tag}>");
                    write_value(out, *ty, t.slots[*slot]);
                    let _ = write!(out, "</{tag}>");
                }
            }
        }
        let _ = write!(out, "<IPOC>{}</IPOC></Sen>", t.ipoc);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::EthernetConfig;

    const SHIPPED: &str = include_str!("../../controller/kuka_rsi_ethernet.xml");

    fn schema(xml: &str) -> Schema {
        Schema::compile(&EthernetConfig::from_str(xml).unwrap()).unwrap()
    }

    #[test]
    fn writes_shipped_layout_byte_exact() {
        let w = Writer::new(&schema(SHIPPED));
        let mut t = SensorTelegram { ipoc: 123456, ..Default::default() };
        t.ak = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        let mut out = Vec::with_capacity(4096);
        w.write(&t, &mut out);
        assert_eq!(
            std::str::from_utf8(&out).unwrap(),
            "<Sen Type=\"KukaRsi\"><AK A1=\"0.000000\" A2=\"1.000000\" A3=\"2.000000\" A4=\"3.000000\" A5=\"4.000000\" A6=\"5.000000\" />\
             <MoveStop>0</MoveStop><EStr/><IPOC>123456</IPOC></Sen>"
        );
    }

    #[test]
    fn writes_stop_flag_message_and_escapes() {
        let w = Writer::new(&schema(SHIPPED));
        let mut t = SensorTelegram { ipoc: 7, ..Default::default() };
        t.slots[6] = 1.0;
        t.estr.push_str("Error: a<b & c>");
        let mut out = Vec::new();
        w.write(&t, &mut out);
        let s = std::str::from_utf8(&out).unwrap();
        assert!(s.contains("<MoveStop>1</MoveStop>"), "{s}");
        assert!(s.contains("<EStr>Error: a&lt;b &amp; c&gt;</EStr>"), "{s}");
        assert!(s.ends_with("<IPOC>7</IPOC></Sen>"));
    }

    #[test]
    fn writes_rkorr_and_user_groups() {
        let s = schema(
            "<ROOT><CONFIG><IP_NUMBER>127.0.0.1</IP_NUMBER><PORT>49152</PORT><SENTYPE>ImFree</SENTYPE><ONLYSEND>FALSE</ONLYSEND></CONFIG>\
             <SEND><ELEMENTS><ELEMENT TAG=\"DEF_RIst\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" /><ELEMENT TAG=\"DEF_AIPos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" /><ELEMENT TAG=\"DEF_ASPos\" TYPE=\"DOUBLE\" INDX=\"INTERNAL\" /></ELEMENTS></SEND><RECEIVE><ELEMENTS>\
             <ELEMENT TAG=\"RKorr.X\" TYPE=\"DOUBLE\" INDX=\"1\" /><ELEMENT TAG=\"RKorr.Y\" TYPE=\"DOUBLE\" INDX=\"2\" /><ELEMENT TAG=\"RKorr.Z\" TYPE=\"DOUBLE\" INDX=\"3\" />\
             <ELEMENT TAG=\"RKorr.A\" TYPE=\"DOUBLE\" INDX=\"4\" /><ELEMENT TAG=\"RKorr.B\" TYPE=\"DOUBLE\" INDX=\"5\" /><ELEMENT TAG=\"RKorr.C\" TYPE=\"DOUBLE\" INDX=\"6\" />\
             <ELEMENT TAG=\"AK.A1\" TYPE=\"DOUBLE\" INDX=\"7\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A2\" TYPE=\"DOUBLE\" INDX=\"8\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A3\" TYPE=\"DOUBLE\" INDX=\"9\" HOLDON=\"1\" />\
             <ELEMENT TAG=\"AK.A4\" TYPE=\"DOUBLE\" INDX=\"10\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A5\" TYPE=\"DOUBLE\" INDX=\"11\" HOLDON=\"1\" /><ELEMENT TAG=\"AK.A6\" TYPE=\"DOUBLE\" INDX=\"12\" HOLDON=\"1\" />\
             <ELEMENT TAG=\"Digout.o1\" TYPE=\"BOOL\" INDX=\"13\" /><ELEMENT TAG=\"Digout.o2\" TYPE=\"BOOL\" INDX=\"14\" /><ELEMENT TAG=\"DiO\" TYPE=\"LONG\" INDX=\"15\" />\
             </ELEMENTS></RECEIVE></ROOT>",
        );
        let w = Writer::new(&s);
        let mut t = SensorTelegram { ipoc: 1, ..Default::default() };
        t.rkorr = [4.0, 7.0, 32.0, 6.0, 0.0, 6.0];
        t.slots[12] = 1.0;
        t.slots[14] = 123.0;
        let mut out = Vec::new();
        w.write(&t, &mut out);
        assert_eq!(
            std::str::from_utf8(&out).unwrap(),
            "<Sen Type=\"ImFree\"><RKorr X=\"4.000000\" Y=\"7.000000\" Z=\"32.000000\" A=\"6.000000\" B=\"0.000000\" C=\"6.000000\" />\
             <AK A1=\"0.000000\" A2=\"0.000000\" A3=\"0.000000\" A4=\"0.000000\" A5=\"0.000000\" A6=\"0.000000\" />\
             <Digout o1=\"1\" o2=\"0\" /><DiO>123</DiO><IPOC>1</IPOC></Sen>"
        );
    }

    #[test]
    fn write_clears_and_reuses_buffer() {
        let w = Writer::new(&schema(SHIPPED));
        let t = SensorTelegram { ipoc: 1, ..Default::default() };
        let mut out = vec![0xFFu8; 10];
        w.write(&t, &mut out);
        assert!(out.starts_with(b"<Sen "));
        let first = out.clone();
        w.write(&t, &mut out);
        assert_eq!(out, first);
    }
}
