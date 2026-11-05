//! Parser for `<Rob>` telegrams. A hand-written tokenizer walks the small XML subset
//! RSI telegrams use; no allocation on the parse path. (quick-xml stays a crate
//! dependency for config parsing and the simulator, but not this hot path: measured
//! with a counting allocator, its own internal Start/End tag-name bookkeeping had to
//! be rebuilt from scratch on every telegram, since it is tied to each call's
//! borrowed input and quick-xml exposes no way to reuse it across calls.)

use super::schema::{Keyword, RobElement, Schema, MAX_SLOTS};

/// `Type` attribute of every `<Rob>` root. The controller always sends `KUKA`; `SENTYPE` in the
/// config names only the `<Sen>` replies (seen on the KR C5 micro, KSS 8.7, RSI 5.0).
pub const ROB_TYPE: &str = "KUKA";

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

/// Token of the RSI telegram subset of XML: one root, one level of children (unknown
/// children may nest a little), attributes in double or single quotes, text content,
/// self-closing tags. `<?...?>` declarations and `<!-- -->` comments are skipped. Entity
/// references are not decoded (the robot sends numbers).
enum Token<'a> {
    /// `<name attrs>` or `<name attrs/>`; `attrs` is the raw text between the name and `>`/`/>`.
    Start { name: &'a str, attrs: &'a str, empty: bool },
    /// `</name>`; the parser matches `name` against the element it closes.
    End { name: &'a str },
    Text(&'a str),
    Eof,
}

/// Validates and slices the raw attribute text of a start tag as UTF-8.
fn raw(b: &[u8]) -> Result<&str, ParseError> {
    std::str::from_utf8(b).map_err(|_| ParseError::Xml("attribute text is not UTF-8".into()))
}

struct Tokenizer<'a> {
    src: &'a [u8],
    pos: usize,
}

impl<'a> Tokenizer<'a> {
    fn new(src: &'a [u8]) -> Self {
        Tokenizer { src, pos: 0 }
    }

    fn peek(&self) -> Option<u8> {
        self.src.get(self.pos).copied()
    }

    fn skip_ws(&mut self) {
        while matches!(self.peek(), Some(b) if b.is_ascii_whitespace()) {
            self.pos += 1;
        }
    }

    fn expect(&mut self, b: u8) -> Result<(), ParseError> {
        if self.peek() == Some(b) {
            self.pos += 1;
            Ok(())
        } else {
            Err(ParseError::Xml(format!("expected {:?} at byte {}", b as char, self.pos)))
        }
    }

    /// Name chars: ASCII alphanumerics, '_', '.', '-', ':'. Empty name is an error.
    fn read_name(&mut self) -> Result<&'a str, ParseError> {
        let start = self.pos;
        while matches!(self.peek(), Some(b) if b.is_ascii_alphanumeric() || matches!(b, b'_' | b'.' | b'-' | b':')) {
            self.pos += 1;
        }
        if self.pos == start {
            return Err(ParseError::Xml(format!("expected a name at byte {start}")));
        }
        std::str::from_utf8(&self.src[start..self.pos]).map_err(|_| ParseError::Xml("name is not UTF-8".into()))
    }

    /// Offset from `pos` of the first `needle` starting at or after `pos + skip`.
    fn find(&self, skip: usize, needle: &[u8]) -> Option<usize> {
        let hay = self.src.get(self.pos + skip..)?;
        hay.windows(needle.len()).position(|w| w == needle).map(|i| i + skip)
    }

    fn next(&mut self) -> Result<Token<'a>, ParseError> {
        loop {
            self.skip_ws();
            let Some(b) = self.peek() else { return Ok(Token::Eof) };
            if b != b'<' {
                // text up to the next '<', trailing whitespace trimmed
                let start = self.pos;
                while matches!(self.peek(), Some(c) if c != b'<') {
                    self.pos += 1;
                }
                let text = std::str::from_utf8(&self.src[start..self.pos]).map_err(|_| ParseError::Xml("text is not UTF-8".into()))?;
                return Ok(Token::Text(text.trim_end()));
            }
            let rest = &self.src[self.pos..];
            if rest.starts_with(b"<?") {
                // The closer may not reuse the opener's '?': `<?>` is not a declaration.
                let end = self.find(2, b"?>").ok_or_else(|| ParseError::Xml("unterminated declaration".into()))?;
                self.pos += end + 2;
                continue;
            }
            if rest.starts_with(b"<!--") {
                // Likewise `<!-->` is not a comment.
                let end = self.find(4, b"-->").ok_or_else(|| ParseError::Xml("unterminated comment".into()))?;
                self.pos += end + 3;
                continue;
            }
            if rest.starts_with(b"</") {
                self.pos += 2;
                let name = self.read_name()?;
                self.skip_ws();
                self.expect(b'>')?;
                return Ok(Token::End { name });
            }
            self.pos += 1;
            let name = self.read_name()?;
            let attr_start = self.pos;
            loop {
                self.skip_ws();
                match self.peek() {
                    None => return Err(ParseError::Xml(format!("unterminated start tag <{name}>"))),
                    Some(b'/') => {
                        let attr_end = self.pos;
                        self.pos += 1;
                        self.expect(b'>')?;
                        return Ok(Token::Start { name, attrs: raw(&self.src[attr_start..attr_end])?, empty: true });
                    }
                    Some(b'>') => {
                        let attr_end = self.pos;
                        self.pos += 1;
                        return Ok(Token::Start { name, attrs: raw(&self.src[attr_start..attr_end])?, empty: false });
                    }
                    Some(_) => {
                        // skip one attribute: name, '=', quoted value
                        self.read_name()?;
                        self.skip_ws();
                        self.expect(b'=')?;
                        self.skip_ws();
                        let q = match self.peek() {
                            Some(q @ (b'"' | b'\'')) => q,
                            _ => return Err(ParseError::Xml(format!("attribute value in <{name}> is not quoted"))),
                        };
                        self.pos += 1;
                        let close = self.src[self.pos..]
                            .iter()
                            .position(|c| *c == q)
                            .ok_or_else(|| ParseError::Xml(format!("unterminated attribute value in <{name}>")))?;
                        self.pos += close + 1;
                    }
                }
            }
        }
    }
}

/// Iterates `name="value"` pairs of a raw attribute region (already validated by the tokenizer).
struct Attrs<'a> {
    src: &'a [u8],
    pos: usize,
}

impl<'a> Iterator for Attrs<'a> {
    type Item = (&'a str, &'a str);

    fn next(&mut self) -> Option<Self::Item> {
        while matches!(self.src.get(self.pos), Some(b) if b.is_ascii_whitespace()) {
            self.pos += 1;
        }
        if self.pos >= self.src.len() {
            return None;
        }
        let name_start = self.pos;
        while matches!(self.src.get(self.pos), Some(b) if !b.is_ascii_whitespace() && *b != b'=') {
            self.pos += 1;
        }
        let name_end = self.pos;
        while matches!(self.src.get(self.pos), Some(b) if b.is_ascii_whitespace()) {
            self.pos += 1;
        }
        if self.src.get(self.pos) != Some(&b'=') {
            return None;
        }
        self.pos += 1;
        while matches!(self.src.get(self.pos), Some(b) if b.is_ascii_whitespace()) {
            self.pos += 1;
        }
        let q = *self.src.get(self.pos)?;
        self.pos += 1;
        let val_start = self.pos;
        while matches!(self.src.get(self.pos), Some(b) if *b != q) {
            self.pos += 1;
        }
        let val_end = self.pos;
        self.pos += 1; // skip closing quote
        // from_utf8 cannot fail here: the tokenizer already validated these same bytes as UTF-8.
        let name = std::str::from_utf8(&self.src[name_start..name_end]).ok()?;
        let value = std::str::from_utf8(&self.src[val_start..val_end]).ok()?;
        Some((name, value))
    }
}

/// Most elements open at once, root included. A telegram needs two (`Rob` and one of its
/// children); the rest is headroom for unknown elements, deeper nesting is rejected.
const MAX_DEPTH: usize = 8;

/// Reusable parser bound to one schema.
pub struct Parser {
    schema: Schema,
    /// Schema elements present in the telegram being parsed.
    seen: Vec<bool>,
    /// `seen` of the last telegram that parsed successfully.
    seen_good: Vec<bool>,
    missing: u64,
    unknown: u64,
}

fn parse_f64(s: &str, what: &str) -> Result<f64, ParseError> {
    let s = s.trim();
    if s.is_empty() {
        return Ok(0.0);
    }
    // `f64::from_str` also accepts NaN, inf and infinity, and overflows to inf ("1e999");
    // none of those may reach joint state.
    match s.parse::<f64>() {
        Ok(v) if v.is_finite() => Ok(v),
        _ => Err(ParseError::Number(format!("{what}={s:?}"))),
    }
}

impl Parser {
    pub fn new(schema: &Schema) -> Parser {
        Parser { schema: schema.clone(), seen: vec![false; schema.rob.len()], seen_good: vec![false; schema.rob.len()], missing: 0, unknown: 0 }
    }

    /// Whether the element of `keyword` was present in the last successfully parsed telegram.
    /// False before the first one and for keywords the schema does not contain. Values of absent
    /// elements are kept from earlier telegrams, so this is how a caller tells a fresh value from
    /// a stale one.
    pub fn seen(&self, keyword: Keyword) -> bool {
        self.schema
            .rob
            .iter()
            .position(|e| matches!(e, RobElement::Keyword(k) if *k == keyword))
            .is_some_and(|i| self.seen_good[i])
    }

    /// Schema elements absent from telegrams so far (cumulative).
    pub fn missing(&self) -> u64 {
        self.missing
    }

    /// Elements not in the schema so far (cumulative).
    pub fn unknown(&self) -> u64 {
        self.unknown
    }

    fn check_root(&self, name: &str, attrs: &str) -> Result<(), ParseError> {
        if name != "Rob" {
            return Err(ParseError::Root(name.to_string()));
        }
        let mut saw_type = false;
        for (key, value) in (Attrs { src: attrs.as_bytes(), pos: 0 }) {
            if key.eq_ignore_ascii_case("TYPE") {
                saw_type = true;
                if value != ROB_TYPE {
                    return Err(ParseError::Sentype { got: value.to_string(), want: ROB_TYPE.to_string() });
                }
            }
        }
        if !saw_type {
            return Err(ParseError::Sentype { got: String::new(), want: ROB_TYPE.to_string() });
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

    fn read_attrs(&self, idx: usize, attrs: &str, out: &mut RobotTelegram) -> Result<(), ParseError> {
        match &self.schema.rob[idx] {
            RobElement::Keyword(k) => {
                let names = k.attrs();
                for (key, value) in (Attrs { src: attrs.as_bytes(), pos: 0 }) {
                    let Some(i) = names.iter().position(|n| *n == key) else { continue };
                    let v = parse_f64(value, key)?;
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
            RobElement::Attrs { attrs: schema_attrs, .. } => {
                for (key, value) in (Attrs { src: attrs.as_bytes(), pos: 0 }) {
                    if let Some(attr) = schema_attrs.iter().find(|x| x.name == key) {
                        out.slots[attr.slot] = parse_f64(value, key)?;
                    }
                }
            }
            RobElement::Text { .. } => {}
        }
        Ok(())
    }

    /// Parse one telegram into `out`. Values of absent elements are left untouched.
    pub fn parse(&mut self, bytes: &[u8], out: &mut RobotTelegram) -> Result<(), ParseError> {
        let mut tok = Tokenizer::new(bytes);
        self.seen.iter_mut().for_each(|s| *s = false);
        // Names of the open elements, borrowed from `bytes`: `open[0]` is the root, `open[1]`
        // the current telegram element, deeper ones are nested unknown elements. `depth` is
        // how many are open.
        let mut open: [&str; MAX_DEPTH] = [""; MAX_DEPTH];
        let mut depth = 0usize;
        let mut root_seen = false;
        let mut got_ipoc = false;
        let mut text_target: Option<TextTarget> = None;
        loop {
            match tok.next()? {
                Token::Eof => break,
                Token::Start { name, attrs, empty } => {
                    match depth {
                        0 => {
                            if root_seen {
                                return Err(ParseError::Xml(format!("element <{name}> after the root element")));
                            }
                            self.check_root(name, attrs)?;
                            root_seen = true;
                            if empty {
                                break; // root self-closed: no children, no IPOC
                            }
                        }
                        1 => {
                            // Each telegram element starts without a text target, and a
                            // self-closed one never gets one: it has no text and no end tag
                            // to clear it, so a later sibling's text would land in it.
                            text_target = None;
                            if name == "IPOC" {
                                if !empty {
                                    text_target = Some(TextTarget::Ipoc);
                                }
                            } else if let Some(idx) = self.element_index(name) {
                                self.seen[idx] = true;
                                match &self.schema.rob[idx] {
                                    RobElement::Text { slot, .. } => {
                                        if !empty {
                                            text_target = Some(TextTarget::Slot(*slot));
                                        }
                                    }
                                    _ => self.read_attrs(idx, attrs, out)?,
                                }
                            } else {
                                self.unknown += 1;
                            }
                        }
                        // Nested deeper than the telegram grammar: not dispatched, only
                        // tracked so its end tag is matched.
                        _ => {}
                    }
                    if !empty {
                        if depth == MAX_DEPTH {
                            return Err(ParseError::Xml(format!("elements nested deeper than {MAX_DEPTH} levels")));
                        }
                        open[depth] = name;
                        depth += 1;
                    }
                }
                Token::End { name } => {
                    let Some(d) = depth.checked_sub(1) else {
                        return Err(ParseError::Xml(format!("end tag </{name}> outside the root element")));
                    };
                    if open[d] != name {
                        return Err(ParseError::Xml(format!("end tag </{name}> does not match <{}>", open[d])));
                    }
                    depth = d;
                    text_target = None;
                }
                Token::Text(s) => {
                    if depth == 2 {
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
            }
        }
        if depth != 0 {
            return Err(ParseError::Xml("unclosed root element".into()));
        }
        if !got_ipoc {
            return Err(ParseError::MissingIpoc);
        }
        self.missing += self.seen.iter().filter(|s| !**s).count() as u64;
        self.seen_good.copy_from_slice(&self.seen);
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

    const SHIPPED: &str = include_str!("../../controller/kuka_rsi_ethernet.xml");

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

    const SHIPPED_TELEGRAM: &[u8] = br#"<Rob Type="KUKA"><RIst X="600.1234" Y="-10.5" Z="780.0" A="180.0" B="70.0" C="-180.0" /><RSol X="600.0" Y="-10.0" Z="780.0" A="180.0" B="70.0" C="-180.0" /><AIPos A1="1.5" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.25" /><ASPos A1="1.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0" /><Delay D="2" /><GearTorque A1="0.1" A2="0.2" A3="0.3" A4="0.4" A5="0.5" A6="0.6" /><ProgStatus R="3" /><OvPro R="100.0" /><IPOC>4004</IPOC></Rob>"#;

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
        let xml = b"<Rob Type=\"KUKA\">\n  <AIPos A6=\"6\" A5=\"5\" A4=\"4\" A3=\"3\" A2=\"2\" A1=\"1\"/>\n  <IPOC> 8 </IPOC>\n</Rob>\n";
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
        let xml = b"<Rob Type=\"KUKA\"><Bogus Q=\"1\" /><Other>7</Other><IPOC>4008</IPOC></Rob>";
        p.parse(xml, &mut t).unwrap();
        assert_eq!(t.aipos[0], 1.5, "kept");
        assert_eq!(t.slots[6], 3.0, "kept");
        assert_eq!(t.ipoc, 4008);
        assert_eq!(p.unknown(), 2);
        assert_eq!(p.missing(), 8);
    }

    #[test]
    fn seen_reports_keywords_of_the_last_good_telegram() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        assert!(!p.seen(Keyword::AIPos), "nothing parsed yet");
        p.parse(SHIPPED_TELEGRAM, &mut t).unwrap();
        assert!(p.seen(Keyword::AIPos) && p.seen(Keyword::ASPos) && p.seen(Keyword::Delay));
        assert!(!p.seen(Keyword::MACur), "not in the schema");

        p.parse(b"<Rob Type=\"KUKA\"><AIPos A1=\"1\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\" /><IPOC>8</IPOC></Rob>", &mut t).unwrap();
        assert!(p.seen(Keyword::AIPos));
        assert!(!p.seen(Keyword::ASPos), "absent from the last telegram, although its value is kept");
        assert_eq!(t.aspos[0], 1.0);

        // A telegram that fails to parse leaves the answer of the last good one.
        assert!(p.parse(b"<Rob Type=\"KUKA\"><ASPos A1=\"2\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\" /></Rob>", &mut t).is_err());
        assert!(p.seen(Keyword::AIPos));
        assert!(!p.seen(Keyword::ASPos));
    }

    /// First telegram from the real controller (KR C5 micro, KSS 8.7, RSI 5.0), monitor config.
    const REAL_MONITOR: &[u8] = include_bytes!("../../tests/fixtures/kr10_monitor_first.xml");
    const SHIPPED_MONITOR: &str = include_str!("../../controller/kuka_rsi_monitor.xml");

    #[test]
    fn parses_the_first_real_telegram() {
        let s = schema(SHIPPED_MONITOR);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        p.parse(REAL_MONITOR, &mut t).unwrap();
        assert_eq!(t.ipoc, 3312758);
        assert_eq!(t.aipos, [-70.1103, -76.3958, 122.9867, -166.6549, 25.7014, -286.8477]);
        assert_eq!(t.aspos[5], -286.8476);
        assert_eq!(t.rist[0], 392.5202);
        assert_eq!(t.delay, 0);
        assert_eq!(&t.slots[..8], &[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 4.0, 100.0]);
        assert_eq!(p.missing(), 0);
        assert_eq!(p.unknown(), 0);
    }

    #[test]
    fn rejects_wrong_root_sentype_and_missing_ipoc() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        assert_eq!(p.parse(b"<Sen Type=\"KukaRsi\"><IPOC>1</IPOC></Sen>", &mut t), Err(ParseError::Root("Sen".into())));
        assert_eq!(
            p.parse(b"<Rob TYPE=\"Other\"><IPOC>1</IPOC></Rob>", &mut t),
            Err(ParseError::Sentype { got: "Other".into(), want: "KUKA".into() })
        );
        assert_eq!(
            p.parse(b"<Rob><IPOC>1</IPOC></Rob>", &mut t),
            Err(ParseError::Sentype { got: String::new(), want: "KUKA".into() })
        );
        assert_eq!(p.parse(b"<Rob Type=\"KUKA\"><Delay D=\"0\" /></Rob>", &mut t), Err(ParseError::MissingIpoc));
        assert!(matches!(p.parse(b"<Rob Type=\"KUKA\"><IPOC>x</IPOC></Rob>", &mut t), Err(ParseError::Number(_))));
        assert!(matches!(p.parse(b"<Rob Type=\"KUKA\"><IPOC>1</IPOC>", &mut t), Err(ParseError::Xml(_))));
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
        p.parse(b"<Rob Type=\"KUKA\"><Digout o1=\"1.000000\" o2=\"0.00\" /><DiL>42</DiL><IPOC>12</IPOC></Rob>", &mut t).unwrap();
        assert_eq!(&t.slots[0..3], &[1.0, 0.0, 42.0]);
    }

    #[test]
    fn parser_is_reusable_across_many_telegrams() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        for i in 0..1000u64 {
            let xml = format!("<Rob Type=\"KUKA\"><AIPos A1=\"{i}\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\" /><IPOC>{}</IPOC></Rob>", i * 4);
            p.parse(xml.as_bytes(), &mut t).unwrap();
            assert_eq!(t.aipos[0], i as f64);
            assert_eq!(t.ipoc, i * 4);
        }
    }

    #[test]
    fn tolerates_declaration_comment_single_quotes_and_nested_unknown() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        let xml = b"<?xml version=\"1.0\"?>\n<!-- comment --><Rob Type='KUKA'><Delay D='7'/><Extra><Deep x=\"1\">z</Deep></Extra><IPOC>12</IPOC></Rob>";
        p.parse(xml, &mut t).unwrap();
        assert_eq!(t.delay, 7);
        assert_eq!(t.ipoc, 12);
        assert_eq!(p.unknown(), 1, "Extra counted once, its child is not a telegram element");
    }

    #[test]
    fn rejects_malformed_tags() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        for bad in [
            &b"<Rob Type=\"KUKA\"><Delay D=7 /><IPOC>1</IPOC></Rob>"[..],
            b"<Rob Type=\"KUKA\"><Delay D=\"7 /><IPOC>1</IPOC></Rob>",
            b"<Rob Type=\"KUKA\"><IPOC>1</IPOC></Rob",
            b"<Rob Type=\"KUKA\"><IPOC>1<IPOC></Rob>",
        ] {
            assert!(matches!(p.parse(bad, &mut t), Err(ParseError::Xml(_))), "{:?}", std::str::from_utf8(bad));
        }
    }

    /// Schema whose first channel (slot 0) is the text element `DiL`.
    fn dil_schema() -> Schema {
        schema(
            "<ROOT><CONFIG><IP_NUMBER>127.0.0.1</IP_NUMBER><PORT>49152</PORT><SENTYPE>T</SENTYPE><ONLYSEND>TRUE</ONLYSEND></CONFIG>\
             <SEND><ELEMENTS><ELEMENT TAG=\"DiL\" TYPE=\"LONG\" INDX=\"1\" /></ELEMENTS></SEND><RECEIVE><ELEMENTS></ELEMENTS></RECEIVE></ROOT>",
        )
    }

    #[test]
    fn self_closing_text_element_does_not_capture_a_later_siblings_text() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        // A self-closed IPOC has no text; the text of a later unknown or keyword sibling is not IPOC.
        assert_eq!(p.parse(b"<Rob Type=\"KUKA\"><IPOC/><Extra>5</Extra></Rob>", &mut t), Err(ParseError::MissingIpoc));
        assert_eq!(p.parse(b"<Rob Type=\"KUKA\"><IPOC/><Delay D=\"3\">5</Delay></Rob>", &mut t), Err(ParseError::MissingIpoc));
        assert_eq!(t.ipoc, 0);

        let mut p = Parser::new(&dil_schema());
        let mut t = RobotTelegram::default();
        p.parse(b"<Rob Type=\"KUKA\"><DiL/><Other>42</Other><IPOC>1</IPOC></Rob>", &mut t).unwrap();
        assert_eq!(t.slots[0], 0.0, "Other's text must not land in the DiL slot");
        assert_eq!(t.ipoc, 1);
        assert_eq!(p.unknown(), 1);
    }

    #[test]
    fn rejects_mismatched_and_stray_end_tags() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        for bad in [
            &b"<Rob Type=\"KUKA\"><IPOC>1</IPOC></Rob></Rob>"[..],
            b"<Rob Type=\"KUKA\"><IPOC>1</Foo></Bar>",
            b"<Rob Type=\"KUKA\"><Extra></Z><AIPos A1=\"9\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\"/><IPOC>1</IPOC></Rob>",
            b"</Rob>",
            b"<Rob Type=\"KUKA\"><Extra><A><B></A></B></Extra><IPOC>1</IPOC></Rob>",
            b"<Rob Type=\"KUKA\"><IPOC>1</IPOC></Rob><Rob Type=\"KUKA\"><IPOC>2</IPOC></Rob>",
        ] {
            assert!(matches!(p.parse(bad, &mut t), Err(ParseError::Xml(_))), "{:?}", std::str::from_utf8(bad));
        }
        assert_eq!(t.aipos[0], 0.0, "AIPos inside the unclosed Extra must not be applied");
    }

    #[test]
    fn nesting_is_limited_to_eight_open_elements() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        // Rob plus `levels` nested unknown elements, all properly closed.
        let nested = |levels: usize| {
            let mut xml = String::from("<Rob Type=\"KUKA\">");
            for i in 0..levels {
                xml.push_str(&format!("<E{i}>"));
            }
            for i in (0..levels).rev() {
                xml.push_str(&format!("</E{i}>"));
            }
            xml.push_str("<IPOC>1</IPOC></Rob>");
            xml
        };
        p.parse(nested(7).as_bytes(), &mut t).unwrap();
        assert_eq!(t.ipoc, 1);
        assert!(matches!(p.parse(nested(8).as_bytes(), &mut t), Err(ParseError::Xml(_))));
    }

    #[test]
    fn declaration_and_comment_closers_do_not_overlap_their_openers() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        for bad in [&b"<?><Rob Type=\"KUKA\"><IPOC>1</IPOC></Rob>"[..], b"<!--><Rob Type=\"KUKA\"><IPOC>1</IPOC></Rob>"] {
            assert!(matches!(p.parse(bad, &mut t), Err(ParseError::Xml(_))), "{:?}", std::str::from_utf8(bad));
        }
        p.parse(b"<?xml version=\"1.0\"?><Rob Type=\"KUKA\"><IPOC>1</IPOC></Rob>", &mut t).unwrap();
        assert_eq!(t.ipoc, 1);
    }

    #[test]
    fn rejects_non_finite_numbers() {
        let s = schema(SHIPPED);
        let mut p = Parser::new(&s);
        let mut t = RobotTelegram::default();
        for v in ["NaN", "inf", "-inf", "infinity", "1e999"] {
            let xml = format!("<Rob Type=\"KUKA\"><AIPos A1=\"{v}\" A2=\"0\" A3=\"0\" A4=\"0\" A5=\"0\" A6=\"0\" /><IPOC>1</IPOC></Rob>");
            assert!(matches!(p.parse(xml.as_bytes(), &mut t), Err(ParseError::Number(_))), "A1={v}");
            assert!(t.aipos[0].is_finite(), "A1={v} reached joint state");
        }
        let mut p = Parser::new(&dil_schema());
        assert!(matches!(p.parse(b"<Rob Type=\"KUKA\"><DiL>NaN</DiL><IPOC>1</IPOC></Rob>", &mut t), Err(ParseError::Number(_))));
        assert!(t.slots[0].is_finite());
    }

    proptest::proptest! {
        #[test]
        fn parses_any_axis_values_in_any_attribute_order(vals in proptest::array::uniform6(-400.0f64..400.0), perm in 0usize..720, ipoc in 0u64..u64::MAX) {
            let s = schema(SHIPPED);
            let mut p = Parser::new(&s);
            let mut t = RobotTelegram::default();
            // Lehmer-code permutation of the six attribute positions.
            let mut order: Vec<usize> = (0..6).collect();
            let mut k = perm;
            let mut permuted = Vec::with_capacity(6);
            for i in (1..=6).rev() { let idx = k % i; k /= i; permuted.push(order.remove(idx)); }
            let names = ["A1", "A2", "A3", "A4", "A5", "A6"];
            let mut xml = String::from("<Rob Type=\"KUKA\"><AIPos");
            for &i in &permuted { xml.push_str(&format!(" {}=\"{:.4}\"", names[i], vals[i])); }
            xml.push_str(&format!(" /><IPOC>{ipoc}</IPOC></Rob>"));
            p.parse(xml.as_bytes(), &mut t).unwrap();
            for i in 0..6 { proptest::prop_assert!((t.aipos[i] - vals[i]).abs() < 1e-4); }
            proptest::prop_assert_eq!(t.ipoc, ipoc);
        }
    }
}
