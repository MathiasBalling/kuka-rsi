//! Wire format: the schema compiled from the Ethernet config, the parser for
//! robot telegrams (`<Rob>`) and the writer for sensor telegrams (`<Sen>`).

pub mod parse;
pub mod schema;

pub use parse::{ParseError, Parser, RobotTelegram};
pub use schema::{Attr, Keyword, RobElement, Schema, SenElement, MAX_SLOTS};
