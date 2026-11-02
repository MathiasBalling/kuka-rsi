//! Proves the real-time hot paths do not allocate. One test function on purpose: the
//! counting allocator is process-global, so this binary must not run other tests in
//! parallel with it. Later hot paths (writer, session) are added inside the same function.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

use kuka_rsi::config::EthernetConfig;
use kuka_rsi::telegram::{Parser, RobotTelegram, Schema, SensorTelegram, Writer};

struct Counting;

static COUNT: AtomicUsize = AtomicUsize::new(0);

unsafe impl GlobalAlloc for Counting {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        COUNT.fetch_add(1, Ordering::Relaxed);
        unsafe { System.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        COUNT.fetch_add(1, Ordering::Relaxed);
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL: Counting = Counting;

fn allocations() -> usize {
    COUNT.load(Ordering::Relaxed)
}

const SHIPPED: &str = include_str!("../../../controller/kuka_rsi_ethernet.xml");
const TELEGRAM: &[u8] = br#"<Rob TYPE="KukaRsi"><RIst X="600.1234" Y="-10.5" Z="780.0" A="180.0" B="70.0" C="-180.0" /><RSol X="600.0" Y="-10.0" Z="780.0" A="180.0" B="70.0" C="-180.0" /><AIPos A1="1.5" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.25" /><ASPos A1="1.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0" /><Delay D="2" /><GearTorque A1="0.1" A2="0.2" A3="0.3" A4="0.4" A5="0.5" A6="0.6" /><ProgStatus R="3" /><OvPro R="100.0" /><IPOC>4004</IPOC></Rob>"#;

#[test]
fn hot_paths_do_not_allocate() {
    let schema = Schema::compile(&EthernetConfig::from_str(SHIPPED).unwrap()).unwrap();

    // Parser: one warm-up parse, then a hundred parses must not touch the allocator.
    let mut parser = Parser::new(&schema);
    let mut tele = RobotTelegram::default();
    parser.parse(TELEGRAM, &mut tele).unwrap();
    let before = allocations();
    for _ in 0..100 {
        parser.parse(TELEGRAM, &mut tele).unwrap();
    }
    assert_eq!(allocations(), before, "Parser::parse must not allocate on the hot path");
    assert_eq!(tele.ipoc, 4004);

    // Writer: into a pre-sized buffer, a hundred telegrams must not touch the allocator.
    let writer = Writer::new(&schema);
    let mut sen = SensorTelegram { ipoc: 4004, ..Default::default() };
    sen.ak = [0.5, -1.25, 2.0, 0.0, 0.125, -0.0625];
    sen.estr.push_str("Error: test");
    let mut out = Vec::with_capacity(4096);
    writer.write(&sen, &mut out);
    let before = allocations();
    for _ in 0..100 {
        writer.write(&sen, &mut out);
    }
    assert_eq!(allocations(), before, "Writer::write must not allocate on the hot path");
    assert!(out.ends_with(b"<IPOC>4004</IPOC></Sen>"));
}
