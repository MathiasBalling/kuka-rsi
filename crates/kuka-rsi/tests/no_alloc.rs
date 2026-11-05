//! Proves the real-time hot paths do not allocate. One test function on purpose: the
//! counting allocator is process-global, so this binary must not run other tests in
//! parallel with it. Later hot paths (writer, session) are added inside the same function.

use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicUsize, Ordering};

use arrayvec::{ArrayString, ArrayVec};
use kuka_rsi::config::EthernetConfig;
use kuka_rsi::config::Limits;
use kuka_rsi::control::{Command, Session};
use kuka_rsi::link::UdpLink;
use kuka_rsi::state::{deg2rad6, RobotState};
use kuka_rsi::telegram::{Keyword, Parser, RobotTelegram, Schema, SensorTelegram, Writer};
use std::net::UdpSocket;
use std::time::Duration;

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

const SHIPPED: &str = include_str!("../controller/kuka_rsi_ethernet.xml");
const TELEGRAM: &[u8] = br#"<Rob Type="KUKA"><RIst X="600.1234" Y="-10.5" Z="780.0" A="180.0" B="70.0" C="-180.0" /><RSol X="600.0" Y="-10.0" Z="780.0" A="180.0" B="70.0" C="-180.0" /><AIPos A1="1.5" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.25" /><ASPos A1="1.0" A2="-90.0" A3="90.0" A4="0.0" A5="90.0" A6="0.0" /><Delay D="2" /><GearTorque A1="0.1" A2="0.2" A3="0.3" A4="0.4" A5="0.5" A6="0.6" /><ProgStatus R="3" /><OvPro R="100.0" /><IPOC>4004</IPOC></Rob>"#;

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
    let started = parser.seen(Keyword::AIPos) && parser.seen(Keyword::ASPos);
    assert_eq!(allocations(), before, "Parser::parse and Parser::seen must not allocate on the hot path");
    assert!(started);
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

    // Link: after one warm-up exchange, a hundred receive/reply cycles must not touch the allocator.
    let mut link = UdpLink::bind("127.0.0.1:0".parse().unwrap(), Duration::from_millis(200)).unwrap();
    let robot = UdpSocket::bind("127.0.0.1:0").unwrap();
    robot.set_read_timeout(Some(Duration::from_millis(200))).unwrap();
    let link_addr = link.local_addr().unwrap();
    let mut rx = vec![0u8; 65536];
    let mut reply = [0u8; 512];
    robot.send_to(TELEGRAM, link_addr).unwrap();
    link.recv_newest(&mut rx).unwrap().unwrap();
    link.send(&out).unwrap();
    robot.recv_from(&mut reply).unwrap();
    let before = allocations();
    for _ in 0..100 {
        robot.send_to(TELEGRAM, link_addr).unwrap();
        let got = link.recv_newest(&mut rx).unwrap().unwrap();
        assert_eq!(got.len, TELEGRAM.len());
        link.send(&out).unwrap();
        robot.recv_from(&mut reply).unwrap();
    }
    assert_eq!(allocations(), before, "UdpLink recv/send must not allocate on the hot path");

    // Session: after start and one queued chunk, a hundred cycle steps must not touch the allocator.
    let limits = Limits::kr10_defaults().with_max_velocity_all(1.0);
    let mut session = Session::new(limits, Duration::from_millis(4), Some(6));
    session.start(&tele);
    let mut state = RobotState { q: deg2rad6(&tele.aipos), q_setpoint: deg2rad6(&tele.aspos), speed_override: 1.0, program_running: true, ..Default::default() };
    let mut target = session.q_start();
    target[0] += 0.02;
    let mut points = ArrayVec::new();
    points.push(target);
    session.apply(Command::Stream { points, cycles_per_point: 50.0, replace: false, batch_remaining: 0, first: true }).unwrap();
    session.apply(Command::Notify(ArrayString::from("hello").unwrap())).unwrap();
    let mut sen = SensorTelegram::default();
    session.step(&tele, &state, &mut sen);
    let before = allocations();
    for _ in 0..100 {
        let q = session.step(&tele, &state, &mut sen);
        state.q = q;
        let _ = session.take_events();
    }
    assert_eq!(allocations(), before, "Session::step must not allocate on the hot path");
    assert!(sen.ak[0] > 0.0);
}
