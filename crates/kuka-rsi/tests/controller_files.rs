//! The shipped controller files must stay consistent with each other and with the driver.

use kuka_rsi::config::{EthernetConfig, Limits};

const ETH: &str = include_str!("../controller/kuka_rsi_ethernet.xml");
const MON: &str = include_str!("../controller/kuka_rsi_monitor.xml");
const RSIX: &str = include_str!("../controller/kuka_rsi.rsix");
const RSIX_MON: &str = include_str!("../controller/kuka_rsi_monitor.rsix");
const SRC: &str = include_str!("../controller/kuka_rsi.src");
const SRC_MON: &str = include_str!("../controller/kuka_rsi_monitor.src");

#[test]
fn ethernet_configs_parse() {
    let e = EthernetConfig::from_str(ETH).unwrap();
    let m = EthernetConfig::from_str(MON).unwrap();
    assert!(!e.onlysend && m.onlysend);
    assert_eq!(e.send, m.send);
    assert_eq!(e.receive, m.receive);
}

#[test]
fn contexts_reference_their_config_and_wire_the_stop_object() {
    assert!(RSIX.contains("ParamValue=\"kuka_rsi_ethernet.xml\""));
    assert!(RSIX_MON.contains("ParamValue=\"kuka_rsi_monitor.xml\""));
    for r in [RSIX, RSIX_MON] {
        assert!(r.contains("<RsiObject ObjTypeId=\"18\" ObjType=\"Stop\" ObjId=\"Stop_1\">"));
        assert!(
            r.contains("<Input InIdx=\"0\" OutObjId=\"RSI\" OutIdx=\"6\" />"),
            "Stop fed by Ethernet output 7 (MoveStop)"
        );
        assert!(r.contains("<Parameter Name=\"Mode\" ParamId=\"0\" ParamValue=\"4\" />"), "ExitMoveCorr");
        assert!(r.contains("<Parameter Name=\"Precision\" ParamId=\"7\" ParamValue=\"4\" />"));
        assert!(r.contains("<Parameter Name=\"Timeout\" ParamId=\"0\" ParamValue=\"100\" />"));
        assert!(r.contains("Construct Name=\"Stop_1\""), "RSIVisual layout entry");
    }
}

/// Every value the context gives parameter `name`: `ParamValue` in `<RsiObjects>`, `Value` in
/// `<Constructs>`. The parameter definitions of the type library carry neither and are skipped.
fn param_values(rsix: &str, name: &str) -> Vec<f64> {
    let needle = format!("Name=\"{name}\"");
    let mut out = Vec::new();
    for (i, _) in rsix.match_indices(&needle) {
        let rest = &rsix[i + needle.len()..];
        let tag = &rest[..rest.find('>').expect("unterminated tag")];
        for attr in [" ParamValue=\"", " Value=\""] {
            if let Some(j) = tag.find(attr) {
                let v = &tag[j + attr.len()..];
                out.push(v[..v.find('"').expect("unterminated value")].parse().expect("numeric value"));
            }
        }
    }
    out
}

/// `AK` is the correction relative to the start pose, so `AxisCorr` and `AxisCorrMon` bound it by
/// the axis range widths: they can never bind before the driver's own joint limits do.
#[test]
fn axis_correction_limits_are_the_range_widths() {
    let l = Limits::kr10_defaults();
    for (ctx, r) in [("kuka_rsi", RSIX), ("kuka_rsi_monitor", RSIX_MON)] {
        for i in 0..6 {
            let width = (l.position_max[i] - l.position_min[i]).to_degrees().round();
            let a = i + 1;
            for (name, want) in
                [(format!("LowerLimA{a}"), -width), (format!("UpperLimA{a}"), width), (format!("MaxA{a}"), width)]
            {
                let got = param_values(r, &name);
                assert_eq!(got.len(), 2, "{ctx}: {name} must be set in <RsiObjects> and <Constructs>, got {got:?}");
                assert!(got.iter().all(|v| *v == want), "{ctx}: {name} is {got:?}, expected {want}");
            }
        }
    }
}

#[test]
fn krl_programs_use_the_expected_calls() {
    assert!(SRC.contains("RSI_CREATE(\"kuka_rsi\", contid, TRUE)"));
    assert!(SRC.contains("RSI_ON(#ABSOLUTE)"));
    assert!(SRC.contains("RSI_MOVECORR()"));
    assert!(SRC.contains("RSI_OFF()"));
    assert!(SRC_MON.contains("RSI_CREATE(\"kuka_rsi_monitor\", contid, TRUE)"));
    assert!(SRC_MON.contains("WAIT SEC"));
    assert!(!SRC_MON.contains("RSI_MOVECORR"));
}

const BUNDLED: [&str; 9] = [
    "kuka_rsi_ethernet.xml",
    "kuka_rsi_monitor.xml",
    "kuka_rsi.rsix",
    "kuka_rsi_monitor.rsix",
    "kuka_rsi.src",
    "kuka_rsi_monitor.src",
    "LICENSE-FZI",
    "LICENSE-MIT",
    "LICENSE-APACHE",
];

/// Where each bundled file lands under the output folder: one folder per target on the robot,
/// so the pendant can copy a whole folder; the licenses stay at the top, on the PC side.
fn local_path(name: &str) -> String {
    match name {
        n if n.ends_with(".xml") || n.ends_with(".rsix") => format!("SensorInterface/{n}"),
        n if n.ends_with(".src") => format!("Program/{n}"),
        n => n.to_string(),
    }
}

#[test]
fn bundled_controller_files_match_the_controller_folder() {
    let names: Vec<&str> = kuka_rsi::controller::FILES.iter().map(|f| f.name).collect();
    assert_eq!(names, BUNDLED);
    for f in kuka_rsi::controller::FILES {
        assert_eq!(f.contents, std::fs::read_to_string(dir_of_shipped(f.name)).unwrap(), "{}", f.name);
    }
    let robot_dir = |n: &str| kuka_rsi::controller::FILES.iter().find(|f| f.name == n).unwrap().robot_dir;
    assert_eq!(robot_dir("kuka_rsi.src"), Some(r"KRC:\R1\Program\"));
    assert_eq!(robot_dir("kuka_rsi.rsix"), Some(r"C:\KRC\ROBOTER\Config\User\Common\SensorInterface\"));
    assert_eq!(robot_dir("LICENSE-FZI"), None);
    assert_eq!(robot_dir("LICENSE-MIT"), None);
}

#[test]
fn write_files_puts_every_bundled_file_in_the_directory() {
    let dir = std::env::temp_dir().join(format!("kuka-rsi-controller-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let written = kuka_rsi::controller::write_files(&dir.join("nested")).unwrap();
    assert_eq!(written.len(), BUNDLED.len());
    for (path, name) in written.iter().zip(BUNDLED) {
        assert_eq!(path, &dir.join("nested").join(local_path(name)));
        let bytes = std::fs::read(path).unwrap();
        let text = String::from_utf8(bytes.clone()).unwrap();
        if name.ends_with(".src") {
            // KRL programs go out with CRLF only, as the controller's editor writes them.
            assert_eq!(text.matches("\r\n").count(), text.matches('\n').count(), "{name}");
            assert_eq!(text.replace("\r\n", "\n"), std::fs::read_to_string(dir_of_shipped(name)).unwrap());
        } else {
            assert_eq!(text, std::fs::read_to_string(dir_of_shipped(name)).unwrap(), "{name}");
        }
    }
    std::fs::remove_dir_all(&dir).unwrap();
}

fn dir_of_shipped(name: &str) -> std::path::PathBuf {
    let crate_dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    // The project's own licenses sit at the crate root, everything else in controller/.
    if name == "LICENSE-MIT" || name == "LICENSE-APACHE" {
        crate_dir.join(name)
    } else {
        crate_dir.join("controller").join(name)
    }
}

#[test]
fn write_files_removes_the_macos_appledouble_files_it_causes() {
    // On FAT/exFAT sticks macOS stores extended attributes in "._<name>" files next to every
    // file and folder it writes; the controller would list them as broken programs and configs.
    let dir = std::env::temp_dir().join(format!("kuka-rsi-appledouble-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("._usb"), b"apple double of the folder").unwrap();
    std::fs::write(dir.join("._unrelated"), b"not ours").unwrap();
    let out = dir.join("usb");
    std::fs::create_dir_all(&out).unwrap();
    std::fs::write(out.join("._kuka_rsi.src"), b"stale").unwrap();

    kuka_rsi::controller::write_files(&out.join("files")).unwrap();
    // Simulate what macOS writes for the new folder and files on a FAT stick, then write again.
    std::fs::write(out.join("._files"), b"x").unwrap();
    std::fs::write(out.join("files").join("SensorInterface").join("._kuka_rsi.rsix"), b"x").unwrap();
    std::fs::write(out.join("files").join("._Program"), b"x").unwrap();
    kuka_rsi::controller::write_files(&out.join("files")).unwrap();

    assert!(!out.join("._files").exists());
    assert!(!out.join("files").join("SensorInterface").join("._kuka_rsi.rsix").exists());
    assert!(!out.join("files").join("._Program").exists(), "the subfolders' AppleDouble files too");
    assert!(out.join("._kuka_rsi.src").exists(), "only AppleDouble files of what it wrote");
    assert!(dir.join("._unrelated").exists());
    std::fs::remove_dir_all(&dir).unwrap();
}

#[test]
fn the_program_starts_and_ends_where_the_robot_stands() {
    // Tested at the robot: no PTP to a fixed pose before RSI starts or after it ends.
    let src = std::fs::read_to_string(dir_of_shipped("kuka_rsi.src")).unwrap();
    let ptps: Vec<&str> = src.lines().map(str::trim).filter(|l| l.starts_with("PTP")).collect();
    assert_eq!(ptps, ["PTP $AXIS_ACT", "PTP $AXIS_ACT"]);
}
