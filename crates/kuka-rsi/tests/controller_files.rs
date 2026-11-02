//! The shipped controller files must stay consistent with each other and with the driver.

use kuka_rsi::config::EthernetConfig;

const ETH: &str = include_str!("../../../controller/kuka_rsi_ethernet.xml");
const MON: &str = include_str!("../../../controller/kuka_rsi_monitor.xml");
const RSIX: &str = include_str!("../../../controller/kuka_rsi.rsix");
const RSIX_MON: &str = include_str!("../../../controller/kuka_rsi_monitor.rsix");
const SRC: &str = include_str!("../../../controller/kuka_rsi.src");
const SRC_MON: &str = include_str!("../../../controller/kuka_rsi_monitor.src");

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
        assert!(r.contains("<Input InIdx=\"0\" OutObjId=\"RSI\" OutIdx=\"6\" />"), "Stop fed by Ethernet output 7 (MoveStop)");
        assert!(r.contains("<Parameter Name=\"Mode\" ParamId=\"0\" ParamValue=\"4\" />"), "ExitMoveCorr");
        assert!(r.contains("<Parameter Name=\"Precision\" ParamId=\"7\" ParamValue=\"4\" />"));
        assert!(r.contains("<Parameter Name=\"Timeout\" ParamId=\"0\" ParamValue=\"100\" />"));
        assert!(r.contains("<Parameter Name=\"UpperLimA2\" ParamId=\"7\" ParamValue=\"45\" />"), "KR10 A2 range");
        assert!(r.contains("Construct Name=\"Stop_1\""), "RSIVisual layout entry");
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
