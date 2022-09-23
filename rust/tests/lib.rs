use assert_cmd::Command;
use predicates::prelude::predicate;

#[test]
fn rtcm3tosbp_help() {
    let mut cmd = Command::cargo_bin("rtcm3tosbp").unwrap();
    cmd.arg("-h")
        .assert()
        .success()
        .stderr(predicate::str::contains("this message"));
}

#[test]
fn rtcm3tosbp_convert() {
    let tempfile = tempfile::NamedTempFile::new().unwrap();
    let mut cmd = Command::cargo_bin("rtcm3tosbp").unwrap();
    cmd.arg("-o")
        .arg("-w")
        .arg("2080:123")
        .arg("../c/rtcm3tosbp/test/data/piksi-5Hz.rtcm3")
        .arg(tempfile.path())
        .assert()
        .success();
    let stat = std::fs::metadata(tempfile.path()).unwrap();
    assert!(stat.len() > 0);
    let converted = std::fs::File::open(tempfile.path()).unwrap();
    let message_count = sbp::iter_messages(converted).count();
    assert_eq!(message_count, 166);
}

#[test]
fn sbp2rtcm_convert() {
    let tempfile = tempfile::NamedTempFile::new().unwrap();
    let mut cmd = Command::cargo_bin("sbp2rtcm").unwrap();
    cmd.arg("../c/rtcm3tosbp/test/data/piksi-gps-glo.sbp")
        .arg(tempfile.path())
        .assert()
        .success();
    let stat = std::fs::metadata(tempfile.path()).unwrap();
    assert!(stat.len() == 22846);
    let converted = std::fs::File::open(tempfile.path()).unwrap();
    let message_count = rtcm::iter_messages(converted).count();
    assert_eq!(message_count, 164);
}

#[test]
fn sbp2rtcm_help() {
    let mut cmd = Command::cargo_bin("sbp2rtcm").unwrap();
    cmd.arg("-h")
        .assert()
        .success()
        .stderr(predicate::str::contains("this message"));
}

#[test]
fn ixcom2sbp_help() {
    let mut cmd = Command::cargo_bin("ixcom2sbp").unwrap();
    cmd.arg("-h")
        .assert()
        .success()
        .stderr(predicate::str::contains("this message"));
}

#[test]
fn ubx2sbp_help() {
    let mut cmd = Command::cargo_bin("ubx2sbp").unwrap();
    cmd.arg("-h")
        .assert()
        .success()
        .stderr(predicate::str::contains("this message"));
}

#[cfg(feature = "nov2sbp")]
#[test]
fn nov2sbp_help() {
    let mut cmd = Command::cargo_bin("nov2sbp").unwrap();
    cmd.arg("-h")
        .assert()
        .success()
        .stderr(predicate::str::contains("this message"));
}

#[cfg(test)]
#[cfg(feature = "json")]
mod json {
    use super::*;

    const RTCM_JSON: &str = r#"{"msg_length":35,"payload":"TsOaWWAAAADthABFuBAA5vBB/dSBB4iyhB78zhACmXBB7QA=","msg_type":1260,"header":{"bds_epoch_time_1s":236121,"ssr_update_interval":6,"multiple_message_indicator":false,"iod_ssr":0,"ssr_provider_id":0,"ssr_solution_id":0,"no_of_satellites":7},"sv_blocks":[{"satellite":{"bds_satellite_id":27,"no_of_code_biases_processed":1},"code":[{"bds_signal_and_tracking_mode_indicator":0,"code_bias":139}]},{"satellite":{"bds_satellite_id":28,"no_of_code_biases_processed":1},"code":[{"bds_signal_and_tracking_mode_indicator":0,"code_bias":115}]},{"satellite":{"bds_satellite_id":30,"no_of_code_biases_processed":1},"code":[{"bds_signal_and_tracking_mode_indicator":0,"code_bias":-70}]},{"satellite":{"bds_satellite_id":36,"no_of_code_biases_processed":1},"code":[{"bds_signal_and_tracking_mode_indicator":0,"code_bias":-955}]},{"satellite":{"bds_satellite_id":37,"no_of_code_biases_processed":1},"code":[{"bds_signal_and_tracking_mode_indicator":0,"code_bias":-519}]},{"satellite":{"bds_satellite_id":39,"no_of_code_biases_processed":1},"code":[{"bds_signal_and_tracking_mode_indicator":0,"code_bias":332}]},{"satellite":{"bds_satellite_id":46,"no_of_code_biases_processed":1},"code":[{"bds_signal_and_tracking_mode_indicator":0,"code_bias":-608}]}],"num_padding_bits":3,"padding":0,"crc":965134}"#;
    const RTCM_BUF: &[u8] = &[
        0xd3, 0x00, 0x23, 0x4e, 0xc3, 0x9a, 0x59, 0x60, 0x00, 0x00, 0x00, 0xed, 0x84, 0x00, 0x45,
        0xb8, 0x10, 0x00, 0xe6, 0xf0, 0x41, 0xfd, 0xd4, 0x81, 0x07, 0x88, 0xb2, 0x84, 0x1e, 0xfc,
        0xce, 0x10, 0x02, 0x99, 0x70, 0x41, 0xed, 0x00, 0x0e, 0xba, 0x0e,
    ];

    #[test]
    fn rtcm_converter_help() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();
        cmd.arg("-h")
            .assert()
            .success()
            .stdout(predicate::str::contains(
                "Convert to and from rtcm data with various formats.",
            ));
    }
    #[test]
    fn rtcm_converter_rtcm2expanded_frame() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();
        cmd.arg("rtcm")
            .arg("expanded")
            .write_stdin(RTCM_BUF)
            .assert()
            .success()
            .stdout(predicate::str::starts_with(RTCM_JSON));
    }

    #[test]
    fn rtcm_converter_expanded2rtcm_frame() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("expanded")
            .arg("rtcm")
            .write_stdin(RTCM_JSON)
            .assert()
            .success()
            .stdout(predicate::eq(RTCM_BUF));
    }

    #[test]
    fn rtcm_converter_rtcm2expanded_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("rtcm")
            .arg("expanded")
            .arg("tests/data/piksi-5Hz_signed.rtcm")
            .assert()
            .success();
    }

    #[test]
    fn rtcm_converter_rtcm2rtcm_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("rtcm")
            .arg("rtcm")
            .arg("tests/data/piksi-5Hz_signed.rtcm")
            .assert()
            .failure()
            .stderr(predicate::str::contains(
                "Input and output formats are the same.",
            ));
    }

    #[test]
    fn rtcm_converter_rtcm2expanded_plus_timestamps_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("rtcm")
            .arg("expanded-plus-timestamps")
            .arg("tests/data/piksi-5Hz_signed.rtcm")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Can only convert"));
    }

    #[test]
    fn rtcm_converter_rtcm2minimal_plus_timestamps_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("rtcm")
            .arg("minimal-plus-timestamps")
            .arg("tests/data/piksi-5Hz_signed.rtcm")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Can only convert"));
    }

    #[test]
    fn rtcm_converter_rtcm2minimal_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("rtcm")
            .arg("minimal")
            .arg("tests/data/piksi-5Hz_signed.rtcm")
            .assert()
            .success();
    }

    #[test]
    fn rtcm_converter_expanded2rtcm_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("expanded")
            .arg("rtcm")
            .arg("tests/data/piksi-5Hz_signed.expanded")
            .assert()
            .success();
    }

    #[test]
    fn rtcm_converter_expanded2expanded_plus_timestamps_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("expanded")
            .arg("expanded-plus-timestamps")
            .arg("tests/data/piksi-5Hz_signed.expanded")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Can only convert"));
    }

    #[test]
    fn rtcm_converter_expanded2minimal_plus_timestamps_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("expanded")
            .arg("minimal-plus-timestamps")
            .arg("tests/data/piksi-5Hz_signed.expanded")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Can only convert"));
    }

    #[test]
    fn rtcm_converter_expanded2expanded_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("expanded")
            .arg("expanded")
            .arg("tests/data/piksi-5Hz_signed.expanded")
            .assert()
            .failure()
            .stderr(predicate::str::contains(
                "Input and output formats are the same.",
            ));
    }

    #[test]
    fn rtcm_converter_expanded2minimal_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("expanded")
            .arg("minimal")
            .arg("tests/data/piksi-5Hz_signed.expanded")
            .assert()
            .success();
    }

    #[test]
    fn rtcm_converter_minimal2rtcm_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("minimal")
            .arg("rtcm")
            .arg("tests/data/piksi-5Hz_signed.minimal")
            .assert()
            .success();
    }

    #[test]
    fn rtcm_converter_minimal2minimal_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("minimal")
            .arg("minimal")
            .arg("tests/data/piksi-5Hz_signed.minimal")
            .assert()
            .failure()
            .stderr(predicate::str::contains(
                "Input and output formats are the same.",
            ));
    }

    #[test]
    fn rtcm_converter_minimal2expanded_plus_timestamps_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("minimal")
            .arg("expanded-plus-timestamps")
            .arg("tests/data/piksi-5Hz_signed.minimal")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Can only convert"));
    }

    #[test]
    fn rtcm_converter_minimal2minimal_plus_timestamps_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("minimal")
            .arg("minimal-plus-timestamps")
            .arg("tests/data/piksi-5Hz_signed.minimal")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Can only convert"));
    }

    #[test]
    fn rtcm_converter_minimal2expanded_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("minimal")
            .arg("expanded")
            .arg("tests/data/piksi-5Hz_signed.minimal")
            .assert()
            .success();
    }

    #[test]
    fn rtcm_converter_expanded_plus_timestamps2minimal_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("expanded-plus-timestamps")
            .arg("minimal")
            .arg("tests/data/piksi-5Hz_signed.expanded_plus_timestamps")
            .assert()
            .success();
    }

    #[test]
    fn rtcm_converter_expanded_plus_timestamps2expanded_plus_timestamps_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("expanded-plus-timestamps")
            .arg("expanded-plus-timestamps")
            .arg("tests/data/piksi-5Hz_signed.expanded_plus_timestamps")
            .assert()
            .failure()
            .stderr(predicate::str::contains(
                "Input and output formats are the same.",
            ));
    }

    #[test]
    fn rtcm_converter_expanded_plus_timestamps2rtcm_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("expanded-plus-timestamps")
            .arg("rtcm")
            .arg("tests/data/piksi-5Hz_signed.expanded_plus_timestamps")
            .assert()
            .success();
    }

    #[test]
    fn rtcm_converter_expanded_plus_timestamps2minimal_plus_timestamps_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("expanded-plus-timestamps")
            .arg("minimal-plus-timestamps")
            .arg("tests/data/piksi-5Hz_signed.expanded_plus_timestamps")
            .assert()
            .success();
    }

    #[test]
    fn rtcm_converter_expanded_plus_timestamps2minimal_plus_timestamps_with_split_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("expanded-plus-timestamps")
            .arg("minimal-plus-timestamps")
            .arg("--split-interval")
            .arg("hour")
            .arg("tests/data/piksi-5Hz_signed.expanded_plus_timestamps")
            .assert()
            .failure()
            .stderr(predicate::str::contains("Currently split-interval can"));
    }

    #[test]
    fn rtcm_converter_minimal_plus_timestamps2expanded_plus_timestamps_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("minimal-plus-timestamps")
            .arg("expanded-plus-timestamps")
            .arg("tests/data/piksi-5Hz_signed.minimal_plus_timestamps")
            .assert()
            .success();
    }

    #[test]
    fn rtcm_converter_minimal_plus_timestamps2minimal_plus_timestamps_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("minimal-plus-timestamps")
            .arg("minimal-plus-timestamps")
            .arg("tests/data/piksi-5Hz_signed.minimal_plus_timestamps")
            .assert()
            .failure()
            .stderr(predicate::str::contains(
                "Input and output formats are the same.",
            ));
    }

    #[test]
    fn rtcm_converter_minimal_plus_timestamps2minimal_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("minimal-plus-timestamps")
            .arg("minimal")
            .arg("tests/data/piksi-5Hz_signed.minimal_plus_timestamps")
            .assert()
            .success();
    }

    #[test]
    fn rtcm_converter_minimal_plus_timestamps2rtcm_file() {
        let mut cmd = Command::cargo_bin("rtcm-converter").unwrap();

        cmd.arg("minimal-plus-timestamps")
            .arg("rtcm")
            .arg("tests/data/piksi-5Hz_signed.minimal_plus_timestamps")
            .assert()
            .success();
    }
}
