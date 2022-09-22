use super::*;
use dencode::{BytesMut, Decoder as _};
use std::{fs, io};

#[test]
fn msg1004() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1004 {
        header: Header1004 {
            reference_station_id: U12(3315),
            gps_epoch_time_tow: U30(75000000),
            synchronous_gnss_flag: Bit1(false),
            no_of_gps_satellite_signals_processed: U5(1),
            gps_divergence_free_smoothing_indicator: Bit1(true),
            gps_smoothing_interval: U3(5),
        },
        satellites: vec![Satellite1004 {
            satellite_id: U6(22),
            l1_code_indicator: Bit1(false),
            l1_pseudorange: U24(5778978),
            l1_phaserange_l1_pseudorange: I20(-6297),
            l1_lock_time_indicator: U7(106),
            integer_l1_pseudorange_modulus_ambiguity: U8(70),
            l1_cnr: U8(203),
            l2_code_indicator: U2(0),
            l2_l1_pseudorange_difference: I14(-6391),
            l2_phaserange_l1_pseudorange: I20(-24834),
            l2_lock_time_indicator: U7(83),
            l2_cnr: U8(141),
        }],
    };

    let raw_msg = fs::read("test_data/1004.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg: Msg1004 = match frame.message {
        Message::Msg1004(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg, expected_msg);
    Ok(())
}

#[test]
fn msg1012() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1012 {
        header: Header1012 {
            reference_station_id: U12(3315),
            glo_epoch_time_tk: U27(85801000),
            synchronous_gnss_flag: Bit1(true),
            no_of_glo_satellite_signals_processed: U5(1),
            glo_divergence_free_smoothing_indicator: Bit1(true),
            glo_smoothing_interval: U3(5),
        },
        satellites: vec![Satellite1012 {
            satellite_id: U6(22),
            l1_code_indicator: Bit1(false),
            satellite_frequency_channel_number: U5(12),
            l1_pseudorange: U25(5778978),
            l1_phaserange_l1_pseudorange: I20(-51332),
            l1_lock_time_indicator: U7(106),
            int_l1_pseudorange_modulus_ambiguity: U7(35),
            l1_cnr: U8(203),
            l2_code_indicator: U2(0),
            l2_l1_pseudorange_difference: I14(-6391),
            l2_phaserange_l1_pseudorange: I20(90471),
            l2_lock_time_indicator: U7(83),
            l2_cnr: U8(141),
        }],
    };

    let raw_msg = fs::read("test_data/1012.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg: Msg1012 = match frame.message {
        Message::Msg1012(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg, expected_msg);
    Ok(())
}

#[test]
fn msg1230_1() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1230 {
        reference_station_id: U12(0),
        glo_code_phase_bias_indicator: Bit1(true),
        reserved: U3(0),
        glo_fdma_signals_mask: U4(15),
        glo_l1_c_a_code_phase_bias: Some(I16(0)),
        glo_l1_p_code_phase_bias: Some(I16(0)),
        glo_l2_c_a_code_phase_bias: Some(I16(0)),
        glo_l2_p_code_phase_bias: Some(I16(0)),
    };

    let raw_msg = fs::read("test_data/1230-1.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg: Msg1230 = match frame.message {
        Message::Msg1230(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg, expected_msg);
    Ok(())
}

#[test]
fn msg1230_2() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1230 {
        reference_station_id: U12(0),
        glo_code_phase_bias_indicator: Bit1(true),
        reserved: U3(0),
        glo_fdma_signals_mask: U4(10),
        glo_l1_c_a_code_phase_bias: Some(I16(1)),
        glo_l1_p_code_phase_bias: None,
        glo_l2_c_a_code_phase_bias: Some(I16(2)),
        glo_l2_p_code_phase_bias: None,
    };

    let raw_msg = fs::read("test_data/1230-2.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg: Msg1230 = match frame.message {
        Message::Msg1230(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg, expected_msg);
    Ok(())
}
