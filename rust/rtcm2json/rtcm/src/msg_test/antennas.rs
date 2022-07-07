use super::*;
use dencode::{BytesMut, Decoder as _};
use std::{fs, io};

// RTCM section 4.2 example
#[test]
fn msg1005() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let raw_msg = fs::read("test_data/1005.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();

    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    assert_eq!(frame.msg_length, 19);
    let msg: Msg1005 = match frame.message {
        Message::Msg1005(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg.reference_station_id.0, 2003);
    assert!(msg.gps.0);
    assert!(!msg.glonass.0);
    assert!(!msg.galileo.0);
    assert_eq!(msg.antenna_reference_point_ecef_x.0, 11141045999);
    assert_eq!(msg.antenna_reference_point_ecef_y.0, -48507297108);
    assert_eq!(msg.antenna_reference_point_ecef_z.0, 39755214643);
    Ok(())
}

#[test]
fn msg1006() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_msg = Msg1006 {
        reference_station_id: U12(2315),
        reserved_itrf_realization_year: U6(0),
        gps: Bit1(true),
        glonass: Bit1(true),
        galileo: Bit1(false),
        reference_station: Bit1(false),
        antenna_reference_point_ecef_x: I38(-26942178790),
        single_receiver_oscillator: Bit1(true),
        reserved: Bit1(false),
        antenna_reference_point_ecef_y: I38(-42640792110),
        quarter_cycle_indicator: U2(0),
        antenna_reference_point_ecef_z: I38(38906470312),
        antenna_height: U16(65535),
    };

    let raw_msg = fs::read("test_data/1006.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg: Msg1006 = match frame.message {
        Message::Msg1006(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg, expected_msg);
    Ok(())
}

#[test]
fn msg1008() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_msg = Msg1008 {
        reference_station_id: U12(55),
        descriptor_counter: U8(14),
        antenna_descriptor: "ADVNULLANTENNA".to_owned(),
        antenna_setup_id: U8(0),
        serial_number_counter: U8(0),
        antenna_serial_number: "".to_owned(),
    };

    let raw_msg = fs::read("test_data/1008.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg: Msg1008 = match frame.message {
        Message::Msg1008(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg, expected_msg);
    Ok(())
}

#[test]
fn msg1033() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_msg = Msg1033 {
        reference_station_id: U12(1132),
        antenna_descriptor_counter: U8(3),
        antenna_descriptor: "ANT".to_owned(),
        antenna_setup_id: U8(32),
        antenna_serial_number_counter: U8(3),
        antenna_serial_number: "123".to_owned(),
        receiver_type_descriptor_counter: U8(3),
        receiver_type_descriptor: "RCV".to_owned(),
        receiver_firmware_version_counter: U8(3),
        receiver_firmware_version: "1.0".to_owned(),
        receiver_serial_number_counter: U8(3),
        receiver_serial_number: "xxx".to_owned(),
    };

    let raw_msg = fs::read("test_data/1033.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg: Msg1033 = match frame.message {
        Message::Msg1033(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg, expected_msg);
    Ok(())
}
