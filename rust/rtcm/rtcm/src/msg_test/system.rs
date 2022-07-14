use super::*;
use dencode::{BytesMut, Decoder as _};
use std::{fs, io};

// RTCM section 3.5.10 example
#[test]
fn msg1029() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1029 {
        reference_station_id: U12(23),
        modified_julian_day_number: U16(132),
        seconds_of_day_utc: U17(59100),
        number_of_characters_to_follow: U7(21),
        number_of_code_units: U8(30),
        character_code_units: "UTF-8 проверка wörter".to_owned(),
    };

    let raw_msg = fs::read("test_data/1029.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg: Msg1029 = match frame.message {
        Message::Msg1029(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg, expected_msg);
    Ok(())
}
