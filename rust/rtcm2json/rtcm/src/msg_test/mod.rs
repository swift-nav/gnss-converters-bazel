mod antennas;
mod ephemeris;
mod msm;
mod observations;
mod ssr;
mod system;

use crate::*;
use dencode::{BytesMut, Decoder as _};
use std::{
    fs::{self, File},
    io,
};

const MSG_UNKNOWN_900_RAW: [u8; 25] = [
    0xd3, 0x00, 0x13, 0x38, 0x47, 0xd3, 0x02, 0x02, 0x98, 0x0e, 0xde, 0xef, 0x34, 0xb4, 0xbd, 0x62,
    0xac, 0x09, 0x41, 0x98, 0x6f, 0x33, 0xe8, 0x7e, 0x12,
];
const RESERVED: u8 = 0x00;

#[test]
fn check_start() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let check_start = fs::read("test_data/check_start.rtcm")?;
    let mut buf = BytesMut::from(&check_start[..]);
    let crc_errors_count = 3;

    for _ in 0..crc_errors_count {
        assert!(matches!(decoder.decode(&mut buf), Err(Error::CrcError(_))));
    }
    // println!("{}", PREAMBLE);
    let frame = decoder.decode(&mut buf).unwrap().unwrap();
    assert!(matches!(frame.message, Message::GloMsm7(_)));
    Ok(())
}

#[test]
fn unknown_message() -> Result<(), io::Error> {
    let raw_msm5 = fs::read("test_data/msm5.rtcm")?;
    let mut decoder = Decoder;
    let mut buffer = BytesMut::new();
    buffer.extend_from_slice(&MSG_UNKNOWN_900_RAW);
    buffer.extend_from_slice(&raw_msm5);

    let frame = decoder.decode(&mut buffer).unwrap().unwrap();
    match frame.message {
        Message::Unknown { msg_type, .. } => assert_eq!(msg_type, 900),
        _ => panic!("wrong messge id"),
    }

    let frame = decoder.decode(&mut buffer).unwrap().unwrap();
    assert!(matches!(frame.message, Message::GpsMsm5(_)));
    Ok(())
}

#[test]
fn two_messages() -> Result<(), io::Error> {
    let raw_msm5 = fs::read("test_data/msm5.rtcm")?;
    let raw_msm7 = fs::read("test_data/msm7.rtcm")?;
    let mut buffer = BytesMut::new();
    buffer.extend_from_slice(&raw_msm5);
    buffer.extend_from_slice(&raw_msm7);

    let mut decoder = Decoder;
    let frame = decoder.decode(&mut buffer).unwrap().unwrap();
    assert!(matches!(frame.message, Message::GpsMsm5(_)));
    let frame = decoder.decode(&mut buffer).unwrap().unwrap();
    assert!(matches!(frame.message, Message::GloMsm7(_)));
    Ok(())
}

#[test]
fn buffer_cleared_on_none() {
    let mut decoder = Decoder;
    let mut buffer = BytesMut::new();
    buffer.extend_from_slice(&[0, 0, 0, 0, 0]);
    let res = decoder.decode(&mut buffer);
    assert!(matches!(res, Ok(None)));
    assert!(buffer.is_empty());
}

#[test]
fn buffer_advanced_on_crc_error() -> Result<(), io::Error> {
    let raw_msg = fs::read("test_data/1005.rtcm")?;
    let mut decoder = Decoder;
    let mut buffer = BytesMut::new();
    buffer.extend_from_slice(&raw_msg[..]);
    buffer.extend_from_slice(&raw_msg[..]);
    // cause crc error in first message
    buffer[5] += 1;
    assert_eq!(buffer.len(), raw_msg.len() * 2);
    let res = decoder.decode(&mut buffer);
    assert!(matches!(res, Err(Error::CrcError(_))));
    assert_eq!(buffer.len(), raw_msg.len() * 2 - 1);
    let frame = decoder.decode(&mut buffer).unwrap().unwrap();
    assert!(matches!(frame.message, Message::Msg1005(_)));
    assert!(buffer.is_empty());
    Ok(())
}

#[test]
fn buffer_advanced_on_deku_error() -> Result<(), io::Error> {
    let raw_msg = fs::read("test_data/deku_error.rtcm")?;
    let mut decoder = Decoder;
    let mut buffer = BytesMut::new();
    buffer.extend_from_slice(&raw_msg[..]);
    let init_buf_len = buffer.len();

    let res = decoder.decode(&mut buffer);
    assert!(matches!(res, Err(Error::ParseError(_))));
    assert_eq!(buffer.len(), init_buf_len - 1);
    Ok(())
}

#[test]
fn reserializing_gives_same_input() -> Result<(), io::Error> {
    let files_to_skip = vec!["check_start.rtcm", "deku_error.rtcm"];

    for test_file in fs::read_dir("test_data")? {
        let test_file = test_file?;
        let test_file_name = test_file.file_name();

        if files_to_skip.contains(&test_file_name.to_string_lossy().as_ref()) {
            continue;
        }

        let file = File::open(test_file.path()).unwrap();

        let frames: Vec<Frame> = iter_messages(file)
            .map(|r| {
                r.expect(&format!(
                    "Couldn't deserialize {}",
                    test_file_name.to_string_lossy()
                ))
            })
            .collect();

        let serialized: Vec<u8> = frames
            .iter()
            .map(|frame| frame.to_bytes().expect("Couldn't serialize"))
            .flatten()
            .collect();

        let reserialize_frames: Vec<Frame> = iter_messages(&serialized[..])
            .map(|r| {
                r.expect(&format!(
                    "Couldn't deserialize {}",
                    test_file_name.to_string_lossy()
                ))
            })
            .collect();

        assert_eq!(
            reserialize_frames,
            frames,
            "{}",
            test_file_name.to_string_lossy()
        );
    }

    Ok(())
}

#[test]
fn check_payload() -> Result<(), io::Error> {
    let raw_msg = fs::read("test_data/1005.rtcm")?;
    let mut decoder = Decoder;

    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(
        frame.payload,
        &raw_msg[FRAME_HEADER_LEN..frame.msg_length as usize + FRAME_HEADER_LEN]
    );
    Ok(())
}
