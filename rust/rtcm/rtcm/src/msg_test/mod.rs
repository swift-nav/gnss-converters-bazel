mod antennas;
mod ephemeris;
mod msm;
mod observations;
mod ssr;
mod system;

use crate::Frame;
use crate::*;
use dencode::{BytesMut, Decoder as _};
use std::{
    fs::{self, File},
    io::{self, Read},
};

const MSG_UNKNOWN_900_RAW: [u8; 25] = [
    0xd3, 0x00, 0x13, 0x38, 0x47, 0xd3, 0x02, 0x02, 0x98, 0x0e, 0xde, 0xef, 0x34, 0xb4, 0xbd, 0x62,
    0xac, 0x09, 0x41, 0x98, 0x6f, 0x33, 0xe8, 0x7e, 0x12,
];
const RESERVED: u8 = 0x00;

#[test]
fn check_start() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
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
    let mut decoder = RtcmDecoder;
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

    let mut decoder = RtcmDecoder;
    let frame = decoder.decode(&mut buffer).unwrap().unwrap();
    assert!(matches!(frame.message, Message::GpsMsm5(_)));
    let frame = decoder.decode(&mut buffer).unwrap().unwrap();
    assert!(matches!(frame.message, Message::GloMsm7(_)));
    Ok(())
}

#[test]
fn buffer_cleared_on_none() {
    let mut decoder = RtcmDecoder;
    let mut buffer = BytesMut::new();
    buffer.extend_from_slice(&[0, 0, 0, 0, 0]);
    let res = decoder.decode(&mut buffer);
    assert!(matches!(res, Ok(None)));
    assert!(buffer.is_empty());
}

#[test]
fn buffer_advanced_on_crc_error() -> Result<(), io::Error> {
    let raw_msg = fs::read("test_data/1005.rtcm")?;
    let mut decoder = RtcmDecoder;
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
    let mut decoder = RtcmDecoder;
    let mut buffer = BytesMut::new();
    buffer.extend_from_slice(&raw_msg[..]);
    let init_buf_len = buffer.len();

    let res = decoder.decode(&mut buffer);
    assert!(matches!(res, Err(Error::ParseError(_))));
    assert_eq!(buffer.len(), init_buf_len - 1);
    Ok(())
}

#[test]
fn buffer_advanced_on_json_error() -> Result<(), io::Error> {
    let mut buffer = BytesMut::new();
    buffer.extend_from_slice(&fs::read("test_data/1004.rtcm")?);
    buffer.extend_from_slice(&fs::read("test_data/1005.rtcm")?);

    let frames: Vec<Frame> = iter_messages_rtcm(&buffer[..])
        .map(|r| r.expect("Couldn't deserialize"))
        .collect();
    let mut serialized_json: Vec<u8> = frames
        .iter()
        .map(|frame| {
            serde_json::to_string(frame)
                .expect("Couldn't serialize to JSON")
                .as_bytes()
                .to_vec()
        })
        .flatten()
        .collect();

    // cause json error in first message
    serialized_json[3] = 10;

    let decoded_frame = {
        let mut iter_messages = iter_messages_json(&serialized_json[..]);
        let mut frame = iter_messages.next();
        loop {
            if !matches!(frame, Some(Err(Error::JsonError(_)))) {
                break;
            }
            frame = iter_messages.next();
        }
        frame
    };
    assert!(matches!(decoded_frame, Some(Ok(_))));
    Ok(())
}

#[test]
fn reserializing_gives_same_frame() -> Result<(), io::Error> {
    let files_to_skip = vec!["check_start.rtcm", "deku_error.rtcm"];

    for test_file in fs::read_dir("test_data")? {
        let test_file = test_file?;
        let test_file_name = test_file.file_name();

        if files_to_skip.contains(&test_file_name.to_string_lossy().as_ref()) {
            continue;
        }

        let file = File::open(test_file.path()).unwrap();

        let frames: Vec<Frame> = iter_messages_rtcm(file)
            .map(|r| {
                r.expect(&format!(
                    "Couldn't deserialize {}",
                    test_file_name.to_string_lossy()
                ))
            })
            .collect();

        let serialized_json: Vec<u8> = frames
            .iter()
            .map(|frame| {
                serde_json::to_string(frame)
                    .expect("Couldn't serialize to JSON")
                    .as_bytes()
                    .to_vec()
            })
            .flatten()
            .collect();

        let decoded_json_frames: Vec<Frame> = iter_messages_json(&serialized_json[..])
            .map(|r| {
                r.expect(&format!(
                    "Couldn't deserialize {}",
                    test_file_name.to_string_lossy()
                ))
            })
            .collect();

        let serialized: Vec<u8> = decoded_json_frames
            .iter()
            .map(|frame| frame.to_bytes().expect("Couldn't serialize"))
            .flatten()
            .collect();

        let reserialize_frames: Vec<Frame> = iter_messages_rtcm(&serialized[..])
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
fn reserializing_gives_same_msg() -> Result<(), io::Error> {
    let files_to_skip = vec!["check_start.rtcm", "deku_error.rtcm"];

    for test_file in fs::read_dir("test_data")? {
        let test_file = test_file?;
        let test_file_name = test_file.file_name();

        if files_to_skip.contains(&test_file_name.to_string_lossy().as_ref()) {
            continue;
        }

        let file = File::open(test_file.path()).unwrap();

        let frames: Vec<Frame> = iter_messages_rtcm(file)
            .map(|r| {
                r.expect(&format!(
                    "Couldn't deserialize {}",
                    test_file_name.to_string_lossy()
                ))
            })
            .collect();

        for frame in frames {
            let mut bit_vec: BitVec<Msb0, u8> = BitVec::new();
            frame
                .message
                .write(&mut bit_vec, (deku::ctx::Endian::Big, frame.msg_length))
                .expect(&format!(
                    "Couldn't serialize {}",
                    test_file_name.to_string_lossy()
                ));

            let (_, reserialize_msg) = Message::read(
                bit_vec.as_bitslice(),
                (deku::ctx::Endian::Big, frame.msg_length),
            )
            .expect(&format!(
                "Couldn't deserialize {}",
                test_file_name.to_string_lossy()
            ));

            assert_eq!(
                reserialize_msg,
                frame.message,
                "{}",
                test_file_name.to_string_lossy()
            );
        }
    }

    Ok(())
}

#[test]
fn check_payload() -> Result<(), io::Error> {
    let raw_msg = fs::read("test_data/1005.rtcm")?;
    let mut decoder = RtcmDecoder;

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

/// Reads a single byte at a time
struct SlowReader {
    inner: Box<dyn Read>,
}

impl Read for SlowReader {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.inner.read_exact(buf[..1].as_mut()) {
            Ok(_) => Ok(1),
            Err(e) => {
                if e.kind() == io::ErrorKind::UnexpectedEof {
                    Ok(0)
                } else {
                    Err(e)
                }
            }
        }
    }
}

/// Test that the decoder can work when the underlying reader doesn't spit out
/// many bytes at a time
#[test]
fn slow_reader() -> Result<(), io::Error> {
    let msg_file = File::open("test_data/1005.rtcm")?;

    let reader = SlowReader {
        inner: Box::new(msg_file),
    };
    let fr = FramedRead::new(reader, RtcmDecoder);
    let frames: Vec<Result<Frame, Error>> = fr.collect();

    assert_eq!(frames.len(), 1);
    assert!(matches!(
        frames[0],
        Ok(Frame {
            message: Message::Msg1005(_),
            ..
        })
    ));

    Ok(())
}

#[test]
fn slow_json() -> Result<(), io::Error> {
    let json = r#"{
        "msg_length": 33,
        "payload": "QJWmFE5VTEwgICAgICAgICAgICAgICAgAAAEU1dGVAAA",
        "msg_type": 1033,
        "reference_station_id": 1446,
        "antenna_descriptor_counter": 20,
        "antenna_descriptor": "NULL                ",
        "antenna_setup_id": 0,
        "antenna_serial_number_counter": 0,
        "antenna_serial_number": "",
        "receiver_type_descriptor_counter": 4,
        "receiver_type_descriptor": "SWFT",
        "receiver_firmware_version_counter": 0,
        "receiver_firmware_version": "",
        "receiver_serial_number_counter": 0,
        "receiver_serial_number": "",
        "num_padding_bits": 0,
        "padding": 0,
        "crc": 5119484
    }"#;

    let reader = SlowReader {
        inner: Box::new(json.as_bytes()),
    };

    let _: Vec<_> = iter_messages_json(reader)
        .map(|r| r.expect("Couldn't deserialize"))
        .collect();

    Ok(())
}
