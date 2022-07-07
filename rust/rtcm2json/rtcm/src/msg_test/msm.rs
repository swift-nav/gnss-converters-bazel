use super::*;
use dencode::{BytesMut, Decoder as _};
use std::{fs, io};

macro_rules! tuple_struct_vec {
    ($typ:expr, $( $x:expr ),* ) => {
        {
            vec![
                $(
                    $typ($x),
                )*
            ]
        }
    };
}

fn replace_msg_type(id: u16, buf: &[u8]) -> Vec<u8> {
    let mut ret_buf = Vec::new();
    ret_buf.extend_from_slice(buf);
    ret_buf[3] = (id >> 4) as u8;
    ret_buf[4] &= 0b00001111;
    ret_buf[4] |= (id as u8) << 4;
    ret_buf
}

fn decode_msm(id: u16, buf: &[u8]) -> Message {
    let msm_buf = replace_msg_type(id, buf);
    let mut bit_vec = BitVec::new();
    bit_vec.extend_from_raw_slice(&msm_buf[FRAME_HEADER_LEN..]);
    match Message::read(&bit_vec.as_bitslice(), (deku::ctx::Endian::Big, 12)) {
        Ok((_, message)) => message,
        Err(err) => panic!("decode error {:?}", err),
    }
}

#[test]
fn msm1_full() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_header = MsmHeader {
        station_id: U12(471),
        tow: U30(419549832),
        multiple_messages: Bit1(false),
        iods1: U3(0),
        reserved: U7(58),
        clock_steering: U2(1),
        external_clock: U2(2),
        divergence_free: Bit1(false),
        smoothing: U3(5),
        sat_mask: U64(372983138235745477),
        sig_mask: U32(2710418044),
        cell_mask: 508,
    };
    let expected_satellite_data = Msm1_2_3SatelliteData {
        rough_ranges_modulo_ms: tuple_struct_vec![
            U10, 989, 1023, 1023, 1023, 992, 0, 0, 42, 683, 687, 114, 860, 938, 295, 603, 430, 939,
            600, 662, 235, 763, 979
        ],
    };
    let expected_signal_data = Msm1SignalData {
        fine_pseudoranges: tuple_struct_vec![I15, 3574, -3652, 904, 11225, -11520, 5410, -6144],
    };

    let raw_msm1 = fs::read("test_data/msm1.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msm1[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msm1: Msm1 = match frame.message {
        Message::GpsMsm1(msm1) => msm1,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msm1.header, expected_header);
    assert_eq!(msm1.sat_data, expected_satellite_data);
    assert_eq!(msm1.signal_data, expected_signal_data);
    Ok(())
}

#[test]
fn msm1() -> Result<(), io::Error> {
    let raw_msm1 = fs::read("test_data/msm1.rtcm")?;
    let msm1 = decode_msm(1071, &raw_msm1[..]);
    assert!(matches!(msm1, Message::GpsMsm1(_)));
    let msm1 = decode_msm(1081, &raw_msm1[..]);
    assert!(matches!(msm1, Message::GloMsm1(_)));
    let msm1 = decode_msm(1091, &raw_msm1[..]);
    assert!(matches!(msm1, Message::GalMsm1(_)));
    let msm1 = decode_msm(1101, &raw_msm1[..]);
    assert!(matches!(msm1, Message::SbasMsm1(_)));
    let msm1 = decode_msm(1111, &raw_msm1[..]);
    assert!(matches!(msm1, Message::QzssMsm1(_)));
    let msm1 = decode_msm(1121, &raw_msm1[..]);
    assert!(matches!(msm1, Message::BdsMsm1(_)));
    Ok(())
}

#[test]
fn msm2_full() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_header = MsmHeader {
        station_id: U12(471),
        tow: U30(419549832),
        multiple_messages: Bit1(false),
        iods1: U3(0),
        reserved: U7(58),
        clock_steering: U2(1),
        external_clock: U2(2),
        divergence_free: Bit1(false),
        smoothing: U3(5),
        sat_mask: U64(372983138235745477),
        sig_mask: U32(2710418044),
        cell_mask: 508,
    };
    let expected_satellite_data = Msm1_2_3SatelliteData {
        rough_ranges_modulo_ms: tuple_struct_vec![
            U10, 989, 1023, 1023, 1023, 992, 0, 0, 42, 683, 687, 114, 860, 938, 295, 603, 430, 939,
            600, 662, 235, 763, 979
        ],
    };
    let expected_signal_data = Msm2SignalData {
        fine_phaseranges: tuple_struct_vec![
            I22, 457585, -1113660, 718441, -2095800, -1245180, -2073614, 1114128
        ],
        phaserange_lock_times: tuple_struct_vec![U4, 2, 0, 0, 0, 0, 0, 0],
        half_cycle_ambiguity_indicators: tuple_struct_vec![
            Bit1, false, false, false, false, false, false, false
        ],
    };

    let raw_msm2 = fs::read("test_data/msm2.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msm2[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msm2: Msm2 = match frame.message {
        Message::GpsMsm2(msm2) => msm2,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msm2.header, expected_header);

    assert_eq!(msm2.sat_data, expected_satellite_data);
    assert_eq!(msm2.signal_data, expected_signal_data);
    Ok(())
}

#[test]
fn msm2() -> Result<(), io::Error> {
    let raw_msm2 = fs::read("test_data/msm2.rtcm")?;
    let msm2 = decode_msm(1072, &raw_msm2[..]);
    assert!(matches!(msm2, Message::GpsMsm2(_)));
    let msm2 = decode_msm(1082, &raw_msm2[..]);
    assert!(matches!(msm2, Message::GloMsm2(_)));
    let msm2 = decode_msm(1092, &raw_msm2[..]);
    assert!(matches!(msm2, Message::GalMsm2(_)));
    let msm2 = decode_msm(1102, &raw_msm2[..]);
    assert!(matches!(msm2, Message::SbasMsm2(_)));
    let msm2 = decode_msm(1112, &raw_msm2[..]);
    assert!(matches!(msm2, Message::QzssMsm2(_)));
    let msm2 = decode_msm(1122, &raw_msm2[..]);
    assert!(matches!(msm2, Message::BdsMsm2(_)));
    Ok(())
}

#[test]
fn msm3_full() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_header = MsmHeader {
        station_id: U12(471),
        tow: U30(419549832),
        multiple_messages: Bit1(false),
        iods1: U3(0),
        reserved: U7(58),
        clock_steering: U2(1),
        external_clock: U2(2),
        divergence_free: Bit1(false),
        smoothing: U3(5),
        sat_mask: U64(372983138235745477),
        sig_mask: U32(2710418044),
        cell_mask: 508,
    };
    let expected_satellite_data = Msm1_2_3SatelliteData {
        rough_ranges_modulo_ms: tuple_struct_vec![
            U10, 989, 1023, 1023, 1023, 992, 0, 0, 42, 683, 687, 114, 860, 938, 295, 603, 430, 939,
            600, 662, 235, 763, 979
        ],
    };
    let expected_signal_data = Msm3SignalData {
        fine_pseudoranges: tuple_struct_vec![I15, 3574, -3652, 904, 11225, -11520, 5410, -6144],
        fine_phaseranges: tuple_struct_vec![
            I22, 590559, -1800192, -2080768, 0, 979446, -1134930, -1332309
        ],
        phaserange_lock_times: tuple_struct_vec![U4, 10, 11, 10, 11, 10, 11, 10],
        half_cycle_ambiguity_indicators: tuple_struct_vec![
            Bit1, true, false, true, true, true, false, true
        ],
    };

    let raw_msm3 = fs::read("test_data/msm3.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msm3[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msm3: Msm3 = match frame.message {
        Message::GpsMsm3(msm3) => msm3,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msm3.header, expected_header);
    assert_eq!(msm3.sat_data, expected_satellite_data);
    assert_eq!(msm3.signal_data, expected_signal_data);
    Ok(())
}

#[test]
fn msm3() -> Result<(), io::Error> {
    let raw_msm3 = fs::read("test_data/msm3.rtcm")?;
    let msm3 = decode_msm(1073, &raw_msm3[..]);
    assert!(matches!(msm3, Message::GpsMsm3(_)));
    let msm3 = decode_msm(1083, &raw_msm3[..]);
    assert!(matches!(msm3, Message::GloMsm3(_)));
    let msm3 = decode_msm(1093, &raw_msm3[..]);
    assert!(matches!(msm3, Message::GalMsm3(_)));
    let msm3 = decode_msm(1103, &raw_msm3[..]);
    assert!(matches!(msm3, Message::SbasMsm3(_)));
    let msm3 = decode_msm(1113, &raw_msm3[..]);
    assert!(matches!(msm3, Message::QzssMsm3(_)));
    let msm3 = decode_msm(1123, &raw_msm3[..]);
    assert!(matches!(msm3, Message::BdsMsm3(_)));
    Ok(())
}

#[test]
fn msm4_full() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_header = MsmHeader {
        station_id: U12(349),
        tow: U30(504821000),
        multiple_messages: Bit1(true),
        iods1: U3(0),
        reserved: U7(0),
        clock_steering: U2(1),
        external_clock: U2(0),
        divergence_free: Bit1(false),
        smoothing: U3(0),
        sat_mask: U64(11552864179677298688),
        sig_mask: U32(1077969152),
        cell_mask: 806292863915,
    };
    let expected_satellite_data = Msm4_6SatelliteData {
        rough_ranges_ms: tuple_struct_vec![U8, 79, 81, 75, 79, 68, 76, 72, 79, 67, 69],
        rough_ranges_modulo_ms: tuple_struct_vec![
            U10, 770, 249, 163, 527, 989, 607, 508, 690, 708, 557
        ],
    };
    let expected_signal_data = Msm4SignalData {
        fine_pseudoranges: tuple_struct_vec![
            I15, 855, 1192, 1209, -137, 317, 420, 6138, 6460, 6453, 8047, 8250, 3649, 3798, 7840,
            7863, -417, -34, 31, 6185, 6594, 6601, -1173, -1006, 7915, 8207, 8190
        ],
        fine_phaseranges: tuple_struct_vec![
            I22, -43090, -43631, -43522, -19396, -19297, -19078, 54822, 55495, 55538, 9391, 12766,
            41086, 361382, 19801, 20879, -61674, -57498, -56540, -55107, -53044, -52591, -109629,
            -104551, 143977, 146931, 147500
        ],
        phaserange_lock_times: tuple_struct_vec![
            U4, 15, 15, 15, 13, 13, 13, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
            15, 15, 15, 15, 15
        ],
        half_cycle_ambiguity_indicators: tuple_struct_vec![
            Bit1, false, false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false, false, false, false, false, false, false,
            false, false, false
        ],
        signal_cnrs: tuple_struct_vec![
            U6, 41, 40, 45, 38, 38, 45, 47, 45, 50, 41, 38, 51, 42, 47, 33, 52, 46, 52, 38, 40, 44,
            56, 51, 55, 51, 56
        ],
    };

    let raw_msm4 = fs::read("test_data/msm4.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msm4[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msm4: Msm4 = match frame.message {
        Message::GpsMsm4(msm4) => msm4,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msm4.header, expected_header);
    assert_eq!(msm4.sat_data, expected_satellite_data);
    assert_eq!(msm4.signal_data, expected_signal_data);
    Ok(())
}

#[test]
fn msm4() -> Result<(), io::Error> {
    let raw_msm4 = fs::read("test_data/msm4.rtcm")?;
    let msm4 = decode_msm(1074, &raw_msm4[..]);
    assert!(matches!(msm4, Message::GpsMsm4(_)));
    let msm4 = decode_msm(1084, &raw_msm4[..]);
    assert!(matches!(msm4, Message::GloMsm4(_)));
    let msm4 = decode_msm(1094, &raw_msm4[..]);
    assert!(matches!(msm4, Message::GalMsm4(_)));
    let msm4 = decode_msm(1104, &raw_msm4[..]);
    assert!(matches!(msm4, Message::SbasMsm4(_)));
    let msm4 = decode_msm(1114, &raw_msm4[..]);
    assert!(matches!(msm4, Message::QzssMsm4(_)));
    let msm4 = decode_msm(1124, &raw_msm4[..]);
    assert!(matches!(msm4, Message::BdsMsm4(_)));
    Ok(())
}

#[test]
fn msm5_full() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_header = MsmHeader {
        station_id: U12(2709),
        tow: U30(333421000),
        multiple_messages: Bit1(true),
        iods1: U3(0),
        reserved: U7(0),
        clock_steering: U2(1),
        external_clock: U2(0),
        divergence_free: Bit1(false),
        smoothing: U3(0),
        sat_mask: U64(5480935206695206912),
        sig_mask: U32(1073742848),
        cell_mask: 178878,
    };
    let expected_satellite_data = Msm5_7SatelliteData {
        rough_ranges_ms: tuple_struct_vec![U8, 69, 82, 73, 67, 78, 76, 75, 74, 81],
        sat_infos: tuple_struct_vec![U4, 15, 15, 15, 15, 15, 15, 15, 15, 15],
        rough_ranges_modulo_ms: tuple_struct_vec![U10, 127, 426, 392, 547, 986, 6, 124, 138, 203],
        rough_phaseranges_rate: tuple_struct_vec![I14, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    let expected_signal_data = Msm5SignalData {
        fine_pseudoranges: tuple_struct_vec![
            I15, 6391, -7782, -7003, -6719, -6907, -6895, 5350, -2109, -1696, 6461, 6764, -5079
        ],
        fine_phaseranges: tuple_struct_vec![
            I22, 203404, -307049, -254628, -259862, -226057, -235307, 144103, -106643, -115816,
            172485, 166656, -202382
        ],
        phaserange_lock_times: tuple_struct_vec![
            U4, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15
        ],
        half_cycle_ambiguity_indicators: tuple_struct_vec![
            Bit1, false, false, false, false, false, false, false, false, false, false, false,
            false
        ],
        signal_cnrs: tuple_struct_vec![U6, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50],
        fine_phaserange_rates: tuple_struct_vec![
            I15, -16384, -16384, -16384, -16384, -16384, -16384, -16384, -16384, -16384, -16384,
            -16384, -16384
        ],
    };

    let raw_msm5 = fs::read("test_data/msm5.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msm5[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msm5: Msm5 = match frame.message {
        Message::GpsMsm5(msm5) => msm5,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msm5.header, expected_header);

    assert_eq!(msm5.sat_data, expected_satellite_data);
    assert_eq!(msm5.signal_data, expected_signal_data);
    Ok(())
}

#[test]
fn msm5() -> Result<(), io::Error> {
    let raw_msm5 = fs::read("test_data/msm5.rtcm")?;
    let msm5 = decode_msm(1075, &raw_msm5[..]);
    assert!(matches!(msm5, Message::GpsMsm5(_)));
    let msm5 = decode_msm(1085, &raw_msm5[..]);
    assert!(matches!(msm5, Message::GloMsm5(_)));
    let msm5 = decode_msm(1095, &raw_msm5[..]);
    assert!(matches!(msm5, Message::GalMsm5(_)));
    let msm5 = decode_msm(1105, &raw_msm5[..]);
    assert!(matches!(msm5, Message::SbasMsm5(_)));
    let msm5 = decode_msm(1115, &raw_msm5[..]);
    assert!(matches!(msm5, Message::QzssMsm5(_)));
    let msm5 = decode_msm(1125, &raw_msm5[..]);
    assert!(matches!(msm5, Message::BdsMsm5(_)));
    Ok(())
}

#[test]
fn msm6_full() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_header = MsmHeader {
        station_id: U12(0),
        tow: U30(467022184),
        multiple_messages: Bit1(true),
        iods1: U3(0),
        reserved: U7(0),
        clock_steering: U2(0),
        external_clock: U2(1),
        divergence_free: Bit1(false),
        smoothing: U3(0),
        sat_mask: U64(558940034514812928),
        sig_mask: U32(1090519040),
        cell_mask: 262142,
    };
    let expected_satellite_data = Msm4_6SatelliteData {
        rough_ranges_ms: tuple_struct_vec![U8, 71, 64, 71, 64, 74, 69, 77, 81, 79],
        rough_ranges_modulo_ms: tuple_struct_vec![U10, 109, 195, 655, 169, 532, 439, 863, 17, 964],
    };
    let expected_signal_data = Msm6SignalData {
        fine_pseudoranges: tuple_struct_vec![
            I20, 85757, 86857, 4845, 5434, -119202, -119695, 101027, 101476, 1771, 12326, 111005,
            111566, 111580, 111638, 31210, 38045, -163144
        ],
        fine_phaseranges: tuple_struct_vec![
            I24, 356711, 349244, 28777, 57464, -462007, -456414, 464089, 484965, 43535, 115092,
            507065, 565289, 458323, 436514, 128176, 210706, -669265
        ],
        phaserange_lock_times: tuple_struct_vec![
            U10, 640, 640, 616, 616, 572, 572, 609, 609, 555, 551, 629, 629, 555, 555, 416, 397,
            594
        ],
        half_cycle_ambiguity_indicators: tuple_struct_vec![
            Bit1, false, false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false
        ],
        signal_cnrs: tuple_struct_vec![
            U10, 682, 641, 827, 803, 724, 724, 820, 828, 705, 623, 711, 672, 690, 631, 574, 523,
            541
        ],
    };

    let raw_msm6 = fs::read("test_data/msm6.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msm6[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msm6: Msm6 = match frame.message {
        Message::GpsMsm6(msm6) => msm6,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msm6.header, expected_header);
    assert_eq!(msm6.sat_data, expected_satellite_data);
    assert_eq!(msm6.signal_data, expected_signal_data);
    Ok(())
}

#[test]
fn msm6() -> Result<(), io::Error> {
    let raw_msm6 = fs::read("test_data/msm6.rtcm")?;
    let msm6 = decode_msm(1076, &raw_msm6);
    assert!(matches!(msm6, Message::GpsMsm6(_)));
    let msm6 = decode_msm(1086, &raw_msm6);
    assert!(matches!(msm6, Message::GloMsm6(_)));
    let msm6 = decode_msm(1096, &raw_msm6);
    assert!(matches!(msm6, Message::GalMsm6(_)));
    let msm6 = decode_msm(1106, &raw_msm6);
    assert!(matches!(msm6, Message::SbasMsm6(_)));
    let msm6 = decode_msm(1116, &raw_msm6);
    assert!(matches!(msm6, Message::QzssMsm6(_)));
    let msm6 = decode_msm(1126, &raw_msm6);
    assert!(matches!(msm6, Message::BdsMsm6(_)));
    Ok(())
}

#[test]
fn msm7_full() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_header = MsmHeader {
        station_id: U12(0),
        tow: U30(467022184),
        multiple_messages: Bit1(true),
        iods1: U3(0),
        reserved: U7(0),
        clock_steering: U2(0),
        external_clock: U2(1),
        divergence_free: Bit1(false),
        smoothing: U3(0),
        sat_mask: U64(558940034514812928),
        sig_mask: U32(1090519040),
        cell_mask: 262142,
    };
    let expected_satellite_data = Msm5_7SatelliteData {
        rough_ranges_ms: tuple_struct_vec![U8, 71, 64, 71, 64, 74, 69, 77, 81, 79],
        sat_infos: tuple_struct_vec![U4, 3, 12, 13, 5, 0, 6, 11, 4, 9],
        rough_ranges_modulo_ms: tuple_struct_vec![U10, 109, 195, 655, 169, 532, 439, 863, 17, 964],
        rough_phaseranges_rate: tuple_struct_vec![
            I14, 710, 32, -647, -130, -705, 596, -181, -631, 394
        ],
    };
    let expected_signal_data = Msm7SignalData {
        fine_pseudoranges: tuple_struct_vec![
            I20, 85757, 86857, 4845, 5434, -119202, -119695, 101027, 101476, 1771, 12326, 111005,
            111566, 111580, 111638, 31210, 38045, -163144
        ],
        fine_phaseranges: tuple_struct_vec![
            I24, 356711, 349244, 28777, 57464, -462007, -456414, 464089, 484965, 43535, 115092,
            507065, 565289, 458323, 436514, 128176, 210706, -669265
        ],
        phaserange_lock_times: tuple_struct_vec![
            U10, 640, 640, 616, 616, 572, 572, 609, 609, 555, 551, 629, 629, 555, 555, 416, 397,
            594
        ],
        half_cycle_ambiguity_indicators: tuple_struct_vec![
            Bit1, false, false, false, false, false, false, false, false, false, false, false,
            false, false, false, false, false, false
        ],
        signal_cnrs: tuple_struct_vec![
            U10, 682, 641, 827, 803, 724, 724, 820, 828, 705, 623, 711, 672, 690, 631, 574, 523,
            541
        ],
        fine_phaserange_rates: tuple_struct_vec![
            I15, 415, 417, -2242, -2207, 1063, 1133, -3100, -3134, -2241, -2151, -2080, -2297,
            1212, 1208, 3353, 3215, -3163
        ],
    };

    let raw_msg = fs::read("test_data/msm7.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    assert_eq!(frame.msg_length, 0xea);
    let msm7: Msm7 = match frame.message {
        Message::GloMsm7(msm7) => msm7,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msm7.header, expected_header);
    assert_eq!(msm7.sat_data, expected_satellite_data);
    assert_eq!(msm7.signal_data, expected_signal_data);
    Ok(())
}

#[test]
fn msm7() -> Result<(), io::Error> {
    let raw_msm7 = fs::read("test_data/msm7.rtcm")?;
    let msm7 = decode_msm(1077, &raw_msm7[..]);
    assert!(matches!(msm7, Message::GpsMsm7(_)));
    let msm7 = decode_msm(1087, &raw_msm7[..]);
    assert!(matches!(msm7, Message::GloMsm7(_)));
    let msm7 = decode_msm(1097, &raw_msm7[..]);
    assert!(matches!(msm7, Message::GalMsm7(_)));
    let msm7 = decode_msm(1107, &raw_msm7[..]);
    assert!(matches!(msm7, Message::SbasMsm7(_)));
    let msm7 = decode_msm(1117, &raw_msm7[..]);
    assert!(matches!(msm7, Message::QzssMsm7(_)));
    let msm7 = decode_msm(1127, &raw_msm7[..]);
    assert!(matches!(msm7, Message::BdsMsm7(_)));
    Ok(())
}
