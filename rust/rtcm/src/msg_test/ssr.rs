use super::*;
use dencode::{BytesMut, Decoder as _};
use std::{fs, io};

#[test]
fn msg1057() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1057 {
        header: Header1057 {
            gps_epoch_time_1s: U20(513555),
            ssr_update_interval: U4(0),
            multiple_message_indicator: Bit1(false),
            satellite_reference_datum: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            no_of_satellites: U6(8),
        },
        satellites: vec![
            Satellite1057 {
                gps_satellite_id: U6(2),
                gps_iode: U8(46),
                delta_radial: I22(-189345),
                delta_along_track: I20(-24271),
                delta_cross_track: I20(-5435),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1057 {
                gps_satellite_id: U6(5),
                gps_iode: U8(67),
                delta_radial: I22(-849),
                delta_along_track: I20(-2),
                delta_cross_track: I20(558),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1057 {
                gps_satellite_id: U6(6),
                gps_iode: U8(24),
                delta_radial: I22(249912),
                delta_along_track: I20(15111),
                delta_cross_track: I20(724),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1057 {
                gps_satellite_id: U6(12),
                gps_iode: U8(66),
                delta_radial: I22(-140839),
                delta_along_track: I20(-3654),
                delta_cross_track: I20(-7554),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1057 {
                gps_satellite_id: U6(17),
                gps_iode: U8(42),
                delta_radial: I22(-81302),
                delta_along_track: I20(45218),
                delta_cross_track: I20(-32805),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1057 {
                gps_satellite_id: U6(19),
                gps_iode: U8(69),
                delta_radial: I22(82455),
                delta_along_track: I20(22996),
                delta_cross_track: I20(11798),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1057 {
                gps_satellite_id: U6(24),
                gps_iode: U8(51),
                delta_radial: I22(260719),
                delta_along_track: I20(25079),
                delta_cross_track: I20(14703),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1057 {
                gps_satellite_id: U6(25),
                gps_iode: U8(90),
                delta_radial: I22(10331),
                delta_along_track: I20(-35298),
                delta_cross_track: I20(-43518),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
        ],
    };

    let raw_msg = fs::read("test_data/1057.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1057 = match frame.message {
        Message::Msg1057(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1057, expected_msg);
    Ok(())
}

#[test]
fn msg1058() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1058 {
        header: Header1058 {
            gps_epoch_time_1s: U20(513554),
            ssr_update_interval: U4(0),
            multiple_message_indicator: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            no_of_satellites: U6(8),
        },
        satellites: vec![
            Satellite1058 {
                gps_satellite_id: U6(2),
                delta_clock_c0: I22(24250),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1058 {
                gps_satellite_id: U6(5),
                delta_clock_c0: I22(-9993),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1058 {
                gps_satellite_id: U6(6),
                delta_clock_c0: I22(-150012),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1058 {
                gps_satellite_id: U6(12),
                delta_clock_c0: I22(103364),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1058 {
                gps_satellite_id: U6(17),
                delta_clock_c0: I22(125950),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1058 {
                gps_satellite_id: U6(19),
                delta_clock_c0: I22(76763),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1058 {
                gps_satellite_id: U6(24),
                delta_clock_c0: I22(-115017),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1058 {
                gps_satellite_id: U6(25),
                delta_clock_c0: I22(-50023),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
        ],
    };

    let raw_msg = fs::read("test_data/1058.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1058 = match frame.message {
        Message::Msg1058(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1058, expected_msg);
    Ok(())
}

#[test]
fn msg1059() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1059 {
        header: Header1059 {
            gps_epoch_time_1s: U20(418560),
            ssr_update_interval: U4(6),
            multiple_message_indicator: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            no_of_satellites: U6(9),
        },
        sv_blocks: vec![
            SvBlock1059 {
                satellite: Satellite1059 {
                    gps_satellite_id: U6(2),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(320),
                    },
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        code_bias: I14(608),
                    },
                ],
            },
            SvBlock1059 {
                satellite: Satellite1059 {
                    gps_satellite_id: U6(6),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(-258),
                    },
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        code_bias: I14(-494),
                    },
                ],
            },
            SvBlock1059 {
                satellite: Satellite1059 {
                    gps_satellite_id: U6(12),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(205),
                    },
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        code_bias: I14(320),
                    },
                ],
            },
            SvBlock1059 {
                satellite: Satellite1059 {
                    gps_satellite_id: U6(17),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(175),
                    },
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        code_bias: I14(266),
                    },
                ],
            },
            SvBlock1059 {
                satellite: Satellite1059 {
                    gps_satellite_id: U6(19),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(234),
                    },
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        code_bias: I14(501),
                    },
                ],
            },
            SvBlock1059 {
                satellite: Satellite1059 {
                    gps_satellite_id: U6(24),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(-229),
                    },
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        code_bias: I14(-422),
                    },
                ],
            },
            SvBlock1059 {
                satellite: Satellite1059 {
                    gps_satellite_id: U6(25),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(-376),
                    },
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        code_bias: I14(-577),
                    },
                ],
            },
            SvBlock1059 {
                satellite: Satellite1059 {
                    gps_satellite_id: U6(28),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(126),
                    },
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        code_bias: I14(262),
                    },
                ],
            },
            SvBlock1059 {
                satellite: Satellite1059 {
                    gps_satellite_id: U6(29),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(156),
                    },
                    Code1059 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        code_bias: I14(235),
                    },
                ],
            },
        ],
    };

    let raw_msg = fs::read("test_data/1059.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1059 = match frame.message {
        Message::Msg1059(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1059, expected_msg);
    Ok(())
}

#[test]
fn msg1060() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1060 {
        header: Header1060 {
            gps_epoch_time_1s: U20(418590),
            ssr_update_interval: U4(5),
            multiple_message_indicator: Bit1(false),
            satellite_reference_datum: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            no_of_satellites: U6(9),
        },
        satellites: vec![
            Satellite1060 {
                gps_satellite_id: U6(2),
                gps_iode: U8(28),
                delta_radial: I22(-1197),
                delta_along_track: I20(1267),
                delta_cross_track: I20(-410),
                dot_delta_radial: I21(-239),
                dot_delta_along_track: I19(113),
                dot_delta_cross_track: I19(-122),
                delta_clock_c0: I22(-5484),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1060 {
                gps_satellite_id: U6(6),
                gps_iode: U8(29),
                delta_radial: I22(420),
                delta_along_track: I20(658),
                delta_cross_track: I20(853),
                dot_delta_radial: I21(105),
                dot_delta_along_track: I19(25),
                dot_delta_cross_track: I19(-83),
                delta_clock_c0: I22(-857),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1060 {
                gps_satellite_id: U6(12),
                gps_iode: U8(17),
                delta_radial: I22(-49),
                delta_along_track: I20(1858),
                delta_cross_track: I20(129),
                dot_delta_radial: I21(-59),
                dot_delta_along_track: I19(-2),
                dot_delta_cross_track: I19(48),
                delta_clock_c0: I22(3376),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1060 {
                gps_satellite_id: U6(17),
                gps_iode: U8(25),
                delta_radial: I22(-90),
                delta_along_track: I20(-1331),
                delta_cross_track: I20(-1646),
                dot_delta_radial: I21(149),
                dot_delta_along_track: I19(-138),
                dot_delta_cross_track: I19(-95),
                delta_clock_c0: I22(-3096),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1060 {
                gps_satellite_id: U6(19),
                gps_iode: U8(63),
                delta_radial: I22(-1517),
                delta_along_track: I20(3983),
                delta_cross_track: I20(1041),
                dot_delta_radial: I21(-221),
                dot_delta_along_track: I19(168),
                dot_delta_cross_track: I19(-7),
                delta_clock_c0: I22(5757),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1060 {
                gps_satellite_id: U6(24),
                gps_iode: U8(72),
                delta_radial: I22(-2744),
                delta_along_track: I20(1687),
                delta_cross_track: I20(1689),
                dot_delta_radial: I21(-156),
                dot_delta_along_track: I19(-37),
                dot_delta_cross_track: I19(95),
                delta_clock_c0: I22(-2008),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1060 {
                gps_satellite_id: U6(25),
                gps_iode: U8(34),
                delta_radial: I22(-1734),
                delta_along_track: I20(3002),
                delta_cross_track: I20(-1475),
                dot_delta_radial: I21(-48),
                dot_delta_along_track: I19(-94),
                dot_delta_cross_track: I19(43),
                delta_clock_c0: I22(-1294),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1060 {
                gps_satellite_id: U6(28),
                gps_iode: U8(97),
                delta_radial: I22(3043),
                delta_along_track: I20(358),
                delta_cross_track: I20(-1502),
                dot_delta_radial: I21(199),
                dot_delta_along_track: I19(-222),
                dot_delta_cross_track: I19(-120),
                delta_clock_c0: I22(1300),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1060 {
                gps_satellite_id: U6(29),
                gps_iode: U8(87),
                delta_radial: I22(553),
                delta_along_track: I20(-1282),
                delta_cross_track: I20(1286),
                dot_delta_radial: I21(90),
                dot_delta_along_track: I19(-76),
                dot_delta_cross_track: I19(54),
                delta_clock_c0: I22(-3753),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
        ],
    };
    let raw_msg = fs::read("test_data/1060.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1060 = match frame.message {
        Message::Msg1060(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1060, expected_msg);
    Ok(())
}

#[test]
fn msg1240() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1240 {
        header: Header1240 {
            gps_epoch_time_1s: U20(513553),
            ssr_update_interval: U4(0),
            multiple_message_indicator: Bit1(false),
            satellite_reference_datum: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            no_of_satellites: U6(7),
        },
        satellites: vec![
            Satellite1240 {
                gal_satellite_id: U6(1),
                gal_iodnav_i_nav: U10(86),
                delta_radial: I22(261433),
                delta_along_track: I20(-35452),
                delta_cross_track: I20(37187),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1240 {
                gal_satellite_id: U6(11),
                gal_iodnav_i_nav: U10(86),
                delta_radial: I22(-486),
                delta_along_track: I20(-38),
                delta_cross_track: I20(-11),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1240 {
                gal_satellite_id: U6(12),
                gal_iodnav_i_nav: U10(86),
                delta_radial: I22(846),
                delta_along_track: I20(2051),
                delta_cross_track: I20(-423),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1240 {
                gal_satellite_id: U6(24),
                gal_iodnav_i_nav: U10(85),
                delta_radial: I22(100693),
                delta_along_track: I20(-14471),
                delta_cross_track: I20(6024),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1240 {
                gal_satellite_id: U6(26),
                gal_iodnav_i_nav: U10(86),
                delta_radial: I22(104094),
                delta_along_track: I20(29219),
                delta_cross_track: I20(-71017),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1240 {
                gal_satellite_id: U6(31),
                gal_iodnav_i_nav: U10(86),
                delta_radial: I22(-252528),
                delta_along_track: I20(-12204),
                delta_cross_track: I20(-15854),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
            Satellite1240 {
                gal_satellite_id: U6(33),
                gal_iodnav_i_nav: U10(81),
                delta_radial: I22(-175758),
                delta_along_track: I20(19850),
                delta_cross_track: I20(12345),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
            },
        ],
    };
    let raw_msg = fs::read("test_data/1240.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1240 = match frame.message {
        Message::Msg1240(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1240, expected_msg);
    Ok(())
}

#[test]
fn msg1241() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1241 {
        header: Header1241 {
            gps_epoch_time_1s: U20(513554),
            ssr_update_interval: U4(0),
            multiple_message_indicator: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            no_of_satellites: U6(7),
        },
        satellites: vec![
            Satellite1241 {
                gal_satellite_id: U6(1),
                delta_clock_c0: I22(-255279),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1241 {
                gal_satellite_id: U6(11),
                delta_clock_c0: I22(-34917),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1241 {
                gal_satellite_id: U6(12),
                delta_clock_c0: I22(32843),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1241 {
                gal_satellite_id: U6(24),
                delta_clock_c0: I22(-198902),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1241 {
                gal_satellite_id: U6(26),
                delta_clock_c0: I22(-155971),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1241 {
                gal_satellite_id: U6(31),
                delta_clock_c0: I22(171390),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1241 {
                gal_satellite_id: U6(33),
                delta_clock_c0: I22(48069),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
        ],
    };
    let raw_msg = fs::read("test_data/1241.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1241 = match frame.message {
        Message::Msg1241(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1241, expected_msg);
    Ok(())
}

#[test]
fn msg1242() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1242 {
        header: Header1242 {
            gal_epoch_time_1s: U20(418560),
            ssr_update_interval: U4(6),
            multiple_message_indicator: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            no_of_satellites: U6(7),
        },
        sv_blocks: vec![
            SvBlock1242 {
                satellite: Satellite1242 {
                    gal_satellite_id: U6(2),
                    no_of_code_biases_processed: U5(4),
                },
                code: vec![
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(2),
                        code_bias: I14(59),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(3),
                        code_bias: I14(56),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(9),
                        code_bias: I14(101),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(10),
                        code_bias: I14(95),
                    },
                ],
            },
            SvBlock1242 {
                satellite: Satellite1242 {
                    gal_satellite_id: U6(11),
                    no_of_code_biases_processed: U5(4),
                },
                code: vec![
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(2),
                        code_bias: I14(433),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(3),
                        code_bias: I14(436),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(9),
                        code_bias: I14(738),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(10),
                        code_bias: I14(743),
                    },
                ],
            },
            SvBlock1242 {
                satellite: Satellite1242 {
                    gal_satellite_id: U6(12),
                    no_of_code_biases_processed: U5(4),
                },
                code: vec![
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(2),
                        code_bias: I14(306),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(3),
                        code_bias: I14(317),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(9),
                        code_bias: I14(521),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(10),
                        code_bias: I14(540),
                    },
                ],
            },
            SvBlock1242 {
                satellite: Satellite1242 {
                    gal_satellite_id: U6(24),
                    no_of_code_biases_processed: U5(4),
                },
                code: vec![
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(2),
                        code_bias: I14(127),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(3),
                        code_bias: I14(131),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(9),
                        code_bias: I14(217),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(10),
                        code_bias: I14(223),
                    },
                ],
            },
            SvBlock1242 {
                satellite: Satellite1242 {
                    gal_satellite_id: U6(25),
                    no_of_code_biases_processed: U5(4),
                },
                code: vec![
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(2),
                        code_bias: I14(-122),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(3),
                        code_bias: I14(-116),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(9),
                        code_bias: I14(-208),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(10),
                        code_bias: I14(-197),
                    },
                ],
            },
            SvBlock1242 {
                satellite: Satellite1242 {
                    gal_satellite_id: U6(31),
                    no_of_code_biases_processed: U5(4),
                },
                code: vec![
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(2),
                        code_bias: I14(-157),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(3),
                        code_bias: I14(-165),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(9),
                        code_bias: I14(-267),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(10),
                        code_bias: I14(-282),
                    },
                ],
            },
            SvBlock1242 {
                satellite: Satellite1242 {
                    gal_satellite_id: U6(36),
                    no_of_code_biases_processed: U5(4),
                },
                code: vec![
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(2),
                        code_bias: I14(-196),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(3),
                        code_bias: I14(-197),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(9),
                        code_bias: I14(-333),
                    },
                    Code1242 {
                        gal_signal_and_tracking_mode_indicator: U5(10),
                        code_bias: I14(-335),
                    },
                ],
            },
        ],
    };
    let raw_msg = fs::read("test_data/1242.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1242 = match frame.message {
        Message::Msg1242(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1242, expected_msg);
    Ok(())
}

#[test]
fn msg1243() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1243 {
        header: Header1243 {
            gal_epoch_time_1s: U20(418590),
            ssr_update_interval: U4(5),
            multiple_message_indicator: Bit1(false),
            satellite_reference_datum: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            no_of_satellites: U6(6),
        },
        satellites: vec![
            Satellite1243 {
                gal_satellite_id: U6(2),
                gal_iodnav_i_nav: U10(55),
                delta_radial: I22(2051),
                delta_along_track: I20(-104),
                delta_cross_track: I20(681),
                dot_delta_radial: I21(31),
                dot_delta_along_track: I19(42),
                dot_delta_cross_track: I19(31),
                delta_clock_c0: I22(-5313),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1243 {
                gal_satellite_id: U6(12),
                gal_iodnav_i_nav: U10(56),
                delta_radial: I22(176),
                delta_along_track: I20(-40),
                delta_cross_track: I20(156),
                dot_delta_radial: I21(-6),
                dot_delta_along_track: I19(8),
                dot_delta_cross_track: I19(-23),
                delta_clock_c0: I22(-3012),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1243 {
                gal_satellite_id: U6(24),
                gal_iodnav_i_nav: U10(56),
                delta_radial: I22(-1177),
                delta_along_track: I20(-642),
                delta_cross_track: I20(47),
                dot_delta_radial: I21(76),
                dot_delta_along_track: I19(-12),
                dot_delta_cross_track: I19(67),
                delta_clock_c0: I22(-1018),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1243 {
                gal_satellite_id: U6(25),
                gal_iodnav_i_nav: U10(50),
                delta_radial: I22(-244),
                delta_along_track: I20(-63),
                delta_cross_track: I20(653),
                dot_delta_radial: I21(-2),
                dot_delta_along_track: I19(20),
                dot_delta_cross_track: I19(70),
                delta_clock_c0: I22(-3396),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1243 {
                gal_satellite_id: U6(31),
                gal_iodnav_i_nav: U10(54),
                delta_radial: I22(-49),
                delta_along_track: I20(-657),
                delta_cross_track: I20(-350),
                dot_delta_radial: I21(39),
                dot_delta_along_track: I19(30),
                dot_delta_cross_track: I19(36),
                delta_clock_c0: I22(-963),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
            Satellite1243 {
                gal_satellite_id: U6(36),
                gal_iodnav_i_nav: U10(56),
                delta_radial: I22(959),
                delta_along_track: I20(113),
                delta_cross_track: I20(397),
                dot_delta_radial: I21(-40),
                dot_delta_along_track: I19(15),
                dot_delta_cross_track: I19(-44),
                delta_clock_c0: I22(-4202),
                delta_clock_c1: I21(0),
                delta_clock_c2: I27(0),
            },
        ],
    };
    let raw_msg = fs::read("test_data/1243.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1243 = match frame.message {
        Message::Msg1243(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1243, expected_msg);
    Ok(())
}

#[test]
fn msg1260() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1260 {
        header: Header1260 {
            bds_epoch_time_1s: U20(418553),
            ssr_update_interval: U4(6),
            multiple_message_indicator: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            no_of_satellites: U6(10),
        },
        sv_blocks: vec![
            SvBlock1260 {
                satellite: Satellite1260 {
                    bds_satellite_id: U6(11),
                    no_of_code_biases_processed: U5(1),
                },
                code: vec![Code1260 {
                    bds_signal_and_tracking_mode_indicator: U5(0),
                    code_bias: I14(0),
                }],
            },
            SvBlock1260 {
                satellite: Satellite1260 {
                    bds_satellite_id: U6(14),
                    no_of_code_biases_processed: U5(1),
                },
                code: vec![Code1260 {
                    bds_signal_and_tracking_mode_indicator: U5(0),
                    code_bias: I14(0),
                }],
            },
            SvBlock1260 {
                satellite: Satellite1260 {
                    bds_satellite_id: U6(23),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(0),
                    },
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(12),
                        code_bias: I14(673),
                    },
                ],
            },
            SvBlock1260 {
                satellite: Satellite1260 {
                    bds_satellite_id: U6(27),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(0),
                    },
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(12),
                        code_bias: I14(112),
                    },
                ],
            },
            SvBlock1260 {
                satellite: Satellite1260 {
                    bds_satellite_id: U6(28),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(0),
                    },
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(12),
                        code_bias: I14(103),
                    },
                ],
            },
            SvBlock1260 {
                satellite: Satellite1260 {
                    bds_satellite_id: U6(33),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(0),
                    },
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(12),
                        code_bias: I14(-2449),
                    },
                ],
            },
            SvBlock1260 {
                satellite: Satellite1260 {
                    bds_satellite_id: U6(37),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(0),
                    },
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(12),
                        code_bias: I14(-267),
                    },
                ],
            },
            SvBlock1260 {
                satellite: Satellite1260 {
                    bds_satellite_id: U6(42),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(0),
                    },
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(12),
                        code_bias: I14(-914),
                    },
                ],
            },
            SvBlock1260 {
                satellite: Satellite1260 {
                    bds_satellite_id: U6(43),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(0),
                    },
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(12),
                        code_bias: I14(34),
                    },
                ],
            },
            SvBlock1260 {
                satellite: Satellite1260 {
                    bds_satellite_id: U6(46),
                    no_of_code_biases_processed: U5(2),
                },
                code: vec![
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        code_bias: I14(0),
                    },
                    Code1260 {
                        bds_signal_and_tracking_mode_indicator: U5(12),
                        code_bias: I14(-285),
                    },
                ],
            },
        ],
    };
    let raw_msg = fs::read("test_data/1260.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1260 = match frame.message {
        Message::Msg1260(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1260, expected_msg);
    Ok(())
}

#[test]
fn msg1261() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1261 {
        header: Header1261 {
            bds_epoch_time_1s: U20(418583),
            ssr_update_interval: U4(5),
            multiple_message_indicator: Bit1(false),
            satellite_reference_datum: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            no_of_satellites: U6(10),
        },
        satellites: vec![
            Satellite1261 {
                bds_satellite_id: U6(11),
                bds_t_oe_modulo: U10(0),
                bds_iod: U8(100),
                delta_radial: I22(0),
                delta_along_track: I20(0),
                delta_cross_track: I20(0),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
                delta_clock_c0: I22(7570),
                delta_clock_c1: I21(1),
                delta_clock_c2: I27(0),
            },
            Satellite1261 {
                bds_satellite_id: U6(14),
                bds_t_oe_modulo: U10(0),
                bds_iod: U8(100),
                delta_radial: I22(0),
                delta_along_track: I20(0),
                delta_cross_track: I20(0),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
                delta_clock_c0: I22(-16390),
                delta_clock_c1: I21(-1),
                delta_clock_c2: I27(0),
            },
            Satellite1261 {
                bds_satellite_id: U6(23),
                bds_t_oe_modulo: U10(0),
                bds_iod: U8(100),
                delta_radial: I22(0),
                delta_along_track: I20(0),
                delta_cross_track: I20(0),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
                delta_clock_c0: I22(-68988),
                delta_clock_c1: I21(-1),
                delta_clock_c2: I27(0),
            },
            Satellite1261 {
                bds_satellite_id: U6(27),
                bds_t_oe_modulo: U10(0),
                bds_iod: U8(100),
                delta_radial: I22(0),
                delta_along_track: I20(0),
                delta_cross_track: I20(0),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
                delta_clock_c0: I22(9264),
                delta_clock_c1: I21(1),
                delta_clock_c2: I27(0),
            },
            Satellite1261 {
                bds_satellite_id: U6(28),
                bds_t_oe_modulo: U10(0),
                bds_iod: U8(100),
                delta_radial: I22(0),
                delta_along_track: I20(0),
                delta_cross_track: I20(0),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
                delta_clock_c0: I22(31620),
                delta_clock_c1: I21(1),
                delta_clock_c2: I27(0),
            },
            Satellite1261 {
                bds_satellite_id: U6(33),
                bds_t_oe_modulo: U10(0),
                bds_iod: U8(100),
                delta_radial: I22(0),
                delta_along_track: I20(0),
                delta_cross_track: I20(0),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
                delta_clock_c0: I22(119193),
                delta_clock_c1: I21(1),
                delta_clock_c2: I27(0),
            },
            Satellite1261 {
                bds_satellite_id: U6(37),
                bds_t_oe_modulo: U10(0),
                bds_iod: U8(100),
                delta_radial: I22(0),
                delta_along_track: I20(0),
                delta_cross_track: I20(0),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
                delta_clock_c0: I22(52433),
                delta_clock_c1: I21(1),
                delta_clock_c2: I27(0),
            },
            Satellite1261 {
                bds_satellite_id: U6(42),
                bds_t_oe_modulo: U10(0),
                bds_iod: U8(100),
                delta_radial: I22(0),
                delta_along_track: I20(0),
                delta_cross_track: I20(0),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
                delta_clock_c0: I22(109060),
                delta_clock_c1: I21(1),
                delta_clock_c2: I27(0),
            },
            Satellite1261 {
                bds_satellite_id: U6(43),
                bds_t_oe_modulo: U10(0),
                bds_iod: U8(100),
                delta_radial: I22(0),
                delta_along_track: I20(0),
                delta_cross_track: I20(0),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
                delta_clock_c0: I22(18893),
                delta_clock_c1: I21(1),
                delta_clock_c2: I27(0),
            },
            Satellite1261 {
                bds_satellite_id: U6(46),
                bds_t_oe_modulo: U10(0),
                bds_iod: U8(100),
                delta_radial: I22(0),
                delta_along_track: I20(0),
                delta_cross_track: I20(0),
                dot_delta_radial: I21(0),
                dot_delta_along_track: I19(0),
                dot_delta_cross_track: I19(0),
                delta_clock_c0: I22(79445),
                delta_clock_c1: I21(1),
                delta_clock_c2: I27(0),
            },
        ],
    };
    let raw_msg = fs::read("test_data/1261.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1261 = match frame.message {
        Message::Msg1261(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1261, expected_msg);
    Ok(())
}

#[test]
fn msg1265() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1265 {
        header: Header1265 {
            gps_epoch_time_1s: U20(418590),
            ssr_update_interval: U4(5),
            multiple_message_indicator: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            dispersive_bias_consistency_indicator: Bit1(false),
            mw_consistency_indicator: Bit1(false),
            no_of_satellites: U6(9),
        },
        satellites: vec![
            Satellite1265 {
                gps_satellite_id: U6(2),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(14),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-31493),
                    },
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-52141),
                    },
                ],
            },
            Satellite1265 {
                gps_satellite_id: U6(6),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(265),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(30868),
                    },
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(50908),
                    },
                ],
            },
            Satellite1265 {
                gps_satellite_id: U6(12),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(319),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-20885),
                    },
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-34228),
                    },
                ],
            },
            Satellite1265 {
                gps_satellite_id: U6(17),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(465),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-11401),
                    },
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-19017),
                    },
                ],
            },
            Satellite1265 {
                gps_satellite_id: U6(19),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(460),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-26182),
                    },
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-43147),
                    },
                ],
            },
            Satellite1265 {
                gps_satellite_id: U6(24),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(408),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(21845),
                    },
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(36928),
                    },
                ],
            },
            Satellite1265 {
                gps_satellite_id: U6(25),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(38),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(35543),
                    },
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(59299),
                    },
                ],
            },
            Satellite1265 {
                gps_satellite_id: U6(28),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(463),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-18526),
                    },
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-29814),
                    },
                ],
            },
            Satellite1265 {
                gps_satellite_id: U6(29),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(305),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-11356),
                    },
                    Phase1265 {
                        gps_signal_and_tracking_mode_indicator: U5(11),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-18560),
                    },
                ],
            },
        ],
    };
    let raw_msg = fs::read("test_data/1265.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1265 = match frame.message {
        Message::Msg1265(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1265, expected_msg);
    Ok(())
}

#[test]
fn msg1267() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1267 {
        header: Header1267 {
            gal_epoch_time_1s: U20(418590),
            ssr_update_interval: U4(5),
            multiple_message_indicator: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            dispersive_bias_consistency_indicator: Bit1(false),
            mw_consistency_indicator: Bit1(false),
            no_of_satellites: U6(7),
        },
        satellites: vec![
            Satellite1267 {
                gal_satellite_id: U6(2),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(30),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(1),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(3882),
                    },
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(8),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(3195),
                    },
                ],
            },
            Satellite1267 {
                gal_satellite_id: U6(11),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(9),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(1),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-47090),
                    },
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(8),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-79911),
                    },
                ],
            },
            Satellite1267 {
                gal_satellite_id: U6(12),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(248),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(1),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-32048),
                    },
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(8),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-54558),
                    },
                ],
            },
            Satellite1267 {
                gal_satellite_id: U6(24),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(184),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(1),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-4551),
                    },
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(8),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-10591),
                    },
                ],
            },
            Satellite1267 {
                gal_satellite_id: U6(25),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(53),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(1),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(31988),
                    },
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(8),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(46568),
                    },
                ],
            },
            Satellite1267 {
                gal_satellite_id: U6(31),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(223),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(1),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(24704),
                    },
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(8),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(38169),
                    },
                ],
            },
            Satellite1267 {
                gal_satellite_id: U6(36),
                no_of_phase_biases_processed: U5(2),
                yaw_angle: U9(12),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(1),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(14076),
                    },
                    Phase1267 {
                        gal_signal_and_tracking_mode_indicator: U5(8),
                        signal_integer_indicator: Bit1(true),
                        signals_wide_lane_integer_indicator: U2(0),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(25527),
                    },
                ],
            },
        ],
    };
    let raw_msg = fs::read("test_data/1267.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1267 = match frame.message {
        Message::Msg1267(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1267, expected_msg);
    Ok(())
}

#[test]
fn msg1270() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = Msg1270 {
        header: Header1270 {
            bds_epoch_time_1s: U20(171921),
            ssr_update_interval: U4(2),
            multiple_message_indicator: Bit1(false),
            iod_ssr: U4(0),
            ssr_provider_id: U16(0),
            ssr_solution_id: U4(0),
            dispersive_bias_consistency_indicator: Bit1(false),
            mw_consistency_indicator: Bit1(true),
            no_of_satellites: U6(10),
        },
        satellites: vec![
            Satellite1270 {
                bds_satellite_id: U6(5),
                no_of_phase_biases_processed: U5(3),
                yaw_angle: U9(0),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(1384),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(6),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(2314),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(3),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(1590),
                    },
                ],
            },
            Satellite1270 {
                bds_satellite_id: U6(6),
                no_of_phase_biases_processed: U5(3),
                yaw_angle: U9(438),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-71),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(6),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-119),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(3),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(975),
                    },
                ],
            },
            Satellite1270 {
                bds_satellite_id: U6(7),
                no_of_phase_biases_processed: U5(3),
                yaw_angle: U9(479),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(2325),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(6),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(3888),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(3),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(3934),
                    },
                ],
            },
            Satellite1270 {
                bds_satellite_id: U6(8),
                no_of_phase_biases_processed: U5(3),
                yaw_angle: U9(59),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(1793),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(6),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(2999),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(3),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(1702),
                    },
                ],
            },
            Satellite1270 {
                bds_satellite_id: U6(9),
                no_of_phase_biases_processed: U5(3),
                yaw_angle: U9(430),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(1072),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(6),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(1793),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(3),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(673),
                    },
                ],
            },
            Satellite1270 {
                bds_satellite_id: U6(10),
                no_of_phase_biases_processed: U5(3),
                yaw_angle: U9(485),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-1639),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(6),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-2740),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(3),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-2109),
                    },
                ],
            },
            Satellite1270 {
                bds_satellite_id: U6(11),
                no_of_phase_biases_processed: U5(3),
                yaw_angle: U9(95),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-1121),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(6),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-1875),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(3),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-2334),
                    },
                ],
            },
            Satellite1270 {
                bds_satellite_id: U6(12),
                no_of_phase_biases_processed: U5(3),
                yaw_angle: U9(59),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(527),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(6),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(882),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(3),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(1090),
                    },
                ],
            },
            Satellite1270 {
                bds_satellite_id: U6(13),
                no_of_phase_biases_processed: U5(3),
                yaw_angle: U9(51),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-163),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(6),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-273),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(3),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(-630),
                    },
                ],
            },
            Satellite1270 {
                bds_satellite_id: U6(14),
                no_of_phase_biases_processed: U5(3),
                yaw_angle: U9(389),
                yaw_rate: I8(0),
                phases: vec![
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(0),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(1554),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(6),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(2598),
                    },
                    Phase1270 {
                        bds_signal_and_tracking_mode_indicator: U5(3),
                        signal_integer_indicator: Bit1(false),
                        signals_wide_lane_integer_indicator: U2(2),
                        signal_discontinuity_counter: U4(0),
                        phase_bias: I20(1259),
                    },
                ],
            },
        ],
    };
    let raw_msg = fs::read("test_data/1270.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1270 = match frame.message {
        Message::Msg1270(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1270, expected_msg);
    Ok(())
}
