use super::*;
use dencode::{BytesMut, Decoder as _};
use std::{fs, io};

#[test]
fn msg1019() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_msg = Msg1019 {
        satellite_id: U6(30),
        week_number: U10(109),
        sv_accuracy: U4(0),
        code_on_l2: U2(1),
        idot: I14(-1204),
        iode: U8(6),
        t_oc: U16(26099),
        a_f2: I8(0),
        a_f1: I16(-44),
        a_f0: I22(-914028),
        iodc: U10(6),
        c_rs: I16(1079),
        delta_n: I16(13228),
        m_0: I32(590289140),
        c_uc: I16(984),
        e: U32(44660822),
        c_us: I16(5421),
        a_1_2: U32(2702028478),
        t_oe: U16(26099),
        c_ic: I16(-19),
        omega_0: I32(-20899457),
        c_is: I16(65),
        i_0: I32(640744167),
        c_rc: I16(5525),
        omega: I32(-1906571568),
        omegadot: I24(-22352),
        t_gd: I8(8),
        sv_health: U6(0),
        l2_p_data_flag: Bit1(true),
        fit_interval: Bit1(false),
    };

    let raw_msg = fs::read("test_data/1019.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1019 = match frame.message {
        Message::Msg1019(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1019, expected_msg);
    Ok(())
}

#[test]
fn msg1020() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_msg = Msg1020 {
        satellite_id: U6(15),
        satellite_freq_channel_nr: U5(7),
        almanac_health: Bit1(false),
        almanac_health_avail_indi: Bit1(false),
        p1: U2(1),
        tk: U12(2944),
        msb_of_bn_word: Bit1(false),
        p2: Bit1(true),
        tb: U7(93),
        xn_tb_first: IS24(144664),
        xn_tb: IS27(3638463),
        xn_tb_second: IS5(-1),
        yn_tb_first: IS24(1678596),
        yn_tb: IS27(-46289486),
        yn_tb_second: IS5(-1),
        zn_tb_first: IS24(-3223854),
        zn_tb: IS27(-23901941),
        zn_tb_second: IS5(3),
        p3: Bit1(true),
        gamma_n_tb: IS11(0),
        m_p: U2(0),
        m_ln_third: Bit1(false),
        tau_n_tb: IS22(-111518),
        m_delta_tau_n: IS5(3),
        en: U5(0),
        m_p4: Bit1(true),
        m_ft: U4(1),
        m_nt: U11(499),
        m_m: U2(1),
        additional_data_avail: Bit1(true),
        na: U11(499),
        tau_c: IS32(-34),
        m_n4: U5(7),
        tau_gps: IS22(11),
        m_ln_fifth: Bit1(false),
        reserved: U7(0),
    };

    let raw_msg = fs::read("test_data/1020.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1020 = match frame.message {
        Message::Msg1020(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1020, expected_msg);
    Ok(())
}

#[test]
fn msg1042() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_msg = Msg1042 {
        satellite_id: U6(37),
        week_number: U13(801),
        sv_urai: U4(0),
        idot: I14(-562),
        aode: U5(1),
        t_oc: U17(52200),
        a_2: I11(0),
        a_1: I22(9061),
        a_0: I24(-7678449),
        aodc: U5(1),
        c_rs: I18(6166),
        delta_n: I16(10188),
        m_0: I32(-2014479910),
        c_uc: I18(10159),
        e: U32(4856889),
        c_us: I18(23802),
        a_1_2: U32(2769616765),
        t_oe: U17(52200),
        c_ic: I18(41),
        omega_0: I32(221934309),
        c_is: I18(79),
        i_0: I32(650510810),
        c_rc: I18(8160),
        omega: I32(-702961900),
        omegadot: I24(-18551),
        t_gd1: I10(-133),
        t_gd2: I10(-133),
        sv_health: Bit1(false),
    };

    let raw_msg = fs::read("test_data/1042.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1042 = match frame.message {
        Message::Msg1042(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1042, expected_msg);
    Ok(())
}

#[test]
fn msg1045() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_msg = Msg1045 {
        satellite_id: U6(4),
        week_number: U12(1184),
        iodnav: U10(23),
        sisa_index: U8(107),
        idot: I14(942),
        t_oc: U14(5350),
        a_f2: I6(0),
        a_f1: I21(-578),
        a_f0: I31(-17535921),
        c_rs: I16(-191),
        delta_n: I16(9340),
        m_0: I32(-848252758),
        c_uc: I16(-92),
        e: U32(3001211),
        c_us: I16(3108),
        a_1_2: U32(2852445960),
        t_oe: U14(5350),
        c_ic: I16(-1),
        omega_0: I32(-907182966),
        c_is: I16(52),
        i_0: I32(653424387),
        c_rc: I16(6880),
        omega: I32(-984667684),
        omegadot: I24(-15409),
        bgd_e5a_e1: I10(-13),
        nav_signal_health_status: U2(0),
        nav_data_validity_status: Bit1(false),
        reserved: U7(0),
    };

    let raw_msg = fs::read("test_data/1045.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1045 = match frame.message {
        Message::Msg1045(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1045, expected_msg);
    Ok(())
}

#[test]
fn msg1046() -> Result<(), io::Error> {
    let mut decoder = Decoder;
    let expected_msg = Msg1046 {
        satellite_id: U6(36),
        week_number: U12(1133),
        iodnav: U10(55),
        sisa_index: U8(107),
        idot: I14(-688),
        t_oc: U14(6950),
        a_f2: I6(0),
        a_f1: I21(-365),
        a_f0: I31(-3549907),
        c_rs: I16(-6161),
        delta_n: I16(7272),
        m_0: I32(-157793765),
        c_uc: I16(-4786),
        e: U32(2136577),
        c_us: I16(3373),
        a_1_2: U32(2852448543),
        t_oe: U14(6950),
        c_ic: I16(-27),
        omega_0: I32(1969600663),
        c_is: I16(-15),
        i_0: I32(680278487),
        c_rc: I16(7156),
        omega: I32(-75400426),
        omegadot: I24(-15095),
        bgd_e5a_e1: I10(24),
        bgd_e5b_e1: I10(28),
        e5b_signal_health_status: U2(0),
        e5b_data_validity_status: Bit1(false),
        e1_b_signal_health_status: U2(0),
        e1_b_data_validity_status: Bit1(false),
        reserved: U2(0),
    };

    let raw_msg = fs::read("test_data/1046.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg1046 = match frame.message {
        Message::Msg1046(msg) => msg,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg1046, expected_msg);
    Ok(())
}
