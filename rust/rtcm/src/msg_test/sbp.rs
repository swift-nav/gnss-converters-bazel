use super::*;
use dencode::{BytesMut, Decoder as _};
use sbp::messages::gnss::{GpsTimeSec, SvId};
use sbp::messages::ssr::{
    GriddedCorrectionHeader, MsgSsrGriddedCorrection, STECResidual, TroposphericDelayCorrection,
};
use std::{fs, io};

#[test]
fn sbp() -> Result<(), io::Error> {
    let mut decoder = RtcmDecoder;
    let expected_msg = SbpFrame {
        msg_type: 1532,
        sender_id: 0,
        length: 128,
        msg: Sbp::MsgSsrGriddedCorrection(MsgSsrGriddedCorrection {
            sender_id: Some(0),
            header: GriddedCorrectionHeader {
                tile_set_id: 0,
                tile_id: 2,
                time: GpsTimeSec {
                    tow: 326910,
                    wn: 2208,
                },
                num_msgs: 36,
                seq_num: 35,
                update_interval: 5,
                iod_atmo: 81,
                tropo_quality_indicator: 0,
            },
            index: 35,
            tropo_delay_correction: TroposphericDelayCorrection {
                hydro: 1,
                wet: -46,
                stddev: 67,
            },
            stec_residuals: vec![
                STECResidual {
                    sv_id: SvId {
                        sat_id: 2,
                        constellation: 0,
                    },
                    residual: -5,
                    stddev: 35,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 3,
                        constellation: 0,
                    },
                    residual: -1,
                    stddev: 37,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 6,
                        constellation: 0,
                    },
                    residual: 1,
                    stddev: 32,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 12,
                        constellation: 0,
                    },
                    residual: 3,
                    stddev: 35,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 14,
                        constellation: 0,
                    },
                    residual: 6,
                    stddev: 33,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 17,
                        constellation: 0,
                    },
                    residual: 4,
                    stddev: 33,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 19,
                        constellation: 0,
                    },
                    residual: 3,
                    stddev: 31,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 24,
                        constellation: 0,
                    },
                    residual: 1,
                    stddev: 33,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 23,
                        constellation: 3,
                    },
                    residual: 4,
                    stddev: 32,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 25,
                        constellation: 3,
                    },
                    residual: 0,
                    stddev: 37,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 28,
                        constellation: 3,
                    },
                    residual: 3,
                    stddev: 34,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 34,
                        constellation: 3,
                    },
                    residual: 0,
                    stddev: 35,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 37,
                        constellation: 3,
                    },
                    residual: 1,
                    stddev: 33,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 39,
                        constellation: 3,
                    },
                    residual: -7,
                    stddev: 82,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 43,
                        constellation: 3,
                    },
                    residual: 3,
                    stddev: 31,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 5,
                        constellation: 5,
                    },
                    residual: -12,
                    stddev: 54,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 12,
                        constellation: 5,
                    },
                    residual: 4,
                    stddev: 33,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 24,
                        constellation: 5,
                    },
                    residual: 1,
                    stddev: 33,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 26,
                        constellation: 5,
                    },
                    residual: 0,
                    stddev: 35,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 31,
                        constellation: 5,
                    },
                    residual: 3,
                    stddev: 31,
                },
                STECResidual {
                    sv_id: SvId {
                        sat_id: 33,
                        constellation: 5,
                    },
                    residual: 4,
                    stddev: 31,
                },
            ],
        }),
    };

    let raw_msg = fs::read("test_data/sbp.rtcm")?;
    let frame = decoder
        .decode(&mut BytesMut::from(&raw_msg[..]))
        .unwrap()
        .unwrap();
    assert_eq!(frame.preamble, PREAMBLE);
    assert_eq!(frame.reserved, RESERVED);
    let msg: SbpFrame = match frame.message {
        Message::SbpMessage { reserved: _, sbp } => sbp,
        _ => panic!("wrong message type"),
    };
    assert_eq!(msg, expected_msg);
    Ok(())
}
