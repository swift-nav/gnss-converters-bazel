mod antennas;
mod data_types;
mod ephemeris;
mod msm;
mod observations;
mod sbp_msg;
mod ssr;
mod system;

pub use antennas::*;
pub use data_types::*;
pub use ephemeris::*;
pub use msm::*;
pub use observations::*;
pub use sbp_msg::*;
pub use ssr::*;
pub use system::*;

use deku::{ctx::Endian, prelude::*};
use serde::{Serialize, Serializer};
use serde_with::SerializeAs;

#[derive(Debug, PartialEq, DekuRead, DekuWrite, Clone, Serialize)]
#[serde(untagged)]
#[deku(
    endian = "endian",
    ctx = "endian: Endian, length: u16",
    type = "u16",
    bits = 12
)]
pub enum Message {
    #[deku(id = "1004")]
    Msg1004(Msg1004),
    #[deku(id = "1005")]
    Msg1005(Msg1005),
    #[deku(id = "1006")]
    Msg1006(Msg1006),
    #[deku(id = "1008")]
    Msg1008(Msg1008),
    #[deku(id = "1012")]
    Msg1012(Msg1012),
    #[deku(id = "1019")]
    Msg1019(Msg1019),
    #[deku(id = "1020")]
    Msg1020(Msg1020),
    #[deku(id = "1029")]
    Msg1029(Msg1029),
    #[deku(id = "1033")]
    Msg1033(Msg1033),
    #[deku(id = "1042")]
    Msg1042(Msg1042),
    #[deku(id = "1045")]
    Msg1045(Msg1045),
    #[deku(id = "1046")]
    Msg1046(Msg1046),
    #[deku(id = "1057")]
    Msg1057(Msg1057),
    #[deku(id = "1058")]
    Msg1058(Msg1058),
    #[deku(id = "1059")]
    Msg1059(Msg1059),
    #[deku(id = "1060")]
    Msg1060(Msg1060),
    #[deku(id = "1071")]
    GpsMsm1(Msm1),
    #[deku(id = "1072")]
    GpsMsm2(Msm2),
    #[deku(id = "1073")]
    GpsMsm3(Msm3),
    #[deku(id = "1074")]
    GpsMsm4(Msm4),
    #[deku(id = "1075")]
    GpsMsm5(Msm5),
    #[deku(id = "1076")]
    GpsMsm6(Msm6),
    #[deku(id = "1077")]
    GpsMsm7(Msm7),
    #[deku(id = "1081")]
    GloMsm1(Msm1),
    #[deku(id = "1082")]
    GloMsm2(Msm2),
    #[deku(id = "1083")]
    GloMsm3(Msm3),
    #[deku(id = "1084")]
    GloMsm4(Msm4),
    #[deku(id = "1085")]
    GloMsm5(Msm5),
    #[deku(id = "1086")]
    GloMsm6(Msm6),
    #[deku(id = "1087")]
    GloMsm7(Msm7),
    #[deku(id = "1091")]
    GalMsm1(Msm1),
    #[deku(id = "1092")]
    GalMsm2(Msm2),
    #[deku(id = "1093")]
    GalMsm3(Msm3),
    #[deku(id = "1094")]
    GalMsm4(Msm4),
    #[deku(id = "1095")]
    GalMsm5(Msm5),
    #[deku(id = "1096")]
    GalMsm6(Msm6),
    #[deku(id = "1097")]
    GalMsm7(Msm7),
    #[deku(id = "1101")]
    SbasMsm1(Msm1),
    #[deku(id = "1102")]
    SbasMsm2(Msm2),
    #[deku(id = "1103")]
    SbasMsm3(Msm3),
    #[deku(id = "1104")]
    SbasMsm4(Msm4),
    #[deku(id = "1105")]
    SbasMsm5(Msm5),
    #[deku(id = "1106")]
    SbasMsm6(Msm6),
    #[deku(id = "1107")]
    SbasMsm7(Msm7),
    #[deku(id = "1111")]
    QzssMsm1(Msm1),
    #[deku(id = "1112")]
    QzssMsm2(Msm2),
    #[deku(id = "1113")]
    QzssMsm3(Msm3),
    #[deku(id = "1114")]
    QzssMsm4(Msm4),
    #[deku(id = "1115")]
    QzssMsm5(Msm5),
    #[deku(id = "1116")]
    QzssMsm6(Msm6),
    #[deku(id = "1117")]
    QzssMsm7(Msm7),
    #[deku(id = "1121")]
    BdsMsm1(Msm1),
    #[deku(id = "1122")]
    BdsMsm2(Msm2),
    #[deku(id = "1123")]
    BdsMsm3(Msm3),
    #[deku(id = "1124")]
    BdsMsm4(Msm4),
    #[deku(id = "1125")]
    BdsMsm5(Msm5),
    #[deku(id = "1126")]
    BdsMsm6(Msm6),
    #[deku(id = "1127")]
    BdsMsm7(Msm7),
    #[deku(id = "1230")]
    Msg1230(Msg1230),
    #[deku(id = "1240")]
    Msg1240(Msg1240),
    #[deku(id = "1241")]
    Msg1241(Msg1241),
    #[deku(id = "1242")]
    Msg1242(Msg1242),
    #[deku(id = "1243")]
    Msg1243(Msg1243),
    // not tested
    #[deku(id = "1258")]
    Msg1258(Msg1258),
    // not tested
    #[deku(id = "1259")]
    Msg1259(Msg1259),
    #[deku(id = "1260")]
    Msg1260(Msg1260),
    #[deku(id = "1261")]
    Msg1261(Msg1261),
    #[deku(id = "1265")]
    Msg1265(Msg1265),
    #[deku(id = "1267")]
    Msg1267(Msg1267),
    #[deku(id = "1270")]
    Msg1270(Msg1270),
    #[deku(id = "4062")]
    SbpMessage {
        #[deku(bits = 4)]
        reserved: u8,
        sbp: SbpFrame,
    },
    #[deku(id_pat = "_")]
    Unknown {
        #[deku(bits = 12)]
        msg_type: u16,
        // 2 bytes = 12 bits (msg_type) + 4 bits (padding)
        #[serde(skip)]
        #[deku(count = "get_payload_length(length)")]
        payload: Vec<u8>,
        #[deku(bits = 4)]
        padding: u8,
    },
}

fn get_payload_length(length: u16) -> usize {
    {
        if length <= 2 {
            0
        } else {
            length as usize - 2
        }
    }
}

impl Default for Message {
    fn default() -> Message {
        Message::Unknown {
            msg_type: 1,
            payload: Vec::new(),
            padding: 0,
        }
    }
}

fn rtcm_msg_type(msg: &Message) -> u16 {
    let default_msg_type = 0;
    match msg.deku_id() {
        Ok(msg_type) => msg_type,
        Err(_) => match *msg {
            Message::Unknown { msg_type, .. } => msg_type,
            _ => default_msg_type,
        },
    }
}

#[derive(Serialize)]
pub struct MessageWithType<'a> {
    msg_type: u16,
    #[serde(flatten)]
    msg: &'a Message,
}

impl SerializeAs<Message> for MessageWithType<'_> {
    fn serialize_as<S>(source: &Message, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let msg_ser = MessageWithType {
            msg_type: rtcm_msg_type(source),
            msg: source,
        };

        msg_ser.serialize(serializer)
    }
}
