use deku::{ctx::Endian, prelude::*};
use serde::Serialize;

#[cfg(feature = "sbp_staging")]
use sbp_staging as sbp;

#[cfg(not(feature = "sbp_staging"))]
use sbp_master as sbp;

#[derive(Debug, PartialEq, DekuRead, Clone, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Sbp {
    msg_type: u16,
    sender_id: u16,
    length: u8,
    #[serde(flatten)]
    #[deku(
        bytes_read = "*length",
        map = "|b: Vec<u8>| -> Result<_, DekuError> { Sbp::map(*msg_type, *sender_id, &b) }"
    )]
    msg: sbp::Sbp,
}

impl DekuWrite<Endian> for Sbp {
    fn write(
        &self,
        output: &mut deku::bitvec::BitVec<deku::bitvec::Msb0, u8>,
        ctx: Endian,
    ) -> Result<(), DekuError> {
        deku::DekuWrite::write(&self.msg_type, output, ctx)?;
        deku::DekuWrite::write(&self.sender_id, output, ctx)?;
        deku::DekuWrite::write(&self.length, output, ctx)?;

        sbp::to_writer(output, &self.msg)
            .map_err(|e| DekuError::Unexpected(format!("SBP to vec failed: {e}")))?;

        Ok(())
    }
}

impl Sbp {
    fn map(msg_type: u16, sender_id: u16, payload: &Vec<u8>) -> Result<sbp::Sbp, DekuError> {
        sbp::Sbp::from_frame(sbp::Frame {
            msg_type,
            sender_id,
            payload: payload.as_slice(),
        })
        .map_err(|err| DekuError::Parse(format!("{}", err)))
    }
}
