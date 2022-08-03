//! # rtcm

#![deny(
    trivial_casts,
    trivial_numeric_casts,
    unsafe_code,
    unstable_features,
    unused_import_braces,
    unused_qualifications
)]

mod base64;
mod crc;
pub mod msg;

use std::io;

use bitvec::prelude::*;
use deku::prelude::*;
use dencode::{Buf, BytesMut, FramedRead};
use msg::*;
use serde::{Deserialize, Serialize};
use serde_with::serde_as;

const CRC_LEN: usize = 3;
/// RTCM Frame preamble
const PREAMBLE: u8 = 0b11010011;
const FRAME_HEADER_LEN: usize =
    /* Preamble */
    1 +
    /* Length + Reserved */
    2;
/// The minimum possible length of an RTCM frame
pub const MIN_FRAME_LEN: usize = FRAME_HEADER_LEN + CRC_LEN;
/// The maximum possible length of an RTCM frame
pub const MAX_FRAME_LEN: usize = MIN_FRAME_LEN + 1024;
const BYTE_SIZE: usize = 8;

/// todo(DEVINFRA-808): We currently only support serializing a frame from a
/// message payload
#[serde_as]
#[derive(Debug, PartialEq, DekuRead, DekuWrite, Serialize, Deserialize)]
#[deku(endian = "big")]
pub struct Frame {
    #[serde(skip_serializing)]
    #[serde(default = "Frame::preamble")]
    pub preamble: u8,
    #[serde(skip)]
    #[deku(bits = 6)]
    pub reserved: u8,
    #[deku(bits = 10)]
    pub msg_length: u16,
    #[deku(
        reader = "Frame::get_payload(deku::rest, *msg_length)",
        writer = "Frame::write_payload(deku::output, payload)"
    )]
    #[serde(with = "base64")]
    pub payload: Vec<u8>,
    #[serde(skip_deserializing)]
    #[serde(flatten)]
    #[serde_as(as = "MessageWithType")]
    #[deku(ctx = "*msg_length", writer = "Frame::write_nothing(deku::output)")]
    pub message: Message,
    #[deku(
        reader = "Frame::get_padding_length(deku::rest, deku::bit_offset, *msg_length)",
        writer = "Frame::write_nothing(deku::output)"
    )]
    pub num_padding_bits: usize,
    #[deku(
        reader = "Frame::get_padding(deku::rest, *num_padding_bits)",
        writer = "Frame::write_nothing(deku::output)"
    )]
    pub padding: u8,
    #[deku(bits = 24)]
    pub crc: u32,
}

impl Frame {
    // The RTCM spec reserves the right to not consume all bits in the message,
    // see section 3.5.16.3.4:
    //
    // "The standard reserves a possibility for potential future MSM extensions.
    // These extensions may be introduced by adding data to the end of a
    // message."
    //
    // This function finds the amount of unconsumed bits in the current message
    // so that we can properly find where the message ends.
    fn get_padding(
        rest: &BitSlice<Msb0, u8>,
        num_padding_bits: usize,
    ) -> Result<(&BitSlice<Msb0, u8>, u8), DekuError> {
        if num_padding_bits > 0 {
            return u8::read(rest, deku::ctx::Size::Bits(num_padding_bits));
        }
        Ok((rest, 0))
    }

    fn get_payload(
        rest: &BitSlice<Msb0, u8>,
        msg_length: u16,
    ) -> Result<(&BitSlice<Msb0, u8>, Vec<u8>), DekuError> {
        let wanted_bits = (msg_length * 8) as usize;
        let readable_bits = rest.len();
        if wanted_bits > readable_bits {
            return Err(DekuError::Incomplete(NeedSize::new(
                wanted_bits - readable_bits,
            )));
        }
        let mut ret = Vec::new();
        ret.extend_from_slice(&rest.as_raw_slice()[0..msg_length as usize]);
        Ok((rest, ret))
    }

    fn write_payload(output: &mut BitVec<Msb0, u8>, payload: &Vec<u8>) -> Result<(), DekuError> {
        DekuWrite::write(payload, output, ())
    }

    fn get_padding_length(
        rest: &BitSlice<Msb0, u8>,
        read_bits: usize,
        msg_length: u16,
    ) -> Result<(&BitSlice<Msb0, u8>, usize), DekuError> {
        let expected_bits = (FRAME_HEADER_LEN + msg_length as usize) * BYTE_SIZE;
        if read_bits > expected_bits {
            return Err(DekuError::Assertion(format!(
                "assertion failed: read_bits <= expected_bits, {} <= {}",
                read_bits, expected_bits
            )));
        }
        if expected_bits > read_bits {
            let padding_bits = expected_bits - read_bits;
            return Ok((rest, padding_bits));
        }
        Ok((rest, 0))
    }

    #[allow(dead_code)]
    fn set_padding(
        output: &mut BitVec<Msb0, u8>,
        padding: u8,
        padding_bits: usize,
    ) -> Result<(), DekuError> {
        u8::write(
            &padding,
            output,
            (deku::ctx::Endian::Big, deku::ctx::Size::Bits(padding_bits)),
        )
    }

    fn write_nothing(_output: &mut BitVec<Msb0, u8>) -> Result<(), DekuError> {
        Ok(())
    }

    pub fn preamble() -> u8 {
        PREAMBLE
    }

    pub fn from_slice(buf: &[u8]) -> Option<Result<Frame, Error>> {
        let mut buf = buf;
        RtcmDecoder::parse_frame(&mut buf)
    }
}

/// Possible errors while decoding an RTCM frame
#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("parsing library error")]
    ParseError(DekuError),
    #[error("crc error")]
    /// CRC mismatch
    CrcError(CrcError),
    #[error("io error")]
    /// IO Error
    IoError(io::Error),
    #[cfg(feature = "json")]
    #[error("JSON error")]
    JsonError(serde_json::Error),
}

impl From<DekuError> for Error {
    fn from(err: DekuError) -> Self {
        Self::ParseError(err)
    }
}

impl From<CrcError> for Error {
    fn from(err: CrcError) -> Self {
        Self::CrcError(err)
    }
}

#[cfg(feature = "json")]
impl From<serde_json::Error> for Error {
    fn from(err: serde_json::Error) -> Self {
        Self::JsonError(err)
    }
}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Self {
        Self::IoError(err)
    }
}

/// Crc mismatch
#[derive(Debug, Clone, Copy)]
pub struct CrcError {
    pub message_crc: u32,
    pub computed_crc: u32,
}

/// RTCM Decoder
#[derive(Debug, Clone, Copy)]
pub struct RtcmDecoder;

impl dencode::Decoder for RtcmDecoder {
    type Item = Frame;
    type Error = Error;

    fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
        let start = match src.iter().position(|b| b == &PREAMBLE) {
            Some(idx) => idx,
            None => {
                src.clear();
                return Ok(None);
            }
        };
        src.advance(start);
        match Self::parse_frame(src) {
            Some(Ok(frame)) => Ok(Some(frame)),
            Some(Err(err)) => {
                if src.remaining() == 0 {
                    return Ok(None);
                }
                src.advance(1);
                Err(err)
            }
            None => {
                src.reserve(MAX_FRAME_LEN);
                Ok(None)
            }
        }
    }
}

impl RtcmDecoder {
    fn parse_frame(buf: &mut impl Buf) -> Option<Result<Frame, Error>> {
        // NOTE: usage of `Buf::chunk` here is not correct for every implementation
        // of `Buf` but it works for `BytesMut` and &[u8] which is all we are using
        match Frame::from_bytes((buf.chunk(), 0)) {
            Ok((_, value)) => {
                let computed_crc =
                    crc::crc24q(&buf.chunk()[0..FRAME_HEADER_LEN + value.msg_length as usize]);
                if computed_crc != value.crc {
                    return Some(Err(Error::from(CrcError {
                        message_crc: value.crc,
                        computed_crc,
                    })));
                }
                buf.advance(FRAME_HEADER_LEN + value.msg_length as usize + CRC_LEN);
                Some(Ok(value))
            }
            Err(DekuError::Incomplete(_)) => None,
            Err(e) => Some(Err(e.into())),
        }
    }
}

pub fn iter_messages<R: io::Read>(input: R) -> impl Iterator<Item = Result<Frame, Error>> {
    FramedRead::new(input, RtcmDecoder)
}

#[cfg(feature = "json")]
pub mod json {

    use super::*;

    #[derive(Debug, Clone, Copy)]
    pub struct DecoderJson;

    impl dencode::Decoder for DecoderJson {
        type Item = crate::Frame;
        type Error = Error;

        fn decode(&mut self, src: &mut BytesMut) -> Result<Option<Self::Item>, Self::Error> {
            match Self::parse_frame(src) {
                Some(Ok((frame, consumed_bytes))) => {
                    src.advance(consumed_bytes);
                    Ok(Some(frame))
                }
                Some(Err(err)) => {
                    src.advance(1);
                    Err(err)
                }
                None => Ok(None),
            }
        }
    }

    impl DecoderJson {
        fn parse_frame(buf: &mut BytesMut) -> Option<Result<(crate::Frame, usize), Error>> {
            let de = serde_json::Deserializer::from_slice(buf.as_ref());
            let mut stream = de.into_iter::<crate::Frame>();
            if let Some(res) = stream.next() {
                match res {
                    Ok(f) => return Some(Ok((f, stream.byte_offset()))),
                    Err(e) => {
                        if e.is_eof() {
                            return None;
                        }
                        return Some(Err(e.into()));
                    }
                }
            }
            None
        }
    }

    pub fn iter_messages<R: io::Read>(
        input: R,
    ) -> impl Iterator<Item = Result<crate::Frame, Error>> {
        FramedRead::new(input, DecoderJson)
    }
}

#[cfg(test)]
mod msg_test;
