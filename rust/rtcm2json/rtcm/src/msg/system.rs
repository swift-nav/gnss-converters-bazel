use super::data_types::*;
use bitvec::prelude::*;
use deku::{ctx::Endian, prelude::*};
use serde::Serialize;
use std::str;

// use 'reader' instead of 'bytes_read' + 'map' because 'bytes_read = 0' ends with decode error
#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1029 {
    pub reference_station_id: U12,
    pub modified_julian_day_number: U16,
    pub seconds_of_day_utc: U17,
    pub number_of_characters_to_follow: U7,
    pub number_of_code_units: U8,
    #[deku(
        reader = "Self::read_utf_8(deku::rest, number_of_code_units.0)",
        writer = "Self::write_utf_8(deku::output, &self.character_code_units)"
    )]
    pub character_code_units: String,
}

impl Msg1029 {
    fn read_utf_8(
        mut rest: &BitSlice<Msb0, u8>,
        counter: u8,
    ) -> Result<(&BitSlice<Msb0, u8>, String), DekuError> {
        let mut ret_val: Vec<u8> = Vec::new();
        let mut value: u8;
        for _ in 0..counter {
            (rest, value) = u8::read(rest, (deku::ctx::Endian::Big, deku::ctx::Size::Bits(8)))?;
            ret_val.push(value);
        }
        match str::from_utf8(&ret_val) {
            Ok(v) => Ok((rest, v.to_string())),
            Err(e) => Err(DekuError::Parse(format!("Invalid UTF-8 sequence: {}", e))),
        }
    }

    fn write_utf_8(output: &mut BitVec<Msb0, u8>, field: &str) -> Result<(), DekuError> {
        for c in field.as_bytes() {
            c.write(output, ())?;
        }
        Ok(())
    }
}
