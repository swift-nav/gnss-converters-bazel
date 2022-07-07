use super::data_types::*;
use bitvec::prelude::*;
use deku::{ctx::Endian, prelude::*};
use serde::Serialize;

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1005 {
    pub reference_station_id: U12,
    pub reserved_itrf_realization_year: U6,
    pub gps: Bit1,
    pub glonass: Bit1,
    pub galileo: Bit1,
    pub reference_station: Bit1,
    pub antenna_reference_point_ecef_x: I38,
    pub single_receiver_oscillator: Bit1,
    pub reserved: Bit1,
    pub antenna_reference_point_ecef_y: I38,
    pub quarter_cycle_indicator: U2,
    pub antenna_reference_point_ecef_z: I38,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1006 {
    pub reference_station_id: U12,
    pub reserved_itrf_realization_year: U6,
    pub gps: Bit1,
    pub glonass: Bit1,
    pub galileo: Bit1,
    pub reference_station: Bit1,
    pub antenna_reference_point_ecef_x: I38,
    pub single_receiver_oscillator: Bit1,
    pub reserved: Bit1,
    pub antenna_reference_point_ecef_y: I38,
    pub quarter_cycle_indicator: U2,
    pub antenna_reference_point_ecef_z: I38,
    pub antenna_height: U16,
}

// use 'reader' instead of 'bytes_read' + 'map' because 'bytes_read = 0' ends with decode error
#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1008 {
    pub reference_station_id: U12,
    pub descriptor_counter: U8,
    #[deku(
        reader = "read_string(deku::rest, descriptor_counter.0)",
        writer = "write_string(deku::output, &self.antenna_descriptor)"
    )]
    pub antenna_descriptor: String,
    pub antenna_setup_id: U8,
    pub serial_number_counter: U8,
    #[deku(
        reader = "read_string(deku::rest, serial_number_counter.0)",
        writer = "write_string(deku::output, &self.antenna_serial_number)"
    )]
    pub antenna_serial_number: String,
}

// use 'reader' instead of 'bytes_read' + 'map' because 'bytes_read = 0' ends with decode error
#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1033 {
    pub reference_station_id: U12,
    pub antenna_descriptor_counter: U8,
    #[deku(
        reader = "read_string(deku::rest, antenna_descriptor_counter.0)",
        writer = "write_string(deku::output, &self.antenna_descriptor)"
    )]
    pub antenna_descriptor: String,
    pub antenna_setup_id: U8,
    pub antenna_serial_number_counter: U8,
    #[deku(
        reader = "read_string(deku::rest, antenna_serial_number_counter.0)",
        writer = "write_string(deku::output, &self.antenna_serial_number)"
    )]
    pub antenna_serial_number: String,
    pub receiver_type_descriptor_counter: U8,
    #[deku(
        reader = "read_string(deku::rest, receiver_type_descriptor_counter.0)",
        writer = "write_string(deku::output, &self.receiver_type_descriptor)"
    )]
    pub receiver_type_descriptor: String,
    pub receiver_firmware_version_counter: U8,
    #[deku(
        reader = "read_string(deku::rest, receiver_firmware_version_counter.0)",
        writer = "write_string(deku::output, &self.receiver_firmware_version)"
    )]
    pub receiver_firmware_version: String,
    pub receiver_serial_number_counter: U8,
    #[deku(
        reader = "read_string(deku::rest, receiver_serial_number_counter.0)",
        writer = "write_string(deku::output, &self.receiver_serial_number)"
    )]
    pub receiver_serial_number: String,
}

fn read_string(
    mut rest: &BitSlice<Msb0, u8>,
    counter: u8,
) -> Result<(&BitSlice<Msb0, u8>, String), DekuError> {
    let mut ret_val: Vec<char> = Vec::new();
    let mut value: u8;
    for _ in 0..counter {
        (rest, value) = u8::read(rest, (deku::ctx::Endian::Big, deku::ctx::Size::Bits(8)))?;
        ret_val.push(value as char);
    }
    Ok((rest, ret_val.into_iter().collect()))
}

/// Parse from String to u8 and write
fn write_string(output: &mut BitVec<Msb0, u8>, field: &str) -> Result<(), DekuError> {
    for c in field.as_bytes() {
        c.write(output, ())?;
    }
    Ok(())
}
