use super::data_types::*;
use deku::{ctx::Endian, prelude::*};
use serde::Serialize;

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1004 {
    pub reference_station_id: U12,
    pub gps_epoch_time_tow: U30,
    pub synchronous_gnss_flag: Bit1,
    pub no_of_gps_satellite_signals_processed: U5,
    pub gps_divergence_free_smoothing_indicator: Bit1,
    pub gps_smoothing_interval: U3,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1004 {
    pub satellite_id: U6,
    pub l1_code_indicator: Bit1,
    pub l1_pseudorange: U24,
    pub l1_phaserange_l1_pseudorange: I20,
    pub l1_lock_time_indicator: U7,
    pub integer_l1_pseudorange_modulus_ambiguity: U8,
    pub l1_cnr: U8,
    pub l2_code_indicator: U2,
    pub l2_l1_pseudorange_difference: I14,
    pub l2_phaserange_l1_pseudorange: I20,
    pub l2_lock_time_indicator: U7,
    pub l2_cnr: U8,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1004 {
    pub header: Header1004,
    #[deku(count = "header.no_of_gps_satellite_signals_processed.0")]
    pub satellites: Vec<Satellite1004>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1012 {
    pub reference_station_id: U12,
    pub glo_epoch_time_tk: U27,
    pub synchronous_gnss_flag: Bit1,
    pub no_of_glo_satellite_signals_processed: U5,
    pub glo_divergence_free_smoothing_indicator: Bit1,
    pub glo_smoothing_interval: U3,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1012 {
    pub satellite_id: U6,
    pub l1_code_indicator: Bit1,
    pub satellite_frequency_channel_number: U5,
    pub l1_pseudorange: U25,
    pub l1_phaserange_l1_pseudorange: I20,
    pub l1_lock_time_indicator: U7,
    pub int_l1_pseudorange_modulus_ambiguity: U7,
    pub l1_cnr: U8,
    pub l2_code_indicator: U2,
    pub l2_l1_pseudorange_difference: I14,
    pub l2_phaserange_l1_pseudorange: I20,
    pub l2_lock_time_indicator: U7,
    pub l2_cnr: U8,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1012 {
    pub header: Header1012,
    #[deku(count = "header.no_of_glo_satellite_signals_processed.0")]
    pub satellites: Vec<Satellite1012>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1230 {
    pub reference_station_id: U12,
    pub glo_code_phase_bias_indicator: Bit1,
    pub reserved: U3,
    pub glo_fdma_signals_mask: U4,
    pub glo_l1_c_a_code_phase_bias: I16,
    pub glo_l1_p_code_phase_bias: I16,
    pub glo_l2_c_a_code_phase_bias: I16,
    pub glo_l2_p_code_phase_bias: I16,
}
