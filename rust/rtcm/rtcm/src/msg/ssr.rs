use super::data_types::*;
use deku::{ctx::Endian, prelude::*};
use serde::Serialize;

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1057 {
    pub gps_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub satellite_reference_datum: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1057 {
    pub gps_satellite_id: U6,
    pub gps_iode: U8,
    pub delta_radial: I22,
    pub delta_along_track: I20,
    pub delta_cross_track: I20,
    pub dot_delta_radial: I21,
    pub dot_delta_along_track: I19,
    pub dot_delta_cross_track: I19,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1057 {
    pub header: Header1057,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1057>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1058 {
    pub gps_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1058 {
    pub gps_satellite_id: U6,
    pub delta_clock_c0: I22,
    pub delta_clock_c1: I21,
    pub delta_clock_c2: I27,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1058 {
    pub header: Header1058,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1058>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1059 {
    pub gps_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1059 {
    pub gps_satellite_id: U6,
    pub no_of_code_biases_processed: U5,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Code1059 {
    pub gps_signal_and_tracking_mode_indicator: U5,
    pub code_bias: I14,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct SvBlock1059 {
    pub satellite: Satellite1059,
    #[deku(count = "satellite.no_of_code_biases_processed.0")]
    pub code: Vec<Code1059>,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1059 {
    pub header: Header1059,
    #[deku(count = "header.no_of_satellites.0")]
    pub sv_blocks: Vec<SvBlock1059>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1060 {
    pub gps_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub satellite_reference_datum: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1060 {
    pub gps_satellite_id: U6,
    pub gps_iode: U8,
    pub delta_radial: I22,
    pub delta_along_track: I20,
    pub delta_cross_track: I20,
    pub dot_delta_radial: I21,
    pub dot_delta_along_track: I19,
    pub dot_delta_cross_track: I19,
    pub delta_clock_c0: I22,
    pub delta_clock_c1: I21,
    pub delta_clock_c2: I27,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1060 {
    pub header: Header1060,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1060>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1240 {
    pub gps_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub satellite_reference_datum: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1240 {
    pub gal_satellite_id: U6,
    pub gal_iodnav_i_nav: U10,
    pub delta_radial: I22,
    pub delta_along_track: I20,
    pub delta_cross_track: I20,
    pub dot_delta_radial: I21,
    pub dot_delta_along_track: I19,
    pub dot_delta_cross_track: I19,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1240 {
    pub header: Header1240,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1240>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1241 {
    pub gps_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1241 {
    pub gal_satellite_id: U6,
    pub delta_clock_c0: I22,
    pub delta_clock_c1: I21,
    pub delta_clock_c2: I27,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1241 {
    pub header: Header1241,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1241>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1242 {
    pub gal_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1242 {
    pub gal_satellite_id: U6,
    pub no_of_code_biases_processed: U5,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Code1242 {
    pub gal_signal_and_tracking_mode_indicator: U5,
    pub code_bias: I14,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct SvBlock1242 {
    pub satellite: Satellite1242,
    #[deku(count = "satellite.no_of_code_biases_processed.0")]
    pub code: Vec<Code1242>,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1242 {
    pub header: Header1242,
    #[deku(count = "header.no_of_satellites.0")]
    pub sv_blocks: Vec<SvBlock1242>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1243 {
    pub gal_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub satellite_reference_datum: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1243 {
    pub gal_satellite_id: U6,
    pub gal_iodnav_i_nav: U10,
    pub delta_radial: I22,
    pub delta_along_track: I20,
    pub delta_cross_track: I20,
    pub dot_delta_radial: I21,
    pub dot_delta_along_track: I19,
    pub dot_delta_cross_track: I19,
    pub delta_clock_c0: I22,
    pub delta_clock_c1: I21,
    pub delta_clock_c2: I27,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1243 {
    pub header: Header1243,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1243>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1258 {
    pub bds_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub satellite_reference_datum: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1258 {
    pub bds_satellite_id: U6,
    pub bds_t_oe_modulo: U10,
    pub bds_iod: U8,
    pub delta_radial: I22,
    pub delta_along_track: I20,
    pub delta_cross_track: I20,
    pub dot_delta_radial: I21,
    pub dot_delta_along_track: I19,
    pub dot_delta_cross_track: I19,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1258 {
    pub header: Header1258,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1258>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1259 {
    pub bds_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub satellite_reference_datum: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1259 {
    pub bds_satellite_id: U6,
    pub delta_clock_c0: I22,
    pub delta_clock_c1: I21,
    pub delta_clock_c2: I27,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1259 {
    pub header: Header1259,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1259>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1260 {
    pub bds_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1260 {
    pub gal_satellite_id: U6,
    pub no_of_code_biases_processed: U5,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Code1260 {
    pub gal_signal_and_tracking_mode_indicator: U5,
    pub code_bias: I14,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct SvBlock1260 {
    pub satellite: Satellite1260,
    #[deku(count = "satellite.no_of_code_biases_processed.0")]
    pub code: Vec<Code1260>,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1260 {
    pub header: Header1260,
    #[deku(count = "header.no_of_satellites.0")]
    pub sv_blocks: Vec<SvBlock1260>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1261 {
    pub bds_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub satellite_reference_datum: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1261 {
    pub bds_satellite_id: U6,
    pub bds_t_oe_modulo: U10,
    pub bds_iod: U8,
    pub delta_radial: I22,
    pub delta_along_track: I20,
    pub delta_cross_track: I20,
    pub dot_delta_radial: I21,
    pub dot_delta_along_track: I19,
    pub dot_delta_cross_track: I19,
    pub delta_clock_c0: I22,
    pub delta_clock_c1: I21,
    pub delta_clock_c2: I27,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1261 {
    pub header: Header1261,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1261>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1265 {
    pub gps_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub dispersive_bias_consistency_indicator: Bit1,
    pub mw_consistency_indicator: Bit1,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1265 {
    pub gps_satellite_id: U6,
    pub no_of_phase_biases_processed: U5,
    pub yaw_angle: U9,
    pub yaw_rate: I8,
    #[deku(count = "no_of_phase_biases_processed.0")]
    pub phases: Vec<Phase1265>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Phase1265 {
    pub gps_signal_and_tracking_mode_indicator: U5,
    pub signal_integer_indicator: Bit1,
    pub signals_wide_lane_integer_indicator: U2,
    pub signal_discontinuity_counter: U4,
    pub phase_bias: I20,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1265 {
    pub header: Header1265,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1265>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1267 {
    pub gal_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub dispersive_bias_consistency_indicator: Bit1,
    pub mw_consistency_indicator: Bit1,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1267 {
    pub gal_satellite_id: U6,
    pub no_of_phase_biases_processed: U5,
    pub yaw_angle: U9,
    pub yaw_rate: I8,
    #[deku(count = "no_of_phase_biases_processed.0")]
    pub phases: Vec<Phase1270>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Phase1267 {
    pub gal_signal_and_tracking_mode_indicator: U5,
    pub signal_integer_indicator: Bit1,
    pub signals_wide_lane_integer_indicator: U2,
    pub signal_discontinuity_counter: U4,
    pub phase_bias: I20,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1267 {
    pub header: Header1267,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1267>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Header1270 {
    pub bds_epoch_time_1s: U20,
    pub ssr_update_interval: U4,
    pub multiple_message_indicator: Bit1,
    pub iod_ssr: U4,
    pub ssr_provider_id: U16,
    pub ssr_solution_id: U4,
    pub dispersive_bias_consistency_indicator: Bit1,
    pub mw_consistency_indicator: Bit1,
    pub no_of_satellites: U6,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Satellite1270 {
    pub bds_satellite_id: U6,
    pub no_of_phase_biases_processed: U5,
    pub yaw_angle: U9,
    pub yaw_rate: I8,
    #[deku(count = "no_of_phase_biases_processed.0")]
    pub phases: Vec<Phase1270>,
}

#[derive(Debug, PartialEq, Clone, Copy, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Phase1270 {
    pub bds_signal_and_tracking_mode_indicator: U5,
    pub signal_integer_indicator: Bit1,
    pub signals_wide_lane_integer_indicator: U2,
    pub signal_discontinuity_counter: U4,
    pub phase_bias: I20,
}

#[derive(Debug, PartialEq, Clone, DekuRead, DekuWrite, Serialize)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msg1270 {
    pub header: Header1270,
    #[deku(count = "header.no_of_satellites.0")]
    pub satellites: Vec<Satellite1270>,
}
