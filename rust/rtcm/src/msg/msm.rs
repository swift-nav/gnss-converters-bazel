use super::data_types::*;
use bitvec::prelude::*;
use deku::{ctx::Endian, prelude::*};
use serde::Serialize;
use std::cmp::min;

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone, Copy)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct MsmHeader {
    pub station_id: U12,
    pub tow: U30,
    pub multiple_messages: Bit1,
    pub iods1: U3,
    pub reserved: U7,
    pub clock_steering: U2,
    pub external_clock: U2,
    pub divergence_free: Bit1,
    pub smoothing: U3,
    pub sat_mask: U64,
    pub sig_mask: U32,
    #[deku(
        reader = "MsmHeader::read_cell_mask(deku::rest, *sat_mask, *sig_mask)",
        writer = "MsmHeader::write_cell_mask(deku::output, *cell_mask, *sat_mask, *sig_mask)"
    )]
    pub cell_mask: u64,
}

impl MsmHeader {
    fn read_cell_mask(
        rest: &BitSlice<Msb0, u8>,
        sat_mask: U64,
        sig_mask: U32,
    ) -> Result<(&BitSlice<Msb0, u8>, u64), DekuError> {
        let nsat = sat_mask.0.count_ones() as usize;
        let nsig = sig_mask.0.count_ones() as usize;
        let size = min(nsat * nsig, 64);
        u64::read(rest, (deku::ctx::Endian::Big, deku::ctx::Size::Bits(size)))
    }

    fn write_cell_mask(
        output: &mut BitVec<Msb0, u8>,
        cell_mask: u64,
        sat_mask: U64,
        sig_mask: U32,
    ) -> Result<(), DekuError> {
        let nsat = sat_mask.0.count_ones() as usize;
        let nsig = sig_mask.0.count_ones() as usize;
        let size = min(nsat * nsig, 64);
        cell_mask.write(
            output,
            (deku::ctx::Endian::Big, deku::ctx::Size::Bits(size)),
        )
    }
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian, nsats: u32")]
pub struct Msm1_2_3SatelliteData {
    #[deku(count = "nsats")]
    pub rough_ranges_modulo_ms: Vec<U10>,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian, nsats: u32")]
pub struct Msm4_6SatelliteData {
    #[deku(count = "nsats")]
    pub rough_ranges_ms: Vec<U8>,
    #[deku(count = "nsats")]
    pub rough_ranges_modulo_ms: Vec<U10>,
}
#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian, nsats: u32")]
pub struct Msm5_7SatelliteData {
    #[deku(count = "nsats")]
    pub rough_ranges_ms: Vec<U8>,
    #[deku(count = "nsats")]
    pub sat_infos: Vec<U4>,
    #[deku(count = "nsats")]
    pub rough_ranges_modulo_ms: Vec<U10>,
    #[deku(count = "nsats")]
    pub rough_phaseranges_rate: Vec<I14>,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian, ncell: u32")]
pub struct Msm1SignalData {
    #[deku(count = "ncell")]
    pub fine_pseudoranges: Vec<I15>,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian, ncell: u32")]
pub struct Msm2SignalData {
    #[deku(count = "ncell")]
    pub fine_phaseranges: Vec<I22>,
    #[deku(count = "ncell")]
    pub phaserange_lock_times: Vec<U4>,
    #[deku(count = "ncell")]
    pub half_cycle_ambiguity_indicators: Vec<Bit1>,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian, ncell: u32")]
pub struct Msm3SignalData {
    #[deku(count = "ncell")]
    pub fine_pseudoranges: Vec<I15>,
    #[deku(count = "ncell")]
    pub fine_phaseranges: Vec<I22>,
    #[deku(count = "ncell")]
    pub phaserange_lock_times: Vec<U4>,
    #[deku(count = "ncell")]
    pub half_cycle_ambiguity_indicators: Vec<Bit1>,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian, ncell: u32")]
pub struct Msm4SignalData {
    #[deku(count = "ncell")]
    pub fine_pseudoranges: Vec<I15>,
    #[deku(count = "ncell")]
    pub fine_phaseranges: Vec<I22>,
    #[deku(count = "ncell")]
    pub phaserange_lock_times: Vec<U4>,
    #[deku(count = "ncell")]
    pub half_cycle_ambiguity_indicators: Vec<Bit1>,
    #[deku(count = "ncell")]
    pub signal_cnrs: Vec<U6>,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian, ncell: u32")]
pub struct Msm5SignalData {
    #[deku(count = "ncell")]
    pub fine_pseudoranges: Vec<I15>,
    #[deku(count = "ncell")]
    pub fine_phaseranges: Vec<I22>,
    #[deku(count = "ncell")]
    pub phaserange_lock_times: Vec<U4>,
    #[deku(count = "ncell")]
    pub half_cycle_ambiguity_indicators: Vec<Bit1>,
    #[deku(count = "ncell")]
    pub signal_cnrs: Vec<U6>,
    #[deku(count = "ncell")]
    pub fine_phaserange_rates: Vec<I15>,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian, ncell: u32")]
pub struct Msm6SignalData {
    #[deku(count = "ncell")]
    pub fine_pseudoranges: Vec<I20>,
    #[deku(count = "ncell")]
    pub fine_phaseranges: Vec<I24>,
    #[deku(count = "ncell")]
    pub phaserange_lock_times: Vec<U10>,
    #[deku(count = "ncell")]
    pub half_cycle_ambiguity_indicators: Vec<Bit1>,
    #[deku(count = "ncell")]
    pub signal_cnrs: Vec<U10>,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian, ncell: u32")]
pub struct Msm7SignalData {
    #[deku(count = "ncell")]
    pub fine_pseudoranges: Vec<I20>,
    #[deku(count = "ncell")]
    pub fine_phaseranges: Vec<I24>,
    #[deku(count = "ncell")]
    pub phaserange_lock_times: Vec<U10>,
    #[deku(count = "ncell")]
    pub half_cycle_ambiguity_indicators: Vec<Bit1>,
    #[deku(count = "ncell")]
    pub signal_cnrs: Vec<U10>,
    #[deku(count = "ncell")]
    pub fine_phaserange_rates: Vec<I15>,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msm1 {
    pub header: MsmHeader,
    #[deku(ctx = "header.sat_mask.0.count_ones()")]
    pub sat_data: Msm1_2_3SatelliteData,
    #[deku(ctx = "header.cell_mask.count_ones()")]
    pub signal_data: Msm1SignalData,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msm2 {
    pub header: MsmHeader,
    #[deku(ctx = "header.sat_mask.0.count_ones()")]
    pub sat_data: Msm1_2_3SatelliteData,
    #[deku(ctx = "header.cell_mask.count_ones()")]
    pub signal_data: Msm2SignalData,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msm3 {
    pub header: MsmHeader,
    #[deku(ctx = "header.sat_mask.0.count_ones()")]
    pub sat_data: Msm1_2_3SatelliteData,
    #[deku(ctx = "header.cell_mask.count_ones()")]
    pub signal_data: Msm3SignalData,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msm4 {
    pub header: MsmHeader,
    #[deku(ctx = "header.sat_mask.0.count_ones()")]
    pub sat_data: Msm4_6SatelliteData,
    #[deku(ctx = "header.cell_mask.count_ones()")]
    pub signal_data: Msm4SignalData,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msm5 {
    pub header: MsmHeader,
    #[deku(ctx = "header.sat_mask.0.count_ones()")]
    pub sat_data: Msm5_7SatelliteData,
    #[deku(ctx = "header.cell_mask.count_ones()")]
    pub signal_data: Msm5SignalData,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msm6 {
    pub header: MsmHeader,
    #[deku(ctx = "header.sat_mask.0.count_ones()")]
    pub sat_data: Msm4_6SatelliteData,
    #[deku(ctx = "header.cell_mask.count_ones()")]
    pub signal_data: Msm6SignalData,
}

#[derive(Debug, PartialEq, Eq, DekuRead, DekuWrite, Serialize, Clone)]
#[deku(endian = "endian", ctx = "endian: Endian")]
pub struct Msm7 {
    pub header: MsmHeader,
    #[deku(ctx = "header.sat_mask.0.count_ones()")]
    pub sat_data: Msm5_7SatelliteData,
    #[deku(ctx = "header.cell_mask.count_ones()")]
    pub signal_data: Msm7SignalData,
}
