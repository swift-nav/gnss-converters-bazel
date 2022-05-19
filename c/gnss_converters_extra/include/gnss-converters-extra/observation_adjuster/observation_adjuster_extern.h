/**
 * Copyright (C) 2021 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_OBSERVATION_ADJUSTER_EXTERN_H
#define GNSS_CONVERTERS_OBSERVATION_ADJUSTER_EXTERN_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Allocate Observation adjuster.
/// vrs_ecef is the ECEF coordinate of the virtual reference station
/// position the user will receive.
/// msm_output_type controls which MSM version of RTCM output is produced.
/// Set to 0 for legacy RTCM output, or 4 for MSM4, or 5 for MSM5 output.
void *obs_adjuster_extern_new(const double vrs_ecef[3],
                              const uint8_t msm_output_type);

/// Free Observations adjuster.
void obs_adjuster_extern_delete(void *ctx);

/// Read SBP frames from station observations.
void obs_adjuster_extern_read_station_observations(
    void *ctx, uint16_t sender, uint16_t type, uint8_t *buf, size_t size);

/// Read SBP frames from station corrections.
void obs_adjuster_extern_read_station_corrections(
    void *ctx, uint16_t sender, uint16_t type, uint8_t *buf, size_t size);

/// Read SBP frames from receiver corrections.
void obs_adjuster_extern_read_receiver_corrections(
    void *ctx, uint16_t sender, uint16_t type, uint8_t *buf, size_t size);

/// Write SBP frames with VRS corrections.
size_t obs_adjuster_extern_write_VRS_corrections_SBP(void *ctx,
                                                     uint8_t *buf,
                                                     size_t size);

/// Write RTCM data with VRS corrections.
size_t obs_adjuster_extern_write_VRS_corrections_RTCM(void *ctx,
                                                      uint8_t *buf,
                                                      size_t size);

#ifdef __cplusplus
}
#endif

#endif  // GNSS_CONVERTERS_OBSERVATION_ADJUSTER_EXTERN_H
