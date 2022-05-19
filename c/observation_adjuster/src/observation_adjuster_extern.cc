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

#include <gnss-converters-extra/observation_adjuster/observation_adjuster_extern.h>
#include <observation_adjuster/observation_adjuster.h>

#include <cassert>

/// Allocate Observation adjuster
void *obs_adjuster_extern_new(const double vrs_ecef[3],
                              const uint8_t msm_output_type) {
  return new ObservationAdjuster(vrs_ecef, msm_output_type);
}

/// Free Observations adjuster.
void obs_adjuster_extern_delete(void *ctx) {
  assert(ctx != nullptr);
  auto obs_adjuster(static_cast<ObservationAdjuster *>(ctx));
  delete obs_adjuster;
}

/// Read SBP frames from station observations.
void obs_adjuster_extern_read_station_observations(
    void *ctx, uint16_t sender, uint16_t type, uint8_t *buf, size_t size) {
  assert(ctx != nullptr);
  assert(buf != nullptr);
  auto obs_adjuster(static_cast<ObservationAdjuster *>(ctx));
  obs_adjuster->read_station_observations(sender, type, buf, size);
}

/// Read SBP frames from station corrections.
void obs_adjuster_extern_read_station_corrections(
    void *ctx, uint16_t sender, uint16_t type, uint8_t *buf, size_t size) {
  assert(ctx != nullptr);
  assert(buf != nullptr);
  auto obs_adjuster(static_cast<ObservationAdjuster *>(ctx));
  obs_adjuster->read_station_corrections(sender, type, buf, size);
}

/// Read SBP frames from receiver corrections.
void obs_adjuster_extern_read_receiver_corrections(
    void *ctx, uint16_t sender, uint16_t type, uint8_t *buf, size_t size) {
  assert(ctx != nullptr);
  assert(buf != nullptr);
  auto obs_adjuster(static_cast<ObservationAdjuster *>(ctx));
  obs_adjuster->read_receiver_corrections(sender, type, buf, size);
}

/// Write SBP frames with VRS corrections.
size_t obs_adjuster_extern_write_VRS_corrections_SBP(void *ctx,
                                                     uint8_t *buf,
                                                     size_t size) {
  assert(ctx != nullptr);
  assert(buf != nullptr);
  auto obs_adjuster(static_cast<ObservationAdjuster *>(ctx));
  return obs_adjuster->write_VRS_corrections_SBP(buf, size);
}

/// Write RTCM data with VRS corrections.
size_t obs_adjuster_extern_write_VRS_corrections_RTCM(void *ctx,
                                                      uint8_t *buf,
                                                      size_t size) {
  assert(ctx != nullptr);
  assert(buf != nullptr);
  auto obs_adjuster(static_cast<ObservationAdjuster *>(ctx));
  return obs_adjuster->write_VRS_corrections_RTCM(buf, size);
}
