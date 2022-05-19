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

#ifndef GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_H
#define GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_H

#include <gnss-converters-extra/sbp_conv.h>
#include <observation_adjuster/internal/observation_adjuster_constants.h>
#include <observation_adjuster/internal/sbp_obs_array.h>
#include <observation_adjuster/message_matcher.h>
#include <observation_adjuster/sbp_packer.h>
#include <observation_adjuster/sbp_unpacker.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

class ObservationAdjuster {
 public:
  /// Initialize the library, generating observations at the location vrs_ecef.
  /// Provide which MSM output version to use (maps to msm_type_e), defaults
  /// to MSM5. Set msm_output_type to 0 to output legacy style RTCM.
  explicit ObservationAdjuster(const double vrs_ecef[3],
                               const uint8_t msm_output_type = 5);

  virtual ~ObservationAdjuster();

  ObservationAdjuster(const ObservationAdjuster &) = delete;
  ObservationAdjuster(ObservationAdjuster &&) = delete;
  ObservationAdjuster &operator=(const ObservationAdjuster &) = delete;
  ObservationAdjuster &operator=(ObservationAdjuster &&other) noexcept = delete;

  /// Read SBP frames from station observations. buf is the payload of the msg.
  void read_station_observations(uint16_t sender,
                                 uint16_t type,
                                 uint8_t *buf,
                                 size_t size);

  /// Read SBP frames from station corrections. buf is the payload of the msg.
  void read_station_corrections(uint16_t sender,
                                uint16_t type,
                                uint8_t *buf,
                                size_t size);

  /// Read SBP frames from receiver corrections. buf is the payload of the msg.
  void read_receiver_corrections(uint16_t sender,
                                 uint16_t type,
                                 uint8_t *buf,
                                 size_t size);

  /// Write SBP frames to VRS corrections.
  size_t write_VRS_corrections_SBP(uint8_t *buf, size_t size);

  /// Write RTCM frames to VRS corrections.
  size_t write_VRS_corrections_RTCM(uint8_t *buf, size_t size);

 private:
  void obs_decoded(const obs_adjuster::StreamType stream_type,
                   const obs_adjuster::TimestampedSbpObsArray<
                       obs_adjuster::MAX_OBS_PER_EPOCH> &obs_array);

  std::mutex mutex_;
  std::array<double, 3> vrs_ecef_;
  std::array<std::unique_ptr<obs_adjuster::SbpUnpacker>,
             obs_adjuster::STREAM_TYPE_COUNT>
      unpackers_;
  obs_adjuster::MessageMatcher message_matcher_;
  obs_adjuster::SbpPacker sbp_packer_;
  sbp_conv_t sbp2rtcm_converter_;
};

#endif  // GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_H
