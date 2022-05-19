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

#ifndef GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_SBP_PACKER_H
#define GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_SBP_PACKER_H

#include <libsbp/cpp/state.h>
#include <observation_adjuster/internal/observation_adjuster_constants.h>
#include <observation_adjuster/internal/sbp_obs_array.h>

#include <array>
#include <deque>
#include <memory>
#include <vector>

namespace obs_adjuster {

class SbpPacker final : private sbp::IWriter, private sbp::State {
 public:
  SbpPacker();
  ~SbpPacker() override = default;

  // Input observation data. SBP MSG_OBS will be available
  // in pop_packed_bytes, split over multiple messages if necessary.
  void pack(const gps_time_t &timestamp,
            const SbpObsArray<obs_adjuster::MAX_OBS_PER_EPOCH> &obs_array);
  // Input the VRS ecef location. SBP MSG_BASE_POS_ECEF will be available
  // in pop_packed_bytes.
  void pack(const std::array<double, 3> &vrs_ecef);
  // Insert a raw SBP message. *buffer shall contain a payload of an SBP msg
  // The payload will be packed into a full SBP message (with header, CRC, etc).
  void pack(const uint16_t sender,
            const uint16_t msg_type,
            const u8 *payload,
            const std::size_t payload_size);

  // Will always return a pointer, but the vector inside may be empty
  std::unique_ptr<std::vector<u8>> pop_packed_bytes(
      const std::size_t max_bytes = cMaxBytesToKeep);

 private:
  s32 write(const u8 *buffer, u32 buffer_length) override;
  void enforce_max_buffer_size();

  static constexpr std::size_t cMaxBytesToKeep = 10 * 1024;  // 10k
  std::unique_ptr<std::deque<u8>> buffer_;
};

}  // namespace obs_adjuster

#endif  // GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_SBP_PACKER_H
