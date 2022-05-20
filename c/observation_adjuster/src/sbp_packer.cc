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

#include <observation_adjuster/sbp_packer.h>
#include <swiftnav/logging.h>

#include <cmath>

namespace obs_adjuster {

const u16 cSenderIdOut = 0;

SbpPacker::SbpPacker() : buffer_(std::make_unique<std::deque<u8>>()) {
  sbp::State::set_writer(this);
}

void SbpPacker::pack(const std::array<double, 3> &vrs_ecef) {
  sbp_msg_base_pos_ecef_t pos_msg{};
  pos_msg.x = vrs_ecef.at(0);
  pos_msg.y = vrs_ecef.at(1);
  pos_msg.z = vrs_ecef.at(2);

  if (sbp::State::send_message(cSenderIdOut, pos_msg) < SBP_OK) {
    assert(false && "Failed to write POS message in sbp_packer.cc");
  }
}

void SbpPacker::pack(const uint16_t sender,
                     const uint16_t msg_type,
                     const u8 *payload,
                     const std::size_t payload_size) {
  send_message(msg_type, sender, static_cast<u8>(payload_size), payload);
}

void SbpPacker::pack(
    const gps_time_t &timestamp,
    const SbpObsArray<obs_adjuster::MAX_OBS_PER_EPOCH> &obs_array) {
  if (obs_array.size() < 1) {
    return;
  }

  const std::size_t max_obs_per_message =
      (SBP_MAX_PAYLOAD_LEN - 11) / 17;  // 14
  const std::size_t nr_messages =
      (obs_array.size() - 1) / max_obs_per_message + 1;

  for (std::size_t msg_idx = 0; msg_idx < nr_messages; ++msg_idx) {
    // Make one obs message
    sbp_msg_obs_t obs_msg{};
    obs_msg.header.t.tow = static_cast<u32>(std::round(timestamp.tow * 1000));
    const double residual_ms =
        timestamp.tow * 1000 - std::round(timestamp.tow * 1000);
    obs_msg.header.t.ns_residual = static_cast<s32>(residual_ms * 1e6);
    obs_msg.header.t.wn = static_cast<u16>(timestamp.wn);

    const std::size_t obs_start_idx = msg_idx * max_obs_per_message;
    // Assume we'll fill the message with obs, if we don't have that many then
    // re-calculate
    std::size_t nr_obs = max_obs_per_message;
    if (obs_start_idx + max_obs_per_message > obs_array.size()) {
      nr_obs = obs_array.size() - obs_start_idx;
    }

    for (std::size_t i = 0; i < nr_obs; i++) {
      sbp_packed_obs_content_t obs_content{};
      if (!obs_array.get(obs_start_idx + i, &obs_content)) {
        assert(false && "Failed to get obs in sbp_packer.cc");
        return;
      }
      obs_msg.obs[i] = obs_content;
    }

    obs_msg.n_obs = static_cast<u8>(nr_obs);
    obs_msg.header.n_obs =
        static_cast<u8>(nr_messages << 4);             // Size of the sequence
    obs_msg.header.n_obs |= static_cast<u8>(msg_idx);  // Sequence counter

    if (sbp::State::send_message(cSenderIdOut, obs_msg) < SBP_OK) {
      log_error("Failed to write OBS message in sbp_packer.cc");
    }
  }
}

std::unique_ptr<std::vector<u8>> SbpPacker::pop_packed_bytes(
    const std::size_t max_bytes) {
  std::size_t bytes_to_return = max_bytes;
  if (bytes_to_return > buffer_->size()) {
    bytes_to_return = buffer_->size();
  }
  std::unique_ptr<std::vector<u8>> to_return =
      std::make_unique<std::vector<u8>>(
          buffer_->begin(),
          buffer_->begin() + static_cast<s64>(bytes_to_return));

  buffer_->erase(buffer_->begin(),
                 buffer_->begin() + static_cast<s64>(bytes_to_return));

  return to_return;
}

s32 SbpPacker::write(const u8 *buffer, u32 buffer_length) {
  // Store the bytes in our internal buffer vector
  std::copy(buffer, buffer + buffer_length, std::back_inserter(*buffer_));
  enforce_max_buffer_size();
  return static_cast<s32>(buffer_length);
}

void SbpPacker::enforce_max_buffer_size() {
  while (buffer_->size() > cMaxBytesToKeep) {
    buffer_->pop_front();
  }
}

}  // namespace obs_adjuster
