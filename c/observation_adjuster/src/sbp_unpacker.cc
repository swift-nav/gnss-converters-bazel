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

#include <observation_adjuster/sbp_unpacker.h>
#include <swiftnav/logging.h>

#include <cstring>
#include <set>
#include <sstream>

namespace obs_adjuster {

SbpUnpacker::SbpUnpacker(const StreamType stream_type)
    : SbpMessageHandler(this), stream_type_(stream_type) {
  obs_unpacker.set_obs_callback(std::bind(
      &SbpUnpacker::handle_unpacked_obs, this, std::placeholders::_1));
}

void SbpUnpacker::process(const uint16_t sender,
                          const uint16_t msg_type,
                          uint8_t* payload,
                          const size_t payload_size) {
  if (payload == nullptr || payload_size < 1) {
    return;
  }

  sbp::State::process_payload(
      sender, msg_type, static_cast<u8>(payload_size), payload);
}

void SbpUnpacker::set_obs_callback(
    const SbpUnpacker::ObsCallbackFn& callback_fn) {
  obs_callback_fn_ = callback_fn;
}

void SbpUnpacker::unset_obs_callback() { obs_callback_fn_ = ObsCallbackFn{}; }

void SbpUnpacker::handle_sbp_msg(uint16_t sender_id,
                                 const sbp_msg_obs_t& msg) noexcept {
  obs_unpacker.unpack_obs(sender_id, msg);
}

void SbpUnpacker::handle_unpacked_obs(
    const TimestampedSbpObsArray<MAX_OBS_PER_EPOCH>& obs_array) {
  // Count unique sat codes
  std::set<uint8_t> codes;

  for (std::size_t i = 0; i < obs_array.obs_.size(); i++) {
    sbp_packed_obs_content_t obs{};
    if (obs_array.obs_.get(i, &obs)) {
      codes.insert(obs.sid.code);
    }
  }

  std::stringstream stream;
  for (const uint8_t code : codes) {
    stream << " " << static_cast<uint32_t>(code);
  }

  log_info(
      "Combined OBS ( %zu ) for stream type %d at tow: %fs (codes: [ %s ])",
      obs_array.obs_.size(),
      static_cast<s32>(stream_type_),
      obs_array.timestamp_.tow,
      stream.str().c_str());

  if (obs_callback_fn_) {
    obs_callback_fn_(stream_type_, obs_array);
  }
}

}  // namespace obs_adjuster
