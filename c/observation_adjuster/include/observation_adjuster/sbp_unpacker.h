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

#ifndef GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_SBP_UNPACKER_H
#define GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_SBP_UNPACKER_H

#include <libsbp/cpp/message_handler.h>
#include <libsbp/cpp/state.h>
#include <observation_adjuster/internal/obs_unpacker.h>
#include <observation_adjuster/internal/sbp_obs_array.h>

#include <functional>

namespace obs_adjuster {

using SbpMessageHandler = sbp::MessageHandler<sbp_msg_obs_t>;

class SbpUnpacker final : private sbp::State, private SbpMessageHandler {
 public:
  explicit SbpUnpacker(const StreamType stream_type);
  ~SbpUnpacker() override = default;

  void process(const uint16_t sender,
               const uint16_t msg_type,
               uint8_t *payload,
               const size_t payload_size);

  using ObsCallbackFn = std::function<void(
      const StreamType, const TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> &)>;
  void set_obs_callback(const ObsCallbackFn &callback_fn);
  void unset_obs_callback();

 private:
  void handle_sbp_msg(uint16_t sender_id,
                      const sbp_msg_obs_t &msg) noexcept override;
  void handle_unpacked_obs(
      const TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> &obs_array);

  StreamType stream_type_;
  ObsCallbackFn obs_callback_fn_;

  // Message unpacker helpers
  ObsUnpacker obs_unpacker;
};

}  // namespace obs_adjuster

#endif  // GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_SBP_UNPACKER_H
