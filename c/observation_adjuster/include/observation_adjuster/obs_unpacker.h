/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OBSERVATION_ADJUSTER_OBS_UNPACKER_H
#define OBSERVATION_ADJUSTER_OBS_UNPACKER_H

#include <libsbp/v4/observation/ObservationHeader.h>
#include <libsbp/v4/observation/PackedObsContent.h>
#include <observation_adjuster/observation_adjuster_constants.h>
#include <observation_adjuster/sbp_obs_array.h>

#include <cstdint>
#include <functional>

namespace obs_adjuster {

class ObsUnpacker final {
 public:
  ObsUnpacker();
  ~ObsUnpacker() = default;

  bool unpack_obs(const uint16_t sender, const sbp_msg_obs_t &msg);

  using ObsCallbackFn =
      std::function<void(const TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> &)>;
  void set_obs_callback(const ObsCallbackFn &callback);
  void unset_callback();

 private:
  void send_out_obs();

  bool sender_id_set_;
  uint16_t sender_id_;
  sbp_observation_header_t sbp_header_;
  TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> sbp_obs_array_;
  ObsCallbackFn obs_callback_;
};

}  // namespace obs_adjuster

#endif  // OBSERVATION_ADJUSTER_OBS_UNPACKER_H
