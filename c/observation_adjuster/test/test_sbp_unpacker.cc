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

#include <gtest/gtest.h>
#include <libsbp/v4/navigation/MSG_POS_ECEF.h>
#include <observation_adjuster/sbp_unpacker.h>
#include <swiftnav/memcpy_s.h>

#include "test_helpers.h"

namespace obs_adjuster_tests {

std::vector<sbp_msg_obs_t> build_obs_messages(const s16 wn,
                                              const u32 tow_ms,
                                              const std::size_t total_nr_obs) {
  std::vector<sbp_msg_obs_t> obs_msgs;

  // 8 (preamble, length, CRC, etc) + 11 (msg header) + 17*N (17 per
  // observation)

  const std::size_t max_obs_per_message =
      (SBP_MAX_PAYLOAD_LEN - 11) / 17;  // 14
  const std::size_t nr_messages = total_nr_obs / max_obs_per_message;

  for (std::size_t i = 0; i < nr_messages; i++) {
    sbp_msg_obs_t obs_msg{};
    obs_msg.header.t.tow = tow_ms;
    obs_msg.header.t.wn = wn;
    const std::size_t nr_obs_remaining = total_nr_obs - i * max_obs_per_message;
    std::size_t nr_obs_in_this_message = nr_obs_remaining;
    if (nr_obs_in_this_message > max_obs_per_message) {
      nr_obs_in_this_message = max_obs_per_message;
    }
    obs_msg.n_obs = nr_obs_in_this_message;
    obs_msg.header.n_obs = (nr_messages << 4);  // Size of the sequence
    obs_msg.header.n_obs |= (i);                // Sequence counter
    for (std::size_t i = 0; i < obs_msg.n_obs; i++) {
      obs_msg.obs[i].flags = 1;  // Code valid
    }
    obs_msgs.push_back(obs_msg);
  }

  return obs_msgs;
}

TEST(SbpUnpacker, HandlesEmptyBuffer) {
  using namespace obs_adjuster;
  // Given an SbpUnpacker instance...
  SbpUnpacker unpacker(StreamType::STATION_OBS);

  // When processing an empty buffer
  unpacker.process(0, 0, nullptr, 0);
  // Then: Expect no crash
}

TEST_F(SbpTestHelper, NavCallback) {
  using namespace obs_adjuster;

  // Given an SbpUnpacker instance...
  SbpUnpacker unpacker(StreamType::STATION_OBS);

  bool obs_callback_received = false;
  StreamType callback_stream_type = StreamType::VRS_CORR;
  TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> nav_array;
  SbpUnpacker::ObsCallbackFn obs_callback(
      [&](const StreamType stream_type,
          const TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> &nav) {
        obs_callback_received = true;
        callback_stream_type = stream_type;
        nav_array = nav;
      });

  // ... with a callback function set..
  unpacker.set_obs_callback(obs_callback);

  const s16 wn = 534;
  const u32 tow_ms = 56712342;
  const std::size_t total_nr_obs = 42;

  std::vector<sbp_msg_obs_t> obs_msgs =
      build_obs_messages(wn, tow_ms, total_nr_obs);

  // ... when processing a sequence of SBP obs messages
  reset_write();
  for (const sbp_msg_obs_t &obs_msg : obs_msgs) {
    ASSERT_EQ(SBP_OK, sbp::State::send_message(1234, obs_msg));
    //    unpacker.process(written_buffer_, bytes_in_buffer_);
    unpacker.process(
        get_sender_id(), get_msg_type(), get_payload(), get_payload_len());
    reset_write();
  }

  // Then expect the callback to be called with the observations combined
  // in one array
  ASSERT_TRUE(obs_callback_received);
  EXPECT_EQ(nav_array.obs_.size(), total_nr_obs);
  EXPECT_FLOAT_EQ(nav_array.timestamp_.tow,
                  static_cast<double>(tow_ms) / 1000.0);
  EXPECT_EQ(nav_array.timestamp_.wn, wn);
  EXPECT_EQ(callback_stream_type, StreamType::STATION_OBS);
}

}  // namespace obs_adjuster_tests
