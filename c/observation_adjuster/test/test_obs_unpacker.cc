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

#include <observation_adjuster/obs_unpacker.h>

namespace obs_adjuster_tests {

class TestObsUnpacker : public ::testing::Test {
 public:
  TestObsUnpacker() : callback_called(false) {
    unpacker_.set_obs_callback(
        std::bind(&TestObsUnpacker::obs_callback, this, std::placeholders::_1));
  };
  virtual ~TestObsUnpacker() override { unpacker_.unset_callback(); };

  void obs_callback(const obs_adjuster::TimestampedSbpObsArray<
                    obs_adjuster::MAX_OBS_PER_EPOCH> &obs_array) {
    callback_called = true;
    received_obs_ = obs_array;
  }

  void clear_obs() {
    callback_called = false;
    received_obs_ =
        obs_adjuster::TimestampedSbpObsArray<obs_adjuster::MAX_OBS_PER_EPOCH>{};
  }

  obs_adjuster::ObsUnpacker unpacker_;
  bool callback_called;
  obs_adjuster::TimestampedSbpObsArray<obs_adjuster::MAX_OBS_PER_EPOCH>
      received_obs_;
};

TEST_F(TestObsUnpacker, SingleMessage) {
  sbp_msg_obs_t obs_msg{};
  // First message of one in total
  obs_msg.header.n_obs = (1 << 4);  // One message in total
  obs_msg.header.n_obs |= (1 - 1);  // Zero indexed counter
  obs_msg.header.t.wn = 567;
  obs_msg.header.t.tow = 928374;

  unpacker_.unpack_obs(0, obs_msg);

  ASSERT_TRUE(callback_called);
  EXPECT_EQ(received_obs_.timestamp_.wn, 567);
  EXPECT_FLOAT_EQ(received_obs_.timestamp_.tow, 928.374);
}

TEST_F(TestObsUnpacker, TwoMessages) {
  sbp_msg_obs_t obs_msg{};
  // First message of two in total
  obs_msg.header.n_obs = (2 << 4);  // Two messages in total
  obs_msg.header.n_obs |= (1 - 1);  // Zero indexed counter
  obs_msg.header.t.wn = 567;
  obs_msg.header.t.tow = 928374;
  obs_msg.header.t.ns_residual = 555;

  obs_msg.n_obs = 5;
  for (std::size_t i = 0; i < 5; i++) {
    obs_msg.obs[i].lock = i + 1;
    obs_msg.obs[i].flags |= 1;  // Code valid
    obs_msg.obs[i].sid.code = 0;
    obs_msg.obs[i].sid.sat = i + 1;
  }

  unpacker_.unpack_obs(0, obs_msg);
  ASSERT_FALSE(callback_called);

  obs_msg.header.n_obs = (2 << 4);  // Two messages in total
  obs_msg.header.n_obs |= (2 - 1);  // Zero indexed counter

  obs_msg.n_obs = 5;
  for (std::size_t i = 0; i < 5; i++) {
    obs_msg.obs[i].lock = i + 1 + 5;
    obs_msg.obs[i].flags |= 1;  // Code valid
    obs_msg.obs[i].sid.code = 0;
    obs_msg.obs[i].sid.sat = i + 1 + 5;
  }

  obs_msg.n_obs = 6;
  // One sat with invalid flags which we don't expect in the output
  obs_msg.obs[5].lock = 10;
  obs_msg.obs[5].flags = 0;  // Code invalid
  obs_msg.obs[5].sid.code = 0;
  obs_msg.obs[5].sid.sat = 5 + 1 + 5;

  unpacker_.unpack_obs(0, obs_msg);
  ASSERT_TRUE(callback_called);
  EXPECT_EQ(received_obs_.timestamp_.wn, 567);
  EXPECT_FLOAT_EQ(received_obs_.timestamp_.tow, 928.374);

  ASSERT_EQ(received_obs_.obs_.size(), 10);
}

TEST_F(TestObsUnpacker, BackwardsTime) {
  sbp_msg_obs_t obs_msg{};
  // First message of two in total
  obs_msg.header.n_obs = (2 << 4);  // Two messages in total
  obs_msg.header.n_obs |= (1 - 1);  // Zero indexed counter
  obs_msg.header.t.wn = 567;
  obs_msg.header.t.tow = 928374;
  obs_msg.n_obs = 1;
  obs_msg.obs[0].flags = 1;  // Code valid

  unpacker_.unpack_obs(0, obs_msg);
  ASSERT_FALSE(callback_called);

  // Now make a message with an earlier timestamp
  obs_msg.header.n_obs |= (2 - 1);  // Zero indexed counter
  obs_msg.header.t.wn = 567;
  obs_msg.header.t.tow = 928370;
  EXPECT_FALSE(unpacker_.unpack_obs(0, obs_msg));
  ASSERT_FALSE(callback_called);
}

}  // namespace obs_adjuster_tests
