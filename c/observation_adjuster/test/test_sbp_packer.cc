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

#include <observation_adjuster/sbp_packer.h>

namespace obs_adjuster {

TEST(SbpPacker, NoMessage) {
  SbpPacker packer;
  std::unique_ptr<std::vector<u8>> buffer_ptr = packer.pop_packed_bytes();
  ASSERT_TRUE(buffer_ptr);
  ASSERT_EQ(buffer_ptr->size(), 0);
}

TEST(SbpPacker, OneMessage) {
  SbpPacker packer;

  SbpObsArray<obs_adjuster::MAX_OBS_PER_EPOCH> obs_array;
  const std::size_t max_obs_per_message =
      (SBP_MAX_PAYLOAD_LEN - 11) / 17;  // 14
  for (std::size_t i = 0; i < max_obs_per_message; ++i) {
    obs_array.add(sbp_packed_obs_content_t{});
  }

  const gps_time_t timestamp{1234.567891234, 2214};
  packer.pack(timestamp, obs_array);

  std::unique_ptr<std::vector<u8>> buffer_ptr = packer.pop_packed_bytes();
  ASSERT_TRUE(buffer_ptr);

  // Preamble + msg type + sender + length + payload + crc
  const std::size_t expected_size = 6 + 17 * max_obs_per_message + 11 + 2;
  EXPECT_EQ(buffer_ptr->size(), expected_size);
}

TEST(SbpPacker, MultipleMessages) {
  SbpPacker packer;

  SbpObsArray<obs_adjuster::MAX_OBS_PER_EPOCH> obs_array;
  const std::size_t max_obs_per_message =
      (SBP_MAX_PAYLOAD_LEN - 11) / 17;     // 14
  const std::size_t nr_observations = 17;  // More than the max of 14 per msg
  for (std::size_t i = 0; i < nr_observations; ++i) {
    obs_array.add(sbp_packed_obs_content_t{});
  }

  const gps_time_t timestamp{1234.567891234, 2214};
  packer.pack(timestamp, obs_array);

  std::unique_ptr<std::vector<u8>> buffer_ptr = packer.pop_packed_bytes();
  ASSERT_TRUE(buffer_ptr);

  std::size_t nr_messages = (nr_observations - 1) / max_obs_per_message + 1;
  // Preamble + msg type + sender + length + payload + crc
  const std::size_t expected_size =
      (6 + 11 + 2) * nr_messages + 17 * nr_observations;
  EXPECT_EQ(buffer_ptr->size(), expected_size);
}

TEST(SbpPacker, PosMessage) {
  SbpPacker packer;

  // Given an ecef position to the pack function
  const std::array<double, 3> pos_ecef{-456.789, 432.456, 9090112.33};
  packer.pack(pos_ecef);

  // When retrieving the packed bytes
  std::unique_ptr<std::vector<u8>> buffer_ptr = packer.pop_packed_bytes();
  ASSERT_TRUE(buffer_ptr);

  // Then the size of the returned byte vector is that of a SBP_MSG_POS_ECEF
  const std::size_t expected_size = 6 + 24 + 2;
  EXPECT_EQ(buffer_ptr->size(), expected_size);

  // and the x coordinate in the message is as in the input position
  double x = 0.0;
  memcpy(&x, buffer_ptr->data() + 6, sizeof(x));
  EXPECT_FLOAT_EQ(x, pos_ecef.at(0));

  // and the y coordinate in the message is as in the input position
  double y = 0.0;
  memcpy(&y, buffer_ptr->data() + 6 + 8, sizeof(y));
  EXPECT_FLOAT_EQ(y, pos_ecef.at(1));

  // and the z coordinate in the message is as in the input position
  double z = 0.0;
  memcpy(&z, buffer_ptr->data() + 6 + 16, sizeof(z));
  EXPECT_FLOAT_EQ(z, pos_ecef.at(2));
}

}  // namespace obs_adjuster
