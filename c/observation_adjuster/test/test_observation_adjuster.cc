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
#include <observation_adjuster/observation_adjuster.h>

#include "test_helpers.h"

namespace obs_adjuster_tests {

TEST_F(SbpTestHelper, PassThroughNonObsSBP) {
  const double vrs_loc[3] = {0.0, 0.0, 0.0};
  ObservationAdjuster obs_adjuster(vrs_loc);

  // Input an ephemeris message in the VRS correction stream and expect
  // to get the bytes output in the adjusted VRS stream

  reset_write();

  sbp_msg_ephemeris_gps_t gps_eph{};
  ASSERT_EQ(SBP_OK, sbp::State::send_message(1234, gps_eph));

  obs_adjuster.read_receiver_corrections(
      get_sender_id(), get_msg_type(), get_payload(), get_payload_len());

  const std::size_t buffer_len = SBP_MAX_FRAME_LEN + 512;
  u8 buffer[buffer_len];
  memset(buffer, 0, buffer_len);

  const std::size_t nr_available =
      obs_adjuster.write_VRS_corrections_SBP(buffer, buffer_len);
  ASSERT_EQ(nr_available, get_payload_len() + SBP_HEADER_LEN + SBP_CRC_LEN);
}

TEST_F(SbpTestHelper, PassThroughNonObsRTCM) {
  const double vrs_loc[3] = {0.0, 0.0, 0.0};
  ObservationAdjuster obs_adjuster(vrs_loc);

  // Input an ephemeris message in the VRS correction stream and expect
  // to get the bytes output in the adjusted VRS stream

  reset_write();
  sbp_msg_ephemeris_gps_t gps_eph{};
  ASSERT_EQ(SBP_OK, sbp::State::send_message(1234, gps_eph));

  obs_adjuster.read_receiver_corrections(
      get_sender_id(), get_msg_type(), get_payload(), get_payload_len());

  const std::size_t buffer_len = SBP_MAX_FRAME_LEN + 512;
  u8 buffer[buffer_len];
  memset(buffer, 0, buffer_len);

  const std::size_t nr_available =
      obs_adjuster.write_VRS_corrections_RTCM(buffer, buffer_len);

  // Size of an GPS ephemeris in RTCM3 is 488 bits + header + CRC
  // --> ceil(488 / 8) + 3 + 3 = 61 + 3 + 3 = 67 bytes
  ASSERT_EQ(nr_available, 67);

  const uint8_t preamble = buffer[0];
  EXPECT_EQ(preamble, 0xD3);  // RTCM3 preamble is 0xD3
}

}  // namespace obs_adjuster_tests
