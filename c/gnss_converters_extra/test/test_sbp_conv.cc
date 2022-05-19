/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <gnss-converters-extra/sbp_conv.h>
#include <gtest/gtest.h>
#include <libsbp/sbp.h>
#include <rtcm3/decode.h>
#include <rtcm3/messages.h>

#include "gnss-converters/sbp_rtcm3.h"

bool msg1005_equals(const rtcm_msg_1005 *lhs, const rtcm_msg_1005 *rhs) {
  if (lhs->stn_id != rhs->stn_id) {
    return false;
  }
  if (lhs->ITRF != rhs->ITRF) {
    return false;
  }
  if (lhs->GPS_ind != rhs->GPS_ind) {
    return false;
  }
  if (lhs->GLO_ind != rhs->GLO_ind) {
    return false;
  }
  if (lhs->GAL_ind != rhs->GAL_ind) {
    return false;
  }
  if (lhs->ref_stn_ind != rhs->ref_stn_ind) {
    return false;
  }
  if (lhs->osc_ind != rhs->osc_ind) {
    return false;
  }
  if (lhs->quart_cycle_ind != rhs->quart_cycle_ind) {
    return false;
  }
  if (fabs(lhs->arp_x - rhs->arp_x) > 0.00005) {
    return false;
  }
  if (fabs(lhs->arp_y - rhs->arp_y) > 0.00005) {
    return false;
  }
  if (fabs(lhs->arp_z - rhs->arp_z) > 0.00005) {
    return false;
  }

  return true;
}

bool msg1006_equals(const rtcm_msg_1006 *lhs, const rtcm_msg_1006 *rhs) {
  if (fabs(lhs->ant_height - rhs->ant_height) > 0.00005) {
    return false;
  }

  return msg1005_equals(&lhs->msg_1005, &rhs->msg_1005);
}

TEST(SbpConv, Basic) {
  const uint8_t rtcm_1006_ev[] = {0xd3, 0x00, 0x15, 0x3e, 0xe0, 0x7b, 0x02,
                                  0xcb, 0x4a, 0x1c, 0x3b, 0x40, 0xbf, 0x63,
                                  0xf9, 0x1e, 0xf4, 0x49, 0x99, 0x80, 0x0a,
                                  0xf6, 0x00, 0x00, 0x51, 0x42, 0xbc};
  sbp_msg_t sbp_msg;
  sbp_msg.base_pos_ecef.x = 4848800.4415999996;
  sbp_msg.base_pos_ecef.y = -261769.65239999999;
  sbp_msg.base_pos_ecef.z = 4123001.1126000001;

  uint8_t sbp_buf[255] = {0};
  uint8_t sbp_buf_len = 0;
  sbp_message_encode(
      sbp_buf, sizeof(sbp_buf), &sbp_buf_len, SbpMsgBasePosEcef, &sbp_msg);

  sbp_conv_t conv = sbp_conv_new(MSM5, "", "");
  // Other elements are set by default
  sbp_conv_set_stn_description_parameters(conv, 0, 1, 1);

  uint8_t rtcm_buf[1029 * 3] = {0};
  size_t rtcm_buffer_size = sbp_conv(conv,
                                     123,
                                     SbpMsgBasePosEcef,
                                     sbp_buf,
                                     sbp_buf_len,
                                     rtcm_buf,
                                     ARRAY_SIZE(rtcm_buf));

  ASSERT_GE(rtcm_buffer_size, ARRAY_SIZE(rtcm_1006_ev));
  EXPECT_EQ(memcmp(rtcm_buf, rtcm_1006_ev, ARRAY_SIZE(rtcm_1006_ev)), 0);
  // because msg 1006 is the first out of 3 messages.

  sbp_conv_delete(conv);
}

struct sbp_conv_s {
  struct rtcm3_out_state state;
  fifo_t fifo;
  uint8_t buf[2048];
};

/** This test just checks the expected behavior at unit test level */
TEST(SbpConv, TestSetLegacyOn) {
  sbp_conv_t conv = sbp_conv_new(MSM5, "", "");
  EXPECT_FALSE(conv->state.send_legacy_obs);
  EXPECT_TRUE(conv->state.send_msm_obs);

  sbp_conv_set_legacy_on(conv);

  EXPECT_TRUE(conv->state.send_legacy_obs);
  EXPECT_FALSE(conv->state.send_msm_obs);
}

TEST(SbpConv, TestAntennaRcvDescriptors) {
  msm_enum _ = MSM_UNKNOWN;
  sbp_conv_t conv = sbp_conv_new(_, "abc", "def");
  EXPECT_STREQ(conv->state.ant_descriptor, "abc");
  EXPECT_STREQ(conv->state.rcv_descriptor, "def");
  ASSERT_TRUE(conv->state.ant_known);
}

TEST(SbpConv, TestAntennaRcvDescriptorsNull) {
  msm_enum _ = MSM_UNKNOWN;
  ASSERT_DEATH(sbp_conv_new(_, NULL, "def"), "");
  ASSERT_DEATH(sbp_conv_new(_, "abc", NULL), "");
  ASSERT_DEATH(sbp_conv_new(_, NULL, NULL), "");
}
