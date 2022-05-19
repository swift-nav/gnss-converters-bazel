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

#ifndef GNSS_CONVERTERS_OBSERVATION_ADJUSTER_TESTS_TEST_HELPERS_H
#define GNSS_CONVERTERS_OBSERVATION_ADJUSTER_TESTS_TEST_HELPERS_H

#include <gtest/gtest.h>
#include <libsbp/v4/navigation/MSG_POS_ECEF.h>
#include <observation_adjuster/sbp_unpacker.h>
#include <swiftnav/memcpy_s.h>

namespace obs_adjuster_tests {

class SbpTestHelper : public ::testing::Test,
                      public sbp::State,
                      public sbp::IWriter {
 public:
  SbpTestHelper() { sbp::State::set_writer(this); }
  virtual ~SbpTestHelper() override = default;

  void reset_write() { msg_bytes_.clear(); }

  virtual s32 write(const u8 *buffer, u32 buffer_length) override {
    for (std::size_t i = 0; i < buffer_length; ++i) {
      msg_bytes_.push_back(buffer[i]);
    }

    return buffer_length;
  }

  uint16_t get_msg_type() {
    uint16_t msg_type = 0;
    if (msg_bytes_.size() >= SBP_FRAME_OFFSET_MSGTYPE + sizeof(msg_type)) {
      memcpy(&msg_type,
             msg_bytes_.data() + SBP_FRAME_OFFSET_MSGTYPE,
             sizeof(msg_type));
    }
    return msg_type;
  }

  uint16_t get_sender_id() {
    uint16_t sender_id = 0;
    if (msg_bytes_.size() >= SBP_FRAME_OFFSET_SENDERID + sizeof(sender_id)) {
      memcpy(&sender_id,
             msg_bytes_.data() + SBP_FRAME_OFFSET_SENDERID,
             sizeof(sender_id));
    }
    return sender_id;
  }

  uint16_t get_payload_len() {
    uint8_t payload_len = 0;
    if (msg_bytes_.size() >= SBP_FRAME_OFFSET_MSGLEN + sizeof(payload_len)) {
      memcpy(&payload_len,
             msg_bytes_.data() + SBP_FRAME_OFFSET_MSGLEN,
             sizeof(payload_len));
    }
    return payload_len;
  }

  uint8_t *get_payload() {
    uint8_t *payload_ptr = nullptr;
    if (msg_bytes_.size() >= SBP_FRAME_OFFSET_MSG + sizeof(payload_ptr)) {
      payload_ptr = msg_bytes_.data() + SBP_FRAME_OFFSET_MSG;
    }
    return payload_ptr;
  }

  std::vector<u8> msg_bytes_;
};

}  // namespace obs_adjuster_tests

#endif  // GNSS_CONVERTERS_OBSERVATION_ADJUSTER_TESTS_TEST_HELPERS_H
