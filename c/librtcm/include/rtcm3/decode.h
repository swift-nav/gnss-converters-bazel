/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_RTCM3_DECODE_H
#define SWIFTNAV_RTCM3_DECODE_H

#include <stdint.h>
#include <swiftnav/bitstream.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <rtcm3/messages.h>

rtcm3_rc rtcm3_decode_999_bitstream(swiftnav_in_bitstream_t *buff,
                                    rtcm_msg_999 *msg_999);
rtcm3_rc rtcm3_decode_1001_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1001);
rtcm3_rc rtcm3_decode_1002_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1002);
rtcm3_rc rtcm3_decode_1003_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1003);
rtcm3_rc rtcm3_decode_1004_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1004);
rtcm3_rc rtcm3_decode_1005_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1005 *msg_1005);
rtcm3_rc rtcm3_decode_1006_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1006 *msg_1006);
rtcm3_rc rtcm3_decode_1007_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1007 *msg_1007);
rtcm3_rc rtcm3_decode_1008_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1008 *msg_1008);
rtcm3_rc rtcm3_decode_1010_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1010);
rtcm3_rc rtcm3_decode_1012_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1012);
rtcm3_rc rtcm3_decode_1013_bitstream(swiftnav_in_bitstream_t *buffer,
                                     rtcm_msg_1013 *msg_1013,
                                     size_t message_length);
rtcm3_rc rtcm3_decode_1029_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1029 *msg_1029);
rtcm3_rc rtcm3_decode_1033_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1033 *msg_1033);
rtcm3_rc rtcm3_decode_1230_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1230 *msg_1230);
rtcm3_rc rtcm3_decode_msm4_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg);
rtcm3_rc rtcm3_decode_msm5_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg);
rtcm3_rc rtcm3_decode_msm6_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg);
rtcm3_rc rtcm3_decode_msm7_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg);
rtcm3_rc rtcm3_decode_4062_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_swift_proprietary *msg);
rtcm3_rc rtcm3_decode_4075_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_ndf *msg);

double rtcm3_decode_lock_time(uint8_t lock);

/**
 * Decode RTCM3 message/payload in bitstream format.
 * @param buff RTCM message/payload in bitstream format
 * @param rtcm_msg (Output) RTCM msg results (msg + msg_type)
 * @return Decoding status
 */
rtcm3_rc rtcm3_decode_payload_bitstream(swiftnav_in_bitstream_t *buff,
                                        rtcm_msg_data_t *rtcm_msg);

/**
 * Decode RTCM3 frame in bitstream format.
 * @param buff RTCM frame in bitstream format
 * @param rtcm_frame (Output) RTCM msg containing decoding results
 * @return Decoding status
 * Note: This function only decodes the content of the frame. CRC verification
 * must be called separately.
 */
rtcm3_rc rtcm3_decode_frame_bitstream(swiftnav_in_bitstream_t *buff,
                                      rtcm_frame_t *rtcm_frame);

/* Backwards compatibility versions, these are all susceptible to buffer
 * overflows */
static inline rtcm3_rc rtcm3_decode_999(const uint8_t buff[],
                                        rtcm_msg_999 *msg_999) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_999_bitstream(&bitstream, msg_999);
}

static inline rtcm3_rc rtcm3_decode_1001(const uint8_t buff[],
                                         rtcm_obs_message *msg_1001) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1001_bitstream(&bitstream, msg_1001);
}

static inline rtcm3_rc rtcm3_decode_1002(const uint8_t buff[],
                                         rtcm_obs_message *msg_1002) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1002_bitstream(&bitstream, msg_1002);
}

static inline rtcm3_rc rtcm3_decode_1003(const uint8_t buff[],
                                         rtcm_obs_message *msg_1003) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1003_bitstream(&bitstream, msg_1003);
}

static inline rtcm3_rc rtcm3_decode_1004(const uint8_t buff[],
                                         rtcm_obs_message *msg_1004) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1004_bitstream(&bitstream, msg_1004);
}

static inline rtcm3_rc rtcm3_decode_1005(const uint8_t buff[],
                                         rtcm_msg_1005 *msg_1005) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1005_bitstream(&bitstream, msg_1005);
}

static inline rtcm3_rc rtcm3_decode_1006(const uint8_t buff[],
                                         rtcm_msg_1006 *msg_1006) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1006_bitstream(&bitstream, msg_1006);
}

static inline rtcm3_rc rtcm3_decode_1007(const uint8_t buff[],
                                         rtcm_msg_1007 *msg_1007) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1007_bitstream(&bitstream, msg_1007);
}

static inline rtcm3_rc rtcm3_decode_1008(const uint8_t buff[],
                                         rtcm_msg_1008 *msg_1008) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1008_bitstream(&bitstream, msg_1008);
}

static inline rtcm3_rc rtcm3_decode_1010(const uint8_t buff[],
                                         rtcm_obs_message *msg_1010) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1010_bitstream(&bitstream, msg_1010);
}

static inline rtcm3_rc rtcm3_decode_1012(const uint8_t buff[],
                                         rtcm_obs_message *msg_1012) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1012_bitstream(&bitstream, msg_1012);
}

static inline rtcm3_rc rtcm3_decode_1029(const uint8_t buff[],
                                         rtcm_msg_1029 *msg_1029) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1029_bitstream(&bitstream, msg_1029);
}

static inline rtcm3_rc rtcm3_decode_1033(const uint8_t buff[],
                                         rtcm_msg_1033 *msg_1033) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1033_bitstream(&bitstream, msg_1033);
}

static inline rtcm3_rc rtcm3_decode_1230(const uint8_t buff[],
                                         rtcm_msg_1230 *msg_1230) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_1230_bitstream(&bitstream, msg_1230);
}

static inline rtcm3_rc rtcm3_decode_msm4(const uint8_t buff[],
                                         rtcm_msm_message *msg) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_msm4_bitstream(&bitstream, msg);
}

static inline rtcm3_rc rtcm3_decode_msm5(const uint8_t buff[],
                                         rtcm_msm_message *msg) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_msm5_bitstream(&bitstream, msg);
}

static inline rtcm3_rc rtcm3_decode_msm6(const uint8_t buff[],
                                         rtcm_msm_message *msg) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_msm6_bitstream(&bitstream, msg);
}

static inline rtcm3_rc rtcm3_decode_msm7(const uint8_t buff[],
                                         rtcm_msm_message *msg) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_msm7_bitstream(&bitstream, msg);
}

static inline rtcm3_rc rtcm3_decode_4062(const uint8_t buff[],
                                         rtcm_msg_swift_proprietary *msg) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_4062_bitstream(&bitstream, msg);
}

static inline rtcm3_rc rtcm3_decode_4075(const uint8_t buff[],
                                         rtcm_msg_ndf *msg) {
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, buff, UINT32_MAX);
  return rtcm3_decode_4075_bitstream(&bitstream, msg);
}

/**
 * Decode RTCM3 message/payload in byte buffer format.
 * @param payload_buff RTCM payload/message
 * @param payload_len RTCM message length (not frame length)
 * @param rtcm_msg (Output) RTCM msg which contains decoding results
 * @return Decoding status
 * Note: Wrapper of rtcm3_decode_payload_bitstream().
 */
static inline rtcm3_rc rtcm3_decode_payload(const uint8_t payload_buff[],
                                            uint32_t payload_len,
                                            rtcm_msg_data_t *rtcm_msg) {
  assert(payload_buff);
  assert(rtcm_msg);

  if (payload_len < RTCM3_MIN_MSG_LEN) {
    return RC_INVALID_MESSAGE;
  }
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, payload_buff, payload_len * 8);
  return rtcm3_decode_payload_bitstream(&bitstream, rtcm_msg);
}

/**
 * Decode RTCM3 frame in byte buffer format.
 * @param frame_buff RTCM frame
 * @param frame_len RTCM frame length
 * @param rtcm_frame (Output) RTCM msg which contains decoding results
 * @return Decoding status
 * Note: Wrapper of rtcm3_decode_frame_bitstream()
 */
static inline rtcm3_rc rtcm3_decode_frame(const uint8_t frame_buff[],
                                          uint32_t frame_len,
                                          rtcm_frame_t *rtcm_frame) {
  assert(frame_buff);
  assert(rtcm_frame);

  if (frame_len < (RTCM3_MSG_OVERHEAD + RTCM3_MIN_MSG_LEN)) {
    return RC_INVALID_MESSAGE;
  }
  swiftnav_in_bitstream_t bitstream;
  swiftnav_in_bitstream_init(&bitstream, frame_buff, frame_len * 8);
  return rtcm3_decode_frame_bitstream(&bitstream, rtcm_frame);
}

/**
 * Extract RTCM message length from RTCM buffer
 * @param frame_buff RTCM buffer (frame)
 * @param frame_len RTCM buffer length (not RTCM frame length)
 * @param payload_len (Output) Exact length of RTCM message
 * @return Decoding status
 */
static inline rtcm3_rc rtcm3_decode_payload_len(const uint8_t frame_buff[],
                                                uint32_t frame_len,
                                                uint16_t *payload_len) {
  if (frame_len < 3) {
    return RC_INVALID_MESSAGE;
  }
  *payload_len = (uint16_t)(((frame_buff[1] & 0x3) << 8) | frame_buff[2]);
  return RC_OK;
}

/**
 * Extract RTCM message type from RTCM frame
 * @param frame_buff RTCM frame
 * @param frame_len RTCM frame length
 * @param msg_type (Output) RTCM message type
 * @return Decoding status
 */
static inline rtcm3_rc rtcm3_decode_msg_type(const uint8_t frame_buff[],
                                             uint32_t frame_len,
                                             uint16_t *msg_type) {
  if (frame_len < 5) {
    return RC_INVALID_MESSAGE;
  }
  *msg_type = (uint16_t)((frame_buff[3] << 4) | ((frame_buff[4] >> 4) & 0xf));
  return RC_OK;
}

/**
 * Get CRC of RTCM3 frame
 * @param frame_buff RTCM frame
 * @param frame_len RTCM frame length
 * @param crc (output)
 * @return Decoding status
 */
static inline rtcm3_rc rtcm3_decode_crc(const uint8_t frame_buff[],
                                        uint32_t frame_len,
                                        uint32_t *crc) {
  uint16_t payload_len = 0;
  rtcm3_rc ret = rtcm3_decode_payload_len(frame_buff, frame_len, &payload_len);
  if (RC_OK != ret) {
    return ret;
  }

  // Minimum condition to extract CRC
  if (frame_len < (RTCM3_MSG_OVERHEAD + payload_len)) {
    return RC_INVALID_MESSAGE;
  }

  uint32_t crc_offset = payload_len + 3;
  *crc = (uint32_t)((frame_buff[crc_offset] << 16) |
                    (frame_buff[crc_offset + 1] << 8) |
                    (frame_buff[crc_offset + 2]));

  return RC_OK;
}

rtcm_msg_type_t rtcm3_msg_num_to_msg_type(uint16_t msg_num);

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_RTCM3_DECODE_H */
