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

/* These encoder functions are provided to allow for easier unit testing however
 * they are not robust to be used in production. Before using with real data, we
 * would need to handle ambiguity rollover and code carrier divergance at least
 */

#ifndef SWIFTNAV_RTCM3_ENCODE_H
#define SWIFTNAV_RTCM3_ENCODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <rtcm3/messages.h>
#include <swiftnav/bitstream.h>

#define RTCM3_ENCODE_LEN(rtcm3_rc_ret, bitstream, encode_len)                 \
  do {                                                                        \
    (encode_len) =                                                            \
        ((RC_OK == (rtcm3_rc_ret)) ? (uint16_t)(((bitstream).offset + 7) / 8) \
                                   : 0);                                      \
  } while (false)

uint8_t rtcm3_encode_lock_time(double time);

rtcm3_rc rtcm3_encode_999_bitstream(swiftnav_out_bitstream_t *buff,
                                    const rtcm_msg_999 *msg_999);
rtcm3_rc rtcm3_encode_1001_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1001);
rtcm3_rc rtcm3_encode_1002_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1002);
rtcm3_rc rtcm3_encode_1003_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1003);
rtcm3_rc rtcm3_encode_1004_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1004);
rtcm3_rc rtcm3_encode_1005_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1005 *msg_1005);
rtcm3_rc rtcm3_encode_1006_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1006 *msg_1006);
rtcm3_rc rtcm3_encode_1007_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1007 *msg_1007);
rtcm3_rc rtcm3_encode_1008_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1008 *msg_1008);
rtcm3_rc rtcm3_encode_1010_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1010);
rtcm3_rc rtcm3_encode_1012_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1012);
rtcm3_rc rtcm3_encode_1013_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1013 *msg_1013);
rtcm3_rc rtcm3_encode_1029_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1029 *msg_1029);
rtcm3_rc rtcm3_encode_1033_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1033 *msg_1033);
rtcm3_rc rtcm3_encode_1230_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1230 *msg_1230);
rtcm3_rc rtcm3_encode_msm4_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msm_message *msg_msm4);
rtcm3_rc rtcm3_encode_msm5_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msm_message *msg_msm5);
rtcm3_rc rtcm3_encode_4062_bitstream(
    swiftnav_out_bitstream_t *buff, const rtcm_msg_swift_proprietary *msg_4062);

rtcm3_rc rtcm3_encode_payload_bitstream(swiftnav_out_bitstream_t *buff,
                                        const void *rtcm_msg,
                                        uint16_t message_type);

rtcm3_rc rtcm3_encode_frame_bitstream(swiftnav_out_bitstream_t *buff,
                                      const void *rtcm_msg,
                                      uint16_t message_type);

static inline uint16_t rtcm3_encode_999(const rtcm_msg_999 *msg_999,
                                        uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_999_bitstream(&bitstream, msg_999);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1001(const rtcm_obs_message *msg_1001,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1001_bitstream(&bitstream, msg_1001);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1002(const rtcm_obs_message *msg_1002,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1002_bitstream(&bitstream, msg_1002);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1003(const rtcm_obs_message *msg_1003,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1003_bitstream(&bitstream, msg_1003);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1004(const rtcm_obs_message *msg_1004,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1004_bitstream(&bitstream, msg_1004);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1005(const rtcm_msg_1005 *msg_1005,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1005_bitstream(&bitstream, msg_1005);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1006(const rtcm_msg_1006 *msg_1006,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1006_bitstream(&bitstream, msg_1006);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1007(const rtcm_msg_1007 *msg_1007,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1007_bitstream(&bitstream, msg_1007);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1008(const rtcm_msg_1008 *msg_1008,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1008_bitstream(&bitstream, msg_1008);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1010(const rtcm_obs_message *msg_1010,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1010_bitstream(&bitstream, msg_1010);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1012(const rtcm_obs_message *msg_1012,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1012_bitstream(&bitstream, msg_1012);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1013(const rtcm_msg_1013 *msg_1013,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1013_bitstream(&bitstream, msg_1013);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1029(const rtcm_msg_1029 *msg_1029,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1029_bitstream(&bitstream, msg_1029);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1033(const rtcm_msg_1033 *msg_1033,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1033_bitstream(&bitstream, msg_1033);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_1230(const rtcm_msg_1230 *msg_1230,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_1230_bitstream(&bitstream, msg_1230);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_msm4(const rtcm_msm_message *msg_msm4,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_msm4_bitstream(&bitstream, msg_msm4);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_msm5(const rtcm_msm_message *msg_msm5,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_msm5_bitstream(&bitstream, msg_msm5);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_4062(const rtcm_msg_swift_proprietary *msg,
                                         uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_4062_bitstream(&bitstream, msg);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}

static inline rtcm3_rc rtcm3_encode_payload(const void *rtcm_msg,
                                            uint8_t *buff,
                                            uint32_t buf_len,
                                            uint16_t message_type,
                                            uint16_t *encode_len) {
  assert(buff);
  if (buf_len == 0) {
    return RC_INVALID_MESSAGE;
  }

  memset(buff, 0, buf_len);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, buf_len * 8);

  rtcm3_rc ret =
      rtcm3_encode_payload_bitstream(&bitstream, rtcm_msg, message_type);
  RTCM3_ENCODE_LEN(ret, bitstream, *encode_len);
  return ret;
}

static inline rtcm3_rc rtcm3_encode_frame(const void *rtcm_msg,
                                          uint8_t *buff,
                                          uint32_t buf_len,
                                          uint16_t message_type,
                                          uint16_t *encode_len) {
  assert(buff);

  if (buf_len <= RTCM3_MSG_OVERHEAD) {
    return RC_INVALID_MESSAGE;
  }

  memset(buff, 0, buf_len);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, buf_len * 8);
  rtcm3_rc ret =
      rtcm3_encode_frame_bitstream(&bitstream, rtcm_msg, message_type);

  RTCM3_ENCODE_LEN(ret, bitstream, *encode_len);
  return ret;
}

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_RTCM3_ENCODE_H */
