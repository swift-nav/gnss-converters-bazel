/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SWIFTNAV_RTCM3_EPH_ENCODE_H
#define SWIFTNAV_RTCM3_EPH_ENCODE_H

#include <rtcm3/encode.h>
#include <rtcm3/messages.h>
#include <swiftnav/bitstream.h>

#define BEIDOU_GEOS_MAX_PRN 5

#ifdef __cplusplus
extern "C" {
#endif

rtcm3_rc rtcm3_encode_gps_eph_bitstream(swiftnav_out_bitstream_t *buff,
                                        const rtcm_msg_eph *msg_1019);
rtcm3_rc rtcm3_encode_glo_eph_bitstream(swiftnav_out_bitstream_t *buff,
                                        const rtcm_msg_eph *msg_1020);
rtcm3_rc rtcm3_encode_bds_eph_bitstream(swiftnav_out_bitstream_t *buff,
                                        const rtcm_msg_eph *msg_1042);
rtcm3_rc rtcm3_encode_gal_eph_inav_bitstream(swiftnav_out_bitstream_t *buff,
                                             const rtcm_msg_eph *msg_eph);
rtcm3_rc rtcm3_encode_gal_eph_fnav_bitstream(swiftnav_out_bitstream_t *buff,
                                             const rtcm_msg_eph *msg_eph);

static inline uint16_t rtcm3_encode_gps_eph(const rtcm_msg_eph *msg_1019,
                                            uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_gps_eph_bitstream(&bitstream, msg_1019);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_glo_eph(const rtcm_msg_eph *msg_1020,
                                            uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_glo_eph_bitstream(&bitstream, msg_1020);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_bds_eph(const rtcm_msg_eph *msg_1042,
                                            uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_bds_eph_bitstream(&bitstream, msg_1042);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_gal_eph_inav(const rtcm_msg_eph *msg_eph,
                                                 uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_gal_eph_inav_bitstream(&bitstream, msg_eph);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}
static inline uint16_t rtcm3_encode_gal_eph_fnav(const rtcm_msg_eph *msg_eph,
                                                 uint8_t buff[]) {
  assert(buff);
  swiftnav_out_bitstream_t bitstream;
  swiftnav_out_bitstream_init(&bitstream, buff, UINT32_MAX);
  rtcm3_rc ret = rtcm3_encode_gal_eph_fnav_bitstream(&bitstream, msg_eph);

  uint16_t encode_len = 0;
  RTCM3_ENCODE_LEN(ret, bitstream, encode_len);
  return encode_len;
}

#ifdef __cplusplus
}
#endif

#endif /* SWIFTNAV_RTCM3_EPH_ENCODE_H */
