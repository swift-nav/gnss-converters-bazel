/*
 * Copyright (C) 2019 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <librtcm/internal/encode_helpers.h>
#include <rtcm3/bits.h>
#include <rtcm3/eph_encode.h>
#include <string.h>

rtcm3_rc rtcm3_encode_gps_eph_bitstream(swiftnav_out_bitstream_t *buff,
                                        const rtcm_msg_eph *msg_1019) {
  assert(buff);
  const uint32_t MSG_NUM = 1019;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  BITSTREAM_ENCODE_U8(buff, msg_1019->sat_id, 6);
  BITSTREAM_ENCODE_U16(buff, msg_1019->wn, 10);
  BITSTREAM_ENCODE_U16(buff, msg_1019->ura, 4);
  BITSTREAM_ENCODE_U8(buff, msg_1019->data.kepler.codeL2, 2);
  BITSTREAM_ENCODE_S16(buff, msg_1019->data.kepler.inc_dot, 14);
  BITSTREAM_ENCODE_U16(buff, msg_1019->data.kepler.iode, 8);
  BITSTREAM_ENCODE_U32(buff, msg_1019->toe, 16);
  BITSTREAM_ENCODE_S16(buff, msg_1019->data.kepler.af2, 8);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.af1, 16);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.af0, 22);
  BITSTREAM_ENCODE_U16(buff, msg_1019->data.kepler.iodc, 10);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.crs, 16);
  BITSTREAM_ENCODE_S16(buff, msg_1019->data.kepler.dn, 16);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.m0, 32);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.cuc, 16);
  BITSTREAM_ENCODE_U32(buff, msg_1019->data.kepler.ecc, 32);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.cus, 16);
  BITSTREAM_ENCODE_U32(buff, msg_1019->data.kepler.sqrta, 32);
  BITSTREAM_ENCODE_U32(buff, msg_1019->data.kepler.toc, 16);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.cic, 16);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.omega0, 32);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.cis, 16);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.inc, 32);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.crc, 16);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.w, 32);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.omegadot, 24);
  BITSTREAM_ENCODE_S32(buff, msg_1019->data.kepler.tgd.gps_s, 8);
  BITSTREAM_ENCODE_U8(buff, msg_1019->health_bits, 6);
  BITSTREAM_ENCODE_U8(buff, msg_1019->data.kepler.L2_data_bit, 1);
  BITSTREAM_ENCODE_U32(buff, msg_1019->fit_interval, 1);

  return RC_OK;
}

rtcm3_rc rtcm3_encode_glo_eph_bitstream(swiftnav_out_bitstream_t *buff,
                                        const rtcm_msg_eph *msg_1020) {
  assert(msg_1020);
  rtcm3_rc ret = RC_OK;

  const uint32_t MSG_NUM = 1020;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  BITSTREAM_ENCODE_U8(buff, msg_1020->sat_id, 6);
  BITSTREAM_ENCODE_U8(buff, msg_1020->data.glo.fcn, 5);
  // Almanac health
  BITSTREAM_ENCODE_U32(buff, (uint32_t)1, 1);
  // Almanac health availability
  BITSTREAM_ENCODE_U32(buff, 0, 1);
  BITSTREAM_ENCODE_U32(buff, msg_1020->fit_interval, 2);
  // T_k
  BITSTREAM_ENCODE_U32(buff, 0, 12);
  BITSTREAM_ENCODE_U8(buff, msg_1020->health_bits, 1);
  // P2
  BITSTREAM_ENCODE_U32(buff, 0, 1);
  BITSTREAM_ENCODE_U8(buff, msg_1020->data.glo.t_b, 7);

  for (size_t i = 0; i < 3; i++) {
    ret =
        rtcm_set_sign_magnitude_bitstream(buff, msg_1020->data.glo.vel[i], 24);
    if (ret != RC_OK) {
      return ret;
    }
    ret =
        rtcm_set_sign_magnitude_bitstream(buff, msg_1020->data.glo.pos[i], 27);
    if (ret != RC_OK) {
      return ret;
    }
    ret = rtcm_set_sign_magnitude_bitstream(buff, msg_1020->data.glo.acc[i], 5);
    if (ret != RC_OK) {
      return ret;
    }
  }
  // P3
  BITSTREAM_ENCODE_U32(buff, 0, 1);
  ret = rtcm_set_sign_magnitude_bitstream(buff, msg_1020->data.glo.gamma, 11);
  if (ret != RC_OK) {
    return ret;
  }
  // P
  BITSTREAM_ENCODE_U32(buff, 0, 2);
  BITSTREAM_ENCODE_U8(buff, msg_1020->health_bits, 1);
  ret = rtcm_set_sign_magnitude_bitstream(buff, msg_1020->data.glo.tau, 22);
  if (ret != RC_OK) {
    return ret;
  }
  ret = rtcm_set_sign_magnitude_bitstream(buff, msg_1020->data.glo.d_tau, 5);
  if (ret != RC_OK) {
    return ret;
  }
  // EN
  BITSTREAM_ENCODE_U32(buff, 0, 5);
  // P4
  BITSTREAM_ENCODE_U32(buff, 0, 1);
  BITSTREAM_ENCODE_U16(buff, msg_1020->ura, 4);
  // NT
  BITSTREAM_ENCODE_U32(buff, 0, 11);
  // M
  BITSTREAM_ENCODE_U32(buff, 0, 2);
  // Additional data
  BITSTREAM_ENCODE_U32(buff, 0, 32);
  BITSTREAM_ENCODE_U32(buff, 0, 32);
  BITSTREAM_ENCODE_U32(buff, 0, 15);

  return RC_OK;
}

rtcm3_rc rtcm3_encode_bds_eph_bitstream(swiftnav_out_bitstream_t *buff,
                                        const rtcm_msg_eph *msg_1042) {
  assert(buff);

  const uint32_t MSG_NUM = 1042;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  BITSTREAM_ENCODE_U8(buff, msg_1042->sat_id, 6);
  BITSTREAM_ENCODE_U16(buff, msg_1042->wn, 13);
  BITSTREAM_ENCODE_U16(buff, msg_1042->ura, 4);
  BITSTREAM_ENCODE_S16(buff, msg_1042->data.kepler.inc_dot, 14);
  BITSTREAM_ENCODE_U16(buff, msg_1042->data.kepler.iode, 5);
  BITSTREAM_ENCODE_U32(buff, msg_1042->data.kepler.toc, 17);
  BITSTREAM_ENCODE_S16(buff, msg_1042->data.kepler.af2, 11);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.af1, 22);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.af0, 24);
  BITSTREAM_ENCODE_U16(buff, msg_1042->data.kepler.iodc, 5);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.crs, 18);
  BITSTREAM_ENCODE_S16(buff, msg_1042->data.kepler.dn, 16);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.m0, 32);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.cuc, 18);
  BITSTREAM_ENCODE_U32(buff, msg_1042->data.kepler.ecc, 32);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.cus, 18);
  BITSTREAM_ENCODE_U32(buff, msg_1042->data.kepler.sqrta, 32);
  BITSTREAM_ENCODE_U32(buff, msg_1042->toe, 17);

  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.cic, 18);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.omega0, 32);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.cis, 18);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.inc, 32);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.crc, 18);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.w, 32);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.omegadot, 24);

  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.tgd.bds_s[0], 10);
  BITSTREAM_ENCODE_S32(buff, msg_1042->data.kepler.tgd.bds_s[1], 10);
  BITSTREAM_ENCODE_U8(buff, msg_1042->health_bits, 1);

  return RC_OK;
}

/** Decode an RTCMv3 GAL (common part) Ephemeris Message
 *
 * \param buff The input data buffer
 * \param msg_eph RTCM message struct
 * \return bit position in the RTCM frame
 */
static rtcm3_rc rtcm3_encode_gal_eph_common_bitstream(
    swiftnav_out_bitstream_t *buff, const rtcm_msg_eph *msg_eph) {
  assert(buff);

  BITSTREAM_ENCODE_U8(buff, msg_eph->sat_id, 6);
  BITSTREAM_ENCODE_U16(buff, msg_eph->wn, 12);
  BITSTREAM_ENCODE_U16(buff, msg_eph->data.kepler.iode, 10);
  BITSTREAM_ENCODE_U16(buff, msg_eph->ura, 8);
  BITSTREAM_ENCODE_S16(buff, msg_eph->data.kepler.inc_dot, 14);
  BITSTREAM_ENCODE_U32(buff, msg_eph->data.kepler.toc, 14);
  BITSTREAM_ENCODE_S16(buff, msg_eph->data.kepler.af2, 6);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.af1, 21);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.af0, 31);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.crs, 16);
  BITSTREAM_ENCODE_S16(buff, msg_eph->data.kepler.dn, 16);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.m0, 32);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.cuc, 16);
  BITSTREAM_ENCODE_U32(buff, msg_eph->data.kepler.ecc, 32);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.cus, 16);
  BITSTREAM_ENCODE_U32(buff, msg_eph->data.kepler.sqrta, 32);
  BITSTREAM_ENCODE_U32(buff, msg_eph->toe, 14);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.cic, 16);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.omega0, 32);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.cis, 16);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.inc, 32);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.crc, 16);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.w, 32);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.omegadot, 24);

  return RC_OK;
}

/** Decode an RTCMv3 GAL (I/NAV message) Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
rtcm3_rc rtcm3_encode_gal_eph_inav_bitstream(swiftnav_out_bitstream_t *buff,
                                             const rtcm_msg_eph *msg_eph) {
  assert(buff);

  const uint32_t MSG_NUM = 1046;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  /* parse common I/NAV and F/NAV part */
  rtcm3_rc ret = rtcm3_encode_gal_eph_common_bitstream(buff, msg_eph);
  if (ret != RC_OK) {
    return ret;
  }
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.tgd.gal_s[0], 10);
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.tgd.gal_s[1], 10);
  BITSTREAM_ENCODE_S8(buff, msg_eph->health_bits, 6);
  /* reserved */
  BITSTREAM_ENCODE_U32(buff, 0, 2);
  return RC_OK;
}

/** Decode an RTCMv3 GAL (F/NAV message) Ephemeris Message
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
rtcm3_rc rtcm3_encode_gal_eph_fnav_bitstream(swiftnav_out_bitstream_t *buff,
                                             const rtcm_msg_eph *msg_eph) {
  assert(buff);

  const uint32_t MSG_NUM = 1045;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  rtcm3_rc ret = rtcm3_encode_gal_eph_common_bitstream(buff, msg_eph);
  if (ret != RC_OK) {
    return ret;
  }
  BITSTREAM_ENCODE_S32(buff, msg_eph->data.kepler.tgd.gal_s[0], 10);
  BITSTREAM_ENCODE_S8(buff, msg_eph->health_bits, 3);
  /* reserved */
  BITSTREAM_ENCODE_U32(buff, 0, 7);

  return RC_OK;
}
