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

#include <assert.h>
#include <encode_helpers.h>
#include <math.h>
#include <rtcm3/encode.h>
#include <rtcm3/eph_encode.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <swiftnav/edc.h>
#include <swiftnav/logging.h>

#include "rtcm3/bits.h"
#include "rtcm3/constants.h"
#include "rtcm3/msm_utils.h"

/** Convert a lock time in seconds into 7-bit RTCMv3 Lock Time Indicator value.
 * See RTCM 10403.1, Table 3.4-2.
 *
 *
 * \param time Lock time in seconds.
 * \return Lock Time Indicator value.
 */
static uint8_t to_lock_ind(double time) {
  if (time < 24) {
    return (uint8_t)time;
  }
  if (time < 72) {
    return (uint8_t)((time + 24) / 2);
  }
  if (time < 168) {
    return (uint8_t)((time + 120) / 4);
  }
  if (time < 360) {
    return (uint8_t)((time + 408) / 8);
  }
  if (time < 744) {
    return (uint8_t)((time + 1176) / 16);
  }
  if (time < 937) {
    return (uint8_t)((time + 3096) / 32);
  }
  return 127;
}

/** Convert a lock time in seconds into a 4-bit RTCMv3 Lock Time Indicator DF402
 * See RTCM 10403.1, Table 3.5-74.
 *
 * \param time Lock time in seconds.
 * \return Lock Time Indicator value 0-15.
 */
uint8_t rtcm3_encode_lock_time(double time) {
  if (time < 0.032) {
    return 0;
  }
  if (time < 0.064) {
    return 1;
  }
  if (time < 0.128) {
    return 2;
  }
  if (time < 0.256) {
    return 3;
  }
  if (time < 0.512) {
    return 4;
  }
  if (time < 1.024) {
    return 5;
  }
  if (time < 2.048) {
    return 6;
  }
  if (time < 4.096) {
    return 7;
  }
  if (time < 8.192) {
    return 8;
  }
  if (time < 16.384) {
    return 9;
  }
  if (time < 32.768) {
    return 10;
  }
  if (time < 65.536) {
    return 11;
  }
  if (time < 131.072) {
    return 12;
  }
  if (time < 262.144) {
    return 13;
  }
  if (time < 524.288) {
    return 14;
  }
  return 15;
}

/** Encode PhaseRange – L1 Pseudorange (DF012, DF018 ,DF042, DF048) */
static int32_t encode_diff_phaserange(double cp_pr, double freq) {
  double phase_unit = (GPS_C / freq) / 0.0005;
  int32_t ppr = lround(cp_pr * phase_unit);

  /* From specification: "Certain ionospheric conditions might cause the GPS
   * L1 Phaserange – L1 Pseudorange to diverge over time across the range
   * limits defined. Under these circumstances the computed value needs to be
   * adjusted (rolled over) by the equivalent of 1500 cycles in order to bring
   * the value back within the range" */

  if (ppr <= -C_2P19) {
    /* add multiples of 1500 cycles */
    cp_pr += 1500 * ceil((-C_2P19 / phase_unit - cp_pr) / 1500);
    ppr = lround(cp_pr * phase_unit);
  } else if (ppr >= C_2P19) {
    /* substract multiples of 1500 cycles */
    cp_pr -= 1500 * ceil((cp_pr - C_2P19 / phase_unit) / 1500);
    ppr = lround(cp_pr * phase_unit);
  }
  return ppr;
}

/** Encode STGSV msg field values */
static rtcm3_rc rtcm3_encode_999_stgsv_fv_base(
    swiftnav_out_bitstream_t *buff,
    const rtcm_999_stgsv_fv *msg_999_stgsv_fv,
    const uint8_t field_mask) {
  assert(buff);

  if (field_mask & RTCM_STGSV_FIELDMASK_EL) {
    BITSTREAM_ENCODE_S8(buff, msg_999_stgsv_fv->el, 8);
  }
  if (field_mask & RTCM_STGSV_FIELDMASK_AZ) {
    BITSTREAM_ENCODE_U16(buff, msg_999_stgsv_fv->az, 9);
  }
  if (field_mask & RTCM_STGSV_FIELDMASK_CN0_B1) {
    BITSTREAM_ENCODE_U8(buff, msg_999_stgsv_fv->cn0_b1, 8);
  }
  if (field_mask & RTCM_STGSV_FIELDMASK_CN0_B2) {
    BITSTREAM_ENCODE_U8(buff, msg_999_stgsv_fv->cn0_b2, 8);
  }
  if (field_mask & RTCM_STGSV_FIELDMASK_CN0_B3) {
    BITSTREAM_ENCODE_U8(buff, msg_999_stgsv_fv->cn0_b3, 8);
  }

  return RC_OK;
}

static rtcm3_rc rtcm3_encode_999_stgsv_base(
    swiftnav_out_bitstream_t *buff, const rtcm_msg_999_stgsv *msg_999_stgsv) {
  assert(buff);

  BITSTREAM_ENCODE_U32(buff, msg_999_stgsv->tow_ms, 30);
  BITSTREAM_ENCODE_U8(buff, msg_999_stgsv->constellation, 4);

  uint64_t sat_mask = 0;
  uint8_t sat_mask_size = (msg_999_stgsv->constellation == RTCM_TESEOV_BDS13
                               ? RTCM_STGSV_SATELLITE_MASK_SIZE_GNSS13
                               : RTCM_STGSV_SATELLITE_MASK_SIZE);
  size_t max_n_sat = MIN(msg_999_stgsv->n_sat, sat_mask_size);
  for (size_t i = 0; i < max_n_sat; i++) {
    sat_mask |= (INT64_C(1) << (RTCM_STGSV_SATELLITE_MASK_SIZE -
                                msg_999_stgsv->field_value[i].sat_id - 1));
  }
  BITSTREAM_ENCODE_U64(buff, sat_mask, 40);
  BITSTREAM_ENCODE_U8(buff, msg_999_stgsv->field_mask, 8);
  BITSTREAM_ENCODE_U8(buff, msg_999_stgsv->mul_msg_ind, 1);

  for (size_t i = 0; i < max_n_sat; i++) {
    rtcm3_encode_999_stgsv_fv_base(
        buff, &msg_999_stgsv->field_value[i], msg_999_stgsv->field_mask);
  }

  return RC_OK;
}

static rtcm3_rc rtcm3_encode_999_restart_base(
    swiftnav_out_bitstream_t *buff,
    const rtcm_msg_999_restart *msg_999_restart) {
  BITSTREAM_ENCODE_U32(buff, msg_999_restart->restart_mask, 32);
  return RC_OK;
}

static rtcm3_rc rtcm3_encode_999_aux_ttff_base(
    swiftnav_out_bitstream_t *buff,
    const rtcm_msg_999_aux_ttff *msg_999_aux_ttff) {
  BITSTREAM_ENCODE_U32(buff, msg_999_aux_ttff->ttff, 32);
  return RC_OK;
}

static rtcm3_rc rtcm3_encode_999_aux_base(swiftnav_out_bitstream_t *buff,
                                          const rtcm_msg_999_aux *msg_999_aux) {
  assert(msg_999_aux);
  BITSTREAM_ENCODE_U8(buff, msg_999_aux->aux_data_type_id, 8);
  switch (msg_999_aux->aux_data_type_id) {
    case RTCM_TESEOV_AUX_TTFF:
      return rtcm3_encode_999_aux_ttff_base(buff, &msg_999_aux->data.ttff);
    default:
      return RC_INVALID_MESSAGE;
  }
}

rtcm3_rc rtcm3_encode_999_bitstream(swiftnav_out_bitstream_t *buff,
                                    const rtcm_msg_999 *msg_999) {
  assert(buff);

  const uint32_t MSG_NUM = 999;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  BITSTREAM_ENCODE_U8(buff, msg_999->sub_type_id, 8);

  switch (msg_999->sub_type_id) {
    case RTCM_TESEOV_RESTART:
      return rtcm3_encode_999_restart_base(buff, &msg_999->data.restart);
    case RTCM_TESEOV_STGSV:
      return rtcm3_encode_999_stgsv_base(buff, &msg_999->data.stgsv);
    case RTCM_TESEOV_AUX:
      return rtcm3_encode_999_aux_base(buff, &msg_999->data.aux);
    default:
      return RC_INVALID_MESSAGE;
  }
}

static rtcm3_rc encode_basic_freq_data(swiftnav_out_bitstream_t *buff,
                                       const rtcm_freq_data *freq_data,
                                       const freq_t freq_enum,
                                       const double *l1_pr) {
  /* Calculate GPS Integer L1 Pseudorange Modulus Ambiguity (DF014). */
  uint8_t amb = (uint8_t)(*l1_pr / PRUNIT_GPS);

  /* Construct L1 pseudorange value as it would be transmitted (DF011). */
  uint32_t calc_l1_pr = (uint32_t)round((*l1_pr - amb * PRUNIT_GPS) / 0.02);

  /* Calculate GPS Pseudorange (DF011/DF016). */
  uint32_t pr =
      (uint32_t)round((freq_data->pseudorange - amb * PRUNIT_GPS) / 0.02);

  double l1_prc = calc_l1_pr * 0.02 + amb * PRUNIT_GPS;

  double freq = 0;
  if (L1_FREQ == freq_enum) {
    freq = GPS_L1_HZ;
    BITSTREAM_ENCODE_U8(buff, (uint8_t)freq_data->code, 1);
    BITSTREAM_ENCODE_U32(
        buff, freq_data->flags.fields.valid_pr ? pr : PR_L1_INVALID, 24);
  } else {
    freq = GPS_L2_HZ;
    BITSTREAM_ENCODE_U8(buff, (uint8_t)freq_data->code, 2);
    BITSTREAM_ENCODE_S32(buff,
                         freq_data->flags.fields.valid_pr
                             ? (int32_t)pr - (int32_t)calc_l1_pr
                             : (int32_t)PR_L2_INVALID,
                         14);
  }

  if (freq_data->flags.fields.valid_cp) {
    /* phaserange - L1 pseudorange */
    double cp_pr = freq_data->carrier_phase - l1_prc / (GPS_C / freq);
    /* encode PhaseRange – L1 Pseudorange (DF012/DF018) and roll over if
     * necessary */
    int32_t ppr = encode_diff_phaserange(cp_pr, freq);
    BITSTREAM_ENCODE_S32(buff, ppr, 20);
  } else {
    BITSTREAM_ENCODE_S32(buff, (int32_t)CP_INVALID, 20);
  }
  uint8_t lock_ind =
      freq_data->flags.fields.valid_lock ? to_lock_ind(freq_data->lock) : 0;
  BITSTREAM_ENCODE_U8(buff, lock_ind, 7);

  return RC_OK;
}

/** Write RTCM header for observation message types 1001..1004.
 *
 * The data message header will be written starting from byte zero of the
 * buffer. If the buffer also contains a frame header then be sure to pass a
 * pointer to the start of the data message rather than a pointer to the start
 * of the frame buffer. The RTCM observation header is 8 bytes (64 bits) long.
 *
 * If the Synchronous GNSS Message Flag is set to `0`, it means that no further
 * GNSS observables referenced to the same Epoch Time will be transmitted. This
 * enables the receiver to begin processing the data immediately after decoding
 * the message. If it is set to `1`, it means that the next message will
 * contain observables of another GNSS source referenced to the same Epoch
 * Time.
 *
 * Divergence-free Smoothing Indicator values:
 *
 * Indicator | Meaning
 * --------- | ----------------------------------
 *     0     | Divergence-free smoothing not used
 *     1     | Divergence-free smoothing used
 *
 * GPS Smoothing Interval indicator values are listed in RTCM 10403.1 Table
 * 3.4-4, reproduced here:
 *
 * Indicator | Smoothing Interval
 * --------- | ------------------
 *  000 (0)  |   No smoothing
 *  001 (1)  |   < 30 s
 *  010 (2)  |   30-60 s
 *  011 (3)  |   1-2 min
 *  100 (4)  |   2-4 min
 *  101 (5)  |   4-8 min
 *  110 (6)  |   >8 min
 *  111 (7)  |   Unlimited
 *
 * \param header pointer to the obs header to encode
 * \param num_sats number of satellites in message
 * \param buff A pointer to the RTCM data message buffer.
 */
static uint16_t rtcm3_encode_header_gps_rtk(swiftnav_out_bitstream_t *buff,
                                            const rtcm_obs_header *header,
                                            uint8_t num_sats) {
  BITSTREAM_ENCODE_U16(buff, header->msg_num, 12);
  BITSTREAM_ENCODE_U16(buff, header->stn_id, 12);
  BITSTREAM_ENCODE_U32(buff, (uint32_t)round(header->tow_ms), 30);
  BITSTREAM_ENCODE_U8(buff, header->sync, 1);
  BITSTREAM_ENCODE_U8(buff, num_sats, 5);
  BITSTREAM_ENCODE_U8(buff, header->div_free, 1);
  BITSTREAM_ENCODE_U8(buff, header->smooth, 3);

  return RC_OK;
}

rtcm3_rc rtcm3_encode_1001_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1001) {
  assert(buff);
  if ((buff->len - buff->offset) < 64) {
    return RC_INVALID_MESSAGE;
  }

  rtcm3_rc ret = RC_INVALID_MESSAGE;

  uint32_t buf_offset_start = buff->offset;
  buff->offset += 64; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1001->header.n_sat; i++) {
    if (msg_1001->sats[i].obs[L1_FREQ].flags.fields.valid_pr &&
        msg_1001->sats[i].obs[L1_FREQ].flags.fields.valid_cp) {
      BITSTREAM_ENCODE_U8(buff, msg_1001->sats[i].svId, 6);
      ret = encode_basic_freq_data(buff,
                                   &msg_1001->sats[i].obs[L1_FREQ],
                                   L1_FREQ,
                                   &msg_1001->sats[i].obs[L1_FREQ].pseudorange);
      if (ret != RC_OK) {
        return ret;
      }
      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }

  uint32_t buf_offset_end = buff->offset;
  buff->offset = buf_offset_start;
  /* Fill in the header */
  ret = rtcm3_encode_header_gps_rtk(buff, &msg_1001->header, num_sats);
  buff->offset = buf_offset_end;

  return ret;
}

/** Encode an RTCMv3 message type 1002 (Extended L1-Only GPS RTK Observables)
 * Message type 1002 has length `64 + n_sat*74` bits. Returned message length
 * is rounded up to the nearest whole byte.
 *
 * \param buff A pointer to the RTCM data message buffer.
 * \param id Reference station ID (DF003).
 * \param t GPS time of epoch (DF004).
 * \param n_sat Number of GPS satellites included in the message (DF006).
 * \param nm Struct containing the observation.
 * \param sync Synchronous GNSS Flag (DF005).
 * \return The message length in bytes.
 */
rtcm3_rc rtcm3_encode_1002_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1002) {
  assert(buff);
  if ((buff->len - buff->offset) < 64) {
    return RC_INVALID_MESSAGE;
  }

  rtcm3_rc ret = RC_INVALID_MESSAGE;

  uint32_t buf_offset_start = buff->offset;
  buff->offset += 64; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1002->header.n_sat; i++) {
    if (msg_1002->sats[i].obs[L1_FREQ].flags.fields.valid_pr &&
        msg_1002->sats[i].obs[L1_FREQ].flags.fields.valid_cp) {
      BITSTREAM_ENCODE_U8(buff, msg_1002->sats[i].svId, 6);
      ret = encode_basic_freq_data(buff,
                                   &msg_1002->sats[i].obs[L1_FREQ],
                                   L1_FREQ,
                                   &msg_1002->sats[i].obs[L1_FREQ].pseudorange);
      if (ret != RC_OK) {
        return ret;
      }

      /* Calculate GPS Integer L1 Pseudorange Modulus Ambiguity (DF014). */
      uint8_t amb =
          (uint8_t)(msg_1002->sats[i].obs[L1_FREQ].pseudorange / PRUNIT_GPS);

      BITSTREAM_ENCODE_U8(buff, amb, 8);
      BITSTREAM_ENCODE_U8(
          buff, (uint8_t)round(msg_1002->sats[i].obs[L1_FREQ].cnr * 4.0), 8);

      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }

  uint32_t buf_offset_end = buff->offset;
  buff->offset = buf_offset_start;
  /* Fill in the header */
  ret = rtcm3_encode_header_gps_rtk(buff, &msg_1002->header, num_sats);
  buff->offset = buf_offset_end;

  return ret;
}

rtcm3_rc rtcm3_encode_1003_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1003) {
  assert(buff);
  if ((buff->len - buff->offset) < 64) {
    return RC_INVALID_MESSAGE;
  }

  rtcm3_rc ret = RC_INVALID_MESSAGE;

  uint32_t buf_offset_start = buff->offset;
  buff->offset += 64; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1003->header.n_sat; i++) {
    flag_bf l1_flags = msg_1003->sats[i].obs[L1_FREQ].flags;
    flag_bf l2_flags = msg_1003->sats[i].obs[L2_FREQ].flags;
    if (l1_flags.fields.valid_pr && l1_flags.fields.valid_cp &&
        l2_flags.fields.valid_pr && l2_flags.fields.valid_cp) {
      BITSTREAM_ENCODE_U8(buff, msg_1003->sats[i].svId, 6);

      ret = encode_basic_freq_data(buff,
                                   &msg_1003->sats[i].obs[L1_FREQ],
                                   L1_FREQ,
                                   &msg_1003->sats[i].obs[L1_FREQ].pseudorange);
      if (ret != RC_OK) {
        return ret;
      }

      ret = encode_basic_freq_data(buff,
                                   &msg_1003->sats[i].obs[L2_FREQ],
                                   L2_FREQ,
                                   &msg_1003->sats[i].obs[L1_FREQ].pseudorange);
      if (ret != RC_OK) {
        return ret;
      }

      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }

  uint32_t buf_offset_end = buff->offset;
  buff->offset = buf_offset_start;
  /* Fill in the header */
  ret = rtcm3_encode_header_gps_rtk(buff, &msg_1003->header, num_sats);
  buff->offset = buf_offset_end;

  return ret;
}

rtcm3_rc rtcm3_encode_1004_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1004) {
  assert(buff);
  if ((buff->len - buff->offset) < 64) {
    return RC_INVALID_MESSAGE;
  }

  rtcm3_rc ret = RC_INVALID_MESSAGE;

  uint32_t buf_offset_start = buff->offset;
  buff->offset += 64; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1004->header.n_sat; i++) {
    flag_bf l1_flags = msg_1004->sats[i].obs[L1_FREQ].flags;
    if (l1_flags.fields.valid_pr && l1_flags.fields.valid_cp) {
      BITSTREAM_ENCODE_U8(buff, msg_1004->sats[i].svId, 6);

      ret = encode_basic_freq_data(buff,
                                   &msg_1004->sats[i].obs[L1_FREQ],
                                   L1_FREQ,
                                   &msg_1004->sats[i].obs[L1_FREQ].pseudorange);
      if (ret != RC_OK) {
        return ret;
      }

      /* Calculate GPS Integer L1 Pseudorange Modulus Ambiguity (DF014). */
      uint8_t amb =
          (uint8_t)(msg_1004->sats[i].obs[L1_FREQ].pseudorange / PRUNIT_GPS);
      BITSTREAM_ENCODE_U8(buff, amb, 8);
      BITSTREAM_ENCODE_U8(
          buff, (uint8_t)round(msg_1004->sats[i].obs[L1_FREQ].cnr * 4.0), 8);

      ret = encode_basic_freq_data(buff,
                                   &msg_1004->sats[i].obs[L2_FREQ],
                                   L2_FREQ,
                                   &msg_1004->sats[i].obs[L1_FREQ].pseudorange);
      if (ret != RC_OK) {
        return ret;
      }
      BITSTREAM_ENCODE_U8(
          buff, (uint8_t)round(msg_1004->sats[i].obs[L2_FREQ].cnr * 4.0), 8);

      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }

  uint32_t buf_offset_end = buff->offset;
  buff->offset = buf_offset_start;
  /* Fill in the header */
  ret = rtcm3_encode_header_gps_rtk(buff, &msg_1004->header, num_sats);
  buff->offset = buf_offset_end;

  return ret;
}

static rtcm3_rc rtcm3_encode_1005_base(swiftnav_out_bitstream_t *buff,
                                       const rtcm_msg_1005 *msg_1005) {
  BITSTREAM_ENCODE_U16(buff, msg_1005->stn_id, 12);
  BITSTREAM_ENCODE_U8(buff, msg_1005->ITRF, 6);
  BITSTREAM_ENCODE_U8(buff, msg_1005->GPS_ind, 1);
  BITSTREAM_ENCODE_U8(buff, msg_1005->GLO_ind, 1);
  BITSTREAM_ENCODE_U8(buff, msg_1005->GAL_ind, 1);
  BITSTREAM_ENCODE_U8(buff, msg_1005->ref_stn_ind, 1);
  BITSTREAM_ENCODE_S64(buff, (int64_t)round(msg_1005->arp_x * 10000.0), 38);
  BITSTREAM_ENCODE_U8(buff, msg_1005->osc_ind, 1);
  BITSTREAM_ENCODE_U8(buff, (uint8_t)0, 1);
  BITSTREAM_ENCODE_S64(buff, (int64_t)round(msg_1005->arp_y * 10000.0), 38);
  BITSTREAM_ENCODE_U8(buff, msg_1005->quart_cycle_ind, 2);
  BITSTREAM_ENCODE_S64(buff, (int64_t)round(msg_1005->arp_z * 10000.0), 38);

  return RC_OK;
}

rtcm3_rc rtcm3_encode_1005_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1005 *msg_1005) {
  assert(msg_1005);
  const uint32_t MSG_NUM = 1005;

  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  return rtcm3_encode_1005_base(buff, msg_1005);
}

rtcm3_rc rtcm3_encode_1006_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1006 *msg_1006) {
  assert(msg_1006);
  const uint32_t MSG_NUM = 1006;

  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  rtcm3_rc ret = rtcm3_encode_1005_base(buff, &msg_1006->msg_1005);
  if (ret != RC_OK) {
    return ret;
  }
  double ant_height = msg_1006->ant_height;
  if (ant_height < 0.0) {
    ant_height = 0.0;
  } else if (ant_height > RTCM_1006_MAX_ANTENNA_HEIGHT_M) {
    ant_height = RTCM_1006_MAX_ANTENNA_HEIGHT_M;
  }
  BITSTREAM_ENCODE_U16(buff, (uint16_t)round(ant_height * 10000.0), 16);

  return RC_OK;
}

static rtcm3_rc rtcm3_encode_1007_base(swiftnav_out_bitstream_t *buff,
                                       const rtcm_msg_1007 *msg_1007) {
  BITSTREAM_ENCODE_U16(buff, msg_1007->stn_id, 12);
  BITSTREAM_ENCODE_U8(buff, msg_1007->ant_descriptor_counter, 8);
  for (uint8_t i = 0; i < msg_1007->ant_descriptor_counter; ++i) {
    BITSTREAM_ENCODE_U8(buff, (uint8_t)msg_1007->ant_descriptor[i], 8);
  }
  BITSTREAM_ENCODE_U8(buff, msg_1007->ant_setup_id, 8);
  return RC_OK;
}

rtcm3_rc rtcm3_encode_1007_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1007 *msg_1007) {
  assert(msg_1007);
  const uint32_t MSG_NUM = 1007;

  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  return rtcm3_encode_1007_base(buff, msg_1007);
}

rtcm3_rc rtcm3_encode_1008_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1008 *msg_1008) {
  assert(msg_1008);

  const uint32_t MSG_NUM = 1008;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  rtcm3_rc ret = rtcm3_encode_1007_base(buff, &msg_1008->msg_1007);
  if (ret != RC_OK) {
    return ret;
  }
  BITSTREAM_ENCODE_U8(buff, msg_1008->ant_serial_num_counter, 8);
  for (uint8_t i = 0; i < msg_1008->ant_serial_num_counter; ++i) {
    BITSTREAM_ENCODE_U8(buff, (uint8_t)msg_1008->ant_serial_num[i], 8);
  }

  return RC_OK;
}

static rtcm3_rc encode_basic_glo_freq_data(swiftnav_out_bitstream_t *buff,
                                           const rtcm_freq_data *freq_data,
                                           const freq_t freq_enum,
                                           const double *l1_pr,
                                           const uint8_t fcn) {
  /* Calculate GPS Integer L1 Pseudorange Modulus Ambiguity (DF044). */
  uint8_t amb = (uint8_t)(*l1_pr / PRUNIT_GLO);

  /* Construct L1 pseudorange value as it would be transmitted (DF041). */
  uint32_t calc_l1_pr = (uint32_t)round((*l1_pr - amb * PRUNIT_GLO) / 0.02);

  /* Calculate GLO Pseudorange (DF041/DF046). */
  uint32_t pr =
      (uint32_t)round((freq_data->pseudorange - amb * PRUNIT_GLO) / 0.02);

  double l1_prc = calc_l1_pr * 0.02 + amb * PRUNIT_GLO;

  double glo_freq = 0.0;
  if (L1_FREQ == freq_enum) {
    glo_freq = GLO_L1_HZ + (fcn - MT1012_GLO_FCN_OFFSET) * GLO_L1_DELTA_HZ;

    BITSTREAM_ENCODE_U8(buff, (uint8_t)0, 1);
    BITSTREAM_ENCODE_U8(buff, fcn, 5);
    BITSTREAM_ENCODE_U32(
        buff, freq_data->flags.fields.valid_pr ? pr : PR_L1_INVALID, 25);
  } else {
    glo_freq = GLO_L2_HZ + (fcn - MT1012_GLO_FCN_OFFSET) * GLO_L2_DELTA_HZ;

    BITSTREAM_ENCODE_U8(buff, (uint8_t)0, 2);
    BITSTREAM_ENCODE_S32(buff,
                         freq_data->flags.fields.valid_pr
                             ? (int32_t)((int32_t)pr - (int32_t)calc_l1_pr)
                             : (int32_t)PR_L2_INVALID,
                         14);
  }

  if (freq_data->flags.fields.valid_cp) {
    /* phaserange - L1 pseudorange */
    double cp_pr = freq_data->carrier_phase - l1_prc / (GPS_C / glo_freq);

    /* Calculate PhaseRange – L1 Pseudorange (DF042/DF048) and roll over if
     * necessary */
    int32_t ppr = encode_diff_phaserange(cp_pr, glo_freq);
    BITSTREAM_ENCODE_S32(buff, ppr, 20);
  } else {
    BITSTREAM_ENCODE_S32(buff, (int32_t)CP_INVALID, 20);
  }
  BITSTREAM_ENCODE_U32(
      buff,
      freq_data->flags.fields.valid_lock ? to_lock_ind(freq_data->lock) : 0,
      7);

  return RC_OK;
}

/** Write RTCM header for observation message types 1009..1012.
 *
 * The data message header will be written starting from byte zero of the
 * buffer. If the buffer also contains a frame header then be sure to pass a
 * pointer to the start of the data message rather than a pointer to the start
 * of the frame buffer. The RTCM observation header is 8 bytes (61 bits) long.
 *
 * If the Synchronous GNSS Message Flag is set to `0`, it means that no further
 * GNSS observables referenced to the same Epoch Time will be transmitted. This
 * enables the receiver to begin processing the data immediately after decoding
 * the message. If it is set to `1`, it means that the next message will
 * contain observables of another GNSS source referenced to the same Epoch
 * Time.
 *
 * Divergence-free Smoothing Indicator values:
 *
 * Indicator | Meaning
 * --------- | ----------------------------------
 *     0     | Divergence-free smoothing not used
 *     1     | Divergence-free smoothing used
 *
 * GLO Smoothing Interval indicator values are listed in RTCM 10403.1 Table
 * 3.4-4, reproduced here:
 *
 * Indicator | Smoothing Interval
 * --------- | ------------------
 *  000 (0)  |   No smoothing
 *  001 (1)  |   < 30 s
 *  010 (2)  |   30-60 s
 *  011 (3)  |   1-2 min
 *  100 (4)  |   2-4 min
 *  101 (5)  |   4-8 min
 *  110 (6)  |   >8 min
 *  111 (7)  |   Unlimited
 *
 * \param header pointer to the obs header to encode
 * \param num_sats number of satellites in message
 * \param buff A pointer to the RTCM data message buffer.
 */
static rtcm3_rc rtcm3_encode_glo_header(swiftnav_out_bitstream_t *buff,
                                        const rtcm_obs_header *header,
                                        uint8_t num_sats) {
  BITSTREAM_ENCODE_U16(buff, header->msg_num, 12);
  BITSTREAM_ENCODE_U16(buff, header->stn_id, 12);
  BITSTREAM_ENCODE_U32(buff, (uint32_t)round(header->tow_ms), 27);
  BITSTREAM_ENCODE_U8(buff, header->sync, 1);
  BITSTREAM_ENCODE_U8(buff, num_sats, 5);
  BITSTREAM_ENCODE_U8(buff, header->div_free, 1);
  BITSTREAM_ENCODE_U8(buff, header->smooth, 3);

  return RC_OK;
}

rtcm3_rc rtcm3_encode_1010_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1010) {
  assert(buff);
  if ((buff->len - buff->offset) < 61) {
    return RC_INVALID_MESSAGE;
  }

  rtcm3_rc ret = RC_INVALID_MESSAGE;

  uint32_t buf_offset_start = buff->offset;
  buff->offset += 61; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1010->header.n_sat; i++) {
    if (msg_1010->sats[i].obs[L1_FREQ].flags.fields.valid_pr &&
        msg_1010->sats[i].obs[L1_FREQ].flags.fields.valid_cp) {
      const rtcm_sat_data *sat_obs = &msg_1010->sats[i];

      BITSTREAM_ENCODE_U8(buff, sat_obs->svId, 6);
      ret = encode_basic_glo_freq_data(buff,
                                       &sat_obs->obs[L1_FREQ],
                                       L1_FREQ,
                                       &sat_obs->obs[L1_FREQ].pseudorange,
                                       sat_obs->fcn);
      if (ret != RC_OK) {
        return ret;
      }

      /* Calculate GPS Integer L1 Pseudorange Modulus Ambiguity (DF014). */
      uint8_t amb = (uint8_t)(sat_obs->obs[L1_FREQ].pseudorange / PRUNIT_GLO);
      BITSTREAM_ENCODE_U8(buff, amb, 7);
      BITSTREAM_ENCODE_U8(
          buff, (uint8_t)round(sat_obs->obs[L1_FREQ].cnr * 4.0), 8);
      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }
  uint32_t buf_offset_end = buff->offset;

  buff->offset = buf_offset_start;
  ret = rtcm3_encode_glo_header(buff, &msg_1010->header, num_sats);
  buff->offset = buf_offset_end;
  return ret;
}

rtcm3_rc rtcm3_encode_1012_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_obs_message *msg_1012) {
  assert(buff);
  if ((buff->len - buff->offset) < 61) {
    return RC_INVALID_MESSAGE;
  }

  rtcm3_rc ret = RC_INVALID_MESSAGE;

  uint32_t buf_offset_start = buff->offset;
  buff->offset += 61; /* Start at end of header. */

  uint8_t num_sats = 0;
  for (uint8_t i = 0; i < msg_1012->header.n_sat; i++) {
    flag_bf l1_flags = msg_1012->sats[i].obs[L1_FREQ].flags;
    if (l1_flags.fields.valid_pr && l1_flags.fields.valid_cp) {
      const rtcm_sat_data *sat_obs = &msg_1012->sats[i];
      BITSTREAM_ENCODE_U8(buff, sat_obs->svId, 6);

      ret = encode_basic_glo_freq_data(buff,
                                       &sat_obs->obs[L1_FREQ],
                                       L1_FREQ,
                                       &sat_obs->obs[L1_FREQ].pseudorange,
                                       sat_obs->fcn);
      if (ret != RC_OK) {
        return ret;
      }

      /* Calculate GLO Integer L1 Pseudorange Modulus Ambiguity (DF014). */
      uint8_t amb = (uint8_t)(sat_obs->obs[L1_FREQ].pseudorange / PRUNIT_GLO);
      BITSTREAM_ENCODE_U8(buff, amb, 7);
      BITSTREAM_ENCODE_U8(
          buff, (uint8_t)round(sat_obs->obs[L1_FREQ].cnr * 4.0), 8);

      ret = encode_basic_glo_freq_data(buff,
                                       &sat_obs->obs[L2_FREQ],
                                       L2_FREQ,
                                       &sat_obs->obs[L1_FREQ].pseudorange,
                                       sat_obs->fcn);
      if (ret != RC_OK) {
        return ret;
      }
      BITSTREAM_ENCODE_U8(
          buff, (uint8_t)round(sat_obs->obs[L2_FREQ].cnr * 4.0), 8);
      ++num_sats;
      if (num_sats >= RTCM_MAX_SATS) {
        /* sanity */
        break;
      }
    }
  }
  uint32_t buf_offset_end = buff->offset;

  buff->offset = buf_offset_start;
  ret = rtcm3_encode_glo_header(buff, &msg_1012->header, num_sats);
  buff->offset = buf_offset_end;
  return ret;
}

rtcm3_rc rtcm3_encode_1013_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1013 *msg_1013) {
  assert(buff);
  assert(msg_1013->message_count <= RTCM_1013_MAX_MESSAGES);

  const uint32_t MSG_NUM = 1013;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  BITSTREAM_ENCODE_U16(buff, msg_1013->reference_station_id, 12);
  BITSTREAM_ENCODE_U16(buff, msg_1013->mjd, 16);
  BITSTREAM_ENCODE_U32(buff, msg_1013->utc, 17);

  if (msg_1013->message_count >= 32) {
    BITSTREAM_ENCODE_U32(buff, 0, 5);
  } else {
    BITSTREAM_ENCODE_U16(buff, msg_1013->message_count, 5);
  }
  BITSTREAM_ENCODE_U8(buff, msg_1013->leap_second, 8);

  for (size_t i = 0; i < msg_1013->message_count; ++i) {
    BITSTREAM_ENCODE_U16(buff, msg_1013->messages[i].id, 12);
    BITSTREAM_ENCODE_U8(buff, msg_1013->messages[i].sync_flag, 1);
    BITSTREAM_ENCODE_U16(buff, msg_1013->messages[i].transmission_interval, 16);
  }

  return RC_OK;
}

rtcm3_rc rtcm3_encode_1029_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1029 *msg_1029) {
  assert(buff);

  const uint32_t MSG_NUM = 1029;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  BITSTREAM_ENCODE_U16(buff, msg_1029->stn_id, 12);
  BITSTREAM_ENCODE_U16(buff, msg_1029->mjd_num, 16);
  BITSTREAM_ENCODE_U32(buff, msg_1029->utc_sec_of_day, 17);
  BITSTREAM_ENCODE_U8(buff, msg_1029->unicode_chars, 7);
  BITSTREAM_ENCODE_U8(buff, msg_1029->utf8_code_units_n, 8);

  for (uint8_t i = 0; i < msg_1029->utf8_code_units_n; i++) {
    BITSTREAM_ENCODE_U8(buff, msg_1029->utf8_code_units[i], 8);
  }

  return RC_OK;
}

rtcm3_rc rtcm3_encode_1033_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1033 *msg_1033) {
  assert(buff);

  const uint32_t MSG_NUM = 1033;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  BITSTREAM_ENCODE_U16(buff, msg_1033->stn_id, 12);

  BITSTREAM_ENCODE_U8(buff, msg_1033->ant_descriptor_counter, 8);
  for (uint8_t i = 0; i < msg_1033->ant_descriptor_counter; ++i) {
    BITSTREAM_ENCODE_U8(buff, (uint8_t)msg_1033->ant_descriptor[i], 8);
  }

  BITSTREAM_ENCODE_S8(buff, msg_1033->ant_setup_id, 8);

  BITSTREAM_ENCODE_U8(buff, msg_1033->ant_serial_num_counter, 8);
  for (uint8_t i = 0; i < msg_1033->ant_serial_num_counter; ++i) {
    BITSTREAM_ENCODE_U8(buff, (uint8_t)msg_1033->ant_serial_num[i], 8);
  }

  BITSTREAM_ENCODE_U8(buff, msg_1033->rcv_descriptor_counter, 8);
  for (uint8_t i = 0; i < msg_1033->rcv_descriptor_counter; ++i) {
    BITSTREAM_ENCODE_U8(buff, (uint8_t)msg_1033->rcv_descriptor[i], 8);
  }

  BITSTREAM_ENCODE_U8(buff, msg_1033->rcv_fw_version_counter, 8);
  for (uint8_t i = 0; i < msg_1033->rcv_fw_version_counter; ++i) {
    BITSTREAM_ENCODE_U8(buff, (uint8_t)msg_1033->rcv_fw_version[i], 8);
  }

  BITSTREAM_ENCODE_U8(buff, msg_1033->rcv_serial_num_counter, 8);
  for (uint8_t i = 0; i < msg_1033->rcv_serial_num_counter; ++i) {
    BITSTREAM_ENCODE_U8(buff, (uint8_t)msg_1033->rcv_serial_num[i], 8);
  }

  return RC_OK;
}

rtcm3_rc rtcm3_encode_1230_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msg_1230 *msg_1230) {
  assert(buff);

  const uint32_t MSG_NUM = 1230;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  BITSTREAM_ENCODE_U16(buff, msg_1230->stn_id, 12);
  BITSTREAM_ENCODE_U8(buff, msg_1230->bias_indicator, 1);  // 8 or 1
  BITSTREAM_ENCODE_U8(buff, (uint8_t)0, 3);                /* 3 reserved bits */
  BITSTREAM_ENCODE_U8(buff, msg_1230->fdma_signal_mask, 4);
  if (msg_1230->fdma_signal_mask & 0x08) {
    int16_t bias = (int16_t)round(msg_1230->L1_CA_cpb_meter * 50);
    BITSTREAM_ENCODE_S16(buff, bias, 16);
  }
  if (msg_1230->fdma_signal_mask & 0x04) {
    int16_t bias = (int16_t)round(msg_1230->L1_P_cpb_meter * 50);
    BITSTREAM_ENCODE_S16(buff, bias, 16);
  }
  if (msg_1230->fdma_signal_mask & 0x02) {
    int16_t bias = (int16_t)round(msg_1230->L2_CA_cpb_meter * 50);
    BITSTREAM_ENCODE_S16(buff, bias, 16);
  }
  if (msg_1230->fdma_signal_mask & 0x01) {
    int16_t bias = (int16_t)round(msg_1230->L2_P_cpb_meter * 50);
    BITSTREAM_ENCODE_S16(buff, bias, 16);
  }

  return RC_OK;
}

/** Following functions support encoding msm msg. */

static rtcm3_rc rtcm3_encode_msm_header(swiftnav_out_bitstream_t *buff,
                                        const rtcm_msm_header *header,
                                        const rtcm_constellation_t cons) {
  assert(buff);

  BITSTREAM_ENCODE_U16(buff, header->msg_num, 12);
  BITSTREAM_ENCODE_U16(buff, header->stn_id, 12);
  if (RTCM_CONSTELLATION_GLO == cons) {
    /* day of the week */
    uint8_t dow = (uint16_t)(header->tow_ms / (24 * 3600 * 1000));
    BITSTREAM_ENCODE_U8(buff, dow, 3);
    /* time of the day */
    uint32_t tod_ms = header->tow_ms - dow * 24 * 3600 * 1000;
    BITSTREAM_ENCODE_U32(buff, tod_ms, 27);
  } else {
    /* for other systems, epoch time is the time of week in ms */
    BITSTREAM_ENCODE_U32(buff, header->tow_ms, 30);
  }

  BITSTREAM_ENCODE_U8(buff, header->multiple, 1);
  BITSTREAM_ENCODE_U8(buff, header->iods, 3);
  BITSTREAM_ENCODE_U8(buff, header->reserved, 7);
  BITSTREAM_ENCODE_U8(buff, header->steering, 2);
  BITSTREAM_ENCODE_U8(buff, header->ext_clock, 2);
  BITSTREAM_ENCODE_U8(buff, header->div_free, 1);
  BITSTREAM_ENCODE_U8(buff, header->smooth, 3);

  for (size_t i = 0; i < MSM_SATELLITE_MASK_SIZE; i++) {
    BITSTREAM_ENCODE_U8(buff, header->satellite_mask[i], 1);
  }
  for (size_t i = 0; i < MSM_SIGNAL_MASK_SIZE; i++) {
    BITSTREAM_ENCODE_U8(buff, header->signal_mask[i], 1);
  }
  uint8_t num_sats =
      count_mask_values(MSM_SATELLITE_MASK_SIZE, header->satellite_mask);
  uint8_t num_sigs =
      count_mask_values(MSM_SIGNAL_MASK_SIZE, header->signal_mask);
  uint8_t cell_mask_size = num_sats * num_sigs;

  for (uint8_t i = 0; i < cell_mask_size; i++) {
    BITSTREAM_ENCODE_U8(buff, header->cell_mask[i], 1);
  }

  return RC_OK;
}

static rtcm3_rc encode_msm_sat_data(swiftnav_out_bitstream_t *buff,
                                    const rtcm_msm_message *msg,
                                    const uint8_t num_sats,
                                    const msm_enum msm_type,
                                    double rough_range_ms[num_sats],
                                    double rough_rate_m_s[num_sats]) {
  assert(buff);

  /* number of integer milliseconds, DF397 */
  uint8_t integer_ms[num_sats];
  for (uint8_t i = 0; i < num_sats; i++) {
    integer_ms[i] = (uint8_t)floor(msg->sats[i].rough_range_ms);
    BITSTREAM_ENCODE_U8(buff, integer_ms[i], 8);
  }
  if (MSM5 == msm_type) {
    for (uint8_t i = 0; i < num_sats; i++) {
      BITSTREAM_ENCODE_U8(buff, msg->sats[i].glo_fcn, 4);
    }
  }

  /* rough range modulo 1 ms, DF398 */
  for (uint8_t i = 0; i < num_sats; i++) {
    double pr = msg->sats[i].rough_range_ms;
    /* remove integer ms part */
    double range_modulo_ms = pr - integer_ms[i];
    uint16_t range_modulo_encoded = (uint16_t)round(1024 * range_modulo_ms);
    BITSTREAM_ENCODE_U16(buff, range_modulo_encoded, 10);
    rough_range_ms[i] = integer_ms[i] + (double)range_modulo_encoded / 1024;
  }

  /* range rate, m/s, DF399*/
  if (MSM5 == msm_type) {
    for (uint8_t i = 0; i < num_sats; i++) {
      int16_t range_rate = (int16_t)msg->sats[i].rough_range_rate_m_s;
      BITSTREAM_ENCODE_S16(buff, range_rate, 14);
      rough_rate_m_s[i] = range_rate;
    }
  }

  return RC_OK;
}

static rtcm3_rc encode_msm_fine_pseudoranges(swiftnav_out_bitstream_t *buff,
                                             const uint8_t num_cells,
                                             const double fine_pr_ms[],
                                             const flag_bf flags[]) {
  assert(buff);
  /* DF400 */
  for (uint16_t i = 0; i < num_cells; i++) {
    if (flags[i].fields.valid_pr && fabs(fine_pr_ms[i]) < C_1_2P10) {
      BITSTREAM_ENCODE_S16(buff, (int16_t)round(fine_pr_ms[i] / C_1_2P24), 15);
    } else {
      BITSTREAM_ENCODE_S32(buff, (int32_t)MSM_PR_INVALID, 15);
    }
  }

  return RC_OK;
}

static rtcm3_rc encode_msm_fine_phaseranges(swiftnav_out_bitstream_t *buff,
                                            const uint8_t num_cells,
                                            const double fine_cp_ms[],
                                            const flag_bf flags[]) {
  assert(buff);

  /* DF401 */
  for (uint16_t i = 0; i < num_cells; i++) {
    if (flags[i].fields.valid_cp && fabs(fine_cp_ms[i]) < C_1_2P8) {
      BITSTREAM_ENCODE_S32(buff, (int32_t)round(fine_cp_ms[i] / C_1_2P29), 22);
    } else {
      BITSTREAM_ENCODE_S32(buff, (int32_t)MSM_CP_INVALID, 22);
    }
  }

  return RC_OK;
}

static rtcm3_rc encode_msm_lock_times(swiftnav_out_bitstream_t *buff,
                                      const uint8_t num_cells,
                                      const double lock_time[],
                                      const flag_bf flags[]) {
  assert(buff);

  /* DF402 */
  for (uint16_t i = 0; i < num_cells; i++) {
    if (flags[i].fields.valid_lock) {
      BITSTREAM_ENCODE_U8(buff, rtcm3_encode_lock_time(lock_time[i]), 4);
    } else {
      BITSTREAM_ENCODE_U8(buff, (uint8_t)0, 4);
    }
  }

  return RC_OK;
}

static rtcm3_rc encode_msm_hca_indicators(swiftnav_out_bitstream_t *buff,
                                          const uint8_t num_cells,
                                          const bool hca_indicator[]) {
  assert(buff);

  /* DF420 */
  for (uint16_t i = 0; i < num_cells; i++) {
    BITSTREAM_ENCODE_U8(buff, hca_indicator[i], 1);
  }
  return RC_OK;
}

static rtcm3_rc encode_msm_cnrs(swiftnav_out_bitstream_t *buff,
                                const uint8_t num_cells,
                                const double cnr[],
                                const flag_bf flags[]) {
  assert(buff);
  /* DF403 */
  for (uint16_t i = 0; i < num_cells; i++) {
    if (flags[i].fields.valid_cnr) {
      BITSTREAM_ENCODE_U8(buff, (uint8_t)round(cnr[i]), 6);
    } else {
      BITSTREAM_ENCODE_U8(buff, (uint8_t)0, 6);
    }
  }
  return RC_OK;
}

static rtcm3_rc encode_msm_fine_phaserangerates(
    swiftnav_out_bitstream_t *buff,
    const uint8_t num_cells,
    const double fine_range_rate_m_s[],
    const flag_bf flags[]) {
  assert(buff);

  /* DF404 */
  for (uint16_t i = 0; i < num_cells; i++) {
    if (flags[i].fields.valid_dop &&
        fabs(fine_range_rate_m_s[i]) < 0.0001 * C_2P14) {
      BITSTREAM_ENCODE_S16(
          buff, (int16_t)round(fine_range_rate_m_s[i] / 0.0001), 15);
    } else {
      BITSTREAM_ENCODE_S32(buff, (int32_t)MSM_DOP_INVALID, 15);
    }
  }

  return RC_OK;
}

static rtcm3_rc rtcm3_encode_msm_internal(swiftnav_out_bitstream_t *buff,
                                          const rtcm_msm_message *msg) {
  rtcm3_rc ret = RC_INVALID_MESSAGE;

  const rtcm_msm_header *header = &msg->header;

  msm_enum msm_type = to_msm_type(header->msg_num);
  if ((MSM4 != msm_type) && (MSM5 != msm_type)) {
    /* Unexpected message type. */
    return 0;
  }

  rtcm_constellation_t cons = to_constellation(header->msg_num);

  if (RTCM_CONSTELLATION_INVALID == cons) {
    /* Invalid or unsupported constellation */
    return 0;
  }

  uint8_t num_sats =
      count_mask_values(MSM_SATELLITE_MASK_SIZE, header->satellite_mask);
  uint8_t num_sigs =
      count_mask_values(MSM_SIGNAL_MASK_SIZE, header->signal_mask);
  uint8_t cell_mask_size = num_sats * num_sigs;
  uint8_t num_cells = count_mask_values(cell_mask_size, header->cell_mask);

  if (num_sats * num_sigs > MSM_MAX_CELLS) {
    /* Too large cell mask, should already have been handled by caller */
    return 0;
  }

  /* Header */
  ret = rtcm3_encode_msm_header(buff, header, cons);
  if (ret != RC_OK) {
    return ret;
  }

  /* Satellite Data */

  double rough_range_ms[num_sats];
  double rough_rate_m_s[num_sats];

  /* Satellite data */
  ret = encode_msm_sat_data(
      buff, msg, num_sats, msm_type, rough_range_ms, rough_rate_m_s);
  if (ret != RC_OK) {
    return ret;
  }

  /* Signal Data */

  double fine_pr_ms[num_cells];
  double fine_cp_ms[num_cells];
  double lock_time[num_cells];
  bool hca_indicator[num_cells];
  double cnr[num_cells];
  double fine_range_rate_m_s[num_cells];
  flag_bf flags[num_cells];

  memset(fine_pr_ms, 0, sizeof(fine_pr_ms));
  memset(fine_cp_ms, 0, sizeof(fine_cp_ms));
  memset(lock_time, 0, sizeof(lock_time));
  memset(hca_indicator, 0, sizeof(hca_indicator));
  memset(cnr, 0, sizeof(cnr));
  memset(fine_range_rate_m_s, 0, sizeof(fine_range_rate_m_s));
  memset(flags, 0, sizeof(flags));

  uint8_t i = 0;
  for (uint8_t sat = 0; sat < num_sats; sat++) {
    for (uint8_t sig = 0; sig < num_sigs; sig++) {
      if (header->cell_mask[sat * num_sigs + sig]) {
        flags[i] = msg->signals[i].flags;
        if (flags[i].fields.valid_pr) {
          fine_pr_ms[i] = msg->signals[i].pseudorange_ms - rough_range_ms[sat];
        }
        if (flags[i].fields.valid_cp) {
          fine_cp_ms[i] =
              msg->signals[i].carrier_phase_ms - rough_range_ms[sat];
        }
        if (flags[i].fields.valid_lock) {
          lock_time[i] = msg->signals[i].lock_time_s;
        }
        hca_indicator[i] = msg->signals[i].hca_indicator;
        if (flags[i].fields.valid_cnr) {
          cnr[i] = msg->signals[i].cnr;
        } else {
          cnr[i] = 0;
        }
        if (MSM5 == msm_type && flags[i].fields.valid_dop) {
          fine_range_rate_m_s[i] =
              msg->signals[i].range_rate_m_s - rough_rate_m_s[sat];
        }
        i++;
      }
    }
  }

  ret = encode_msm_fine_pseudoranges(buff, num_cells, fine_pr_ms, flags);
  if (ret != RC_OK) {
    return ret;
  }
  ret = encode_msm_fine_phaseranges(buff, num_cells, fine_cp_ms, flags);
  if (ret != RC_OK) {
    return ret;
  }
  ret = encode_msm_lock_times(buff, num_cells, lock_time, flags);
  if (ret != RC_OK) {
    return ret;
  }
  ret = encode_msm_hca_indicators(buff, num_cells, hca_indicator);
  if (ret != RC_OK) {
    return ret;
  }
  ret = encode_msm_cnrs(buff, num_cells, cnr, flags);
  if (ret != RC_OK) {
    return ret;
  }
  if (MSM5 == msm_type) {
    ret = encode_msm_fine_phaserangerates(
        buff, num_cells, fine_range_rate_m_s, flags);
    if (ret != RC_OK) {
      return ret;
    }
  }

  return RC_OK;
}

/** MSM4 encoder
 *
 * \param msg The input RTCM message struct
 * \param buff Data buffer large enough to hold the message (at worst 742 bytes)
 *             (see RTCM 10403.3 Table 3.5-71)
 * \return
 */
rtcm3_rc rtcm3_encode_msm4_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msm_message *msg_msm4) {
  assert(buff);
  if (MSM4 != to_msm_type(msg_msm4->header.msg_num)) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  return rtcm3_encode_msm_internal(buff, msg_msm4);
}

/** MSM5 encoder
 *
 * \param msg The input RTCM message struct
 * \param buff Data buffer large enough to hold the message (at worst 742 bytes)
 *             (see RTCM 10403.3 Table 3.5-71)
 * \return
 */
rtcm3_rc rtcm3_encode_msm5_bitstream(swiftnav_out_bitstream_t *buff,
                                     const rtcm_msm_message *msg_msm5) {
  assert(buff);
  if (MSM5 != to_msm_type(msg_msm5->header.msg_num)) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  return rtcm3_encode_msm_internal(buff, msg_msm5);
}

/** Encode the Swift Proprietary Message
 *
 * \param msg The input RTCM message struct
 * \param buff Data buffer large enough to hold the message
 * \return Number of bytes written or 0 on failure
 */
rtcm3_rc rtcm3_encode_4062_bitstream(
    swiftnav_out_bitstream_t *buff,
    const rtcm_msg_swift_proprietary *msg_4062) {
  assert(buff);

  const uint32_t MSG_NUM = 4062;
  BITSTREAM_ENCODE_U32(buff, MSG_NUM, 12);
  /* These 4 bits are currently reserved, and should always be 0 */
  BITSTREAM_ENCODE_U8(buff, (uint8_t)0, 4);
  BITSTREAM_ENCODE_U16(buff, msg_4062->msg_type, 16);
  BITSTREAM_ENCODE_U16(buff, msg_4062->sender_id, 16);
  BITSTREAM_ENCODE_U8(buff, msg_4062->len, 8);
  for (size_t i = 0; i < msg_4062->len; ++i) {
    BITSTREAM_ENCODE_U8(buff, msg_4062->data[i], 8);
  }

  return RC_OK;
}

rtcm3_rc rtcm3_encode_payload_bitstream(swiftnav_out_bitstream_t *buff,
                                        const void *rtcm_msg,
                                        uint16_t message_type) {
  assert(buff);

  if (buff->len == 0) {
    return RC_INVALID_MESSAGE;
  }

  rtcm3_rc ret = RC_OK;
  switch (message_type) {
    case 999: {
      const rtcm_msg_999 *msg_999 = (const rtcm_msg_999 *)rtcm_msg;
      ret = rtcm3_encode_999_bitstream(buff, msg_999);
      break;
    }
    case 1001: {
      const rtcm_obs_message *msg_1001 = (const rtcm_obs_message *)rtcm_msg;
      ret = rtcm3_encode_1001_bitstream(buff, msg_1001);
      break;
    }
    case 1002: {
      const rtcm_obs_message *msg_1002 = (const rtcm_obs_message *)rtcm_msg;
      ret = rtcm3_encode_1002_bitstream(buff, msg_1002);
      break;
    }
    case 1003: {
      const rtcm_obs_message *msg_1003 = (const rtcm_obs_message *)rtcm_msg;
      ret = rtcm3_encode_1003_bitstream(buff, msg_1003);
      break;
    }
    case 1004: {
      const rtcm_obs_message *msg_1004 = (const rtcm_obs_message *)rtcm_msg;
      ret = rtcm3_encode_1004_bitstream(buff, msg_1004);
      break;
    }
    case 1005: {
      const rtcm_msg_1005 *msg_1005 = (const rtcm_msg_1005 *)rtcm_msg;
      ret = rtcm3_encode_1005_bitstream(buff, msg_1005);
      break;
    }
    case 1006: {
      const rtcm_msg_1006 *msg_1006 = (const rtcm_msg_1006 *)rtcm_msg;
      ret = rtcm3_encode_1006_bitstream(buff, msg_1006);
      break;
    }
    case 1007: {
      const rtcm_msg_1007 *msg_1007 = (const rtcm_msg_1007 *)rtcm_msg;
      ret = rtcm3_encode_1007_bitstream(buff, msg_1007);
      break;
    }
    case 1008: {
      const rtcm_msg_1008 *msg_1008 = (const rtcm_msg_1008 *)rtcm_msg;
      ret = rtcm3_encode_1008_bitstream(buff, msg_1008);
      break;
    }
    case 1010: {
      const rtcm_obs_message *msg_1010 = (const rtcm_obs_message *)rtcm_msg;
      ret = rtcm3_encode_1010_bitstream(buff, msg_1010);
      break;
    }
    case 1012: {
      const rtcm_obs_message *msg_1012 = (const rtcm_obs_message *)rtcm_msg;
      ret = rtcm3_encode_1012_bitstream(buff, msg_1012);
      break;
    }
    case 1013: {
      const rtcm_msg_1013 *msg_1013 = (const rtcm_msg_1013 *)rtcm_msg;
      ret = rtcm3_encode_1013_bitstream(buff, msg_1013);
      break;
    }
    case 1019: {
      const rtcm_msg_eph *msg_1019 = (const rtcm_msg_eph *)rtcm_msg;
      ret = rtcm3_encode_gps_eph_bitstream(buff, msg_1019);
      break;
    }
    case 1020: {
      const rtcm_msg_eph *msg_1020 = (const rtcm_msg_eph *)rtcm_msg;
      ret = rtcm3_encode_glo_eph_bitstream(buff, msg_1020);
      break;
    }
    case 1029: {
      const rtcm_msg_1029 *msg_1029 = (const rtcm_msg_1029 *)rtcm_msg;
      ret = rtcm3_encode_1029_bitstream(buff, msg_1029);
      break;
    }
    case 1033: {
      const rtcm_msg_1033 *msg_1033 = (const rtcm_msg_1033 *)rtcm_msg;
      ret = rtcm3_encode_1033_bitstream(buff, msg_1033);
      break;
    }
    case 1042: {
      const rtcm_msg_eph *msg_1042 = (const rtcm_msg_eph *)rtcm_msg;
      ret = rtcm3_encode_bds_eph_bitstream(buff, msg_1042);
      break;
    }
    case 1045: {
      const rtcm_msg_eph *msg_1045 = (const rtcm_msg_eph *)rtcm_msg;
      ret = rtcm3_encode_gal_eph_fnav_bitstream(buff, msg_1045);
      break;
    }
    case 1046: {
      const rtcm_msg_eph *msg_1046 = (const rtcm_msg_eph *)rtcm_msg;
      ret = rtcm3_encode_gal_eph_inav_bitstream(buff, msg_1046);
      break;
    }
    case 1230: {
      const rtcm_msg_1230 *msg_1230 = (const rtcm_msg_1230 *)rtcm_msg;
      ret = rtcm3_encode_1230_bitstream(buff, msg_1230);
      break;
    }
    case 1074:
    case 1084:
    case 1094:
    case 1114:
    case 1124: {
      const rtcm_msm_message *msg_msm = (const rtcm_msm_message *)rtcm_msg;
      ret = rtcm3_encode_msm4_bitstream(buff, msg_msm);
      break;
    }
    case 1075:
    case 1085:
    case 1095:
    case 1115:
    case 1125: {
      const rtcm_msm_message *msg_msm = (const rtcm_msm_message *)rtcm_msg;
      ret = rtcm3_encode_msm5_bitstream(buff, msg_msm);
      break;
    }
    case 4062: {
      const rtcm_msg_swift_proprietary *swift_msg =
          (const rtcm_msg_swift_proprietary *)rtcm_msg;
      ret = rtcm3_encode_4062_bitstream(buff, swift_msg);
      break;
    }
    default:
      log_error("%d is not a supported message", message_type);
      ret = RC_MESSAGE_TYPE_MISMATCH;
      break;
  }

  return ret;
}

rtcm3_rc rtcm3_encode_frame_bitstream(swiftnav_out_bitstream_t *buff,
                                      const void *rtcm_msg,
                                      uint16_t message_type) {
  assert(buff);
  assert((buff->offset % 8) == 0);  // frame start at full byte

  if ((buff->len - buff->offset) <= (RTCM3_MSG_OVERHEAD * 8)) {
    return RC_INVALID_MESSAGE;
  }

  uint32_t buf_offset_start = buff->offset;
  buff->offset += 24;  // 3 bytes
  rtcm3_rc ret = rtcm3_encode_payload_bitstream(buff, rtcm_msg, message_type);
  if (RC_OK != ret) {
    log_info("Error encoding RTCM message %u", message_type);
    return ret;
  }

  uint16_t message_size =
      (uint16_t)(((buff->offset - buf_offset_start) + 7) / 8) - 3;
  assert(message_size <= RTCM3_MAX_MSG_LEN);

  buff->offset = buf_offset_start;
  BITSTREAM_ENCODE_U8(
      buff, (uint8_t)(RTCM3_PREAMBLE), 8);      /* write the header */
  BITSTREAM_ENCODE_U32(buff, 0, 6);             /* reserved bits */
  BITSTREAM_ENCODE_U16(buff, message_size, 10); /* message size */
  // Already encode payload

  // Round to nearest byte
  buff->offset = (message_size + 3) * 8 + buf_offset_start;
  uint32_t crc =
      crc24q(buff->data + (buf_offset_start + 7) / 8, message_size + 3, 0);
  BITSTREAM_ENCODE_U32(buff, crc, 24);

  return RC_OK;
}
