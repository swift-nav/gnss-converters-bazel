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
#include <librtcm/internal/decode_helpers.h>
#include <math.h>
#include <rtcm3/bits.h>
#include <rtcm3/constants.h>
#include <rtcm3/decode.h>
#include <rtcm3/eph_decode.h>
#include <rtcm3/librtcm_utils.h>
#include <rtcm3/ssr_decode.h>
#include <rtcm3/sta_decode.h>
#include <stdio.h>
#include <string.h>

static void init_sat_data(rtcm_sat_data *sat_data) {
  for (uint8_t freq = 0; freq < NUM_FREQS; ++freq) {
    sat_data->obs[freq].flags.data = 0;
  }
}

/* Convert the 7-bit Lock Time Indicator (DF013, DF019, DF043, DF049) into
 * integer seconds */
/* RTCM 10403.3 Table 3.4-2 */
static uint32_t from_lock_ind(uint8_t lock) {
  if (lock < 24) {
    return lock;
  }
  if (lock < 48) {
    return 2 * lock - 24;
  }
  if (lock < 72) {
    return 4 * lock - 120;
  }
  if (lock < 96) {
    return 8 * lock - 408;
  }
  if (lock < 120) {
    return 16 * lock - 1176;
  }
  if (lock < 127) {
    return 32 * lock - 3096;
  }
  return 937;
}

/* Convert the 4-bit Lock Time Indicator DF402 into seconds. */
/* RTCM 10403.3 Table 3.5-74 */
double rtcm3_decode_lock_time(uint8_t lock) {
  /* Discard the MSB nibble */
  lock &= 0x0F;

  if (lock == 0) {
    return 0;
  }
  return (double)(32 << (lock - 1)) / 1000;
}

/* Convert the Extended Lock Time Indicator DF407 into milliseconds. */
/* RTCM 10403.3 Table 3.5-75 */
static uint32_t from_msm_lock_ind_ext(uint16_t lock) {
  if (lock < 64) {
    return lock;
  }
  if (lock < 96) {
    return 2 * lock - 64;
  }
  if (lock < 128) {
    return 4 * lock - 256;
  }
  if (lock < 160) {
    return 8 * lock - 768;
  }
  if (lock < 192) {
    return 16 * lock - 2048;
  }
  if (lock < 224) {
    return 32 * lock - 5120;
  }
  if (lock < 256) {
    return 64 * lock - 12288;
  }
  if (lock < 288) {
    return 128 * lock - 28672;
  }
  if (lock < 320) {
    return 256 * lock - 65536;
  }
  if (lock < 352) {
    return 512 * lock - 147456;
  }
  if (lock < 384) {
    return 1024 * lock - 327680;
  }
  if (lock < 416) {
    return 2048 * lock - 720896;
  }
  if (lock < 448) {
    return 4096 * lock - 1572864;
  }
  if (lock < 480) {
    return 8192 * lock - 3407872;
  }
  if (lock < 512) {
    return 16384 * lock - 7340032;
  }
  if (lock < 544) {
    return 32768 * lock - 15728640;
  }
  if (lock < 576) {
    return 65536 * lock - 33554432;
  }
  if (lock < 608) {
    return 131072 * lock - 71303168;
  }
  if (lock < 640) {
    return 262144 * lock - 150994944;
  }
  if (lock < 672) {
    return 524288 * lock - 318767104;
  }
  if (lock < 704) {
    return 1048576 * lock - 671088640;
  }
  return 67108864;
}

static rtcm3_rc decode_basic_gps_l1_freq_data(swiftnav_in_bitstream_t *buff,
                                              rtcm_freq_data *freq_data,
                                              uint32_t *pr,
                                              int32_t *phr_pr_diff) {
  BITSTREAM_DECODE_U8(buff, freq_data->code, 1);
  BITSTREAM_DECODE_U32(buff, *pr, 24);
  BITSTREAM_DECODE_S32(buff, *phr_pr_diff, 20);

  uint8_t lock_ind;
  BITSTREAM_DECODE_U8(buff, lock_ind, 7);
  freq_data->lock = from_lock_ind(lock_ind);
  return RC_OK;
}

static rtcm3_rc decode_basic_glo_l1_freq_data(swiftnav_in_bitstream_t *buff,
                                              rtcm_freq_data *freq_data,
                                              uint32_t *pr,
                                              int32_t *phr_pr_diff,
                                              uint8_t *fcn) {
  BITSTREAM_DECODE_U8(buff, freq_data->code, 1);
  BITSTREAM_DECODE_U8(buff, *fcn, 5);
  BITSTREAM_DECODE_U32(buff, *pr, 25);
  BITSTREAM_DECODE_S32(buff, *phr_pr_diff, 20);
  uint8_t lock_ind;
  BITSTREAM_DECODE_U8(buff, lock_ind, 7);
  freq_data->lock = from_lock_ind(lock_ind);
  return RC_OK;
}

static rtcm3_rc decode_basic_l2_freq_data(swiftnav_in_bitstream_t *buff,
                                          rtcm_freq_data *freq_data,
                                          int32_t *pr,
                                          int32_t *phr_pr_diff) {
  BITSTREAM_DECODE_U8(buff, freq_data->code, 2);
  BITSTREAM_DECODE_S32(buff, *pr, 14);
  BITSTREAM_DECODE_S32(buff, *phr_pr_diff, 20);

  uint32_t lock_ind;
  BITSTREAM_DECODE_U32(buff, lock_ind, 7);
  freq_data->lock = from_lock_ind(lock_ind);
  return RC_OK;
}

static rtcm3_rc rtcm3_read_header(swiftnav_in_bitstream_t *buff,
                                  rtcm_obs_header *header) {
  BITSTREAM_DECODE_U16(buff, header->msg_num, 12);
  BITSTREAM_DECODE_U16(buff, header->stn_id, 12);
  BITSTREAM_DECODE_U32(buff, header->tow_ms, 30);
  BITSTREAM_DECODE_U8(buff, header->sync, 1);
  BITSTREAM_DECODE_U8(buff, header->n_sat, 5);
  BITSTREAM_DECODE_U8(buff, header->div_free, 1);
  BITSTREAM_DECODE_U8(buff, header->smooth, 3);
  return RC_OK;
}

static rtcm3_rc rtcm3_read_glo_header(swiftnav_in_bitstream_t *buff,
                                      rtcm_obs_header *header) {
  BITSTREAM_DECODE_U16(buff, header->msg_num, 12);
  BITSTREAM_DECODE_U16(buff, header->stn_id, 12);
  BITSTREAM_DECODE_U32(buff, header->tow_ms, 27);
  BITSTREAM_DECODE_U8(buff, header->sync, 1);
  BITSTREAM_DECODE_U8(buff, header->n_sat, 5);
  BITSTREAM_DECODE_U8(buff, header->div_free, 1);
  BITSTREAM_DECODE_U8(buff, header->smooth, 3);
  return RC_OK;
}

/* unwrap underflowed uint30 value to a wrapped tow_ms value */
static uint32_t normalize_bds2_tow(const uint32_t tow_ms) {
  if (tow_ms >= C_2P30 - BDS_SECOND_TO_GPS_SECOND * 1000) {
    uint32_t negative_tow_ms = C_2P30 - tow_ms;
    return RTCM_MAX_TOW_MS + 1 - negative_tow_ms;
  }
  return tow_ms;
}

static rtcm3_rc rtcm3_read_msm_header(swiftnav_in_bitstream_t *buff,
                                      const rtcm_constellation_t cons,
                                      rtcm_msm_header *header) {
  BITSTREAM_DECODE_U16(buff, header->stn_id, 12);
  if (RTCM_CONSTELLATION_GLO == cons) {
    /* skip the day of week, it is handled in gnss_converters */
    swiftnav_in_bitstream_remove(buff, 3);
    /* for GLONASS, the epoch time is the time of day in ms */
    BITSTREAM_DECODE_U32(buff, header->tow_ms, 27);
  } else if (RTCM_CONSTELLATION_BDS == cons) {
    /* Beidou time can be negative (at least for some Septentrio base stations),
     * so normalize it first */
    uint32_t bds2_tow;
    BITSTREAM_DECODE_U32(buff, bds2_tow, 30);
    header->tow_ms = normalize_bds2_tow(bds2_tow);
  } else {
    /* for other systems, epoch time is the time of week in ms */
    BITSTREAM_DECODE_U32(buff, header->tow_ms, 30);
  }
  BITSTREAM_DECODE_U8(buff, header->multiple, 1);
  BITSTREAM_DECODE_U8(buff, header->iods, 3);
  BITSTREAM_DECODE_U8(buff, header->reserved, 7);
  BITSTREAM_DECODE_U8(buff, header->steering, 2);
  BITSTREAM_DECODE_U8(buff, header->ext_clock, 2);
  BITSTREAM_DECODE_U8(buff, header->div_free, 1);
  BITSTREAM_DECODE_U8(buff, header->smooth, 3);

  for (uint8_t i = 0; i < MSM_SATELLITE_MASK_SIZE; i++) {
    BITSTREAM_DECODE_BOOL(buff, header->satellite_mask[i], 1);
  }
  for (uint8_t i = 0; i < MSM_SIGNAL_MASK_SIZE; i++) {
    BITSTREAM_DECODE_BOOL(buff, header->signal_mask[i], 1);
  }
  uint8_t num_sats =
      count_mask_values(MSM_SATELLITE_MASK_SIZE, header->satellite_mask);
  uint8_t num_sigs =
      count_mask_values(MSM_SIGNAL_MASK_SIZE, header->signal_mask);
  uint8_t cell_mask_size = num_sats * num_sigs;

  for (uint8_t i = 0; i < cell_mask_size; i++) {
    BITSTREAM_DECODE_BOOL(buff, header->cell_mask[i], 1);
  }
  return RC_OK;
}

static uint8_t construct_L1_code(rtcm_freq_data *l1_freq_data,
                                 int32_t pr,
                                 double amb_correction) {
  l1_freq_data->pseudorange = 0.02 * pr + amb_correction;
  if (pr != (int)PR_L1_INVALID) {
    return 1;
  }
  return 0;
}

static uint8_t construct_L1_phase(rtcm_freq_data *l1_freq_data,
                                  int32_t phr_pr_diff,
                                  double freq) {
  l1_freq_data->carrier_phase =
      (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) / (GPS_C / freq);
  if (phr_pr_diff != (int)CP_INVALID) {
    return 1;
  }
  return 0;
}

static uint8_t construct_L2_code(rtcm_freq_data *l2_freq_data,
                                 const rtcm_freq_data *l1_freq_data,
                                 int32_t pr) {
  l2_freq_data->pseudorange = 0.02 * pr + l1_freq_data->pseudorange;
  if (pr != (int)PR_L2_INVALID) {
    return 1;
  }
  return 0;
}

static uint8_t construct_L2_phase(rtcm_freq_data *l2_freq_data,
                                  const rtcm_freq_data *l1_freq_data,
                                  int32_t phr_pr_diff,
                                  double freq) {
  l2_freq_data->carrier_phase =
      (l1_freq_data->pseudorange + 0.0005 * phr_pr_diff) / (GPS_C / freq);
  if (phr_pr_diff != (int)CP_INVALID) {
    return 1;
  }
  return 0;
}

static rtcm3_rc get_cnr(rtcm_freq_data *freq_data,
                        swiftnav_in_bitstream_t *buff) {
  uint32_t cnr;
  BITSTREAM_DECODE_U32(buff, cnr, 8);
  if (cnr == 0) {
    freq_data->flags.fields.valid_cnr = 0;
    return RC_OK;
  }
  freq_data->cnr = 0.25 * cnr;
  freq_data->flags.fields.valid_cnr = 1;
  return RC_OK;
}

/** Decode an RTCMv3 message type 1001 (L1-Only GPS RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : TOW sanity check fail
 */
rtcm3_rc rtcm3_decode_1001_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1001) {
  assert(msg_1001);
  rtcm3_rc ret = rtcm3_read_header(buff, &msg_1001->header);
  if (ret != RC_OK) {
    return ret;
  }

  if (msg_1001->header.msg_num != 1001) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_1001->header.tow_ms > RTCM_MAX_TOW_MS) {
    return RC_INVALID_MESSAGE;
  }

  for (uint8_t i = 0; i < msg_1001->header.n_sat; i++) {
    init_sat_data(&msg_1001->sats[i]);

    BITSTREAM_DECODE_U8(buff, msg_1001->sats[i].svId, 6);

    rtcm_freq_data *l1_freq_data = &msg_1001->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t phr_pr_diff;
    ret =
        decode_basic_gps_l1_freq_data(buff, l1_freq_data, &l1_pr, &phr_pr_diff);
    if (ret != RC_OK) {
      return ret;
    }

    l1_freq_data->flags.fields.valid_pr =
        construct_L1_code(l1_freq_data, l1_pr, 0);
    l1_freq_data->flags.fields.valid_cp =
        construct_L1_phase(l1_freq_data, phr_pr_diff, GPS_L1_HZ);
    l1_freq_data->flags.fields.valid_lock = l1_freq_data->flags.fields.valid_cp;
  }

  return RC_OK;
}

/** Decode an RTCMv3 message type 1002 (Extended L1-Only GPS RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : TOW sanity check fail
 */
rtcm3_rc rtcm3_decode_1002_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1002) {
  assert(msg_1002);
  rtcm3_rc ret = rtcm3_read_header(buff, &msg_1002->header);
  if (ret != RC_OK) {
    return ret;
  }

  if (msg_1002->header.msg_num != 1002) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_1002->header.tow_ms > RTCM_MAX_TOW_MS) {
    return RC_INVALID_MESSAGE;
  }

  for (uint8_t i = 0; i < msg_1002->header.n_sat; i++) {
    init_sat_data(&msg_1002->sats[i]);

    BITSTREAM_DECODE_U8(buff, msg_1002->sats[i].svId, 6);

    rtcm_freq_data *l1_freq_data = &msg_1002->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t phr_pr_diff;
    ret =
        decode_basic_gps_l1_freq_data(buff, l1_freq_data, &l1_pr, &phr_pr_diff);
    if (ret != RC_OK) {
      return ret;
    }

    uint8_t amb;
    BITSTREAM_DECODE_U8(buff, amb, 8);
    ret = get_cnr(l1_freq_data, buff);
    if (ret != RC_OK) {
      return ret;
    }
    l1_freq_data->flags.fields.valid_pr =
        construct_L1_code(l1_freq_data, l1_pr, amb * PRUNIT_GPS);
    l1_freq_data->flags.fields.valid_cp =
        construct_L1_phase(l1_freq_data, phr_pr_diff, GPS_L1_HZ);
    l1_freq_data->flags.fields.valid_lock = l1_freq_data->flags.fields.valid_cp;
  }

  return RC_OK;
}

/** Decode an RTCMv3 message type 1003 (L1/L2 GPS RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : TOW sanity check fail
 */
rtcm3_rc rtcm3_decode_1003_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1003) {
  assert(msg_1003);
  rtcm3_rc ret = rtcm3_read_header(buff, &msg_1003->header);
  if (ret != RC_OK) {
    return ret;
  }

  if (msg_1003->header.msg_num != 1003) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_1003->header.tow_ms > RTCM_MAX_TOW_MS) {
    return RC_INVALID_MESSAGE;
  }

  for (uint8_t i = 0; i < msg_1003->header.n_sat; i++) {
    init_sat_data(&msg_1003->sats[i]);

    BITSTREAM_DECODE_U8(buff, msg_1003->sats[i].svId, 6);

    rtcm_freq_data *l1_freq_data = &msg_1003->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t l2_pr;
    int32_t phr_pr_diff;
    ret =
        decode_basic_gps_l1_freq_data(buff, l1_freq_data, &l1_pr, &phr_pr_diff);
    if (ret != RC_OK) {
      return ret;
    }

    l1_freq_data->flags.fields.valid_pr =
        construct_L1_code(l1_freq_data, l1_pr, 0);
    l1_freq_data->flags.fields.valid_cp =
        construct_L1_phase(l1_freq_data, phr_pr_diff, GPS_L1_HZ);
    l1_freq_data->flags.fields.valid_lock = l1_freq_data->flags.fields.valid_cp;

    rtcm_freq_data *l2_freq_data = &msg_1003->sats[i].obs[L2_FREQ];

    ret = decode_basic_l2_freq_data(buff, l2_freq_data, &l2_pr, &phr_pr_diff);
    if (ret != RC_OK) {
      return ret;
    }

    l2_freq_data->flags.fields.valid_pr =
        construct_L2_code(l2_freq_data, l1_freq_data, l2_pr);
    l2_freq_data->flags.fields.valid_cp =
        construct_L2_phase(l2_freq_data, l1_freq_data, phr_pr_diff, GPS_L2_HZ);
    l2_freq_data->flags.fields.valid_lock = l2_freq_data->flags.fields.valid_cp;
  }

  return RC_OK;
}

/** Decode an RTCMv3 message type 1004 (Extended L1/L2 GPS RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : TOW sanity check fail
 */
rtcm3_rc rtcm3_decode_1004_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1004) {
  assert(msg_1004);
  rtcm3_rc ret = rtcm3_read_header(buff, &msg_1004->header);
  if (ret != RC_OK) {
    return ret;
  }

  if (msg_1004->header.msg_num != 1004) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_1004->header.tow_ms > RTCM_MAX_TOW_MS) {
    return RC_INVALID_MESSAGE;
  }

  for (uint8_t i = 0; i < msg_1004->header.n_sat; i++) {
    init_sat_data(&msg_1004->sats[i]);

    BITSTREAM_DECODE_U8(buff, msg_1004->sats[i].svId, 6);

    rtcm_freq_data *l1_freq_data = &msg_1004->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t l2_pr;
    int32_t phr_pr_diff;
    ret =
        decode_basic_gps_l1_freq_data(buff, l1_freq_data, &l1_pr, &phr_pr_diff);
    if (ret != RC_OK) {
      return ret;
    }

    uint32_t amb;
    BITSTREAM_DECODE_U32(buff, amb, 8);

    ret = get_cnr(l1_freq_data, buff);
    if (ret != RC_OK) {
      return ret;
    }
    l1_freq_data->flags.fields.valid_pr =
        construct_L1_code(l1_freq_data, l1_pr, amb * PRUNIT_GPS);
    l1_freq_data->flags.fields.valid_cp =
        construct_L1_phase(l1_freq_data, phr_pr_diff, GPS_L1_HZ);
    l1_freq_data->flags.fields.valid_lock = l1_freq_data->flags.fields.valid_cp;

    rtcm_freq_data *l2_freq_data = &msg_1004->sats[i].obs[L2_FREQ];

    ret = decode_basic_l2_freq_data(buff, l2_freq_data, &l2_pr, &phr_pr_diff);
    if (ret != RC_OK) {
      return ret;
    }

    ret = get_cnr(l2_freq_data, buff);
    if (ret != RC_OK) {
      return ret;
    }
    l2_freq_data->flags.fields.valid_pr =
        construct_L2_code(l2_freq_data, l1_freq_data, l2_pr);
    l2_freq_data->flags.fields.valid_cp =
        construct_L2_phase(l2_freq_data, l1_freq_data, phr_pr_diff, GPS_L2_HZ);
    l2_freq_data->flags.fields.valid_lock = l2_freq_data->flags.fields.valid_cp;
  }

  return RC_OK;
}

static rtcm3_rc rtcm3_decode_1005_base(swiftnav_in_bitstream_t *buff,
                                       rtcm_msg_1005 *msg_1005) {
  BITSTREAM_DECODE_U16(buff, msg_1005->stn_id, 12);
  BITSTREAM_DECODE_U8(buff, msg_1005->ITRF, 6);
  BITSTREAM_DECODE_U8(buff, msg_1005->GPS_ind, 1);
  BITSTREAM_DECODE_U8(buff, msg_1005->GLO_ind, 1);
  BITSTREAM_DECODE_U8(buff, msg_1005->GAL_ind, 1);
  BITSTREAM_DECODE_U8(buff, msg_1005->ref_stn_ind, 1);
  s64 tmp;
  BITSTREAM_DECODE_S64(buff, tmp, 38);
  msg_1005->arp_x = (double)(tmp) / 10000.0;
  BITSTREAM_DECODE_U8(buff, msg_1005->osc_ind, 1);
  swiftnav_in_bitstream_remove(buff, 1);
  BITSTREAM_DECODE_S64(buff, tmp, 38);
  msg_1005->arp_y = (double)(tmp) / 10000.0;
  BITSTREAM_DECODE_U8(buff, msg_1005->quart_cycle_ind, 2);
  BITSTREAM_DECODE_S64(buff, tmp, 38);
  msg_1005->arp_z = (double)(tmp) / 10000.0;

  return RC_OK;
}

/** Decode an RTCMv3 message type 1005 (Stationary RTK Reference Station ARP)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
rtcm3_rc rtcm3_decode_1005_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1005 *msg_1005) {
  assert(msg_1005);
  uint32_t msg_num;
  BITSTREAM_DECODE_U32(buff, msg_num, 12);

  if (msg_num != 1005) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  return rtcm3_decode_1005_base(buff, msg_1005);
}

/** Decode an RTCMv3 message type 1005 (Stationary RTK Reference Station ARP
 * with antenna height)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
rtcm3_rc rtcm3_decode_1006_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1006 *msg_1006) {
  assert(msg_1006);
  uint32_t msg_num;
  BITSTREAM_DECODE_U32(buff, msg_num, 12);

  if (msg_num != 1006) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  rtcm3_decode_1005_base(buff, &msg_1006->msg_1005);
  uint32_t tmp;
  BITSTREAM_DECODE_U32(buff, tmp, 16);
  msg_1006->ant_height = (double)(tmp) / 10000.0;
  return RC_OK;
}

static rtcm3_rc decode_string(swiftnav_in_bitstream_t *buff,
                              u8 *len,
                              char *str) {
  BITSTREAM_DECODE_U8(buff, *len, 8);
  if (*len > RTCM_MAX_STRING_LEN) {
    return RC_INVALID_MESSAGE;
  }
  for (u32 i = 0; i < *len; i++) {
    BITSTREAM_DECODE_U8(buff, str[i], 8);
  }
  return RC_OK;
}

static rtcm3_rc rtcm3_decode_1007_base(swiftnav_in_bitstream_t *buff,
                                       rtcm_msg_1007 *msg_1007) {
  BITSTREAM_DECODE_U16(buff, msg_1007->stn_id, 12);
  if (decode_string(buff,
                    &msg_1007->ant_descriptor_counter,
                    msg_1007->ant_descriptor) != RC_OK) {
    return RC_INVALID_MESSAGE;
  }
  BITSTREAM_DECODE_U8(buff, msg_1007->ant_setup_id, 8);

  return RC_OK;
}

/** Decode an RTCMv3 message type 1007 (Antenna Descriptor)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : String length too large
 *
 */
rtcm3_rc rtcm3_decode_1007_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1007 *msg_1007) {
  assert(msg_1007);
  uint32_t msg_num;
  BITSTREAM_DECODE_U32(buff, msg_num, 12);

  if (msg_num != 1007) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  return rtcm3_decode_1007_base(buff, msg_1007);
}

/** Decode an RTCMv3 message type 1008 (Antenna Descriptor & Serial Number)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : String length too large
 */
rtcm3_rc rtcm3_decode_1008_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1008 *msg_1008) {
  assert(msg_1008);
  uint32_t msg_num;
  BITSTREAM_DECODE_U32(buff, msg_num, 12);

  if (msg_num != 1008) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  rtcm3_rc ret = rtcm3_decode_1007_base(buff, &msg_1008->msg_1007);
  if (RC_OK != ret) {
    return ret;
  }

  return decode_string(
      buff, &msg_1008->ant_serial_num_counter, msg_1008->ant_serial_num);
}

/** Decode an RTCMv3 message type 1010 (Extended L1-Only GLO RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : TOW sanity check fail
 */
rtcm3_rc rtcm3_decode_1010_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1010) {
  assert(msg_1010);
  rtcm3_rc ret = rtcm3_read_glo_header(buff, &msg_1010->header);
  if (ret != RC_OK) {
    return ret;
  }

  if (msg_1010->header.msg_num != 1010) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_1010->header.tow_ms > RTCM_GLO_MAX_TOW_MS) {
    return RC_INVALID_MESSAGE;
  }

  for (uint8_t i = 0; i < msg_1010->header.n_sat; i++) {
    init_sat_data(&msg_1010->sats[i]);

    BITSTREAM_DECODE_U8(buff, msg_1010->sats[i].svId, 6);

    rtcm_freq_data *l1_freq_data = &msg_1010->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t phr_pr_diff;
    ret = decode_basic_glo_l1_freq_data(
        buff, l1_freq_data, &l1_pr, &phr_pr_diff, &msg_1010->sats[i].fcn);
    if (ret != RC_OK) {
      return ret;
    }

    uint8_t amb;
    BITSTREAM_DECODE_U8(buff, amb, 7);

    ret = get_cnr(l1_freq_data, buff);
    if (ret != RC_OK) {
      return ret;
    }

    int8_t glo_fcn = msg_1010->sats[i].fcn - MT1012_GLO_FCN_OFFSET;
    l1_freq_data->flags.fields.valid_pr =
        construct_L1_code(l1_freq_data, l1_pr, PRUNIT_GLO * amb);
    l1_freq_data->flags.fields.valid_cp =
        (msg_1010->sats[i].fcn <= MT1012_GLO_MAX_FCN) &&
        construct_L1_phase(
            l1_freq_data, phr_pr_diff, GLO_L1_HZ + glo_fcn * GLO_L1_DELTA_HZ);
    l1_freq_data->flags.fields.valid_lock = l1_freq_data->flags.fields.valid_cp;
  }

  return RC_OK;
}

/** Decode an RTCMv3 message type 1012 (Extended L1/L2 GLO RTK Observables)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : TOW sanity check fail
 */
rtcm3_rc rtcm3_decode_1012_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_obs_message *msg_1012) {
  assert(msg_1012);
  rtcm3_rc ret = rtcm3_read_glo_header(buff, &msg_1012->header);
  if (ret != RC_OK) {
    return ret;
  }

  if (msg_1012->header.msg_num != 1012) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (msg_1012->header.tow_ms > RTCM_GLO_MAX_TOW_MS) {
    return RC_INVALID_MESSAGE;
  }

  for (uint8_t i = 0; i < msg_1012->header.n_sat; i++) {
    init_sat_data(&msg_1012->sats[i]);

    BITSTREAM_DECODE_U8(buff, msg_1012->sats[i].svId, 6);

    rtcm_freq_data *l1_freq_data = &msg_1012->sats[i].obs[L1_FREQ];

    uint32_t l1_pr;
    int32_t l2_pr;
    int32_t phr_pr_diff;
    ret = decode_basic_glo_l1_freq_data(
        buff, l1_freq_data, &l1_pr, &phr_pr_diff, &msg_1012->sats[i].fcn);
    if (ret != RC_OK) {
      return ret;
    }

    uint32_t amb;
    BITSTREAM_DECODE_U32(buff, amb, 7);

    int8_t glo_fcn = msg_1012->sats[i].fcn - MT1012_GLO_FCN_OFFSET;
    ret = get_cnr(l1_freq_data, buff);
    if (ret != RC_OK) {
      return ret;
    }
    l1_freq_data->flags.fields.valid_pr =
        construct_L1_code(l1_freq_data, l1_pr, amb * PRUNIT_GLO);
    l1_freq_data->flags.fields.valid_cp =
        (msg_1012->sats[i].fcn <= MT1012_GLO_MAX_FCN) &&
        construct_L1_phase(
            l1_freq_data, phr_pr_diff, GLO_L1_HZ + glo_fcn * GLO_L1_DELTA_HZ);
    l1_freq_data->flags.fields.valid_lock = l1_freq_data->flags.fields.valid_cp;

    rtcm_freq_data *l2_freq_data = &msg_1012->sats[i].obs[L2_FREQ];

    ret = decode_basic_l2_freq_data(buff, l2_freq_data, &l2_pr, &phr_pr_diff);
    if (ret != RC_OK) {
      return ret;
    }

    ret = get_cnr(l2_freq_data, buff);
    if (ret != RC_OK) {
      return ret;
    }
    l2_freq_data->flags.fields.valid_pr =
        construct_L2_code(l2_freq_data, l1_freq_data, l2_pr);
    l2_freq_data->flags.fields.valid_cp =
        construct_L2_phase(l2_freq_data,
                           l1_freq_data,
                           phr_pr_diff,
                           GLO_L2_HZ + glo_fcn * GLO_L2_DELTA_HZ);
    l2_freq_data->flags.fields.valid_lock = l2_freq_data->flags.fields.valid_cp;
  }

  return RC_OK;
}

/**
 * Decode a RTCMv3 message type 1013 (System Parameters)
 *
 * @param buffer input data buffer
 * @param msg_1013 message struct
 * @param message_length length of the RTCM message (not the RTCM frame) in
 * bytes (valid range is [9, 1023])
 * @return - RC_OK : Success
 *         - RC_MESSAGE_TYPE_MISMATCH : Message type mismatched
 *         - RC_INVALID_MESSAGE : Buffer overflow would have taken place
 */
rtcm3_rc rtcm3_decode_1013_bitstream(swiftnav_in_bitstream_t *buffer,
                                     rtcm_msg_1013 *msg_1013,
                                     size_t message_length) {
  assert(msg_1013);
  uint16_t message_number;

  if (message_length > RTCM3_MAX_MSG_LEN ||
      8 * message_length < RTCM_1013_MIN_MSG_LEN_BITS) {
    return RC_INVALID_MESSAGE;
  }

  BITSTREAM_DECODE_U16(buffer, message_number, 12);
  if (message_number != 1013) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BITSTREAM_DECODE_U16(buffer, msg_1013->reference_station_id, 12);
  BITSTREAM_DECODE_U16(buffer, msg_1013->mjd, 16);
  BITSTREAM_DECODE_U32(buffer, msg_1013->utc, 17);
  BITSTREAM_DECODE_U16(buffer, msg_1013->message_count, 5);

  // based on the specs for DF053, if the number of message IDs is zero, it can
  // mean that either there are zero messages provided OR there are more than
  // 31 messages (limit of the field can hold) at which point the actual size is
  // based on the RTCM message length
  if (msg_1013->message_count == 0) {
    msg_1013->message_count =
        (8 * message_length - RTCM_1013_MIN_MSG_LEN_BITS) /
        RTCM_1013_MESSAGE_SIZE_BITS;
  }

  BITSTREAM_DECODE_U8(buffer, msg_1013->leap_second, 8);
  for (size_t i = 0; i < msg_1013->message_count; ++i) {
    BITSTREAM_DECODE_U16(buffer, msg_1013->messages[i].id, 12);
    BITSTREAM_DECODE_U8(buffer, msg_1013->messages[i].sync_flag, 1);
    BITSTREAM_DECODE_U16(
        buffer, msg_1013->messages[i].transmission_interval, 16);
  }

  return RC_OK;
}

/** Decode an RTCMv3 message type 1029 (Unicode Text String Message)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
rtcm3_rc rtcm3_decode_1029_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1029 *msg_1029) {
  assert(msg_1029);
  uint32_t msg_num;
  BITSTREAM_DECODE_U32(buff, msg_num, 12);

  if (msg_num != 1029) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BITSTREAM_DECODE_U16(buff, msg_1029->stn_id, 12);

  BITSTREAM_DECODE_U16(buff, msg_1029->mjd_num, 16);

  BITSTREAM_DECODE_U32(buff, msg_1029->utc_sec_of_day, 17);

  BITSTREAM_DECODE_U8(buff, msg_1029->unicode_chars, 7);

  BITSTREAM_DECODE_U8(buff, msg_1029->utf8_code_units_n, 8);
  for (uint8_t i = 0; i < msg_1029->utf8_code_units_n; ++i) {
    BITSTREAM_DECODE_U8(buff, msg_1029->utf8_code_units[i], 8);
  }

  return RC_OK;
}

/** Decode an RTCMv3 message type 1033 (Rcv and Ant descriptor)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : String length too large
 */
rtcm3_rc rtcm3_decode_1033_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1033 *msg_1033) {
  assert(msg_1033);
  uint32_t msg_num;
  BITSTREAM_DECODE_U32(buff, msg_num, 12);

  if (msg_num != 1033) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  /* make sure all the strings gets initialized */
  memset(msg_1033, 0, sizeof(*msg_1033));

  BITSTREAM_DECODE_U16(buff, msg_1033->stn_id, 12);

  rtcm3_rc ret = decode_string(
      buff, &msg_1033->ant_descriptor_counter, msg_1033->ant_descriptor);
  if (ret != RC_OK) {
    return ret;
  }

  BITSTREAM_DECODE_U8(buff, msg_1033->ant_setup_id, 8);

  ret = decode_string(
      buff, &msg_1033->ant_serial_num_counter, msg_1033->ant_serial_num);
  if (ret != RC_OK) {
    return ret;
  }

  ret = decode_string(
      buff, &msg_1033->rcv_descriptor_counter, msg_1033->rcv_descriptor);
  if (ret != RC_OK) {
    return ret;
  }

  ret = decode_string(
      buff, &msg_1033->rcv_fw_version_counter, msg_1033->rcv_fw_version);
  if (ret != RC_OK) {
    return ret;
  }

  ret = decode_string(
      buff, &msg_1033->rcv_serial_num_counter, msg_1033->rcv_serial_num);
  if (ret != RC_OK) {
    return ret;
  }

  return RC_OK;
}

/** Decode an RTCMv3 message type 1230 (Code-Phase Bias Message)
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 */
rtcm3_rc rtcm3_decode_1230_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_1230 *msg_1230) {
  assert(msg_1230);
  uint32_t msg_num;
  BITSTREAM_DECODE_U32(buff, msg_num, 12);

  if (msg_num != 1230) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BITSTREAM_DECODE_U16(buff, msg_1230->stn_id, 12);
  BITSTREAM_DECODE_U8(buff, msg_1230->bias_indicator, 1);
  /* 3 Reserved bits */
  swiftnav_in_bitstream_remove(buff, 3);
  BITSTREAM_DECODE_U8(buff, msg_1230->fdma_signal_mask, 4);
  if (msg_1230->fdma_signal_mask & 0x08) {
    s32 tmp;
    BITSTREAM_DECODE_S32(buff, tmp, 16);
    msg_1230->L1_CA_cpb_meter = tmp * 0.02;
  } else {
    msg_1230->L1_CA_cpb_meter = 0.0;
  }
  if (msg_1230->fdma_signal_mask & 0x04) {
    s32 tmp;
    BITSTREAM_DECODE_S32(buff, tmp, 16);
    msg_1230->L1_P_cpb_meter = tmp * 0.02;
  } else {
    msg_1230->L1_P_cpb_meter = 0.0;
  }
  if (msg_1230->fdma_signal_mask & 0x02) {
    s32 tmp;
    BITSTREAM_DECODE_S32(buff, tmp, 16);
    msg_1230->L2_CA_cpb_meter = tmp * 0.02;
  } else {
    msg_1230->L2_CA_cpb_meter = 0.0;
  }
  if (msg_1230->fdma_signal_mask & 0x01) {
    s32 tmp;
    BITSTREAM_DECODE_S32(buff, tmp, 16);
    msg_1230->L2_P_cpb_meter = tmp * 0.02;
  } else {
    msg_1230->L2_P_cpb_meter = 0.0;
  }

  return RC_OK;
}

static rtcm3_rc decode_msm_sat_data(swiftnav_in_bitstream_t *buff,
                                    const uint8_t num_sats,
                                    const msm_enum msm_type,
                                    double rough_range_ms[],
                                    bool rough_range_valid[],
                                    uint8_t sat_info[],
                                    bool sat_info_valid[],
                                    double rough_rate_m_s[],
                                    bool rough_rate_valid[]) {
  /* number of integer milliseconds, DF397 */
  for (uint8_t i = 0; i < num_sats; i++) {
    uint32_t range_ms;
    BITSTREAM_DECODE_U32(buff, range_ms, 8);
    rough_range_ms[i] = range_ms;
    rough_range_valid[i] = (MSM_ROUGH_RANGE_INVALID != range_ms);
  }

  /* satellite info (constellation-dependent, currently only GLO uses this to
   * deliver FCN) */
  for (uint8_t i = 0; i < num_sats; i++) {
    if (MSM5 == msm_type || MSM7 == msm_type) {
      BITSTREAM_DECODE_U8(buff, sat_info[i], 4);
      sat_info_valid[i] = true;
    } else {
      sat_info[i] = 0;
      sat_info_valid[i] = false;
    }
  }

  /* rough range modulo 1 ms, DF398 */
  for (uint8_t i = 0; i < num_sats; i++) {
    uint32_t rough_pr;
    BITSTREAM_DECODE_U32(buff, rough_pr, 10);
    if (rough_range_valid[i]) {
      rough_range_ms[i] += (double)rough_pr / 1024;
    }
  }

  /* range rate, m/s, DF399*/
  for (uint8_t i = 0; i < num_sats; i++) {
    if (MSM5 == msm_type || MSM7 == msm_type) {
      int16_t rate;
      BITSTREAM_DECODE_S16(buff, rate, 14);
      rough_rate_m_s[i] = (double)rate;
      rough_rate_valid[i] = (MSM_ROUGH_RATE_INVALID != rate);
    } else {
      rough_rate_m_s[i] = 0;
      rough_rate_valid[i] = false;
    }
  }
  return RC_OK;
}

static rtcm3_rc decode_msm_fine_pseudoranges(swiftnav_in_bitstream_t *buff,
                                             const uint8_t num_cells,
                                             double fine_pr_ms[],
                                             flag_bf flags[]) {
  /* DF400 */
  for (uint16_t i = 0; i < num_cells; i++) {
    int16_t decoded;
    BITSTREAM_DECODE_S16(buff, decoded, 15);
    flags[i].fields.valid_pr = (decoded != MSM_PR_INVALID);
    fine_pr_ms[i] = (double)decoded * C_1_2P24;
  }
  return RC_OK;
}

static rtcm3_rc decode_msm_fine_pseudoranges_extended(
    swiftnav_in_bitstream_t *buff,
    const uint8_t num_cells,
    double fine_pr_ms[],
    flag_bf flags[]) {
  /* DF405 */
  for (uint16_t i = 0; i < num_cells; i++) {
    int32_t decoded;
    BITSTREAM_DECODE_S32(buff, decoded, 20);
    flags[i].fields.valid_pr = (decoded != MSM_PR_EXT_INVALID);
    fine_pr_ms[i] = (double)decoded * C_1_2P29;
  }
  return RC_OK;
}

static rtcm3_rc decode_msm_fine_phaseranges(swiftnav_in_bitstream_t *buff,
                                            const uint8_t num_cells,
                                            double fine_cp_ms[],
                                            flag_bf flags[]) {
  /* DF401 */
  for (uint16_t i = 0; i < num_cells; i++) {
    int32_t decoded;
    BITSTREAM_DECODE_S32(buff, decoded, 22);
    flags[i].fields.valid_cp = (decoded != MSM_CP_INVALID);
    fine_cp_ms[i] = (double)decoded * C_1_2P29;
  }
  return RC_OK;
}

static rtcm3_rc decode_msm_fine_phaseranges_extended(
    swiftnav_in_bitstream_t *buff,
    const uint8_t num_cells,
    double fine_cp_ms[],
    flag_bf flags[]) {
  /* DF406 */
  for (uint16_t i = 0; i < num_cells; i++) {
    int32_t decoded;
    BITSTREAM_DECODE_S32(buff, decoded, 24);
    flags[i].fields.valid_cp = (decoded != MSM_CP_EXT_INVALID);
    fine_cp_ms[i] = (double)decoded * C_1_2P31;
  }
  return RC_OK;
}

static rtcm3_rc decode_msm_lock_times(swiftnav_in_bitstream_t *buff,
                                      const uint8_t num_cells,
                                      double lock_time[],
                                      flag_bf flags[]) {
  /* DF402 */
  for (uint16_t i = 0; i < num_cells; i++) {
    uint32_t lock_ind;
    BITSTREAM_DECODE_U32(buff, lock_ind, 4);
    lock_time[i] = rtcm3_decode_lock_time(lock_ind);
    flags[i].fields.valid_lock = 1;
  }
  return RC_OK;
}

static rtcm3_rc decode_msm_lock_times_extended(swiftnav_in_bitstream_t *buff,
                                               const uint8_t num_cells,
                                               double lock_time[],
                                               flag_bf flags[]) {
  /* DF407 */
  for (uint16_t i = 0; i < num_cells; i++) {
    uint16_t lock_ind;
    BITSTREAM_DECODE_U16(buff, lock_ind, 10);
    lock_time[i] = (double)from_msm_lock_ind_ext(lock_ind) / 1000;
    flags[i].fields.valid_lock = 1;
  }
  return RC_OK;
}

static rtcm3_rc decode_msm_hca_indicators(swiftnav_in_bitstream_t *buff,
                                          const uint8_t num_cells,
                                          bool hca_indicator[]) {
  /* DF420 */
  for (uint16_t i = 0; i < num_cells; i++) {
    BITSTREAM_DECODE_BOOL(buff, hca_indicator[i], 1);
  }
  return RC_OK;
}

static rtcm3_rc decode_msm_cnrs(swiftnav_in_bitstream_t *buff,
                                const uint8_t num_cells,
                                double cnr[],
                                flag_bf flags[]) {
  /* DF403 */
  for (uint16_t i = 0; i < num_cells; i++) {
    uint32_t decoded;
    BITSTREAM_DECODE_U32(buff, decoded, 6);
    flags[i].fields.valid_cnr = (decoded != 0);
    cnr[i] = (double)decoded;
  }
  return RC_OK;
}

static rtcm3_rc decode_msm_cnrs_extended(swiftnav_in_bitstream_t *buff,
                                         const uint8_t num_cells,
                                         double cnr[],
                                         flag_bf flags[]) {
  /* DF408 */
  for (uint16_t i = 0; i < num_cells; i++) {
    uint32_t decoded;
    BITSTREAM_DECODE_U32(buff, decoded, 10);
    flags[i].fields.valid_cnr = (decoded != 0);
    cnr[i] = (double)decoded * C_1_2P4;
  }
  return RC_OK;
}

static rtcm3_rc decode_msm_fine_phaserangerates(swiftnav_in_bitstream_t *buff,
                                                const uint8_t num_cells,
                                                double *fine_range_rate_m_s,
                                                flag_bf *flags) {
  /* DF404 */
  for (uint16_t i = 0; i < num_cells; i++) {
    int32_t decoded;
    BITSTREAM_DECODE_S32(buff, decoded, 15);
    fine_range_rate_m_s[i] = (double)decoded * 0.0001;
    flags[i].fields.valid_dop = (decoded != MSM_DOP_INVALID);
  }
  return RC_OK;
}

/** Decode an RTCMv3 Multi System Messages 4-7
 *
 * \param buff The input data buffer
 * \param msm_type MSM4, MSM5, MSM6 or MSM7
 * \param msg The parsed RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Cell mask too large or invalid TOW
 */
static rtcm3_rc rtcm3_decode_msm_internal(swiftnav_in_bitstream_t *buff,
                                          const uint16_t msm_type,
                                          rtcm_msm_message *msg) {
  if (MSM4 != msm_type && MSM5 != msm_type && MSM6 != msm_type &&
      MSM7 != msm_type) {
    /* Invalid message type requested */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BITSTREAM_DECODE_U16(buff, msg->header.msg_num, 12);
  if (msm_type != to_msm_type(msg->header.msg_num)) {
    /* Message number does not match the requested message type */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  rtcm_constellation_t cons = to_constellation(msg->header.msg_num);
  if (RTCM_CONSTELLATION_INVALID == cons) {
    /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  rtcm3_rc ret = rtcm3_read_msm_header(buff, cons, &msg->header);
  if (ret != RC_OK) {
    return ret;
  }

  if (RTCM_CONSTELLATION_GLO != cons) {
    if (msg->header.tow_ms > RTCM_MAX_TOW_MS) {
      return RC_INVALID_MESSAGE;
    }
  } else if (msg->header.tow_ms > RTCM_GLO_MAX_TOW_MS) { /* GLO */
    return RC_INVALID_MESSAGE;
  }

  uint8_t num_sats =
      count_mask_values(MSM_SATELLITE_MASK_SIZE, msg->header.satellite_mask);
  uint8_t num_sigs =
      count_mask_values(MSM_SIGNAL_MASK_SIZE, msg->header.signal_mask);

  if (num_sats * num_sigs > MSM_MAX_CELLS) {
    /* Too large cell mask, most probably a parsing error */
    return RC_INVALID_MESSAGE;
  }

  uint8_t cell_mask_size = num_sats * num_sigs;
  uint8_t num_cells = count_mask_values(cell_mask_size, msg->header.cell_mask);

  /* Satellite Data */

  double rough_range_ms[num_sats];
  double rough_rate_m_s[num_sats];
  uint8_t sat_info[num_sats];
  bool rough_range_valid[num_sats];
  bool rough_rate_valid[num_sats];
  bool sat_info_valid[num_sats];

  ret = decode_msm_sat_data(buff,
                            num_sats,
                            msm_type,
                            rough_range_ms,
                            rough_range_valid,
                            sat_info,
                            sat_info_valid,
                            rough_rate_m_s,
                            rough_rate_valid);
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

  for (uint8_t i = 0; i < num_cells; i++) {
    fine_pr_ms[i] = 0;
    fine_cp_ms[i] = 0;
    lock_time[i] = 0;
    hca_indicator[i] = false;
    cnr[i] = 0;
    fine_range_rate_m_s[i] = 0;
    flags[i].data = 0;
  }

  if (MSM4 == msm_type || MSM5 == msm_type) {
    ret = decode_msm_fine_pseudoranges(buff, num_cells, fine_pr_ms, flags);
    if (ret != RC_OK) {
      return ret;
    }
    ret = decode_msm_fine_phaseranges(buff, num_cells, fine_cp_ms, flags);
    if (ret != RC_OK) {
      return ret;
    }
    ret = decode_msm_lock_times(buff, num_cells, lock_time, flags);
    if (ret != RC_OK) {
      return ret;
    }
  } else if (MSM6 == msm_type || MSM7 == msm_type) {
    ret = decode_msm_fine_pseudoranges_extended(
        buff, num_cells, fine_pr_ms, flags);
    if (ret != RC_OK) {
      return ret;
    }
    ret = decode_msm_fine_phaseranges_extended(
        buff, num_cells, fine_cp_ms, flags);
    if (ret != RC_OK) {
      return ret;
    }
    ret = decode_msm_lock_times_extended(buff, num_cells, lock_time, flags);
    if (ret != RC_OK) {
      return ret;
    }
  } else {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  ret = decode_msm_hca_indicators(buff, num_cells, hca_indicator);
  if (ret != RC_OK) {
    return ret;
  }

  if (MSM4 == msm_type || MSM5 == msm_type) {
    ret = decode_msm_cnrs(buff, num_cells, cnr, flags);
    if (ret != RC_OK) {
      return ret;
    }
  } else if (MSM6 == msm_type || MSM7 == msm_type) {
    ret = decode_msm_cnrs_extended(buff, num_cells, cnr, flags);
    if (ret != RC_OK) {
      return ret;
    }
  } else {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  if (MSM5 == msm_type || MSM7 == msm_type) {
    ret = decode_msm_fine_phaserangerates(
        buff, num_cells, fine_range_rate_m_s, flags);
    if (ret != RC_OK) {
      return ret;
    }
  }

  uint8_t i = 0;
  for (uint8_t sat = 0; sat < num_sats; sat++) {
    msg->sats[sat].rough_range_ms = rough_range_ms[sat];
    msg->sats[sat].rough_range_rate_m_s = rough_rate_m_s[sat];
    if (RTCM_CONSTELLATION_GLO == cons && !sat_info_valid[sat]) {
      msg->sats[sat].glo_fcn = MSM_GLO_FCN_UNKNOWN;
    } else {
      msg->sats[sat].glo_fcn = sat_info[sat];
    }

    for (uint8_t sig = 0; sig < num_sigs; sig++) {
      if (msg->header.cell_mask[sat * num_sigs + sig]) {
        if (rough_range_valid[sat] && flags[i].fields.valid_pr) {
          msg->signals[i].pseudorange_ms = rough_range_ms[sat] + fine_pr_ms[i];
        } else {
          msg->signals[i].pseudorange_ms = 0;
          flags[i].fields.valid_pr = false;
        }
        if (rough_range_valid[sat] && flags[i].fields.valid_cp) {
          msg->signals[i].carrier_phase_ms =
              rough_range_ms[sat] + fine_cp_ms[i];
        } else {
          msg->signals[i].carrier_phase_ms = 0;
          flags[i].fields.valid_cp = false;
        }
        msg->signals[i].lock_time_s = lock_time[i];
        msg->signals[i].hca_indicator = hca_indicator[i];
        if (flags[i].fields.valid_cnr) {
          msg->signals[i].cnr = cnr[i];
        } else {
          msg->signals[i].cnr = 0;
        }
        if (rough_rate_valid[sat] && flags[i].fields.valid_dop) {
          /* convert Doppler into Hz */
          msg->signals[i].range_rate_m_s =
              rough_rate_m_s[sat] + fine_range_rate_m_s[i];
        } else {
          msg->signals[i].range_rate_m_s = 0;
          flags[i].fields.valid_dop = 0;
        }
        msg->signals[i].flags = flags[i];
        i++;
      }
    }
  }

  return RC_OK;
}

/** Decode an RTCMv3 Multi System Message 4
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Cell mask too large or invalid TOW
 */
rtcm3_rc rtcm3_decode_msm4_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg) {
  assert(msg);
  return rtcm3_decode_msm_internal(buff, MSM4, msg);
}

/** Decode an RTCMv3 Multi System Message 5
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Cell mask too large or invalid TOW
 */
rtcm3_rc rtcm3_decode_msm5_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg) {
  assert(msg);
  return rtcm3_decode_msm_internal(buff, MSM5, msg);
}

/** Decode an RTCMv3 Multi System Message 6
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Cell mask too large or invalid TOW
 */
rtcm3_rc rtcm3_decode_msm6_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg) {
  assert(msg);
  return rtcm3_decode_msm_internal(buff, MSM6, msg);
}

/** Decode an RTCMv3 Multi System Message 7
 *
 * \param buff The input data buffer
 * \param RTCM message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Cell mask too large or invalid TOW
 */
rtcm3_rc rtcm3_decode_msm7_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msm_message *msg) {
  assert(msg);
  return rtcm3_decode_msm_internal(buff, MSM7, msg);
}

/** Decode Wrapped SBP Message
 *
 * \param buff The input data buffer
 * \param msg SBP message struct
 * \return  - RC_OK : Success
 */
static rtcm3_rc rtcm3_decode_wrapped_sbp(swiftnav_in_bitstream_t *buff,
                                         rtcm_msg_wrapped_sbp *msg) {
  BITSTREAM_DECODE_U16(buff, msg->msg_type, 16);
  BITSTREAM_DECODE_U16(buff, msg->sender_id, 16);
  BITSTREAM_DECODE_U8(buff, msg->len, 8);
  for (uint8_t i = 0; i < msg->len; ++i) {
    BITSTREAM_DECODE_U8(buff, msg->data[i], 8);
  }

  return RC_OK;
}

/** Decode Wrapped Swift RTCM Message
 *
 * \return  - RC_OK : Success
 */
static rtcm3_rc rtcm3_decode_wrapped_swift_rtcm(void) {
  return RC_INVALID_MESSAGE;
}

/** Decode Swift Proprietary Message
 *
 * \param buff The input data buffer
 * \param msg  message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Unknown protocol of wrapped message
 */
rtcm3_rc rtcm3_decode_4062_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_swift_proprietary *msg) {
  assert(msg);
  uint32_t msg_num;
  BITSTREAM_DECODE_U32(buff, msg_num, 12);

  if (msg_num != SWIFT_PROPRIETARY_MSG) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BITSTREAM_DECODE_U8(buff, msg->protocol_version, 4);
  const rtcm_msg_protocol_t protocol_type =
      to_protocol_type(msg->protocol_version);

  rtcm3_rc return_code = RC_INVALID_MESSAGE;
  if (protocol_type == WRAPPED_SBP) {
    return_code = rtcm3_decode_wrapped_sbp(buff, &msg->wrapped_msg.sbp);
  } else if (protocol_type == WRAPPED_SWIFT_RTCM) {
    return_code = rtcm3_decode_wrapped_swift_rtcm();
  }

  return return_code;
}

/** Decode Navigation Data Frame
 *
 * \param buff The input data buffer
 * \param msg  message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Nonzero reserved bits (invalid format)
 */
rtcm3_rc rtcm3_decode_4075_bitstream(swiftnav_in_bitstream_t *buff,
                                     rtcm_msg_ndf *msg) {
  assert(msg);
  uint16_t msg_num;
  BITSTREAM_DECODE_U16(buff, msg_num, 12);

  if (msg_num != 4075) { /* Unexpected message type. */
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  BITSTREAM_DECODE_U16(buff, msg->stn_id, 12);

  uint8_t reserved_bits;
  BITSTREAM_DECODE_U8(buff, reserved_bits, 2);

  /* These bits are reserved for future use, if they aren't 0 it must be a
     new format we don't know how to handle. */
  if (reserved_bits != 0) {
    return RC_INVALID_MESSAGE;
  }

  BITSTREAM_DECODE_U8(buff, msg->frame_count, 6);

  if (msg->frame_count > MAX_NDF_FRAMES) {
    return RC_INVALID_MESSAGE;
  }

  for (uint8_t i = 0; i < msg->frame_count; i++) {
    BITSTREAM_DECODE_U8(buff, msg->frames[i].sat_sys, 4);
    BITSTREAM_DECODE_U8(buff, msg->frames[i].sat_num, 6);
    BITSTREAM_DECODE_U8(buff, msg->frames[i].ext_sat_info, 4);
    BITSTREAM_DECODE_U8(buff, msg->frames[i].sig_type, 5);
    BITSTREAM_DECODE_U32(buff, msg->frames[i].epoch_time, 30);
    BITSTREAM_DECODE_U8(buff, msg->frames[i].continuous_tracking, 1);
    BITSTREAM_DECODE_U16(buff, msg->frames[i].frame_data_size_bits, 12);

    uint16_t remaining_bits = msg->frames[i].frame_data_size_bits;
    if (remaining_bits > MAX_NDF_FRAME_SIZE_BITS) {
      return RC_INVALID_MESSAGE;
    }

    uint16_t j = 0;
    while (remaining_bits > 0) {
      uint32_t next_bits = remaining_bits < 32 ? remaining_bits : 32;
      BITSTREAM_DECODE_U32(buff, msg->frames[i].frame_data[j++], next_bits);
      remaining_bits -= next_bits;
    }
  }

  return RC_OK;
}

static rtcm3_rc rtcm3_decode_999_stgsv_field_value_base(
    swiftnav_in_bitstream_t *buff,
    rtcm_999_stgsv_sat_signal *msg_999_stgsv_fv,
    const uint8_t field_mask) {
  if (field_mask & RTCM_STGSV_FIELDMASK_EL) {
    BITSTREAM_DECODE_S8(buff, msg_999_stgsv_fv->el, 8);
  }
  if (field_mask & RTCM_STGSV_FIELDMASK_AZ) {
    BITSTREAM_DECODE_U16(buff, msg_999_stgsv_fv->az, 9);
  }
  if (field_mask & RTCM_STGSV_FIELDMASK_CN0_B1) {
    BITSTREAM_DECODE_U8(buff, msg_999_stgsv_fv->cn0_b1, 8);
  }
  if (field_mask & RTCM_STGSV_FIELDMASK_CN0_B2) {
    BITSTREAM_DECODE_U8(buff, msg_999_stgsv_fv->cn0_b2, 8);
  }
  if (field_mask & RTCM_STGSV_FIELDMASK_CN0_B3) {
    BITSTREAM_DECODE_U8(buff, msg_999_stgsv_fv->cn0_b3, 8);
  }

  return RC_OK;
}

static uint8_t rtcm3_teseov_count_sat_active(uint64_t sat_mask,
                                             uint8_t sat_mask_size,
                                             uint8_t sat_active[]) {
  uint8_t n_sat = 0;
  for (uint8_t i = 0; i < sat_mask_size; i++) {
    if (sat_mask & (INT64_C(1) << (RTCM_TESEOV_SATELLITE_MASK_SIZE - i - 1))) {
      sat_active[n_sat] = i;
      n_sat++;
    }
  }
  return n_sat;
}

static rtcm3_rc rtcm3_decode_999_stgsv_base(swiftnav_in_bitstream_t *buff,
                                            rtcm_msg_999_stgsv *msg_999_stgsv) {
  uint64_t sat_mask_64 = 0;

  BITSTREAM_DECODE_U32(buff, msg_999_stgsv->tow_ms, 30);
  BITSTREAM_DECODE_U8(buff, msg_999_stgsv->constellation, 4);
  BITSTREAM_DECODE_U64(buff, sat_mask_64, RTCM_TESEOV_SATELLITE_MASK_SIZE);
  BITSTREAM_DECODE_U8(buff, msg_999_stgsv->field_mask, 8);
  BITSTREAM_DECODE_U8(buff, msg_999_stgsv->mul_msg_ind, 1);

  uint8_t sat_mask_size = (msg_999_stgsv->constellation == RTCM_TESEOV_BDS13
                               ? RTCM_TESEOV_SATELLITE_MASK_SIZE_GNSS13
                               : RTCM_TESEOV_SATELLITE_MASK_SIZE);

  uint8_t sat_active_arr[RTCM_TESEOV_SATELLITE_MASK_SIZE] = {0};
  msg_999_stgsv->n_sat =
      rtcm3_teseov_count_sat_active(sat_mask_64, sat_mask_size, sat_active_arr);

  rtcm3_rc rc_status_fv_base = RC_OK;
  for (uint8_t i = 0; i < msg_999_stgsv->n_sat; i++) {
    msg_999_stgsv->field_value[i].sat_id = sat_active_arr[i];

    rc_status_fv_base = rtcm3_decode_999_stgsv_field_value_base(
        buff, &msg_999_stgsv->field_value[i], msg_999_stgsv->field_mask);
    if (rc_status_fv_base != RC_OK) {
      return rc_status_fv_base;
    }
  }

  return RC_OK;
}

static rtcm3_rc rtcm3_decode_999_restart_base(
    swiftnav_in_bitstream_t *buff, rtcm_msg_999_restart *msg_999_restart) {
  BITSTREAM_DECODE_U32(buff, msg_999_restart->restart_mask, 32);
  return RC_OK;
}

static rtcm3_rc rtcm3_decode_999_aux_ttff(
    swiftnav_in_bitstream_t *buff, rtcm_msg_999_aux_ttff *msg_999_aux_ttff) {
  BITSTREAM_DECODE_U32(buff, msg_999_aux_ttff->ttff, 32);
  return RC_OK;
}

static rtcm3_rc rtcm3_decode_999_aux_base(swiftnav_in_bitstream_t *buff,
                                          rtcm_msg_999_aux *msg_999_aux) {
  BITSTREAM_DECODE_U8(buff, msg_999_aux->aux_data_type_id, 8);

  switch (msg_999_aux->aux_data_type_id) {
    case RTCM_TESEOV_AUX_TTFF:
      return rtcm3_decode_999_aux_ttff(buff, &msg_999_aux->data.ttff);
    default:
      return RC_INVALID_MESSAGE;
  }
}

/** Decode 999 message
 *
 * \param buff The input data buffer
 * \param msg  message struct
 * \return  - RC_OK : Success
 *          - RC_MESSAGE_TYPE_MISMATCH : Message type mismatch
 *          - RC_INVALID_MESSAGE : Nonzero reserved bits (invalid format)
 */
rtcm3_rc rtcm3_decode_999_bitstream(swiftnav_in_bitstream_t *buff,
                                    rtcm_msg_999 *msg_999) {
  assert(msg_999);
  uint32_t msg_num;
  BITSTREAM_DECODE_U32(buff, msg_num, 12);
  BITSTREAM_DECODE_U8(buff, msg_999->sub_type_id, 8);

  if (msg_num != 999) {
    return RC_MESSAGE_TYPE_MISMATCH;
  }

  switch (msg_999->sub_type_id) {
    case RTCM_TESEOV_RESTART:
      return rtcm3_decode_999_restart_base(buff, &msg_999->data.restart);
    case RTCM_TESEOV_STGSV:
      return rtcm3_decode_999_stgsv_base(buff, &msg_999->data.stgsv);
    case RTCM_TESEOV_AUX:
      return rtcm3_decode_999_aux_base(buff, &msg_999->data.aux);
    default:
      return RC_INVALID_MESSAGE;
  }
}

rtcm3_rc rtcm3_decode_payload_bitstream(swiftnav_in_bitstream_t *buff,
                                        rtcm_msg_data_t *rtcm_msg) {
  assert(buff);
  assert(rtcm_msg);

  // Check payload/message buffer must be full bytes
  if ((buff->len % 8 != 0) || (buff->offset % 8 != 0)) {
    return RC_INVALID_MESSAGE;
  }
  uint16_t payload_len = (buff->len - buff->offset) / 8U;

  if (payload_len < RTCM3_MIN_MSG_LEN) {  // #bytes to decode message size
    return RC_INVALID_MESSAGE;
  }

  BITSTREAM_DECODE_U16(buff, rtcm_msg->msg_num, 12);
  buff->offset -= 12;

  rtcm_msg->msg_type = rtcm3_msg_num_to_msg_type(rtcm_msg->msg_num);

  rtcm3_rc ret = RC_OK;
  switch (rtcm_msg->msg_num) {
    case 999: {
      rtcm_msg_999 *msg_999 = &rtcm_msg->message.msg_999;
      ret = rtcm3_decode_999_bitstream(buff, msg_999);
      break;
    }
    case 1001:
    case 1003:
      break;
    case 1002: {
      rtcm_obs_message *msg_obs = &rtcm_msg->message.msg_obs;
      ret = rtcm3_decode_1002_bitstream(buff, msg_obs);
      break;
    }
    case 1004: {
      rtcm_obs_message *msg_obs = &rtcm_msg->message.msg_obs;
      ret = rtcm3_decode_1004_bitstream(buff, msg_obs);
      break;
    }
    case 1005: {
      rtcm_msg_1005 *msg_1005 = &rtcm_msg->message.msg_1005;
      ret = rtcm3_decode_1005_bitstream(buff, msg_1005);
      break;
    }
    case 1006: {
      rtcm_msg_1006 *msg_1006 = &rtcm_msg->message.msg_1006;
      ret = rtcm3_decode_1006_bitstream(buff, msg_1006);
      break;
    }
    case 1007:
    case 1008:
      break;
    case 1010: {
      rtcm_obs_message *msg_obs = &rtcm_msg->message.msg_obs;
      ret = rtcm3_decode_1010_bitstream(buff, msg_obs);
      break;
    }
    case 1012: {
      rtcm_obs_message *msg_obs = &rtcm_msg->message.msg_obs;
      ret = rtcm3_decode_1012_bitstream(buff, msg_obs);
      break;
    }
    case 1013: {
      rtcm_msg_1013 *rtcm_1013 = &rtcm_msg->message.msg_1013;
      ret = rtcm3_decode_1013_bitstream(buff, rtcm_1013, payload_len);
      break;
    }
    case 1019: {
      rtcm_msg_eph *msg_eph = &rtcm_msg->message.msg_eph;
      ret = rtcm3_decode_gps_eph_bitstream(buff, msg_eph);
      break;
    }
    case 1020: {
      rtcm_msg_eph *msg_eph = &rtcm_msg->message.msg_eph;
      ret = rtcm3_decode_glo_eph_bitstream(buff, msg_eph);
      break;
    }
    case 1042: {
      rtcm_msg_eph *msg_eph = &rtcm_msg->message.msg_eph;
      ret = rtcm3_decode_bds_eph_bitstream(buff, msg_eph);
      break;
    }
    case 1044: {
      rtcm_msg_eph *msg_eph = &rtcm_msg->message.msg_eph;
      ret = rtcm3_decode_qzss_eph_bitstream(buff, msg_eph);
      break;
    }
    case 1045: {
      rtcm_msg_eph *msg_eph = &rtcm_msg->message.msg_eph;
      ret = rtcm3_decode_gal_eph_fnav_bitstream(buff, msg_eph);
      break;
    }

    case 1046: {
      rtcm_msg_eph *msg_eph = &rtcm_msg->message.msg_eph;
      ret = rtcm3_decode_gal_eph_inav_bitstream(buff, msg_eph);
      break;
    }
    case 1029: {
      rtcm_msg_1029 *msg_1029 = &rtcm_msg->message.msg_1029;
      ret = rtcm3_decode_1029_bitstream(buff, msg_1029);
      break;
    }
    case 1033: {
      rtcm_msg_1033 *msg_1033 = &rtcm_msg->message.msg_1033;
      ret = rtcm3_decode_1033_bitstream(buff, msg_1033);
      break;
    }
    case 1230: {
      rtcm_msg_1230 *msg_1230 = &rtcm_msg->message.msg_1230;
      ret = rtcm3_decode_1230_bitstream(buff, msg_1230);
      break;
    }

    /* The following two chunks of messages handle converting separate SSR orbit
     * correction and SSR clock correction messages into a single SBP message
     * containing both orbit and clock corrections. This code makes the
     * following assumptions:
     *  1. Each pair of orbit and clock messages contain data for the same group
     * of satellites, this implies that we don't support the multimessage flag
     * in the RTCM message. We do match the clock correction satid with the
     * orbit correction satid, so the corrections don't have to be in the same
     * order but we will silently drop any correction that we can't find a
     * matching pair for.
     *  2. The clock and orbit messages are sent at the same frequency. The
     * epoch time of the two messages must match for them to be considered
     * pairs.
     */
    case 1057:
    case 1063:
    case 1240:
    case 1246:
    case 1258: {
      rtcm_msg_orbit *msg_orbit = &rtcm_msg->message.msg_orbit;
      ret = rtcm3_decode_orbit_bitstream(buff, msg_orbit);
      break;
    }
    case 1058:
    case 1064:
    case 1241:
    case 1247:
    case 1259: {
      rtcm_msg_clock *msg_clock = &rtcm_msg->message.msg_clock;
      ret = rtcm3_decode_clock_bitstream(buff, msg_clock);
      break;
    }
    case 1059:
    case 1065:
    case 1242:
    case 1248:
    case 1260: {
      rtcm_msg_code_bias *msg_code_bias = &rtcm_msg->message.msg_code_bias;
      ret = rtcm3_decode_code_bias_bitstream(buff, msg_code_bias);
      break;
    }
    case 1060:
    case 1066:
    case 1243:
    case 1249:
    case 1261: {
      rtcm_msg_orbit_clock *msg_orbit_clock =
          &rtcm_msg->message.msg_orbit_clock;
      ret = rtcm3_decode_orbit_clock_bitstream(buff, msg_orbit_clock);
      break;
    }
    case 1265:
    case 1266:
    case 1267:
    case 1268:
    case 1269:
    case 1270: {
      rtcm_msg_phase_bias *msg_phase_bias = &rtcm_msg->message.msg_phase_bias;
      ret = rtcm3_decode_phase_bias_bitstream(buff, msg_phase_bias);
      break;
    }
    case 1074:
    case 1084:
    case 1094:
    case 1124: {
      rtcm_msm_message *msg_msm = &rtcm_msg->message.msg_msm;
      ret = rtcm3_decode_msm4_bitstream(buff, msg_msm);
      break;
    }
    case 1075:
    case 1085:
    case 1095:
    case 1125: {
      rtcm_msm_message *msg_msm = &rtcm_msg->message.msg_msm;
      ret = rtcm3_decode_msm5_bitstream(buff, msg_msm);
      break;
    }
    case 1076:
    case 1086:
    case 1096:
    case 1126: {
      rtcm_msm_message *msg_msm = &rtcm_msg->message.msg_msm;
      ret = rtcm3_decode_msm6_bitstream(buff, msg_msm);
      break;
    }
    case 1077:
    case 1087:
    case 1097:
    case 1127: {
      rtcm_msm_message *msg_msm = &rtcm_msg->message.msg_msm;
      ret = rtcm3_decode_msm7_bitstream(buff, msg_msm);
      break;
    }
    case 1104:
    case 1105:
    case 1106:
    case 1107:
      /* MSM4-MSM7 of SBAS messages suppressed for now */
    case 1114:
    case 1115:
    case 1116:
    case 1117:
      /* MSM4-MSM7 of QZSS messages suppressed for now */
      break;
    case 1071:
    case 1072:
    case 1073:
      /* GPS MSM1 - MSM3 */
    case 1081:
    case 1082:
    case 1083:
      /* GLO MSM1 - MSM3 */
    case 1091:
    case 1092:
    case 1093:
      /* GAL MSM1 - MSM3 */
    case 1101:
    case 1102:
    case 1103:
      /* SBAS MSM1 - MSM3 */
    case 1111:
    case 1112:
    case 1113:
      /* QZSS MSM1 - MSM3 */
    case 1121:
    case 1122:
    case 1123: {
      /* BDS MSM1 - MSM3 */

      /* MSM1-3 messages (1xx3) are currently not supported, warn the user once
       * if these messages are seen - only warn once as these messages can be
       * present in streams that contain MSM4-7 or 1004 and 1012 so are valid */
      rtcm_msm_message *msg_msm = &rtcm_msg->message.msg_msm;
      uint32_t stn_id = 0;
      if (swiftnav_in_bitstream_getbitu(buff, &stn_id, 12, 24)) {
        msg_msm->header.stn_id = stn_id;
      }
      break;
    }
    case 4062: {
      rtcm_msg_swift_proprietary *msg_swift = &rtcm_msg->message.msg_swift_prop;
      ret = rtcm3_decode_4062_bitstream(buff, msg_swift);
      break;
    }
    case 4075: {
      rtcm_msg_ndf *msg_ndf = &rtcm_msg->message.msg_ndf;
      ret = rtcm3_decode_4075_bitstream(buff, msg_ndf);
      break;
    }
    default:
      ret = RC_INVALID_MESSAGE;
      break;
  }

  return ret;
}

rtcm3_rc rtcm3_decode_frame_bitstream(swiftnav_in_bitstream_t *buff,
                                      rtcm_frame_t *rtcm_frame) {
  assert(buff);
  assert(rtcm_frame);
  assert(buff->offset % 8 == 0);

  uint32_t buff_offset_start = buff->offset;

  // Check minimum buffer size
  if ((buff->len - buff_offset_start) <
      ((RTCM3_MSG_OVERHEAD + RTCM3_MIN_MSG_LEN) * 8U)) {
    return RC_INVALID_MESSAGE;
  }

  // Decode general info extracted from RTCM frame
  // Decode Preamble
  uint8_t rtcm_preamble = 0;
  BITSTREAM_DECODE_U8(buff, rtcm_preamble, 8);  // 8 bit preamble
  if (rtcm_preamble != RTCM3_PREAMBLE) {
    return RC_INVALID_MESSAGE;
  }

  // Decode Reserved bits
  BITSTREAM_DECODE_U8(buff, rtcm_frame->reserve, 6);  // 6 bit reserved

  // Decode message length
  BITSTREAM_DECODE_U16(buff, rtcm_frame->payload_len, 10);  // 10 bit msg length

  // Decode CRC
  if ((buff->len - buff_offset_start) <
      ((RTCM3_MSG_OVERHEAD + rtcm_frame->payload_len) * 8U)) {
    return RC_INVALID_MESSAGE;
  }
  buff->offset = (buff_offset_start + (rtcm_frame->payload_len + 3) * 8U);
  BITSTREAM_DECODE_U32(buff, rtcm_frame->crc, 24);
  // Note: this step just decodes the CRC content. It must be verified
  // separately.

  // Decode payload
  swiftnav_in_bitstream_t payload_bitstream;
  swiftnav_in_bitstream_init(
      &payload_bitstream, buff->data + 3, rtcm_frame->payload_len * 8U);
  rtcm3_rc ret =
      rtcm3_decode_payload_bitstream(&payload_bitstream, &rtcm_frame->data);
  if (RC_OK != ret) {
    return ret;
  }

  return RC_OK;
}

rtcm_msg_type_t rtcm3_msg_num_to_msg_type(uint16_t msg_num) {
  switch (msg_num) {
    case 999:
      return Rtcm999;
    case 1001:
    case 1003:
      // not support
    case 1002:
    case 1004:
    case 1010:
    case 1012:
      return RtcmObs;
    case 1005:
      return Rtcm1005;
    case 1006:
      return Rtcm1006;
    case 1007:
      return Rtcm1007;
    case 1008:
      return Rtcm1008;
    case 1013:
      return Rtcm1013;
    case 1019:
    case 1020:
    case 1042:
    case 1044:
    case 1045:
    case 1046:
      return RtcmEph;
    case 1029:
      return Rtcm1029;
    case 1033:
      return Rtcm1033;
    case 1230:
      return Rtcm1230;
    case 1071:
    case 1072:
    case 1073:
      /* GPS MSM1 - MSM3 */
    case 1081:
    case 1082:
    case 1083:
      /* GLO MSM1 - MSM3 */
    case 1091:
    case 1092:
    case 1093:
      /* GAL MSM1 - MSM3 */
    case 1101:
    case 1102:
    case 1103:
      /* SBAS MSM1 - MSM3 */
    case 1111:
    case 1112:
    case 1113:
      /* QZSS MSM1 - MSM3 */
    case 1121:
    case 1122:
    case 1123:
      /* BDS MSM1 - MSM3 */
    case 1104:
    case 1105:
    case 1106:
    case 1107:
      /* SBAS MSM4-MSM7 */
    case 1114:
    case 1115:
    case 1116:
    case 1117:
      /* QZSS MSM4-MSM7 */
    case 1074:
    case 1084:
    case 1094:
    case 1124:
      // MSM4
    case 1075:
    case 1085:
    case 1095:
    case 1125:
      // MSM5
    case 1076:
    case 1086:
    case 1096:
    case 1126:
      // MSM6
    case 1077:
    case 1087:
    case 1097:
    case 1127:
      // MSM7
      return RtcmMSM;
    case 1057:
    case 1063:
    case 1240:
    case 1246:
    case 1258:
      return RtcmOrbit;
    case 1058:
    case 1064:
    case 1241:
    case 1247:
    case 1259:
      return RtcmClock;
    case 1059:
    case 1065:
    case 1242:
    case 1248:
    case 1260:
      return RtcmCodeBias;
    case 1060:
    case 1066:
    case 1243:
    case 1249:
    case 1261:
      return RtcmOrbitClock;
    case 1265:
    case 1266:
    case 1267:
    case 1268:
    case 1269:
    case 1270:
      return RtcmPhaseBias;
    case 4062:
      return RtcmSwiftProp;
    case 4075:
      return RtcmNdf;
    default:
      return RtcmUnsupported;
  }
}
