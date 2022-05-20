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

#include <check.h>
#include <gnss-converters/rtcm3_sbp.h>
#include <gnss-converters/sbp_rtcm3.h>
#include <gnss_converters/src/common.h>
#include <gnss_converters/src/rtcm3_sbp_internal.h>
#include <gnss_converters/src/rtcm3_utils.h>
#include <gnss_converters/src/sbp_rtcm3_internal.h>
#include <libsbp/legacy/logging.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FREQ_TOL 1e-3

static struct rtcm3_sbp_state state;
static struct rtcm3_out_state out_state;

static const gps_time_t setup_gps_time = {.wn = 1945, .tow = 211190};
static const int8_t setup_leap_seconds = 18;

/* fixture globals and functions */

void utils_setup(void) {
  memset(&state, 0, sizeof(state));
  memset(&out_state, 0, sizeof(out_state));
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&setup_gps_time, &setup_leap_seconds, &state);

  sbp2rtcm_set_leap_second(&setup_leap_seconds, &out_state);
}

/* end fixtures */

START_TEST(test_compute_glo_time) {
  for (u8 day = 0; day < 7; day++) {
    for (u8 hour = 0; hour < 24; hour++) {
      for (u8 min = 0; min < 60; min++) {
        for (u8 sec = 0; sec < 60; sec++) {
          u32 tod = hour * HOUR_SECS + min * MINUTE_SECS + sec;
          gps_time_t rover_time = {.tow = day * DAY_SECS + tod, .wn = 1945};
          rtcm2sbp_set_time(&rover_time, &setup_leap_seconds, &state);
          u32 glo_tod_ms = (tod + UTC_SU_OFFSET * HOUR_SECS) * SECS_MS;
          if (glo_tod_ms > DAY_SECS * SECS_MS) {
            glo_tod_ms -= DAY_SECS * SECS_MS;
          }
          gps_time_t expected_time = rover_time;
          expected_time.tow += setup_leap_seconds;
          if (expected_time.tow >= WEEK_SECS) {
            expected_time.tow -= WEEK_SECS;
            expected_time.wn++;
          }

          gps_time_t obs_time;
          compute_glo_time(glo_tod_ms, &obs_time, &rover_time, &state);
          ck_assert_uint_eq(obs_time.wn, expected_time.wn);
          ck_assert_uint_eq((u32)rint(obs_time.tow),
                            (u32)rint(expected_time.tow));
        }
      }
    }
  }
}
END_TEST

START_TEST(test_glo_time_conversion) {
  for (u32 tow = 0; tow < WEEK_SECS; tow++) {
    gps_time_t rover_time = {.tow = tow, .wn = 1945};
    rtcm2sbp_set_time(&rover_time, &setup_leap_seconds, &state);

    u32 glo_tod_ms =
        compute_glo_tod_ms((u32)rint(rover_time.tow * SECS_MS), &out_state);

    gps_time_t obs_time;
    compute_glo_time(glo_tod_ms, &obs_time, &rover_time, &state);
    ck_assert_uint_eq(obs_time.wn, rover_time.wn);
    ck_assert_uint_eq((u32)rint(obs_time.tow), (u32)rint(rover_time.tow));
  }
}
END_TEST

START_TEST(test_msm_sid_conversion) {
  rtcm_msm_header header;
  /* GPS message */

  header.msg_num = 1074;
  /* PRNs 1, 2 and 20 */
  memset((void *)&header.satellite_mask, 0, sizeof(header.satellite_mask));
  header.satellite_mask[0] = true;  /* PRN 1 */
  header.satellite_mask[1] = true;  /* PRN 2 */
  header.satellite_mask[63] = true; /* PRN 64, invalid except for BDS  */
  /* signal ids 2 (L1CA) and 15 (L2CM) */
  memset((void *)&header.signal_mask, 0, sizeof(header.signal_mask));
  header.signal_mask[1] = true;
  header.signal_mask[14] = true;

  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_GPS);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 1);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 2);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), PRN_INVALID);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_GPS_L1CA);
  ck_assert_uint_eq(msm_signal_to_code(&header, 1), CODE_GPS_L2CM);
  double freq;
  ck_assert(msm_signal_frequency(&header, 0, 0, &freq) &&
            fabs(freq - GPS_L1_HZ) < FREQ_TOL);
  ck_assert(msm_signal_frequency(&header, 1, 0, &freq) &&
            fabs(freq - GPS_L2_HZ) < FREQ_TOL);

  /* GLO */
  header.msg_num = 1084;
  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_GLO);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 1);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 2);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), PRN_INVALID);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_GLO_L1OF);
  ck_assert_uint_eq(msm_signal_to_code(&header, 1), CODE_INVALID);
  uint8_t fcn = 3;
  ck_assert(msm_signal_frequency(&header, 0, fcn + MSM_GLO_FCN_OFFSET, &freq) &&
            fabs(freq - (GLO_L1_HZ + fcn * GLO_L1_DELTA_HZ)) < FREQ_TOL);
  ck_assert(!msm_signal_frequency(&header, 1, fcn + MSM_GLO_FCN_OFFSET, &freq));

  /* GAL */
  header.msg_num = 1094;
  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_GAL);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 1);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 2);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), PRN_INVALID);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_GAL_E1C);
  ck_assert_uint_eq(msm_signal_to_code(&header, 1), CODE_GAL_E7Q);
  ck_assert(msm_signal_frequency(&header, 0, 0, &freq) &&
            fabs(freq - GAL_E1_HZ) < FREQ_TOL);
  ck_assert(msm_signal_frequency(&header, 1, 0, &freq) &&
            fabs(freq - GAL_E7_HZ) < FREQ_TOL);

  /* SBAS */
  header.msg_num = 1104;
  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_SBAS);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 120);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 121);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), PRN_INVALID);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_SBAS_L1CA);
  ck_assert(msm_signal_frequency(&header, 0, 0, &freq) &&
            fabs(freq - SBAS_L1_HZ) < FREQ_TOL);

  /* QZS PRNs start from 193 */
  header.msg_num = 1114;
  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_QZS);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 193);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 194);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), PRN_INVALID);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_QZS_L1CA);
  ck_assert_uint_eq(msm_signal_to_code(&header, 1), CODE_QZS_L2CM);
  ck_assert(msm_signal_frequency(&header, 0, 0, &freq) &&
            fabs(freq - QZS_L1_HZ) < FREQ_TOL);
  ck_assert(msm_signal_frequency(&header, 1, 0, &freq) &&
            fabs(freq - QZS_L2_HZ) < FREQ_TOL);

  /* BDS */
  header.msg_num = 1124;
  header.signal_mask[13] = true;
  ck_assert_uint_eq(to_constellation(header.msg_num), CONSTELLATION_BDS);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 0), 1);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 1), 2);
  ck_assert_uint_eq(msm_sat_to_prn(&header, 2), 64);
  ck_assert_uint_eq(msm_signal_to_code(&header, 0), CODE_BDS2_B1);
  ck_assert_uint_eq(msm_signal_to_code(&header, 1), CODE_BDS2_B2);
  ck_assert(msm_signal_frequency(&header, 0, 0, &freq) &&
            fabs(freq - BDS2_B11_HZ) < FREQ_TOL);
  ck_assert(msm_signal_frequency(&header, 1, 0, &freq) &&
            fabs(freq - BDS2_B2_HZ) < FREQ_TOL);
}
END_TEST

START_TEST(test_msm_code_prn_conversion) {
  rtcm_msm_header header;

  for (u8 cons = 0; cons < RTCM_CONSTELLATION_COUNT; cons++) {
    header.msg_num = to_msm_msg_num(cons, MSM4);
    for (u8 sat_id = 0; sat_id < MSM_SATELLITE_MASK_SIZE; sat_id++) {
      header.satellite_mask[sat_id] = true;
      u8 prn = msm_sat_to_prn(&header, sat_id);
      if (PRN_INVALID == prn) {
        continue;
      }
      /* check that each valid PRN convers back to the original satellite id */
      ck_assert_uint_eq(sat_id, prn_to_msm_sat_id(prn, cons));
      /* index is the same in this case when all mask bits are filled */
      ck_assert_uint_eq(sat_id, prn_to_msm_sat_index(&header, prn));
    }

    for (u8 signal_id = 0; signal_id < MSM_SIGNAL_MASK_SIZE; signal_id++) {
      header.signal_mask[signal_id] = true;
      code_t code = msm_signal_to_code(&header, signal_id);
      if (CODE_INVALID == code) {
        continue;
      }
      /* check that each valid code converts back to the original signal id */
      u8 converted_signal_id = code_to_msm_signal_id(code, cons);

      if (RTCM_CONSTELLATION_GPS == cons && signal_id == 3) {
        /* RTCM GPS 1W maps into L1P */
        ck_assert_uint_eq(converted_signal_id, 2);
      } else if (RTCM_CONSTELLATION_GPS == cons && signal_id == 9) {
        /* RTCM GPS 1W maps into L2P */
        ck_assert_uint_eq(converted_signal_id, 8);
      } else {
        /* everything else is 1-to-1 */
        ck_assert_uint_eq(converted_signal_id, signal_id);
        ck_assert_uint_eq(code_to_msm_signal_index(&header, code), signal_id);
      }
    }
  }
}
END_TEST

START_TEST(test_msm_glo_fcn) {
  rtcm_msm_header header;

  header.msg_num = 1084;
  memset((void *)&header.satellite_mask, 0, sizeof(header.satellite_mask));
  header.satellite_mask[0] = true;  /* PRN 1 */
  header.satellite_mask[2] = true;  /* PRN 3 */
  header.satellite_mask[63] = true; /* invalid PRN 64 */
  /* signal id 2 (L1CA)) */
  memset((void *)&header.signal_mask, 0, sizeof(header.signal_mask));
  header.signal_mask[1] = true;

  uint8_t sat_info[2] = {3 + MSM_GLO_FCN_OFFSET, -6 + MSM_GLO_FCN_OFFSET};

  uint8_t glo_fcn;

  /* FCN for both satellites available in sat_info */
  ck_assert(msm_get_glo_fcn(&header, 0, sat_info[0], NULL, &glo_fcn));
  ck_assert_uint_eq(glo_fcn, 3 + MSM_GLO_FCN_OFFSET);

  ck_assert(msm_get_glo_fcn(&header, 1, sat_info[1], NULL, &glo_fcn));
  ck_assert_uint_eq(glo_fcn, -6 + MSM_GLO_FCN_OFFSET);

  /* sat info invalid for first satellite, FCN not available */
  sat_info[0] = MSM_GLO_FCN_UNKNOWN;
  ck_assert(!msm_get_glo_fcn(&header, 0, sat_info[0], NULL, &glo_fcn));

  /* FCN map indexed by PRN */
  uint8_t fcn_map[RTCM_MAX_SATS];
  fcn_map[0] = MSM_GLO_FCN_UNKNOWN;
  fcn_map[1] = 2 + MSM_GLO_FCN_OFFSET; /* PRN 1 */
  fcn_map[2] = MSM_GLO_FCN_UNKNOWN;
  fcn_map[3] = MSM_GLO_FCN_UNKNOWN; /* PRN 3 */

  /* FCN for first satellite from FCN MAP */
  ck_assert(msm_get_glo_fcn(&header, 0, sat_info[0], fcn_map, &glo_fcn));
  ck_assert_uint_eq(glo_fcn, 2 + MSM_GLO_FCN_OFFSET);

  /* FCN MAP invalid for second satellite, value in sat_info used */
  ck_assert(msm_get_glo_fcn(&header, 1, sat_info[1], fcn_map, &glo_fcn));
  ck_assert_uint_eq(glo_fcn, -6 + MSM_GLO_FCN_OFFSET);

  /* both FCN map and sat_info invalid, invalid FCN returned */
  sat_info[1] = MSM_GLO_FCN_UNKNOWN;
  ck_assert(!msm_get_glo_fcn(&header, 1, sat_info[1], fcn_map, &glo_fcn));

  /* add valid value in the FCN map */
  fcn_map[3] = -5 + MSM_GLO_FCN_OFFSET;
  ck_assert(msm_get_glo_fcn(&header, 1, sat_info[1], fcn_map, &glo_fcn));
  ck_assert_uint_eq(glo_fcn, -5 + MSM_GLO_FCN_OFFSET);
}
END_TEST

START_TEST(test_msm_add_to_header) {
  rtcm_msm_header header;
  memset(&header, 0, sizeof(header));
  header.msg_num = 1074;
  msm_add_to_header(&header, CODE_GPS_L1CA, 1);
  msm_add_to_header(&header, CODE_GPS_L1CA, 2);
  msm_add_to_header(&header, CODE_GPS_L1CA, 3);
  msm_add_to_header(&header, CODE_GPS_L2CM, 1);

  ck_assert_uint_eq(msm_get_num_satellites(&header), 3);
  ck_assert_uint_eq(msm_get_num_signals(&header), 2);

  /* invalid code, should get rejected */
  ck_assert(!msm_add_to_header(&header, CODE_INVALID, 1));
  /* invalid PRN, should get rejected */
  ck_assert(!msm_add_to_header(&header, CODE_GPS_L1CA, 50));

  msm_add_to_header(&header, CODE_GPS_L1P, 1);
  msm_add_to_header(&header, CODE_GPS_L2P, 1);
  msm_add_to_header(&header, CODE_GPS_L2CL, 1);
  msm_add_to_header(&header, CODE_GPS_L2CX, 1);
  msm_add_to_header(&header, CODE_GPS_L5I, 1);
  msm_add_to_header(&header, CODE_GPS_L5Q, 1);
  msm_add_to_header(&header, CODE_GPS_L5X, 1);
  msm_add_to_header(&header, CODE_GPS_L1CI, 1);
  msm_add_to_header(&header, CODE_GPS_L1CQ, 1);
  ck_assert_uint_eq(msm_get_num_signals(&header), 11);

  ck_assert(msm_add_to_header(&header, CODE_GPS_L1CA, 4));
  ck_assert_uint_eq(msm_get_num_satellites(&header), 4);
  ck_assert(msm_add_to_header(&header, CODE_GPS_L1CA, 5));
  ck_assert_uint_eq(msm_get_num_satellites(&header), 5);

  /* adding sixth satellite fails because cell mask holds only 64 items */
  ck_assert(!msm_add_to_header(&header, CODE_GPS_L1CA, 6));

  /* can still add signals to existing satellites */
  ck_assert(msm_add_to_header(&header, CODE_GPS_L1P, 5));

  msm_add_to_cell_mask(&header, CODE_GPS_L1CA, 1);
  msm_add_to_cell_mask(&header, CODE_GPS_L1CA, 2);
  msm_add_to_cell_mask(&header, CODE_GPS_L1CA, 3);
  msm_add_to_cell_mask(&header, CODE_GPS_L2CM, 1);
  ck_assert_uint_eq(msm_get_num_cells(&header), 4);

  /* satellite not in cell mask, should get rejected */
  ck_assert(!msm_add_to_cell_mask(&header, CODE_GPS_L1CA, 6));
  /* signal not in cell mask, should get rejected */
  ck_assert(!msm_add_to_cell_mask(&header, CODE_GPS_L1CX, 1));
  /* invalid code, should get rejected */
  ck_assert(!msm_add_to_cell_mask(&header, CODE_INVALID, 1));
  /* invalid PRN, should get rejected */
  ck_assert(!msm_add_to_cell_mask(&header, CODE_GPS_L1CA, 50));
}
END_TEST

START_TEST(test_ura_uri_convertor) {
  uint8_t output_ura;
  for (int i = 0; i < 15; i++) {
    float uri = convert_ura_to_uri(i);
    output_ura = convert_gps_uri_to_ura(uri);
    ck_assert_uint_eq(i, output_ura);
  }
}
END_TEST

START_TEST(test_sisa_meters_convertor) {
  uint8_t output_sisa;
  for (int i = 0; i < 64; i++) {
    float ura = convert_sisa_to_meters(i);
    output_sisa = convert_meters_to_sisa(ura);
    ck_assert_uint_eq(i, output_sisa);
  }
}
END_TEST

// Test base functions for constellation, prn vs sid conversion and validation.
/** Copied from rtcm3_utils.c */
typedef struct {
  u16 first_prn;
  u16 num_sats;
} prn_table_element_t;

static const prn_table_element_t prn_table[RTCM_CONSTELLATION_COUNT] = {
    [RTCM_CONSTELLATION_GPS] = {GPS_FIRST_PRN, NUM_SATS_GPS},
    [RTCM_CONSTELLATION_SBAS] = {SBAS_FIRST_PRN, NUM_SATS_SBAS},
    [RTCM_CONSTELLATION_GLO] = {GLO_FIRST_PRN, NUM_SATS_GLO},
    [RTCM_CONSTELLATION_BDS] = {BDS_FIRST_PRN, NUM_SATS_BDS},
    [RTCM_CONSTELLATION_QZS] = {QZS_FIRST_PRN, NUM_SATS_QZS},
    [RTCM_CONSTELLATION_GAL] = {GAL_FIRST_PRN, NUM_SATS_GAL},
};

START_TEST(test_constellation_sbp2rtcm_roundtrip) {
  rtcm_constellation_t rtcm_cons;
  for (rtcm_cons = RTCM_CONSTELLATION_GPS; rtcm_cons < RTCM_CONSTELLATION_COUNT;
       rtcm_cons++) {
    constellation_t cons = constellation_rtcm2sbp(rtcm_cons);
    ck_assert(rtcm_cons == constellation_sbp2rtcm(cons));
  }

  // Invalid cases
  ck_assert(constellation_rtcm2sbp(RTCM_CONSTELLATION_INVALID) ==
            CONSTELLATION_INVALID);
  ck_assert(constellation_rtcm2sbp(RTCM_CONSTELLATION_COUNT) ==
            CONSTELLATION_INVALID);
  ck_assert(constellation_sbp2rtcm(CONSTELLATION_INVALID) ==
            RTCM_CONSTELLATION_INVALID);
  ck_assert(constellation_sbp2rtcm(CONSTELLATION_COUNT) ==
            RTCM_CONSTELLATION_INVALID);
}

START_TEST(test_constellation_teseov2rtcm) {
  // Valid cases
  ck_assert(constellation_teseov2rtcm(RTCM_TESEOV_GPS) ==
            RTCM_CONSTELLATION_GPS);
  ck_assert(constellation_teseov2rtcm(RTCM_TESEOV_GLO) ==
            RTCM_CONSTELLATION_GLO);
  ck_assert(constellation_teseov2rtcm(RTCM_TESEOV_QZS) ==
            RTCM_CONSTELLATION_QZS);
  ck_assert(constellation_teseov2rtcm(RTCM_TESEOV_GAL) ==
            RTCM_CONSTELLATION_GAL);
  ck_assert(constellation_teseov2rtcm(RTCM_TESEOV_SBAS) ==
            RTCM_CONSTELLATION_SBAS);
  ck_assert(constellation_teseov2rtcm(RTCM_TESEOV_BDS7) ==
            RTCM_CONSTELLATION_BDS);
  ck_assert(constellation_teseov2rtcm(RTCM_TESEOV_BDS13) ==
            RTCM_CONSTELLATION_BDS);

  // Invalid cases
  ck_assert(constellation_teseov2rtcm(RTCM_TESEOV_BDS13 + 1) ==
            RTCM_CONSTELLATION_INVALID);
}

START_TEST(test_satellite_id_teseov2rtcm) {
  rtcm_teseov_constellation_t teseov_cons[] = {RTCM_TESEOV_GPS,
                                               RTCM_TESEOV_GLO,
                                               RTCM_TESEOV_QZS,
                                               RTCM_TESEOV_GAL,
                                               RTCM_TESEOV_SBAS,
                                               RTCM_TESEOV_BDS7};
  uint8_t upper_bound;
  for (size_t i = 0; i < ARRAY_SIZE(teseov_cons); i++) {
    ck_assert(teseov_constellation_valid(teseov_cons[i]));

    ck_assert(teseov_sid_valid(teseov_cons[i], 0));
    ck_assert_uint_eq(satellite_id_teseov2rtcm(teseov_cons[i], 0), 0);

    upper_bound = RTCM_TESEOV_SATELLITE_MASK_SIZE - 1;
    ck_assert(teseov_sid_valid(teseov_cons[i], upper_bound));
    ck_assert_uint_eq(satellite_id_teseov2rtcm(teseov_cons[i], upper_bound),
                      upper_bound);

    // invalid sid
    ck_assert(
        !teseov_sid_valid(teseov_cons[i], RTCM_TESEOV_SATELLITE_MASK_SIZE));
  }

  ck_assert(teseov_constellation_valid(RTCM_TESEOV_BDS13));
  ck_assert_uint_eq(satellite_id_teseov2rtcm(RTCM_TESEOV_BDS13, 0),
                    RTCM_TESEOV_SATELLITE_MASK_SIZE);
  upper_bound = RTCM_TESEOV_SATELLITE_MASK_SIZE_GNSS13 - 1;
  ck_assert(teseov_sid_valid(RTCM_TESEOV_BDS13, upper_bound));
  ck_assert_uint_eq(satellite_id_teseov2rtcm(RTCM_TESEOV_BDS13, upper_bound),
                    (upper_bound + RTCM_TESEOV_SATELLITE_MASK_SIZE));

  // invalid sid
  ck_assert(!teseov_sid_valid(RTCM_TESEOV_BDS13,
                              RTCM_TESEOV_SATELLITE_MASK_SIZE_GNSS13));

  // invalid constellation
  ck_assert(!teseov_constellation_valid(RTCM_TESEOV_BDS13 + 1));
}

START_TEST(test_rtcm_cons_sid_valid) {
  rtcm_constellation_t rtcm_cons;
  for (rtcm_cons = RTCM_CONSTELLATION_GPS; rtcm_cons < RTCM_CONSTELLATION_COUNT;
       rtcm_cons++) {
    // assert rtcm_constellation_valid()
    ck_assert(rtcm_constellation_valid(rtcm_cons));

    // assert rtcm_sid_valid()
    for (uint8_t sid = 0; sid < prn_table[rtcm_cons].num_sats; sid++) {
      ck_assert(rtcm_sid_valid(rtcm_cons, sid));
    }
    ck_assert(!rtcm_sid_valid(rtcm_cons, prn_table[rtcm_cons].num_sats));
    ck_assert(!rtcm_sid_valid(rtcm_cons, UINT8_MAX));
  }

  // assert rtcm_constellation_valid()
  ck_assert(!rtcm_constellation_valid(RTCM_CONSTELLATION_INVALID));
  ck_assert(!rtcm_constellation_valid(RTCM_CONSTELLATION_COUNT));
}

START_TEST(test_satellite_id2prn_roundtrip) {
  for (rtcm_constellation_t rtcm_cons = RTCM_CONSTELLATION_GPS;
       rtcm_cons < RTCM_CONSTELLATION_COUNT;
       rtcm_cons++) {
    ck_assert(rtcm_constellation_valid(rtcm_cons));

    // Test id<->prn conversion
    for (uint8_t sid = 0; sid < prn_table[rtcm_cons].num_sats; sid++) {
      ck_assert(rtcm_sid_valid(rtcm_cons, sid));
      uint8_t prn = satellite_id2prn(rtcm_cons, sid);
      ck_assert(prn_valid(rtcm_cons, prn));
      ck_assert_uint_eq(prn, (sid + prn_table[rtcm_cons].first_prn));

      uint8_t sid_rt = satellite_prn2id(rtcm_cons, prn);
      ck_assert(rtcm_sid_valid(rtcm_cons, sid_rt));
      ck_assert_uint_eq(sid_rt, (prn - prn_table[rtcm_cons].first_prn));

      ck_assert_uint_eq(sid_rt, sid);
    }

    // Test number of satellite each constellation
    ck_assert_uint_eq(constellation_to_num_sats(rtcm_cons),
                      prn_table[rtcm_cons].num_sats);
  }
}

Suite *utils_suite(void) {
  Suite *s = suite_create("Utils");

  TCase *tc_utils = tcase_create("Utilities");
  tcase_add_checked_fixture(tc_utils, utils_setup, NULL);
  tcase_add_test(tc_utils, test_compute_glo_time);
  tcase_add_test(tc_utils, test_glo_time_conversion);
  tcase_add_test(tc_utils, test_msm_sid_conversion);
  tcase_add_test(tc_utils, test_msm_code_prn_conversion);
  tcase_add_test(tc_utils, test_msm_glo_fcn);
  tcase_add_test(tc_utils, test_msm_add_to_header);
  tcase_add_test(tc_utils, test_ura_uri_convertor);
  tcase_add_test(tc_utils, test_sisa_meters_convertor);
  suite_add_tcase(s, tc_utils);

  TCase *tc_cons_utils = tcase_create("Constellation Utils");
  tcase_add_checked_fixture(tc_cons_utils, utils_setup, NULL);
  tcase_add_test(tc_cons_utils, test_constellation_sbp2rtcm_roundtrip);
  tcase_add_test(tc_cons_utils, test_constellation_teseov2rtcm);
  tcase_add_test(tc_cons_utils, test_satellite_id_teseov2rtcm);
  tcase_add_test(tc_cons_utils, test_rtcm_cons_sid_valid);
  tcase_add_test(tc_cons_utils, test_satellite_id2prn_roundtrip);
  suite_add_tcase(s, tc_cons_utils);

  return s;
}
