/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <assert.h>
#include <check.h>
#include <gnss-converters/internal/sbp_nmea_internal.h>
#include <gnss-converters/sbp_nmea.h>
#include <libsbp/sbp.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "nmea_truth.h"

static sbp_msg_callbacks_node_t gps_time_callback_node;
static sbp_msg_callbacks_node_t utc_time_callback_node;
static sbp_msg_callbacks_node_t pos_llh_cov_callback_node;
static sbp_msg_callbacks_node_t vel_ned_callback_node;
static sbp_msg_callbacks_node_t dops_callback_node;
static sbp_msg_callbacks_node_t age_correction_callback_node;
static sbp_msg_callbacks_node_t baseline_heading_callback_node;
static sbp_msg_callbacks_node_t sv_az_el_callback_node;
static sbp_msg_callbacks_node_t measurement_state_callback_node;
static sbp_msg_callbacks_node_t observation_callback_node;

static bool nmea_gpgga_processed = false;
static bool nmea_gprmc_processed = false;
static bool nmea_gpvtg_processed = false;
static bool nmea_gpgll_processed = false;
static bool nmea_gpzda_processed = false;
/*static bool nmea_gphdt_processed = false;*/
static bool nmea_gsa_processed = false;
static bool nmea_gpgst_processed = false;
static bool nmea_gpgsv_processed = false;

static int msg_count[SBP2NMEA_NMEA_CNT];
static bool start_count[SBP2NMEA_NMEA_CNT];

static float cog_thd = 0.1f;
static double cog_stationary_thd = 0;

int32_t read_file(uint8_t *buff, uint32_t n, void *context) {
  FILE *f = (FILE *)context;
  return (int32_t)(fread(buff, 1, n, f));
}

void nmea_callback_gpgga(char msg[], void *ctx) {
  (void)ctx;
  if (strstr((char *)msg, "GGA")) {
    if ((strstr((char *)msg, "193508.20") || start_count[SBP2NMEA_NMEA_GGA]) &&
        msg_count[SBP2NMEA_NMEA_GGA] < 10) {
      nmea_gpgga_processed = true;
      start_count[SBP2NMEA_NMEA_GGA] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gpgga_truth[msg_count[SBP2NMEA_NMEA_GGA]++]) == 0);
    }
  }
}

void nmea_callback_gprmc(char msg[], void *ctx) {
  (void)ctx;
  if (strstr((char *)msg, "RMC")) {
    if ((strstr((char *)msg, "193502.00") || start_count[SBP2NMEA_NMEA_RMC]) &&
        msg_count[SBP2NMEA_NMEA_RMC] < 10) {
      nmea_gprmc_processed = true;
      start_count[SBP2NMEA_NMEA_RMC] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gprmc_truth[msg_count[SBP2NMEA_NMEA_RMC]++]) == 0);
    }
  }
}

void nmea_callback_gpvtg(char msg[], void *ctx) {
  (void)ctx;
  if (strstr((char *)msg, "VTG")) {
    if ((strstr((char *)msg, "$GPVTG,,T,,M,0.02,N,0.04,K,A*25") ||
         start_count[SBP2NMEA_NMEA_VTG]) &&
        msg_count[SBP2NMEA_NMEA_VTG] < 10) {
      nmea_gpvtg_processed = true;
      start_count[SBP2NMEA_NMEA_VTG] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gpvtg_truth[msg_count[SBP2NMEA_NMEA_VTG]++]) == 0);
    }
  }
}

void nmea_callback_gpvtg_cog_forced(char msg[], void *ctx) {
  (void)ctx;
  if (strstr((char *)msg, "VTG")) {
    static int msg_count_cog_forced = 0;
    static bool start_count_cog_forced = false;
    if ((strstr((char *)msg, "$GPVTG,206.6,T,,M,0.02,N,0.04,K,A*09") ||
         start_count_cog_forced) &&
        msg_count_cog_forced < 10) {
      nmea_gpvtg_processed = true;
      start_count_cog_forced = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gpvtg_cog_forced_truth[msg_count_cog_forced++]) == 0);
    }
  }
}

void nmea_callback_gpvtg_cog_forced_stationary_thresh(char msg[], void *ctx) {
  (void)ctx;
  if (strstr((char *)msg, "VTG")) {
    static int msg_count_cog_forced = 0;
    static bool start_count_cog_forced = false;
    if ((strstr((char *)msg, "$GPVTG,206.6,T,,M,0.02,N,0.04,K,A*09") ||
         start_count_cog_forced) &&
        msg_count_cog_forced < 10) {
      nmea_gpvtg_processed = true;
      start_count_cog_forced = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gpvtg_cog_forced_truth_stationary_thresh
                           [msg_count_cog_forced++]) == 0);
    }
  }
}

void nmea_callback_gpgll(char msg[], void *ctx) {
  (void)ctx;
  if (strstr((char *)msg, "GLL")) {
    if ((strstr((char *)msg, "193502.00") || start_count[SBP2NMEA_NMEA_GLL]) &&
        msg_count[SBP2NMEA_NMEA_GLL] < 10) {
      nmea_gpgll_processed = true;
      start_count[SBP2NMEA_NMEA_GLL] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gpgll_truth[msg_count[SBP2NMEA_NMEA_GLL]++]) == 0);
    }
  }
}

void nmea_callback_gpzda(char msg[], void *ctx) {
  (void)ctx;
  if (strstr((char *)msg, "ZDA")) {
    if ((strstr((char *)msg, "193502.00") || start_count[SBP2NMEA_NMEA_ZDA]) &&
        msg_count[SBP2NMEA_NMEA_ZDA] < 10) {
      nmea_gpzda_processed = true;
      start_count[SBP2NMEA_NMEA_ZDA] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(strcmp((char *)msg,
                       gpzda_truth[msg_count[SBP2NMEA_NMEA_ZDA]++]) == 0);
    }
  }
}

/*void nmea_callback_gphdt(u8 msg[]) {
  if(strstr((char*)msg,"GPHDT")){
    static int msg_count = 0;
    static bool start_count = false;
    nmea_gphdt_processed = true;
    if((strstr((char *)msg,"171104.30") || start_count) && msg_count < 10) {
      start_count = true;
      msg [ strcspn((char *)msg, "\r\n") ] = '\0';
      ck_assert(strcmp((char *)msg,gpgga_truth[msg_count]) == 0);
      msg_count++;
    }
  }
} */

void nmea_callback_gsa(char msg[], void *ctx) {
  (void)ctx;
  if (strstr((char *)msg, "GSA")) {
    if ((strstr((char *)msg,
                "$GNGSA,A,3,414,421,426,429,435,436,,,,,,,1.8,0.9,1.6*21") ||
         start_count[SBP2NMEA_NMEA_GSA]) &&
        msg_count[SBP2NMEA_NMEA_GSA] < 10) {
      nmea_gsa_processed = true;
      start_count[SBP2NMEA_NMEA_GSA] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(
          strcmp((char *)msg, gsa_truth[msg_count[SBP2NMEA_NMEA_GSA]++]) == 0);
    }
  }
}

void nmea_callback_gpgst(char msg[], void *ctx) {
  (void)ctx;
  if (strstr((char *)msg, "GST")) {
    if ((strstr((char *)msg, "193501.80") || start_count[SBP2NMEA_NMEA_GST]) &&
        msg_count[SBP2NMEA_NMEA_GST] < 10) {
      nmea_gpgst_processed = true;
      start_count[SBP2NMEA_NMEA_GST] = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(
          strcmp((char *)msg, gst_truth[msg_count[SBP2NMEA_NMEA_GST]++]) == 0);
    }
  }
}

void nmea_callback_gpgsv(char msg[], void *ctx) {
  (void)ctx;
  if (strstr((char *)msg, "GSV")) {
    if (msg_count[SBP2NMEA_NMEA_GSV] < 10) {
      nmea_gpgsv_processed = true;
      msg[strcspn((char *)msg, "\r\n")] = '\0';
      ck_assert(
          strcmp((char *)msg, gsv_truth[msg_count[SBP2NMEA_NMEA_GSV]++]) == 0);
    }
  }
}

void gps_time_callback(uint16_t sender_id,
                       sbp_msg_type_t msg_type,
                       const sbp_msg_t *msg,
                       void *context) {
  (void)sender_id;
  (void)msg_type;
  sbp2nmea(context, msg, SBP2NMEA_SBP_GPS_TIME);
}

void utc_time_callback(uint16_t sender_id,
                       sbp_msg_type_t msg_type,
                       const sbp_msg_t *msg,
                       void *context) {
  (void)sender_id;
  (void)msg_type;
  sbp2nmea(context, msg, SBP2NMEA_SBP_UTC_TIME);
}

void pos_llh_cov_callback(uint16_t sender_id,
                          sbp_msg_type_t msg_type,
                          const sbp_msg_t *msg,
                          void *context) {
  (void)sender_id;
  (void)msg_type;
  sbp2nmea(context, msg, SBP2NMEA_SBP_POS_LLH_COV);
}

void vel_ned_callback(uint16_t sender_id,
                      sbp_msg_type_t msg_type,
                      const sbp_msg_t *msg,
                      void *context) {
  (void)sender_id;
  (void)msg_type;
  sbp2nmea(context, msg, SBP2NMEA_SBP_VEL_NED);
}

void dops_callback(uint16_t sender_id,
                   sbp_msg_type_t msg_type,
                   const sbp_msg_t *msg,
                   void *context) {
  (void)sender_id;
  (void)msg_type;
  sbp2nmea(context, msg, SBP2NMEA_SBP_DOPS);
}

void age_correction_callback(uint16_t sender_id,
                             sbp_msg_type_t msg_type,
                             const sbp_msg_t *msg,
                             void *context) {
  (void)sender_id;
  (void)msg_type;
  sbp2nmea(context, msg, SBP2NMEA_SBP_AGE_CORR);
}

void baseline_heading_callback(uint16_t sender_id,
                               sbp_msg_type_t msg_type,
                               const sbp_msg_t *msg,
                               void *context) {
  (void)sender_id;
  (void)msg_type;
  sbp2nmea(context, msg, SBP2NMEA_SBP_HDG);
}

void sv_az_el_callback(uint16_t sender_id,
                       sbp_msg_type_t msg_type,
                       const sbp_msg_t *msg,
                       void *context) {
  (void)sender_id;
  (void)msg_type;
  sbp2nmea(context, msg, SBP2NMEA_SBP_SV_AZ_EL);
}

void measurement_state_callback(uint16_t sender_id,
                                sbp_msg_type_t msg_type,
                                const sbp_msg_t *msg,
                                void *context) {
  (void)sender_id;
  (void)msg_type;
  sbp2nmea(context, msg, SBP2NMEA_SBP_MEASUREMENT_STATE);
}

void observation_callback(uint16_t sender_id,
                          sbp_msg_type_t msg_type,
                          const sbp_msg_t *msg,
                          void *context) {
  (void)sender_id;
  (void)msg_type;
  sbp2nmea_obs(context, &msg->obs);
}

void sbp_init(sbp_state_t *sbp_state, void *ctx) {
  sbp_state_init(sbp_state);
  sbp_callback_register(sbp_state,
                        SBP_MSG_GPS_TIME,
                        &gps_time_callback,
                        ctx,
                        &gps_time_callback_node);
  sbp_callback_register(sbp_state,
                        SBP_MSG_UTC_TIME,
                        &utc_time_callback,
                        ctx,
                        &utc_time_callback_node);
  sbp_callback_register(sbp_state,
                        SBP_MSG_POS_LLH_COV,
                        &pos_llh_cov_callback,
                        ctx,
                        &pos_llh_cov_callback_node);
  sbp_callback_register(sbp_state,
                        SBP_MSG_VEL_NED,
                        &vel_ned_callback,
                        ctx,
                        &vel_ned_callback_node);
  sbp_callback_register(
      sbp_state, SBP_MSG_DOPS, &dops_callback, ctx, &dops_callback_node);
  sbp_callback_register(sbp_state,
                        SBP_MSG_AGE_CORRECTIONS,
                        &age_correction_callback,
                        ctx,
                        &age_correction_callback_node);
  sbp_callback_register(sbp_state,
                        SBP_MSG_BASELINE_HEADING,
                        &baseline_heading_callback,
                        ctx,
                        &baseline_heading_callback_node);
  sbp_callback_register(sbp_state,
                        SBP_MSG_SV_AZ_EL,
                        &sv_az_el_callback,
                        ctx,
                        &sv_az_el_callback_node);
  sbp_callback_register(sbp_state,
                        SBP_MSG_MEASUREMENT_STATE,
                        &measurement_state_callback,
                        ctx,
                        &measurement_state_callback_node);
  sbp_callback_register(sbp_state,
                        SBP_MSG_OBS,
                        &observation_callback,
                        ctx,
                        &observation_callback_node);
}

void test_NMEA(const char *filename,
               void (*cb_sbp_to_nmea)(char msg[], void *ctx)) {
  memset(&msg_count, 0, sizeof(msg_count));
  memset(&start_count, 0, sizeof(start_count));

  sbp2nmea_t state;
  memset(&state, 0, sizeof(state));

  sbp2nmea_init(&state, SBP2NMEA_MODE_GNSS, cb_sbp_to_nmea, NULL);
  sbp2nmea_base_id_set(&state, 33);
  sbp2nmea_soln_freq_set(&state, 10);
  sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_GGA);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_RMC);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_VTG);
  /*sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_HDT);*/
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_GLL);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_ZDA);
  sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_GSA);
  sbp2nmea_rate_set(&state, 1, SBP2NMEA_NMEA_GST);
  sbp2nmea_rate_set(&state, 10, SBP2NMEA_NMEA_GSV);
  sbp2nmea_cog_threshold_set(&state, cog_thd);
  sbp2nmea_cog_stationary_threshold_set(&state, cog_stationary_thd);

  sbp_state_t sbp_state_;
  sbp_init(&sbp_state_, &state);

  FILE *fp = fopen(filename, "rb");
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }
  sbp_state_set_io_context(&sbp_state_, fp);
  while (!feof(fp)) {
    sbp_process(&sbp_state_, &read_file);
  }
  fclose(fp);
  return;
}

void nmea_setup_basic(void) { return; }

START_TEST(test_nmea_gpgga) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpgga);
  ck_assert(nmea_gpgga_processed);
}
END_TEST

START_TEST(test_nmea_gprmc) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gprmc);
  ck_assert(nmea_gprmc_processed);
}
END_TEST

START_TEST(test_nmea_gpvtg) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpvtg);
  ck_assert(nmea_gpvtg_processed);
  nmea_gpvtg_processed = false;
  cog_thd = 0;
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp",
            nmea_callback_gpvtg_cog_forced);
  ck_assert(nmea_gpvtg_processed);

  /* Test that we only recompute the COG if the SOG is smaller than the
  specified threshold */
  nmea_gpvtg_processed = false;
  cog_stationary_thd = 0.0057;  // Corresponds to 0.011 knots/s
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp",
            nmea_callback_gpvtg_cog_forced_stationary_thresh);
  ck_assert(nmea_gpvtg_processed);
}
END_TEST

/*START_TEST(test_nmea_gphdt) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp",
             nmea_callback_gphdt);
  ck_assert(nmea_gphdt_processed);
}
END_TEST */

START_TEST(test_nmea_gpgll) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpgll);
  ck_assert(nmea_gpgll_processed);
}
END_TEST

START_TEST(test_nmea_gpzda) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpzda);
  ck_assert(nmea_gpzda_processed);
}
END_TEST

START_TEST(test_nmea_gsa) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gsa);
  ck_assert(nmea_gsa_processed);
}
END_TEST

START_TEST(test_nmea_gpgst) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/nmea.sbp", nmea_callback_gpgst);
  ck_assert(nmea_gpgst_processed);
}
END_TEST

START_TEST(test_nmea_gpgsv) {
  test_NMEA(RELATIVE_PATH_PREFIX "/data/azel-sbp.sbp", nmea_callback_gpgsv);
  ck_assert(nmea_gpgsv_processed);
}
END_TEST

static bool check_utc_time_string(const sbp_msg_utc_time_t *msg_time,
                                  const char utc_time[],
                                  const char utc_timedate_trunc[],
                                  const char utc_timedate[]) {
  char utc_str[23];
  get_utc_time_string(true, false, false, msg_time, utc_str, sizeof(utc_str));
  if (strncmp(utc_str, utc_time, 9)) {
    fprintf(stderr, "expected %s, got %s\n", utc_time, utc_str);
    return false;
  }
  if (utc_timedate_trunc) {
    get_utc_time_string(true, true, true, msg_time, utc_str, sizeof(utc_str));
    if (strncmp(utc_str, utc_timedate_trunc, 16)) {
      fprintf(stderr, "expected %s, got %s\n", utc_timedate_trunc, utc_str);
      return false;
    }
  }
  if (utc_timedate) {
    get_utc_time_string(true, true, false, msg_time, utc_str, sizeof(utc_str));
    if (strncmp(utc_str, utc_timedate, 20)) {
      fprintf(stderr, "expected %s, got %s\n", utc_timedate, utc_str);
      return false;
    }
  }
  return true;
}

START_TEST(test_nmea_time_string) {
  sbp_msg_utc_time_t msg_time;
  msg_time.flags = 1;
  msg_time.tow = 0;
  msg_time.year = 2018;
  msg_time.month = 1;
  msg_time.day = 1;
  msg_time.hours = 17;
  msg_time.minutes = 9;
  msg_time.seconds = 31;
  msg_time.ns = 0;

  ck_assert(check_utc_time_string(
      &msg_time, "170931.00", "170931.00,010118", "170931.00,01,01,2018"));

  msg_time.ns = 4000000;
  ck_assert(check_utc_time_string(
      &msg_time, "170931.00", "170931.00,010118", "170931.00,01,01,2018"));

  msg_time.ns = 5000000;
  ck_assert(check_utc_time_string(
      &msg_time, "170931.01", "170931.01,010118", "170931.01,01,01,2018"));

  msg_time.ns = 994999999;
  ck_assert(check_utc_time_string(
      &msg_time, "170931.99", "170931.99,010118", "170931.99,01,01,2018"));

  msg_time.ns = 999999999;
  ck_assert(check_utc_time_string(
      &msg_time, "170932.00", "170932.00,010118", "170932.00,01,01,2018"));

  msg_time.seconds = 59;
  ck_assert(check_utc_time_string(
      &msg_time, "171000.00", "171000.00,010118", "171000.00,01,01,2018"));

  msg_time.minutes = 59;
  ck_assert(check_utc_time_string(
      &msg_time, "180000.00", "180000.00,010118", "180000.00,01,01,2018"));

  msg_time.hours = 23;
  ck_assert(check_utc_time_string(
      &msg_time, "000000.00", "000000.00,020118", "000000.00,02,01,2018"));

  msg_time.day = 31;
  ck_assert(check_utc_time_string(
      &msg_time, "000000.00", "000000.00,010218", "000000.00,01,02,2018"));

  msg_time.month = 12;
  ck_assert(check_utc_time_string(
      &msg_time, "000000.00", "000000.00,010119", "000000.00,01,01,2019"));

  /* leap second */
  msg_time.year = 2016;
  /* TODO: this fails, see the note in nmea.c/get_utc_time_string */
  check_utc_time_string(
      &msg_time, "235960.00", "235960.00,311216", "235960.00,31,12,2016");

  /* leap day */
  msg_time.month = 2;
  msg_time.day = 28;
  ck_assert(check_utc_time_string(
      &msg_time, "000000.00", "000000.00,290216", "000000.00,29,02,2016"));
}
END_TEST

START_TEST(test_check_nmea_rate) {
  // Invalid Entry
  ck_assert(!check_nmea_rate(0, 453568, 10.0));

  // Test cases with solutions at 200ms with output rate at 1s.
  ck_assert(!check_nmea_rate(5, 519092899, 5.0));
  ck_assert(check_nmea_rate(5, 519092900, 5.0));
  ck_assert(check_nmea_rate(5, 519092901, 5.0));
  ck_assert(check_nmea_rate(5, 519092999, 5.0));
  ck_assert(check_nmea_rate(5, 519093001, 5.0));
  ck_assert(check_nmea_rate(5, 519093099, 5.0));
  ck_assert(!check_nmea_rate(5, 519093100, 5.0));

  // Test cases with solutions at 100ms with output rate at 1.9s.
  ck_assert(!check_nmea_rate(19, 1900 - 125, 10.0));
  ck_assert(!check_nmea_rate(19, 1900 - 75, 10.0));
  ck_assert(!check_nmea_rate(19, 1900 - 51, 10.0));
  ck_assert(check_nmea_rate(19, 1900 - 50, 10.0));
  ck_assert(check_nmea_rate(19, 1900 - 49, 10.0));
  ck_assert(check_nmea_rate(19, 1900, 10.0));
  ck_assert(check_nmea_rate(19, 1900 + 49, 10.0));
  ck_assert(!check_nmea_rate(19, 1900 + 50, 10.0));
  ck_assert(!check_nmea_rate(19, 1900 + 75, 10.0));
  ck_assert(!check_nmea_rate(19, 1900 + 125, 10.0));
}

Suite *nmea_suite(void) {
  Suite *s = suite_create("NMEA");

  TCase *tc_nmea = tcase_create("NMEA");
  tcase_add_checked_fixture(tc_nmea, nmea_setup_basic, NULL);
  tcase_add_test(tc_nmea, test_nmea_gpgga);
  tcase_add_test(tc_nmea, test_nmea_gprmc);
  tcase_add_test(tc_nmea, test_nmea_gpvtg);
  /*tcase_add_test(tc_nmea, test_nmea_gphdt);*/
  tcase_add_test(tc_nmea, test_nmea_gpgll);
  tcase_add_test(tc_nmea, test_nmea_gpzda);
  tcase_add_test(tc_nmea, test_nmea_gsa);
  tcase_add_test(tc_nmea, test_nmea_gpgst);
  tcase_add_test(tc_nmea, test_nmea_gpgsv);
  tcase_add_test(tc_nmea, test_nmea_time_string);
  tcase_add_test(tc_nmea, test_check_nmea_rate);
  suite_add_tcase(s, tc_nmea);

  return s;
}
