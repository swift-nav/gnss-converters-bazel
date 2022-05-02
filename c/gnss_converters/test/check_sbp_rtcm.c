/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <check.h>
#include <gnss-converters/sbp_rtcm3.h>
#include <gnss-converters/time_truth_v2.h>
#include <libsbp/v4/observation.h>
#include <swiftnav/signal.h>

#include "time_truth.h"

static const sbp_msg_t sbp_glonass_ephemeride = {
    .ephemeris_glo = {
        .common.sid.sat = 25,
        .common.sid.code = CODE_GLO_L1OF,
        .common.toe.wn = 2022,
        .common.toe.tow = 220518,
        .common.fit_interval = 2400,
        .common.valid = 1,
        .common.health_bits = 1,
        .common.ura = (float)10.0,
        .gamma = (float)(2.4 * 1e-31),
        .tau = (float)(1.38 * 1e-10),
        .d_tau = (float)(2.56 * 1e-10),
        .pos[0] = 647838.345,
        .pos[1] = 875308.747,
        .pos[2] = 234597.325,
        .vel[0] = 24.435,
        .vel[1] = 72.7643,
        .vel[2] = 55.876,
        .acc[0] = (float)(1.3425 * 1e-6),
        .acc[1] = (float)(1.765 * 1e-6),
        .acc[2] = (float)(2.23 * 1e-6),
        .fcn = 15,
        .iod = 4,
    }};

static const uint8_t rtcm_glonass_ephemeride[] = {
    0xd3, 0x00, 0x2d, 0x3f, 0xc6, 0x5d, 0x20, 0x01, 0x41, 0x00, 0x64,
    0x16, 0x02, 0x87, 0xd6, 0xa1, 0x01, 0x2a, 0x0b, 0x03, 0x6b, 0x4f,
    0x02, 0x00, 0xe4, 0xde, 0x00, 0xea, 0x98, 0xe2, 0x00, 0x02, 0x00,
    0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xc5, 0x78, 0xbd};
static const uint8_t rtcm_glonass_ephemeride_unhealthy[] = {
    0xd3, 0x00, 0x2d, 0x3f, 0xc6, 0x5d, 0x20, 0x01, 0x00, 0x00, 0x64,
    0x16, 0x02, 0x87, 0xd6, 0xa1, 0x01, 0x2a, 0x0b, 0x03, 0x6b, 0x4f,
    0x02, 0x00, 0xe4, 0xde, 0x00, 0xea, 0x98, 0xe2, 0x00, 0x02, 0x00,
    0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xb9, 0x02, 0x41};

static size_t sbp_rtcm_callback_invoked = 0;
static void fixture(void) { sbp_rtcm_callback_invoked = 0; }

typedef struct {
  uint16_t length;
  const uint8_t *frame;
} test_glonass_context_t;
static int32_t test_glonass_callback(uint8_t *frame, u16 length, void *ctx) {
  ck_assert_ptr_ne(ctx, NULL);
  ++sbp_rtcm_callback_invoked;

  test_glonass_context_t *context = (test_glonass_context_t *)ctx;
  ck_assert_uint_eq(context->length, length);
  ck_assert_mem_eq(context->frame, frame, length);
  return length;
}

static bool test_glonass_unix_time_callback(int64_t *now) {
  *now = 1539090900;
  return true;
}

START_TEST(test_glo_no_leap_seconds) {
  test_glonass_context_t context = {
      .length = ARRAY_SIZE(rtcm_glonass_ephemeride_unhealthy),
      .frame = rtcm_glonass_ephemeride_unhealthy};

  struct rtcm3_out_state state;
  sbp2rtcm_init(&state, test_glonass_callback, &context);
  sbp2rtcm_sbp_cb(0, SbpMsgEphemerisGlo, &sbp_glonass_ephemeride, &state);
  ck_assert_uint_eq(sbp_rtcm_callback_invoked, 1);
}
END_TEST

START_TEST(test_glo_user_leap_seconds) {
  static const int8_t leap_seconds = 18;

  test_glonass_context_t context = {
      .length = ARRAY_SIZE(rtcm_glonass_ephemeride),
      .frame = rtcm_glonass_ephemeride};

  struct rtcm3_out_state state;
  sbp2rtcm_init(&state, test_glonass_callback, &context);
  sbp2rtcm_set_leap_second(&leap_seconds, &state);
  sbp2rtcm_sbp_cb(0, SbpMsgEphemerisGlo, &sbp_glonass_ephemeride, &state);
  ck_assert_uint_eq(sbp_rtcm_callback_invoked, 1);
}
END_TEST

START_TEST(test_glo_unix_clock_callback) {
  test_glonass_context_t context = {
      .length = ARRAY_SIZE(rtcm_glonass_ephemeride),
      .frame = rtcm_glonass_ephemeride};

  struct rtcm3_out_state state;
  sbp2rtcm_init(&state, test_glonass_callback, &context);
  sbp2rtcm_set_unix_time_callback_function(test_glonass_unix_time_callback,
                                           &state);
  sbp2rtcm_sbp_cb(0, SbpMsgEphemerisGlo, &sbp_glonass_ephemeride, &state);
  ck_assert_uint_eq(sbp_rtcm_callback_invoked, 1);
}
END_TEST

START_TEST(test_glo_time_truth) {
  static const gps_time_t gps_time = {.wn = 2022, .tow = 220518};
  static const int8_t leap_seconds = 18;

  test_glonass_context_t context = {
      .length = ARRAY_SIZE(rtcm_glonass_ephemeride),
      .frame = rtcm_glonass_ephemeride};

  ObservationTimeEstimator *observation_time_estimator;
  EphemerisTimeEstimator *ephemeris_time_estimator;
  Rtcm1013TimeEstimator *leap_second_time_estimator;

  time_truth_reset(time_truth);
  ck_assert(time_truth_request_observation_time_estimator(
      time_truth, TIME_TRUTH_SOURCE_LOCAL, &observation_time_estimator));
  ck_assert(time_truth_request_ephemeris_time_estimator(
      time_truth, TIME_TRUTH_SOURCE_LOCAL, &ephemeris_time_estimator));
  ck_assert(time_truth_request_rtcm_1013_time_estimator(
      time_truth, TIME_TRUTH_SOURCE_LOCAL, &leap_second_time_estimator));

  time_truth_observation_estimator_push(observation_time_estimator,
                                        gps_time.tow * SECS_MS);
  for (uint16_t i = 0; i < NUM_SATS_GPS; ++i) {
    time_truth_ephemeris_estimator_push(
        ephemeris_time_estimator, (gnss_signal_t){i, CODE_GPS_L1CA}, gps_time);
  }
  time_truth_rtcm_1013_estimator_push(
      leap_second_time_estimator, gps_time, leap_seconds);

  struct rtcm3_out_state state;
  sbp2rtcm_init(&state, test_glonass_callback, &context);
  sbp2rtcm_set_time_truth(time_truth, NULL, &state);
  sbp2rtcm_sbp_cb(0, SbpMsgEphemerisGlo, &sbp_glonass_ephemeride, &state);
  ck_assert_uint_eq(sbp_rtcm_callback_invoked, 1);
}
END_TEST

Suite *sbp_rtcm_suite(void) {
  Suite *suite = suite_create("SBP to RTCM Converter");

  TCase *core_test_cases = tcase_create("Core");
  tcase_add_checked_fixture(core_test_cases, fixture, NULL);
  tcase_add_test(core_test_cases, test_glo_no_leap_seconds);
  tcase_add_test(core_test_cases, test_glo_user_leap_seconds);
  tcase_add_test(core_test_cases, test_glo_unix_clock_callback);
  tcase_add_test(core_test_cases, test_glo_time_truth);
  suite_add_tcase(suite, core_test_cases);

  return suite;
}
