/*
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

#include <check.h>
#include <libsbp/observation.h>
#include <libsbp/sbp.h>
#include <math.h>
#include <rtcm3/decode.h>
#include <rtcm3/encode.h>
#include <rtcm3/eph_decode.h>
#include <rtcm3/eph_encode.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/bits.h>
#include <swiftnav/edc.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/sid_set.h>

#include "check_rtcm3.h"
#include "check_suites.h"
#include "config.h"

static uint8_t iobuf[4096];
static size_t iobuf_read_idx = 0;
static size_t iobuf_write_idx = 0;
static size_t iobuf_len = 0;
static struct {
  uint16_t msg_id;
  uint16_t sender_id;
  uint8_t len;
  uint8_t payload[256];
} sbp_out_msgs[10];
static size_t n_sbp_out = 0;
time_truth_t time_truth;
static struct rtcm3_sbp_state rtcm2sbp_state;
static struct rtcm3_out_state sbp2rtcm_state;

static int read_iobuf(uint8_t *buf, size_t len, void *ctx) {
  (void)ctx;
  if ((iobuf_len - iobuf_read_idx) < len) {
    len = iobuf_len - iobuf_read_idx;
  }
  memcpy(buf, iobuf + iobuf_read_idx, len);
  iobuf_read_idx += len;
  return (int)len;
}

static s32 save_rtcm_to_iobuf(uint8_t *buf, uint16_t len, void *ctx) {
  (void)ctx;
  assert(iobuf_write_idx + len < sizeof(iobuf));
  memcpy(iobuf + iobuf_write_idx, buf, len);
  iobuf_write_idx += len;
  return len;
}

static void save_sbp_out_cb(
    uint16_t msg_id, uint8_t len, uint8_t *buf, uint16_t sender_id, void *ctx) {
  (void)ctx;
  assert(n_sbp_out < 10);
  sbp_out_msgs[n_sbp_out].msg_id = msg_id;
  sbp_out_msgs[n_sbp_out].sender_id = sender_id;
  sbp_out_msgs[n_sbp_out].len = len;
  memcpy(sbp_out_msgs[n_sbp_out].payload, buf, len);
  n_sbp_out++;
}

static void reset_test_fixture() {
  iobuf_read_idx = 0;
  iobuf_write_idx = 0;
  iobuf_len = 0;
  n_sbp_out = 0;
  time_truth_init(&time_truth);
  rtcm2sbp_init(&rtcm2sbp_state, &time_truth, save_sbp_out_cb, NULL, NULL);
  sbp2rtcm_init(&sbp2rtcm_state, save_rtcm_to_iobuf, NULL);
}

static void reset_test_fixture_unknown(void) { reset_test_fixture(); }

static void reset_test_fixture_solved(uint16_t wn, double tow) {
  reset_test_fixture();
  time_truth_update(
      &time_truth, TIME_TRUTH_EPH_GAL, (gps_time_t){.wn = wn, .tow = tow});
}

static void setup_example_frame() {
  setbitu(iobuf, 0, 8, RTCM3_PREAMBLE);
  setbitu(iobuf, 8, 6, 0);
  setbitu(iobuf, 14, 10, iobuf_len);
  iobuf_len += 3;
  setbitu(iobuf, iobuf_len * 8, 24, crc24q(iobuf, iobuf_len, 0));
  iobuf_len += 3;
  iobuf_read_idx = 0;
}

static void setup_example_1002(uint32_t tow) {
  rtcm_obs_message obs;
  memset(&obs, 0, sizeof(obs));
  obs.header.msg_num = 1002;
  obs.header.tow_ms = tow * 1000;
  obs.header.n_sat = 1;
  obs.sats[0].svId = 1;
  obs.sats[0].fcn = 1;
  obs.sats[0].obs[L1_FREQ].code = 1;
  obs.sats[0].obs[L1_FREQ].pseudorange = 1;
  obs.sats[0].obs[L1_FREQ].carrier_phase = 1;
  obs.sats[0].obs[L1_FREQ].lock = 1;
  obs.sats[0].obs[L1_FREQ].cnr = 1;
  obs.sats[0].obs[L1_FREQ].flags.valid_cp = 1;
  obs.sats[0].obs[L1_FREQ].flags.valid_pr = 1;
  iobuf_len = (size_t)rtcm3_encode_1002(&obs, iobuf + 3);
  setup_example_frame();
}

static void setup_example_1004(uint32_t tow) {
  rtcm_obs_message obs;
  memset(&obs, 0, sizeof(obs));
  obs.header.msg_num = 1004;
  obs.header.tow_ms = tow * 1000;
  obs.header.n_sat = 1;
  obs.sats[0].svId = 1;
  obs.sats[0].fcn = 1;
  obs.sats[0].obs[L1_FREQ].code = 1;
  obs.sats[0].obs[L1_FREQ].pseudorange = 1;
  obs.sats[0].obs[L1_FREQ].carrier_phase = 1;
  obs.sats[0].obs[L1_FREQ].lock = 1;
  obs.sats[0].obs[L1_FREQ].cnr = 1;
  obs.sats[0].obs[L1_FREQ].flags.valid_cp = 1;
  obs.sats[0].obs[L1_FREQ].flags.valid_pr = 1;
  iobuf_len = (size_t)rtcm3_encode_1004(&obs, iobuf + 3);
  setup_example_frame();
}

static void setup_example_1010(uint32_t tow) {
  rtcm_obs_message obs;
  memset(&obs, 0, sizeof(obs));
  obs.header.msg_num = 1010;
  obs.header.tow_ms = tow * 1000;
  obs.header.n_sat = 1;
  obs.sats[0].svId = 1;
  obs.sats[0].fcn = 1;
  obs.sats[0].obs[L1_FREQ].code = 1;
  obs.sats[0].obs[L1_FREQ].pseudorange = 1;
  obs.sats[0].obs[L1_FREQ].carrier_phase = 1;
  obs.sats[0].obs[L1_FREQ].lock = 1;
  obs.sats[0].obs[L1_FREQ].cnr = 1;
  obs.sats[0].obs[L1_FREQ].flags.valid_cp = 1;
  obs.sats[0].obs[L1_FREQ].flags.valid_pr = 1;
  iobuf_len = (size_t)rtcm3_encode_1010(&obs, iobuf + 3);
  setup_example_frame();
}

static void setup_example_1012(uint32_t tow) {
  rtcm_obs_message obs;
  memset(&obs, 0, sizeof(obs));
  obs.header.msg_num = 1012;
  obs.header.tow_ms = tow * 1000;
  obs.header.n_sat = 1;
  obs.sats[0].svId = 1;
  obs.sats[0].fcn = 1;
  obs.sats[0].obs[L1_FREQ].code = 1;
  obs.sats[0].obs[L1_FREQ].pseudorange = 1;
  obs.sats[0].obs[L1_FREQ].carrier_phase = 1;
  obs.sats[0].obs[L1_FREQ].lock = 1;
  obs.sats[0].obs[L1_FREQ].cnr = 1;
  obs.sats[0].obs[L1_FREQ].flags.valid_cp = 1;
  obs.sats[0].obs[L1_FREQ].flags.valid_pr = 1;
  iobuf_len = (size_t)rtcm3_encode_1012(&obs, iobuf + 3);
  setup_example_frame();
}

static void setup_example_msm4(uint16_t msg_num, uint32_t tow) {
  rtcm_msm_message obs;
  memset(&obs, 0, sizeof(obs));
  obs.header.msg_num = msg_num;
  obs.header.tow_ms = tow * 1000;
  obs.header.satellite_mask[1] = true;
  obs.header.signal_mask[1] = true;
  obs.header.cell_mask[0] = true;
  obs.sats[0].rough_range_ms = 74.2900390625;
  obs.sats[0].rough_range_rate_m_s = 615;
  obs.signals[0].pseudorange_ms = 74.290083542466164;
  obs.signals[0].range_rate_m_s = 614.95159999999;
  obs.signals[0].carrier_phase_ms = 74.290111145935953;
  obs.signals[0].hca_indicator = false;
  obs.signals[0].lock_time_s = 770.048;
  obs.signals[0].cnr = 27.875;
  obs.signals[0].flags.valid_cnr = 1;
  obs.signals[0].flags.valid_pr = 1;
  obs.signals[0].flags.valid_cp = 1;
  obs.signals[0].flags.valid_dop = 1;
  obs.signals[0].flags.valid_lock = 1;
  iobuf_len = (size_t)rtcm3_encode_msm4(&obs, iobuf + 3);
  setup_example_frame();
}

static void setup_example_msm5(uint16_t msg_num, uint32_t tow) {
  rtcm_msm_message obs;
  memset(&obs, 0, sizeof(obs));
  obs.header.msg_num = msg_num;
  obs.header.tow_ms = tow * 1000;
  obs.header.satellite_mask[1] = true;
  obs.header.signal_mask[1] = true;
  obs.header.cell_mask[0] = true;
  obs.sats[0].rough_range_ms = 74.2900390625;
  obs.sats[0].rough_range_rate_m_s = 615;
  obs.signals[0].pseudorange_ms = 74.290083542466164;
  obs.signals[0].range_rate_m_s = 614.95159999999;
  obs.signals[0].carrier_phase_ms = 74.290111145935953;
  obs.signals[0].hca_indicator = false;
  obs.signals[0].lock_time_s = 770.048;
  obs.signals[0].cnr = 27.875;
  obs.signals[0].flags.valid_cnr = 1;
  obs.signals[0].flags.valid_pr = 1;
  obs.signals[0].flags.valid_cp = 1;
  obs.signals[0].flags.valid_dop = 1;
  obs.signals[0].flags.valid_lock = 1;
  iobuf_len = (size_t)rtcm3_encode_msm5(&obs, iobuf + 3);
  setup_example_frame();
}

// Get an example GPS orbit for testing
static void setup_example_gps_eph(uint16_t wn, uint32_t tow) {
  msg_ephemeris_gps_t eph;
  eph.common.sid.sat = 25;
  eph.common.sid.code = CODE_GPS_L1CA;
  eph.common.toe.wn = wn;
  eph.common.toe.tow = tow * 1000;
  eph.common.ura = (float)2.8;
  eph.common.fit_interval = 14400;
  eph.common.valid = 1;
  eph.common.health_bits = 0;
  eph.tgd = (float)(-3 * 1e-10);
  eph.c_rc = 167.140625;
  eph.c_rs = -18.828125;
  eph.c_uc = -9.0105459094047546e-07;
  eph.c_us = 9.4850547611713409e-06;
  eph.c_ic = -4.0978193283081055e-08;
  eph.c_is = 1.0104849934577942e-07;
  eph.dn = 3.9023054038264214e-09;
  eph.m0 = 0.39869951815527438;
  eph.ecc = 0.00043709692545235157;
  eph.sqrta = 5282.6194686889648;
  eph.omega0 = 2.2431156200949509;
  eph.omegadot = -6.6892072037584707e-09;
  eph.w = 0.39590413040186828;
  eph.inc = 0.95448398903792575;
  eph.inc_dot = -6.2716898124832475e-10;
  eph.af0 = -0.00050763087347149849;
  eph.af1 = -1.3019807454384136e-11;
  eph.af2 = 0.000000;
  eph.toc.wn = 2022;
  eph.toc.tow = 460800;
  eph.iodc = 250;
  eph.iode = 250;

  iobuf_write_idx = 0;
  struct rtcm3_out_state state;
  sbp2rtcm_init(&state, save_rtcm_to_iobuf, NULL);
  sbp2rtcm_sbp_gps_eph_cb(0x1000, sizeof(eph), (const uint8_t *)&eph, &state);
  iobuf_read_idx = 0;
  iobuf_len = iobuf_write_idx;
}

// Get an example GLO orbit for testing
static void setup_example_glo_eph(uint16_t wn, uint32_t tow) {
  msg_ephemeris_glo_t eph;
  eph.common.sid.sat = 25;
  eph.common.sid.code = CODE_GLO_L1OF;
  eph.common.toe.wn = wn;
  eph.common.toe.tow = tow * 1000;
  eph.common.fit_interval = 2400;
  eph.common.valid = 1;
  eph.common.health_bits = 1;
  eph.common.ura = (float)10.0;
  eph.gamma = (float)(2.4 * 1e-31);
  eph.tau = (float)(1.38 * 1e-10);
  eph.d_tau = (float)(2.56 * 1e-10);
  eph.pos[0] = 647838.345;
  eph.pos[1] = 875308.747;
  eph.pos[2] = 234597.325;
  eph.vel[0] = 24.435;
  eph.vel[1] = 72.7643;
  eph.vel[2] = 55.876;
  eph.acc[0] = (float)(1.3425 * 1e-6);
  eph.acc[1] = (float)(1.765 * 1e-6);
  eph.acc[2] = (float)(2.23 * 1e-6);
  eph.fcn = 15;
  eph.iod = 4;

  iobuf_write_idx = 0;
  struct rtcm3_out_state state;
  sbp2rtcm_init(&state, save_rtcm_to_iobuf, NULL);
  sbp2rtcm_sbp_glo_eph_cb(0x1000, sizeof(eph), (const uint8_t *)&eph, &state);
  iobuf_read_idx = 0;
  iobuf_len = iobuf_write_idx;
}

// Get an example BDS orbit for testing
static void setup_example_bds_eph(uint16_t wn, uint32_t tow) {
  msg_ephemeris_bds_t eph;
  eph.common.sid.sat = 25;
  eph.common.sid.code = CODE_BDS2_B1;
  eph.common.toe.wn = wn;
  eph.common.toe.tow = tow * 1000;
  eph.common.ura = (float)2.8;
  eph.common.fit_interval = 3 * HOUR_SECS;
  eph.common.valid = 1;
  eph.common.health_bits = 0;
  eph.tgd1 = (float)(-3 * 1e-10);
  eph.tgd2 = (float)(-4 * 1e-10);
  eph.c_rc = 167.140625;
  eph.c_rs = -18.828125;
  eph.c_uc = -9.0105459094047546e-07;
  eph.c_us = 9.4850547611713409e-06;
  eph.c_ic = -4.0978193283081055e-08;
  eph.c_is = 1.0104849934577942e-07;
  eph.dn = 3.9023054038264214e-09;
  eph.m0 = 0.39869951815527438;
  eph.ecc = 0.00043709692545235157;
  eph.sqrta = 5282.6194686889648;
  eph.omega0 = 2.2431156200949509;
  eph.omegadot = -6.6892072037584707e-09;
  eph.w = 0.39590413040186828;
  eph.inc = 0.95448398903792575;
  eph.inc_dot = -6.2716898124832475e-10;
  eph.af0 = -0.00050763087347149849;
  eph.af1 = -1.3019807454384136e-11;
  eph.af2 = 0.000000;
  eph.toc.wn = 2022;
  eph.toc.tow = 460814;
  eph.iodc = 954;
  eph.iode = 250;

  iobuf_write_idx = 0;
  struct rtcm3_out_state state;
  sbp2rtcm_init(&state, save_rtcm_to_iobuf, NULL);
  sbp2rtcm_sbp_bds_eph_cb(0x1000, sizeof(eph), (const uint8_t *)&eph, &state);
  iobuf_read_idx = 0;
  iobuf_len = iobuf_write_idx;
}

// Get an example GAL orbit for testing
static void setup_example_gal_eph(uint16_t wn, uint32_t tow) {
  msg_ephemeris_gal_t eph;
  eph.source = EPH_SOURCE_GAL_INAV;
  eph.common.sid.sat = 25;
  eph.common.sid.code = CODE_GAL_E1B;
  eph.common.toe.wn = wn;
  eph.common.toe.tow = tow * 1000;
  eph.common.ura = (float)2.8;
  eph.common.fit_interval = 4 * HOUR_SECS;
  eph.common.valid = 1;
  eph.common.health_bits = 0;
  eph.bgd_e1e5a = (float)(-3 * 1e-10);
  eph.bgd_e1e5b = (float)(-2 * 1e-10);
  eph.c_rc = 167.140625;
  eph.c_rs = -18.828125;
  eph.c_uc = -9.0105459094047546e-07;
  eph.c_us = 9.4850547611713409e-06;
  eph.c_ic = -4.0978193283081055e-08;
  eph.c_is = 1.0104849934577942e-07;
  eph.dn = 3.9023054038264214e-09;
  eph.m0 = 0.39869951815527438;
  eph.ecc = 0.00043709692545235157;
  eph.sqrta = 5282.6194686889648;
  eph.omega0 = 2.2431156200949509;
  eph.omegadot = -6.6892072037584707e-09;
  eph.w = 0.39590413040186828;
  eph.inc = 0.95448398903792575;
  eph.inc_dot = -6.2716898124832475e-10;
  eph.af0 = -0.00050763087347149849;
  eph.af1 = -1.3019807454384136e-11;
  eph.af2 = 0.000000;
  eph.toc.wn = 2022;
  eph.toc.tow = 460800;
  eph.iodc = 954;
  eph.iode = 954;

  iobuf_write_idx = 0;
  struct rtcm3_out_state state;
  sbp2rtcm_init(&state, save_rtcm_to_iobuf, NULL);
  sbp2rtcm_sbp_gal_eph_cb(0x1000, sizeof(eph), (const uint8_t *)&eph, &state);
  iobuf_read_idx = 0;
  iobuf_len = iobuf_write_idx;
}

struct eph_example_pair {
  void (*function)(uint16_t, uint32_t);
  uint16_t msg_id;
};

static const int MSG4_MSG_NUMS[] = {1074, 1084, 1094, 1124};
static const int MSG5_MSG_NUMS[] = {1075, 1085, 1095, 1125};
static void (*const OBS_EXAMPLES[])(uint32_t) = {setup_example_1002,
                                                 setup_example_1004,
                                                 setup_example_1010,
                                                 setup_example_1012};
static const struct eph_example_pair EPH_EXAMPLES[] = {
    {setup_example_bds_eph, SBP_MSG_EPHEMERIS_BDS},
    {setup_example_gal_eph, SBP_MSG_EPHEMERIS_GAL},
    {setup_example_glo_eph, SBP_MSG_EPHEMERIS_GLO},
    {setup_example_gps_eph, SBP_MSG_EPHEMERIS_GPS}};

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an unknown reference absolute
 * GPS time.
 *
 * Test case: A BDS/GAL/GLO/GPS RTCM observation message gets pushed into the
 * converter. Between each observation message the time truth state machine is
 * reset back to its initial state.
 *
 * Expected result: No SBP message will be produced
 */
START_TEST(test_strs_1) {
  for (size_t i = 0; i < sizeof(OBS_EXAMPLES) / sizeof(OBS_EXAMPLES[0]); ++i) {
    reset_test_fixture_unknown();
    OBS_EXAMPLES[i](100);
    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);
    ck_assert(n_sbp_out == 0);
    ck_assert(iobuf_read_idx == iobuf_len);
  }

  for (size_t i = 0; i < sizeof(MSG4_MSG_NUMS) / sizeof(MSG4_MSG_NUMS[0]);
       ++i) {
    reset_test_fixture_unknown();
    setup_example_msm4(MSG4_MSG_NUMS[i], 100);
    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);
    ck_assert(n_sbp_out == 0);
    ck_assert(iobuf_read_idx == iobuf_len);
  }

  for (size_t i = 0; i < sizeof(MSG5_MSG_NUMS) / sizeof(MSG5_MSG_NUMS[0]);
       ++i) {
    reset_test_fixture_unknown();
    setup_example_msm5(MSG5_MSG_NUMS[i], 100);
    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);
    ck_assert(n_sbp_out == 0);
    ck_assert(iobuf_read_idx == iobuf_len);
  }
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an unknown reference absolute
 * GPS time.
 *
 * Test case: A BDS ephemeris message gets pushed into the converter
 *
 * Expected result: The msg_ephemeris_qzss_t::common::toe::wn and
 * msg_ephemeris_qzss_t::common::toe::tow must match that of the associate BDS
 * ephemeris message
 */
START_TEST(test_strs_2) {
  setup_example_bds_eph(2500, 100);

  ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

  ck_assert(n_sbp_out == 1);
  ck_assert(iobuf_read_idx == iobuf_len);
  ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_EPHEMERIS_BDS);
  msg_ephemeris_bds_t *sbp_msg =
      (msg_ephemeris_bds_t *)(sbp_out_msgs[0].payload);
  ck_assert(sbp_msg->common.toe.wn == 2500);
  ck_assert(sbp_msg->common.toe.tow == 100);
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an unknown reference absolute
 * GPS time.
 *
 * Test case: A GAL ephemeris message gets pushed into the converter
 *
 * Expected result: The msg_ephemeris_gal_t::common::toe::wn and
 * msg_ephemeris_gal_t::common::toe::tow must match that of the associate GAL
 * ephemeris message
 */
START_TEST(test_strs_3) {
  setup_example_gal_eph(2500, 100);

  ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

  ck_assert(n_sbp_out == 1);
  ck_assert(iobuf_read_idx == iobuf_len);
  ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_EPHEMERIS_GAL);
  msg_ephemeris_gal_t *sbp_msg =
      (msg_ephemeris_gal_t *)(sbp_out_msgs[0].payload);
  ck_assert(sbp_msg->common.toe.wn == 2500);
  ck_assert(sbp_msg->common.toe.tow == 100);
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an unknown reference absolute
 * GPS time.
 *
 * Test case: A GLO ephemeris message gets pushed into the converter
 *
 * Expected result:  No SBP message will be produced
 */
START_TEST(test_strs_4) {
  setup_example_glo_eph(2500, 100);

  ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

  ck_assert(n_sbp_out == 0);
  ck_assert(iobuf_read_idx == iobuf_len);
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an unknown reference absolute
 * GPS time.
 *
 * Test case: A GPS ephemeris message gets pushed into the converter
 *
 * Expected result:  No SBP message will be produced
 */
START_TEST(test_strs_5) {
  setup_example_gps_eph(2500, 100);

  ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

  ck_assert(n_sbp_out == 0);
  ck_assert(iobuf_read_idx == iobuf_len);
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Absolute GPS time is set to be WN: 2000 and TOW: 500.
 *
 * A BDS/GAL/GLO/GPS RTCM observation message gets pushed into the converter
 * with a +1 second TOW offset (TOW: 501) to the time truth.
 *
 * Expected result: Each SBP observation message should have their
 * msg_obs_t::header:t:wn match WN: 2000 and msg_obs_t::header:t:tow set to
 * match TOW: 501
 */
START_TEST(test_strs_6) {
  msg_obs_t *sbp_obs;

  const uint16_t init_wn = 2000;
  const uint32_t init_tow = 500;
  const uint16_t wn = 2000;
  const uint32_t tow = 501;

  for (size_t i = 0; i < sizeof(OBS_EXAMPLES) / sizeof(OBS_EXAMPLES[0]); ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    OBS_EXAMPLES[i](tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }

  for (size_t i = 0; i < sizeof(MSG4_MSG_NUMS) / sizeof(MSG4_MSG_NUMS[0]);
       ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    setup_example_msm4(MSG4_MSG_NUMS[i], tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }

  for (size_t i = 0; i < sizeof(MSG5_MSG_NUMS) / sizeof(MSG5_MSG_NUMS[0]);
       ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    setup_example_msm5(MSG5_MSG_NUMS[i], tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Absolute GPS time is set to be WN: 2000 and TOW: 500.
 *
 * A BDS/GAL/GLO/GPS RTCM observation message gets pushed into the converter
 * with a -1 second TOW offset (TOW: 499) to the time truth.
 *
 * Expected result: Each SBP observation message should have their
 * msg_obs_t::header:t:wn match WN: 2000 and msg_obs_t::header:t:tow set to
 * match TOW: 499
 */
START_TEST(test_strs_7) {
  msg_obs_t *sbp_obs;

  const uint16_t init_wn = 2000;
  const uint32_t init_tow = 500;
  const uint16_t wn = 2000;
  const uint32_t tow = 499;

  for (size_t i = 0; i < sizeof(OBS_EXAMPLES) / sizeof(OBS_EXAMPLES[0]); ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    OBS_EXAMPLES[i](tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }

  for (size_t i = 0; i < sizeof(MSG4_MSG_NUMS) / sizeof(MSG4_MSG_NUMS[0]);
       ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    setup_example_msm4(MSG4_MSG_NUMS[i], tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }

  for (size_t i = 0; i < sizeof(MSG5_MSG_NUMS) / sizeof(MSG5_MSG_NUMS[0]);
       ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    setup_example_msm5(MSG5_MSG_NUMS[i], tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Absolute GPS time is set to be WN: 2000 and TOW: 500.
 *
 * A BDS/GAL/GLO/GPS RTCM observation message gets pushed into the converter
 * with a +302400 second (half week) TOW offset (TOW: 302900) to the time truth.
 *
 * Expected result: Each SBP observation message should have their
 * msg_obs_t::header:t:wn match WN: 1999 and msg_obs_t::header:t:tow set to
 * match TOW: 302900
 */
START_TEST(test_strs_8) {
  msg_obs_t *sbp_obs;

  const uint16_t init_wn = 2000;
  const uint32_t init_tow = 500;
  const uint16_t wn = 1999;
  const uint32_t tow = 302900;

  for (size_t i = 0; i < sizeof(OBS_EXAMPLES) / sizeof(OBS_EXAMPLES[0]); ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    OBS_EXAMPLES[i](tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }

  for (size_t i = 0; i < sizeof(MSG4_MSG_NUMS) / sizeof(MSG4_MSG_NUMS[0]);
       ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    setup_example_msm4(MSG4_MSG_NUMS[i], tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }

  for (size_t i = 0; i < sizeof(MSG5_MSG_NUMS) / sizeof(MSG5_MSG_NUMS[0]);
       ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    setup_example_msm5(MSG5_MSG_NUMS[i], tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Absolute GPS time is set to be WN: 2000 and TOW: 400000.
 *
 * A BDS/GAL/GLO/GPS RTCM observation message gets pushed into the converter
 * with a -302400 second (half week) TOW offset (TOW: 97600) to the time truth.
 *
 * Expected result: Each SBP observation message should have their
 * msg_obs_t::header:t:wn match WN: 2001 and msg_obs_t::header:t:tow set to
 * match TOW: 97600
 */
START_TEST(test_strs_9) {
  msg_obs_t *sbp_obs;

  const uint16_t init_wn = 2000;
  const uint32_t init_tow = 400000;
  const uint16_t wn = 2001;
  const uint32_t tow = 97600;

  for (size_t i = 0; i < sizeof(OBS_EXAMPLES) / sizeof(OBS_EXAMPLES[0]); ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    OBS_EXAMPLES[i](tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }

  for (size_t i = 0; i < sizeof(MSG4_MSG_NUMS) / sizeof(MSG4_MSG_NUMS[0]);
       ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    setup_example_msm4(MSG4_MSG_NUMS[i], tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }

  for (size_t i = 0; i < sizeof(MSG5_MSG_NUMS) / sizeof(MSG5_MSG_NUMS[0]);
       ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    setup_example_msm5(MSG5_MSG_NUMS[i], tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_OBS);
    ck_assert(sbp_out_msgs[0].len ==
              sizeof(msg_obs_t) + sizeof(packed_obs_content_t));
    sbp_obs = (msg_obs_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_obs->header.n_obs == 0x10);
    ck_assert(sbp_obs->header.t.wn == wn);
    ck_assert(sbp_obs->header.t.tow == tow);
  }
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case:
 *
 * Absolute GPS time is set to be WN: 2000 and TOW: 500.
 *
 * A BDS/GAL/GLO/GPS ephemeris message gets pushed into the converter with a +1
 * second TOW offset (TOW: 501) to the time truth.
 *
 * Expected result: The SBP message’s *::common::toe::wn and *::common::toe::tow
 * must match that of the associate ephemeris message
 */
START_TEST(test_strs_10) {
  msg_ephemeris_bds_t *sbp_bds_eph;

  const uint16_t init_wn = 2000;
  const uint32_t init_tow = 500;
  const uint16_t wn = 2000;
  const uint32_t tow = 501;

  for (size_t i = 0; i < sizeof(EPH_EXAMPLES) / sizeof(EPH_EXAMPLES[0]); ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    EPH_EXAMPLES[i].function(wn, tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == EPH_EXAMPLES[i].msg_id);
    sbp_bds_eph = (msg_ephemeris_bds_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_bds_eph->common.toe.wn == wn);
    ck_assert(sbp_bds_eph->common.toe.tow == tow);
  }
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Absolute GPS time is set to be WN: 2000 and TOW: 500.
 *
 * A BDS/GAL/GLO/GPS ephemeris message gets pushed into the converter with a -1
 * second TOW offset (TOW: 499) to the time truth.
 *
 * Expected result: The SBP message’s *::common::toe::wn and *::common::toe::tow
 * must match that of the associate ephemeris message
 */
START_TEST(test_strs_11) {
  msg_ephemeris_bds_t *sbp_bds_eph;

  const uint16_t init_wn = 2000;
  const uint32_t init_tow = 500;
  const uint16_t wn = 2000;
  const uint32_t tow = 499;

  for (size_t i = 0; i < sizeof(EPH_EXAMPLES) / sizeof(EPH_EXAMPLES[0]); ++i) {
    reset_test_fixture_solved(init_wn, init_tow);
    EPH_EXAMPLES[i].function(wn, tow);

    ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

    ck_assert(n_sbp_out == 1);
    ck_assert(iobuf_read_idx == iobuf_len);
    ck_assert(sbp_out_msgs[0].msg_id == EPH_EXAMPLES[i].msg_id);
    sbp_bds_eph = (msg_ephemeris_bds_t *)(sbp_out_msgs[0].payload);
    ck_assert(sbp_bds_eph->common.toe.wn == wn);
    ck_assert(sbp_bds_eph->common.toe.tow == tow);
  }
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Truth time state machine is setup with a known absolute GPS time
 * of WN: 1023 and TOW: 604799 (a second before GPS’s first roll over of WN:
 * 1024 and TOW: 0).
 *
 * A GPS ephemeris message gets pushed into the converter with WN: 0 and TOW: 0.
 *
 * Expected result: The SBP message’s common::toe::wn and common::toe::tow must
 * match WN: 1024 and TOW: 0
 */
START_TEST(test_strs_12) {
  reset_test_fixture_solved(1023, 604799);
  setup_example_gps_eph(1024, 0);  // Will be encoded as wn == 0

  ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

  ck_assert(n_sbp_out == 1);
  ck_assert(iobuf_read_idx == iobuf_len);
  ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_EPHEMERIS_GPS);
  msg_ephemeris_gps_t *sbp_msg =
      (msg_ephemeris_gps_t *)(sbp_out_msgs[0].payload);
  ck_assert(sbp_msg->common.toe.wn == 1024);
  ck_assert(sbp_msg->common.toe.tow == 0);
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Truth time state machine is setup with a known absolute GPS time
 * of WN: 1024 and TOW: 0 (at GPS’s first roll over).
 *
 * A GPS ephemeris message gets pushed into the converter with WN: 0 and TOW:
 * 604799.
 *
 * Expected result: The SBP message’s common::toe::wn and common::toe::tow must
 * match WN: 1023 and TOW: 604799
 */
START_TEST(test_strs_13) {
  reset_test_fixture_solved(1024, 0);
  setup_example_gps_eph(0, 604799);

  ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

  ck_assert(n_sbp_out == 1);
  ck_assert(iobuf_read_idx == iobuf_len);
  ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_EPHEMERIS_GPS);
  msg_ephemeris_gps_t *sbp_msg =
      (msg_ephemeris_gps_t *)(sbp_out_msgs[0].payload);
  ck_assert(sbp_msg->common.toe.wn == 1023);
  ck_assert(sbp_msg->common.toe.tow == 604799);
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Truth time state machine is setup with a known absolute GPS time
 * of WN: 5119 and TOW: 604799 (GAL’s first roll over happens a second before
 * WN: 5120 TOW: 0, its WN is calculated as GAL_WEEK_TO_GPS_WEEK + 2^12).
 *
 * A GAL ephemeris message gets pushed into the converter with WN: 0 and TOW: 0.
 *
 * Expected result: The SBP message’s common::toe::wn and common::toe::tow must
 * match WN: 5120 and TOW: 0
 */
START_TEST(test_strs_14) {
  reset_test_fixture_solved(5119, 604799);
  setup_example_gal_eph(0, 0);

  ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

  ck_assert(n_sbp_out == 1);
  ck_assert(iobuf_read_idx == iobuf_len);
  ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_EPHEMERIS_GAL);
  msg_ephemeris_gal_t *sbp_msg =
      (msg_ephemeris_gal_t *)(sbp_out_msgs[0].payload);
  ck_assert(sbp_msg->common.toe.wn == 5120);
  ck_assert(sbp_msg->common.toe.tow == 0);
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Truth time state machine is setup with a known absolute GPS time
 * of WN: 5120 and TOW: 0 (GAL’s first roll over).
 *
 * A GAL ephemeris message gets pushed into the converter with WN: 5119 and TOW:
 * 604799.
 *
 * Expected result: The SBP message’s common::toe::wn and common::toe::tow must
 * match WN: 5119 and TOW: 604799
 */
START_TEST(test_strs_15) {
  reset_test_fixture_solved(5120, 0);
  setup_example_gal_eph(5119, 604799);

  ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

  ck_assert(n_sbp_out == 1);
  ck_assert(iobuf_read_idx == iobuf_len);
  ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_EPHEMERIS_GAL);
  msg_ephemeris_gal_t *sbp_msg =
      (msg_ephemeris_gal_t *)(sbp_out_msgs[0].payload);
  ck_assert(sbp_msg->common.toe.wn == 5119);
  ck_assert(sbp_msg->common.toe.tow == 604799);
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Truth time state machine is setup with a known absolute GPS time
 * of WN: 9548 and TOW: 13 (BDS’s first roll over happens a seconds before WN:
 * 9548 and TOW: 14, the WN is equal to BDS_WEEK_TO_GPS_WEEK + 2 ^ 13 and TOW:
 * BDS_SECOND_TO_GPS_SECOND).
 *
 * A BDS ephemeris message gets pushed into the converter with WN: 0 and TOW: 0.
 *
 * Expected result: The SBP message’s common::toe::wn and common::toe::tow must
 * match WN: 9548 and TOW: 14
 */
START_TEST(test_strs_16) {
  reset_test_fixture_solved(9548, 13);
  setup_example_bds_eph(0, 0);

  ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

  ck_assert(n_sbp_out == 1);
  ck_assert(iobuf_read_idx == iobuf_len);
  ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_EPHEMERIS_BDS);
  msg_ephemeris_gal_t *sbp_msg =
      (msg_ephemeris_gal_t *)(sbp_out_msgs[0].payload);
  ck_assert(sbp_msg->common.toe.wn == 9548);
  ck_assert(sbp_msg->common.toe.tow == 14);
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Truth time state machine is setup with a known absolute GPS time
 * of WN: 9548 and TOW: 14 (BDS’s first roll over).
 *
 * A BDS ephemeris message gets pushed into the converter with WN: 9548 and
 * TOW: 13.
 *
 * Expected result: Truth time state machine is setup with a known absolute GPS
 * time of WN: 9548 and TOW: 14 (BDS’s first roll over).
 *
 * A BDS ephemeris message gets pushed into the converter with WN: 9548 and
 * TOW: 13.
 */
START_TEST(test_strs_17) {
  reset_test_fixture_solved(9548, 14);
  setup_example_bds_eph(9548, 13);

  ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

  ck_assert(n_sbp_out == 1);
  ck_assert(iobuf_read_idx == iobuf_len);
  ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_EPHEMERIS_BDS);
  msg_ephemeris_gal_t *sbp_msg =
      (msg_ephemeris_gal_t *)(sbp_out_msgs[0].payload);
  ck_assert(sbp_msg->common.toe.wn == 9548);
  ck_assert(sbp_msg->common.toe.tow == 13);
}
END_TEST

/*
 * Setup: The following tests cases apply to the RTCM to SBP converter which has
 * been setup with a time truth object that has an known reference absolute GPS
 * time. With all these test cases, the time truth object should never be
 * updated.
 *
 * Test case: Truth time state machine is setup with a known absolute GPS time
 * of WN: 1512 and TOW: 345615 (a second after 01-01-2009 leap seconds).
 *
 * Two GLO observation message gets pushed into the converter
 *
 * Expected result: RTCM to SBP converter determines leap second to be equal to
 * 15 once.
 */
START_TEST(test_strs_18) {
  reset_test_fixture_solved(1512, 345615);
  setup_example_glo_eph(1512, 345615);

  ck_assert(rtcm2sbp_process(&rtcm2sbp_state, read_iobuf) == (int)iobuf_len);

  ck_assert(n_sbp_out == 1);
  ck_assert(iobuf_read_idx == iobuf_len);
  ck_assert(sbp_out_msgs[0].msg_id == SBP_MSG_EPHEMERIS_GLO);
  msg_ephemeris_gal_t *sbp_msg =
      (msg_ephemeris_gal_t *)(sbp_out_msgs[0].payload);
  ck_assert(sbp_msg->common.toe.wn == 1512);
  ck_assert(sbp_msg->common.toe.tow == 345615);
  ck_assert(rtcm2sbp_state.leap_second_known);
  ck_assert(rtcm2sbp_state.leap_seconds == 15);
}
END_TEST

Suite *rtcm3_time_suite(void) {
  Suite *s = suite_create("RTCMv3 time");

  (void)test_strs_1;
  (void)test_strs_2;
  (void)test_strs_3;
  (void)test_strs_4;
  (void)test_strs_5;
  (void)test_strs_6;
  (void)test_strs_7;
  (void)test_strs_8;
  (void)test_strs_9;
  (void)test_strs_10;
  (void)test_strs_11;
  (void)test_strs_12;
  (void)test_strs_13;
  (void)test_strs_14;
  (void)test_strs_15;
  (void)test_strs_16;
  (void)test_strs_17;
  (void)test_strs_18;

  TCase *tc_rtcm3_time = tcase_create("time truth");
  tcase_add_checked_fixture(tc_rtcm3_time, reset_test_fixture_unknown, NULL);
  // tcase_add_test(tc_rtcm3_time, test_strs_1);
  // tcase_add_test(tc_rtcm3_time, test_strs_2);
  // tcase_add_test(tc_rtcm3_time, test_strs_3);
  // tcase_add_test(tc_rtcm3_time, test_strs_4);
  // tcase_add_test(tc_rtcm3_time, test_strs_5);
  // tcase_add_test(tc_rtcm3_time, test_strs_6);
  // tcase_add_test(tc_rtcm3_time, test_strs_7);
  // tcase_add_test(tc_rtcm3_time, test_strs_8);
  // tcase_add_test(tc_rtcm3_time, test_strs_9);
  // tcase_add_test(tc_rtcm3_time, test_strs_10);
  // tcase_add_test(tc_rtcm3_time, test_strs_11);
  // tcase_add_test(tc_rtcm3_time, test_strs_12);
  // tcase_add_test(tc_rtcm3_time, test_strs_13);
  // tcase_add_test(tc_rtcm3_time, test_strs_14);
  // tcase_add_test(tc_rtcm3_time, test_strs_15);
  // tcase_add_test(tc_rtcm3_time, test_strs_16);
  // tcase_add_test(tc_rtcm3_time, test_strs_17);
  // tcase_add_test(tc_rtcm3_time, test_strs_18);
  suite_add_tcase(s, tc_rtcm3_time);

  return s;
}
