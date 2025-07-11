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

#include <assert.h>
#include <ephemeris/sbas.h>
#include <gnss-converters/internal/rtcm3_sbp_internal.h>
#include <gnss-converters/internal/rtcm3_utils.h>
#include <math.h>
#include <swiftnav/constants.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/float_equality.h>

float convert_glo_ft_to_meters(const uint8_t ft) {
  /* Convert between RTCM/GLO FT ("GLONASS-M predicted satellite user range
   * accuracy") index to a number in meters.
   * See table 4.4 in GLO signal specification.
   * Indices 1, 3, and 5 are hard-coded according to spec, and 15 is hard-coded
   * according to SBP/Piksi convention. */
  switch (ft) {
    case 0:
      return 1.0f;
    case 1:
      return 2.0f;
    case 2:
      return 2.5f;
    case 3:
      return 4.0f;
    case 4:
      return 5.0f;
    case 5:
      return 7.0f;
    case 6:
      return 10.0f;
    case 7:
      return 12.0f;
    case 8:
      return 14.0f;
    case 9:
      return 16.0f;
    case 10:
      return 32.0f;
    case 11:
      return 64.0f;
    case 12:
      return 128.0f;
    case 13:
      return 256.0f;
    case 14:
      return 512.0f;
    case 15:
      return 6144.0f;
    default:
      return -1;
  }
}

float convert_bds_ura_to_meters(const uint8_t ura) {
  /* (meters See: BDS ICD Section 5.2.4.: to define nominal
values, N = 0-6: use 2^(1+N/2) (round to one
decimal place i.e. 2.8, 5.7 and 11.3) , N=
7-15:use 2^(N-2), 8192 specifies use at own
risk) */
  if (ura >= ARRAY_SIZE(g_bds_ura_table)) {
    return -1;
  }
  float m = g_bds_ura_table[ura];
  if (float_equal(INVALID_URA_VALUE, m)) {
    return 8192.0f;
  }
  return m;
}

/** Calculate the GLO ephemeris curve fit interval.
 *
 * \param fit_interval_flag The curve fit interval flag. 0 is 4 hours, 1 is >4
 * hours.
 * \return the curve fit interval in seconds.
 */
u32 rtcm3_decode_fit_interval_glo(const u8 p1) {
  switch (p1) {
    case 0:
      return (60 + 10) * 60;
    case 1:
      return (30 + 10) * 60;
    case 2:
      return (45 + 10) * 60;
    case 3:
      return (60 + 10) * 60;
    default:
      return (60 + 10) * 60;
  }
}

gps_time_t week_rollover_adjustment(gps_time_t constellation_time,
                                    gps_time_t absolute_gps_reference_time,
                                    uint8_t constellation_wn_resolution,
                                    gps_time_t gps_constellation_offset) {
  const uint16_t week_rollover_period = 1 << constellation_wn_resolution;
  const double week_rollover_period_seconds =
      ((double)week_rollover_period) * WEEK_SECS;
  const double gps_constellation_offset_seconds =
      ((double)gps_constellation_offset.wn) * WEEK_SECS +
      gps_constellation_offset.tow;

  gps_time_t absolute_reference_time = absolute_gps_reference_time;
  add_secs(&absolute_reference_time, -gps_constellation_offset_seconds);

  gps_time_t reference_time = absolute_reference_time;
  reference_time.wn = absolute_reference_time.wn % week_rollover_period;

  double delta = gpsdifftime(&constellation_time, &reference_time);
  if (delta < 0) {
    delta += week_rollover_period_seconds;
  }

  gps_time_t adjusted_time = absolute_reference_time;
  add_secs(&adjusted_time, delta);
  add_secs(&adjusted_time, gps_constellation_offset_seconds);

  return adjusted_time;
}

static bool sbp_gps_time_sec_valid(const sbp_gps_time_sec_t *t) {
  gps_time_t time_to_verify = {.wn = t->wn, .tow = t->tow};
  return gps_time_valid(&time_to_verify);
}

bool rtcm3_gps_eph_to_sbp(const rtcm_msg_eph *msg_eph,
                          sbp_msg_ephemeris_gps_t *sbp_gps_eph,
                          struct rtcm3_sbp_state *state) {
  assert(msg_eph);
  assert(sbp_gps_eph);
  assert(RTCM_CONSTELLATION_GPS == msg_eph->constellation);

  /* GPS week is 10 bit (DF076) */
  static const uint16_t GPS_WN_RESOLUTION = 10;
  static const gps_time_t GPS_GPS_OFFSET = {.wn = 0, .tow = 0};

  const gps_time_t toe_time = week_rollover_adjustment(
      (gps_time_t){.wn = msg_eph->wn, .tow = msg_eph->toe * GPS_TOE_RESOLUTION},
      (gps_time_t){.wn = state->gps_week_reference, .tow = 0},
      GPS_WN_RESOLUTION,
      GPS_GPS_OFFSET);

  if (state->ephemeris_time_estimator != NULL && msg_eph->health_bits == 0) {
    gnss_signal_t gnss_signal = {.sat = msg_eph->sat_id, .code = CODE_GPS_L1CA};
    time_truth_ephemeris_estimator_push(
        state->ephemeris_time_estimator, gnss_signal, toe_time);
  }

  sbp_gps_eph->common.toe.wn = toe_time.wn;
  sbp_gps_eph->common.toe.tow = msg_eph->toe * GPS_TOE_RESOLUTION;

  if (!sbp_gps_time_sec_valid(&sbp_gps_eph->common.toe)) {
    return false;
  }

  sbp_gps_eph->common.sid.sat = msg_eph->sat_id;
  sbp_gps_eph->common.sid.code = CODE_GPS_L1CA;
  sbp_gps_eph->common.ura = convert_ura_to_uri(msg_eph->ura);
  sbp_gps_eph->common.fit_interval = rtcm3_decode_fit_interval_gps(
      msg_eph->fit_interval, msg_eph->data.kepler.iodc);
  sbp_gps_eph->common.valid =
      (msg_eph->data.kepler.iodc & 0xFF) == msg_eph->data.kepler.iode;
  sbp_gps_eph->common.health_bits = msg_eph->health_bits;

  sbp_gps_eph->tgd = (float)(msg_eph->data.kepler.tgd.gps_s * C_1_2P31);

  sbp_gps_eph->c_rs = (float)(msg_eph->data.kepler.crs * C_1_2P5);
  sbp_gps_eph->c_rc = (float)(msg_eph->data.kepler.crc * C_1_2P5);
  sbp_gps_eph->c_uc = (float)(msg_eph->data.kepler.cuc * C_1_2P29);
  sbp_gps_eph->c_us = (float)(msg_eph->data.kepler.cus * C_1_2P29);
  sbp_gps_eph->c_ic = (float)(msg_eph->data.kepler.cic * C_1_2P29);
  sbp_gps_eph->c_is = (float)(msg_eph->data.kepler.cis * C_1_2P29);

  sbp_gps_eph->dn = msg_eph->data.kepler.dn * C_1_2P43 * M_PI;
  sbp_gps_eph->m0 = msg_eph->data.kepler.m0 * C_1_2P31 * M_PI;
  sbp_gps_eph->ecc = msg_eph->data.kepler.ecc * C_1_2P33;
  sbp_gps_eph->sqrta = msg_eph->data.kepler.sqrta * C_1_2P19;
  sbp_gps_eph->omega0 = msg_eph->data.kepler.omega0 * C_1_2P31 * M_PI;
  sbp_gps_eph->omegadot = msg_eph->data.kepler.omegadot * C_1_2P43 * M_PI;
  sbp_gps_eph->w = msg_eph->data.kepler.w * C_1_2P31 * M_PI;
  sbp_gps_eph->inc = msg_eph->data.kepler.inc * C_1_2P31 * M_PI;
  sbp_gps_eph->inc_dot = msg_eph->data.kepler.inc_dot * C_1_2P43 * M_PI;

  sbp_gps_eph->af0 = (float)(msg_eph->data.kepler.af0 * C_1_2P31);
  sbp_gps_eph->af1 = (float)(msg_eph->data.kepler.af1 * C_1_2P43);
  sbp_gps_eph->af2 = (float)(msg_eph->data.kepler.af2 * C_1_2P55);

  sbp_gps_eph->iode = msg_eph->data.kepler.iode;
  sbp_gps_eph->iodc = msg_eph->data.kepler.iodc;

  sbp_gps_eph->toc.wn = toe_time.wn;
  sbp_gps_eph->toc.tow = msg_eph->data.kepler.toc * GPS_TOC_RESOLUTION;

  return sbp_gps_time_sec_valid(&sbp_gps_eph->toc);
}

void rtcm3_qzss_eph_to_sbp(const rtcm_msg_eph *msg_eph,
                           sbp_msg_ephemeris_qzss_t *sbp_qzss_eph,
                           struct rtcm3_sbp_state *state) {
  (void)state;
  assert(msg_eph);
  assert(sbp_qzss_eph);
  assert(RTCM_CONSTELLATION_QZS == msg_eph->constellation);
  /* QZSS week is 10 bit (DF452) */
  sbp_qzss_eph->common.toe.wn =
      gps_adjust_week_cycle(msg_eph->wn, state->gps_week_reference);
  sbp_qzss_eph->common.toe.tow = msg_eph->toe * GPS_TOE_RESOLUTION;
  sbp_qzss_eph->common.sid.sat = msg_eph->sat_id + QZS_FIRST_PRN - 1;
  sbp_qzss_eph->common.sid.code = CODE_QZS_L1CA;
  sbp_qzss_eph->common.ura = convert_ura_to_uri(msg_eph->ura);
  sbp_qzss_eph->common.fit_interval = rtcm3_decode_fit_interval_gps(
      msg_eph->fit_interval, msg_eph->data.kepler.iodc);
  sbp_qzss_eph->common.valid =
      msg_eph->data.kepler.iodc == msg_eph->data.kepler.iode;
  sbp_qzss_eph->common.health_bits = msg_eph->health_bits;

  sbp_qzss_eph->tgd = (float)(msg_eph->data.kepler.tgd.gps_s * C_1_2P31);

  sbp_qzss_eph->c_rs = (float)(msg_eph->data.kepler.crs * C_1_2P5);
  sbp_qzss_eph->c_rc = (float)(msg_eph->data.kepler.crc * C_1_2P5);
  sbp_qzss_eph->c_uc = (float)(msg_eph->data.kepler.cuc * C_1_2P29);
  sbp_qzss_eph->c_us = (float)(msg_eph->data.kepler.cus * C_1_2P29);
  sbp_qzss_eph->c_ic = (float)(msg_eph->data.kepler.cic * C_1_2P29);
  sbp_qzss_eph->c_is = (float)(msg_eph->data.kepler.cis * C_1_2P29);

  sbp_qzss_eph->dn = msg_eph->data.kepler.dn * C_1_2P43 * M_PI;
  sbp_qzss_eph->m0 = msg_eph->data.kepler.m0 * C_1_2P31 * M_PI;
  sbp_qzss_eph->ecc = msg_eph->data.kepler.ecc * C_1_2P33;
  sbp_qzss_eph->sqrta = msg_eph->data.kepler.sqrta * C_1_2P19;
  sbp_qzss_eph->omega0 = msg_eph->data.kepler.omega0 * C_1_2P31 * M_PI;
  sbp_qzss_eph->omegadot = msg_eph->data.kepler.omegadot * C_1_2P43 * M_PI;
  sbp_qzss_eph->w = msg_eph->data.kepler.w * C_1_2P31 * M_PI;
  sbp_qzss_eph->inc = msg_eph->data.kepler.inc * C_1_2P31 * M_PI;
  sbp_qzss_eph->inc_dot = msg_eph->data.kepler.inc_dot * C_1_2P43 * M_PI;

  sbp_qzss_eph->af0 = (float)(msg_eph->data.kepler.af0 * C_1_2P31);
  sbp_qzss_eph->af1 = (float)(msg_eph->data.kepler.af1 * C_1_2P43);
  sbp_qzss_eph->af2 = (float)(msg_eph->data.kepler.af2 * C_1_2P55);

  sbp_qzss_eph->iode = msg_eph->data.kepler.iode;
  sbp_qzss_eph->iodc = msg_eph->data.kepler.iodc;

  sbp_qzss_eph->toc.wn = sbp_qzss_eph->common.toe.wn;
  sbp_qzss_eph->toc.tow = msg_eph->data.kepler.toc * GPS_TOC_RESOLUTION;
}

bool rtcm3_glo_eph_to_sbp(const rtcm_msg_eph *msg_eph,
                          sbp_msg_ephemeris_glo_t *sbp_glo_eph,
                          struct rtcm3_sbp_state *state) {
  assert(msg_eph);
  assert(sbp_glo_eph);
  assert(RTCM_CONSTELLATION_GLO == msg_eph->constellation);

  gps_time_t rover_time;
  if (!rtcm_get_gps_time(&rover_time, state)) {
    return false;
  }

  gps_time_t toe;
  if (!compute_glo_time(msg_eph->data.glo.t_b * 15 * MINUTE_SECS * SECS_MS,
                        &toe,
                        &rover_time,
                        state)) {
    return false;
  }
  if (toe.wn == (s16)INVALID_TIME) {
    return false;
  }
  sbp_glo_eph->common.toe.wn = toe.wn;
  sbp_glo_eph->common.toe.tow = (u32)rint(toe.tow);
  if (msg_eph->sat_id == 0) {
    return false;
  }
  sbp_glo_eph->common.sid.sat = msg_eph->sat_id;
  sbp_glo_eph->common.sid.code = CODE_GLO_L1OF;
  sbp_glo_eph->common.ura = convert_glo_ft_to_meters(msg_eph->ura);
  sbp_glo_eph->common.fit_interval =
      rtcm3_decode_fit_interval_glo(msg_eph->fit_interval);
  sbp_glo_eph->common.valid = 1;
  sbp_glo_eph->common.health_bits = msg_eph->health_bits;

  sbp_glo_eph->gamma = (float)(msg_eph->data.glo.gamma * C_1_2P40);
  sbp_glo_eph->tau = (float)(msg_eph->data.glo.tau * C_1_2P30);
  sbp_glo_eph->d_tau = (float)(msg_eph->data.glo.d_tau * C_1_2P30);

  sbp_glo_eph->pos[0] = msg_eph->data.glo.pos[0] * C_1_2P11 * 1000;
  sbp_glo_eph->pos[1] = msg_eph->data.glo.pos[1] * C_1_2P11 * 1000;
  sbp_glo_eph->pos[2] = msg_eph->data.glo.pos[2] * C_1_2P11 * 1000;

  sbp_glo_eph->vel[0] = msg_eph->data.glo.vel[0] * C_1_2P20 * 1000;
  sbp_glo_eph->vel[1] = msg_eph->data.glo.vel[1] * C_1_2P20 * 1000;
  sbp_glo_eph->vel[2] = msg_eph->data.glo.vel[2] * C_1_2P20 * 1000;

  sbp_glo_eph->acc[0] = (float)(msg_eph->data.glo.acc[0] * C_1_2P30 * 1000);
  sbp_glo_eph->acc[1] = (float)(msg_eph->data.glo.acc[1] * C_1_2P30 * 1000);
  sbp_glo_eph->acc[2] = (float)(msg_eph->data.glo.acc[2] * C_1_2P30 * 1000);

  sbp_glo_eph->fcn = msg_eph->data.glo.fcn + 1;
  sbp_glo_eph->iod = (msg_eph->data.glo.t_b * 15 * MINUTE_SECS) & 127u;
  return true;
}

bool rtcm3_gal_eph_to_sbp(const rtcm_msg_eph *msg_eph,
                          const u8 source,
                          sbp_msg_ephemeris_gal_t *sbp_gal_eph,
                          struct rtcm3_sbp_state *state) {
  assert(msg_eph);
  assert(sbp_gal_eph);
  assert(RTCM_CONSTELLATION_GAL == msg_eph->constellation);

  /* Galileo week is 12 bit (DF289) and starts from GPS WN 1024 */
  static const uint16_t GAL_WN_RESOLUTION = 12;
  static const gps_time_t GAL_GPS_OFFSET = {.wn = GAL_WEEK_TO_GPS_WEEK,
                                            .tow = 0};

  const gps_time_t toe_time = week_rollover_adjustment(
      (gps_time_t){.wn = msg_eph->wn,
                   .tow = msg_eph->toe * GALILEO_TOE_RESOLUTION},
      (gps_time_t){.wn = state->gps_week_reference, .tow = 0},
      GAL_WN_RESOLUTION,
      GAL_GPS_OFFSET);

  if (state->ephemeris_time_estimator != NULL && msg_eph->health_bits == 0) {
    gnss_signal_t gnss_signal = {.sat = msg_eph->sat_id, .code = CODE_GAL_E1B};
    time_truth_ephemeris_estimator_push(
        state->ephemeris_time_estimator, gnss_signal, toe_time);
  }

  sbp_gal_eph->common.toe.wn = toe_time.wn;
  sbp_gal_eph->common.toe.tow = msg_eph->toe * GALILEO_TOE_RESOLUTION;

  if (!sbp_gps_time_sec_valid(&sbp_gal_eph->common.toe)) {
    return false;
  }

  sbp_gal_eph->common.sid.sat = msg_eph->sat_id;
  sbp_gal_eph->common.sid.code = CODE_GAL_E1B;
  sbp_gal_eph->common.ura = convert_sisa_to_meters(msg_eph->ura);
  /* Fit interval is hardcoded to 4 hours, as not present in RTCM fields */
  sbp_gal_eph->common.fit_interval = 4 * HOUR_SECS;
  sbp_gal_eph->common.valid = 1;
  sbp_gal_eph->common.health_bits = msg_eph->health_bits;

  sbp_gal_eph->bgd_e1e5a =
      (float)(msg_eph->data.kepler.tgd.gal_s[0] * C_1_2P32);
  sbp_gal_eph->bgd_e1e5b =
      (float)(msg_eph->data.kepler.tgd.gal_s[1] * C_1_2P32);

  sbp_gal_eph->c_rs = (float)(msg_eph->data.kepler.crs * C_1_2P5);
  sbp_gal_eph->c_rc = (float)(msg_eph->data.kepler.crc * C_1_2P5);
  sbp_gal_eph->c_uc = (float)(msg_eph->data.kepler.cuc * C_1_2P29);
  sbp_gal_eph->c_us = (float)(msg_eph->data.kepler.cus * C_1_2P29);
  sbp_gal_eph->c_ic = (float)(msg_eph->data.kepler.cic * C_1_2P29);
  sbp_gal_eph->c_is = (float)(msg_eph->data.kepler.cis * C_1_2P29);

  sbp_gal_eph->dn = msg_eph->data.kepler.dn * C_1_2P43 * M_PI;
  sbp_gal_eph->m0 = msg_eph->data.kepler.m0 * C_1_2P31 * M_PI;
  sbp_gal_eph->ecc = msg_eph->data.kepler.ecc * C_1_2P33;
  sbp_gal_eph->sqrta = msg_eph->data.kepler.sqrta * C_1_2P19;
  sbp_gal_eph->omega0 = msg_eph->data.kepler.omega0 * C_1_2P31 * M_PI;
  sbp_gal_eph->omegadot = msg_eph->data.kepler.omegadot * C_1_2P43 * M_PI;
  sbp_gal_eph->w = msg_eph->data.kepler.w * C_1_2P31 * M_PI;
  sbp_gal_eph->inc = msg_eph->data.kepler.inc * C_1_2P31 * M_PI;
  sbp_gal_eph->inc_dot = msg_eph->data.kepler.inc_dot * C_1_2P43 * M_PI;

  sbp_gal_eph->af0 = msg_eph->data.kepler.af0 * C_1_2P34;
  sbp_gal_eph->af1 = msg_eph->data.kepler.af1 * C_1_2P46;
  sbp_gal_eph->af2 = (float)(msg_eph->data.kepler.af2 * C_1_2P59);

  sbp_gal_eph->iode = msg_eph->data.kepler.iode;
  sbp_gal_eph->iodc = msg_eph->data.kepler.iode;

  sbp_gal_eph->toc.wn = toe_time.wn;
  sbp_gal_eph->toc.tow = msg_eph->data.kepler.toc * GALILEO_TOC_RESOLUTION;
  sbp_gal_eph->source = source;

  return sbp_gps_time_sec_valid(&sbp_gal_eph->toc);
}

void rtcm3_bds_eph_to_sbp(const rtcm_msg_eph *msg_eph,
                          sbp_msg_ephemeris_bds_t *sbp_bds_eph,
                          struct rtcm3_sbp_state *state) {
  (void)state;
  assert(msg_eph);
  assert(sbp_bds_eph);
  assert(RTCM_CONSTELLATION_BDS == msg_eph->constellation);

  /* Beidou week is 13 bit (DF489) and starts from GPS WN 1356 */
  static const uint16_t BDS_WN_RESOLUTION = 13;
  static const gps_time_t GPS_BDS_OFFSET = {.wn = BDS_WEEK_TO_GPS_WEEK,
                                            .tow = BDS_SECOND_TO_GPS_SECOND};

  const gps_time_t toe_time = week_rollover_adjustment(
      (gps_time_t){.wn = msg_eph->wn,
                   .tow = msg_eph->toe * BEIDOU_TOE_RESOLUTION},
      (gps_time_t){.wn = state->gps_week_reference, .tow = 0},
      BDS_WN_RESOLUTION,
      GPS_BDS_OFFSET);

  const gps_time_t toc_time = week_rollover_adjustment(
      (gps_time_t){.wn = msg_eph->wn,
                   .tow = msg_eph->data.kepler.toc * BEIDOU_TOC_RESOLUTION},
      (gps_time_t){.wn = state->gps_week_reference, .tow = 0},
      BDS_WN_RESOLUTION,
      GPS_BDS_OFFSET);

  if (state->ephemeris_time_estimator != NULL && msg_eph->health_bits == 0) {
    gnss_signal_t gnss_signal = {.sat = msg_eph->sat_id, .code = CODE_BDS2_B1};
    time_truth_ephemeris_estimator_push(
        state->ephemeris_time_estimator, gnss_signal, toe_time);
  }

  sbp_bds_eph->common.toe.wn = toe_time.wn;
  sbp_bds_eph->common.toe.tow = toe_time.tow;
  sbp_bds_eph->common.sid.sat = msg_eph->sat_id;
  sbp_bds_eph->common.sid.code = CODE_BDS2_B1;
  sbp_bds_eph->common.ura = convert_bds_ura_to_meters(msg_eph->ura);
  /* Fit interval is hardcoded to 3 hours, as not present in RTCM fields */
  sbp_bds_eph->common.fit_interval = 3 * HOUR_SECS;
  sbp_bds_eph->common.valid = 1;
  sbp_bds_eph->common.health_bits = msg_eph->health_bits;

  sbp_bds_eph->tgd1 = (float)(msg_eph->data.kepler.tgd.bds_s[0] * 1e-10);
  sbp_bds_eph->tgd2 = (float)(msg_eph->data.kepler.tgd.bds_s[1] * 1e-10);

  sbp_bds_eph->c_rs = (float)(msg_eph->data.kepler.crs * C_1_2P6);
  sbp_bds_eph->c_rc = (float)(msg_eph->data.kepler.crc * C_1_2P6);
  sbp_bds_eph->c_uc = (float)(msg_eph->data.kepler.cuc * C_1_2P31);
  sbp_bds_eph->c_us = (float)(msg_eph->data.kepler.cus * C_1_2P31);
  sbp_bds_eph->c_ic = (float)(msg_eph->data.kepler.cic * C_1_2P31);
  sbp_bds_eph->c_is = (float)(msg_eph->data.kepler.cis * C_1_2P31);

  sbp_bds_eph->dn = msg_eph->data.kepler.dn * C_1_2P43 * M_PI;
  sbp_bds_eph->m0 = msg_eph->data.kepler.m0 * C_1_2P31 * M_PI;
  sbp_bds_eph->ecc = msg_eph->data.kepler.ecc * C_1_2P33;
  sbp_bds_eph->sqrta = msg_eph->data.kepler.sqrta * C_1_2P19;
  sbp_bds_eph->omega0 = msg_eph->data.kepler.omega0 * C_1_2P31 * M_PI;
  sbp_bds_eph->omegadot = msg_eph->data.kepler.omegadot * C_1_2P43 * M_PI;
  sbp_bds_eph->w = msg_eph->data.kepler.w * C_1_2P31 * M_PI;
  sbp_bds_eph->inc = msg_eph->data.kepler.inc * C_1_2P31 * M_PI;
  sbp_bds_eph->inc_dot = msg_eph->data.kepler.inc_dot * C_1_2P43 * M_PI;

  sbp_bds_eph->af0 = msg_eph->data.kepler.af0 * C_1_2P33;
  sbp_bds_eph->af1 = (float)(msg_eph->data.kepler.af1 * C_1_2P50);
  sbp_bds_eph->af2 = (float)(msg_eph->data.kepler.af2 * C_1_2P66);

  sbp_bds_eph->iode = msg_eph->data.kepler.iode;
  sbp_bds_eph->iodc = msg_eph->data.kepler.iodc;

  // What happens if toc and toe straddle the week boundary?
  sbp_bds_eph->toc.wn = toc_time.wn;
  sbp_bds_eph->toc.tow = toc_time.tow;
}

void handle_ndf_frame(const rtcm_msg_ndf *msg_ndf,
                      struct rtcm3_sbp_state *state) {
  assert(msg_ndf);

  for (uint8_t i = 0; i < msg_ndf->frame_count; i++) {
    u32 frame_data_size_words = msg_ndf->frames[i].frame_data_size_bits / 32;
    if (msg_ndf->frames[i].frame_data_size_bits % 32 > 0) {
      frame_data_size_words++;
    }

    switch (msg_ndf->frames[i].sat_sys) {
      case NDF_SYS_SBAS:
        sbas_decode_subframe(&state->eph_data,
                             msg_ndf->frames[i].epoch_time,
                             msg_ndf->frames[i].sat_num + 120,
                             msg_ndf->frames[i].frame_data,
                             frame_data_size_words,
                             rtcm_stn_to_sbp_sender_id(msg_ndf->stn_id),
                             state->context,
                             state->cb_rtcm_to_sbp);
        break;
      case NDF_SYS_GPS:
      case NDF_SYS_GLO:
      case NDF_SYS_GAL:
      case NDF_SYS_QZS:
      case NDF_SYS_BDS:
      default:
        break;
    }
  }
}
