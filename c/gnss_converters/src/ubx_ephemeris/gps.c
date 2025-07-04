/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swift-nav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <gnss-converters/internal/common.h>
#include <gnss-converters/ubx_sbp.h>
#include <libsbp/v4/gnss.h>
#include <libsbp/v4/observation.h>
#include <string.h>
#include <swiftnav/common.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/signal.h>

#define GPS_L1CA_PREAMBLE 0x8b

static void pack_ephemeris_gps(const ephemeris_t *e,
                               sbp_msg_ephemeris_gps_t *msg) {
  const ephemeris_kepler_t *k = &e->data.kepler;
  pack_ephemeris_common_content(e, &msg->common);
  msg->tgd = k->tgd.gps_s[0];
  msg->c_rs = (float)k->crs;
  msg->c_rc = (float)k->crc;
  msg->c_uc = (float)k->cuc;
  msg->c_us = (float)k->cus;
  msg->c_ic = (float)k->cic;
  msg->c_is = (float)k->cis;
  msg->dn = k->dn;
  msg->m0 = k->m0;
  msg->ecc = k->ecc;
  msg->sqrta = k->sqrta;
  msg->omega0 = k->omega0;
  msg->omegadot = k->omegadot;
  msg->w = k->w;
  msg->inc = k->inc;
  msg->inc_dot = k->inc_dot;
  msg->af0 = (float)k->af0;
  msg->af1 = (float)k->af1;
  msg->af2 = (float)k->af2;
  msg->toc.tow = (u32)round(k->toc.tow);
  msg->toc.wn = k->toc.wn;
  msg->iode = k->iode;
  msg->iodc = k->iodc;
}

/**
 * Decodes GPS L1CA subframes.
 * Reference: ICD IS-GPS-200H
 * @param data context data
 * @param prn transmitter's PRN
 * @param subframe the array of full subframe (30 bit words)
 * @param sz must be 10 (number of 30 bit words per one GPS L1CA subframe)
 */
void gps_decode_subframe(struct ubx_sbp_state *data,
                         int prn,
                         const u32 words[],
                         int sz) {
  assert(data);
  assert(words);
  if (prn < GPS_FIRST_PRN || prn >= (GPS_FIRST_PRN + NUM_SATS_GPS) ||
      10 != sz) {
    return;
  }

  /* we have to figure out if the provided subframe is
     from GPS L1CA or L2C by looking at the preamble
     (https://portal.u-blox.com/s/question/0D52p00008fxLLxCAM/why-is-there-no-signal-identifier-in-the-ubxrawsfrbx-message)
   */
  unsigned preamble = (words[0] >> 22U) & 0xFFU;
  if (GPS_L1CA_PREAMBLE != preamble) {
    /* do not support L2C subframes ATM */
    return;
  }

  int sf_idx = ((words[1] >> 8) & 0x7U) - 1;
  if ((sf_idx < 0) || (sf_idx > 2)) {
    return;
  }

  struct sat_data *sat = &data->eph_data.gps_sat_data[prn - 1];
  sat->vmask |= 1U << sf_idx;
  memcpy(&sat->sf[sf_idx].words, words, sz * 4);
  if (0x7 != (sat->vmask & 0x7U)) {
    return; /* not all SF 1,2,3 are available yet */
  }

  u8 iodc8_sf1 = (sat->sf[0].words[7] >> 22U) & 0xFFU;
  u8 iode8_sf2 = (sat->sf[1].words[2] >> 22U) & 0xFFU;
  u8 iode8_sf3 = (sat->sf[2].words[9] >> 22U) & 0xFFU;

  if ((iodc8_sf1 != iode8_sf2) || (iode8_sf2 != iode8_sf3)) {
    // Invalidate the out of date subframes (i.e. all but sf_idx, which is this
    // one).
    invalidate_subframes(sat, /*mask=*/0x7 & ~(1U << sf_idx));
    return;
  }

  s32 tot_tow_s = TOW_UNKNOWN;
  /* HOW contains TOW for the _next_ subframe transmission start */
  s32 tow_6s = (sat->sf[0].words[1] >> 13U) & 0x1ffffU;
  if (0 == tow_6s) {
    tot_tow_s = WEEK_SECS - 6; /* Week rollover after the current subframe */
  } else if (tow_6s < WEEK_SECS / 6) {
    tot_tow_s = (tow_6s - 1) * 6;
  } else {
    // Subframe 1 must be corrupted if the ToW is greater than WEEK_SECS, so
    // invalidate it (subframe 1 has index 0, so this bit is the very first one,
    // hence the 0x1 mask).
    invalidate_subframes(sat, /*mask=*/0x1);
    return;
  }

  /* Now let's actually decode the ephemeris... */
  u32 frame_words[3][8];
  for (int i = 0; i < 3; i++) {
    memcpy(&frame_words[i][0], &sat->sf[i].words[2], 8 * sizeof(u32));
  }
  ephemeris_t e;
  memset(&e, 0, sizeof(e));
  e.sid.sat = prn;
  e.sid.code = CODE_GPS_L1CA;
  decode_ephemeris((const u32(*)[8])frame_words, &e, tot_tow_s);
  if (!e.valid) {
    invalidate_subframes(sat, /*mask=*/0x7);
    return;
  }
  sbp_msg_t sbp_msg;
  sbp_msg_ephemeris_gps_t *msg = &sbp_msg.ephemeris_gps;
  memset(msg, 0, sizeof(*msg));
  pack_ephemeris_gps(&e, msg);

  if (data->ephemeris_time_estimator != NULL) {
    const gnss_signal_t gnss_signal = {.sat = msg->common.sid.sat,
                                       .code = msg->common.sid.code};
    const gps_time_t gps_time = {.wn = (int16_t)msg->common.toe.wn,
                                 .tow = msg->common.toe.tow};

    time_truth_ephemeris_estimator_push(
        data->ephemeris_time_estimator, gnss_signal, gps_time);
  }

  assert(data->cb_ubx_to_sbp);
  data->cb_ubx_to_sbp(
      data->sender_id, SbpMsgEphemerisGps, &sbp_msg, data->context);
  invalidate_subframes(sat, /*mask=*/0x7);
}
