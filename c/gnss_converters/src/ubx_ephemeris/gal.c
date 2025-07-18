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
#include <swiftnav/bits.h>
#include <swiftnav/common.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_capabilities.h>
#include <swiftnav/signal.h>

static void invalidate_pages(struct gal_sat_data *sat, unsigned mask) {
  assert(sat);
  sat->vmask &= ~mask;
}

/**
 * Packs Galileo ephemeris data.
 * @param e the ephemeris to pack.
 * @param m the packed ephemeris.
 * @param src the GAL ephemeris source (0 - I/NAV, 1 - F/NAV)
 */
static void pack_ephemeris_gal(const ephemeris_t *e,
                               sbp_msg_ephemeris_gal_t *msg,
                               int src) {
  assert(e);
  assert(msg);
  assert((0 == src) || (1 == src));

  memset(msg, 0, sizeof(*msg));

  const ephemeris_kepler_t *k = &e->data.kepler;
  pack_ephemeris_common_content(e, &msg->common);
  msg->bgd_e1e5a = k->tgd.gal_s[0];
  msg->bgd_e1e5b = k->tgd.gal_s[1];
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
  msg->af0 = k->af0;
  msg->af1 = (float)k->af1;
  msg->af2 = (float)k->af2;
  msg->toc.tow = (u32)round(k->toc.tow);
  msg->toc.wn = k->toc.wn;
  msg->iode = k->iode;
  msg->iodc = k->iodc;
  msg->source = (u8)src;
}

/** pages 1,2,3,4,5 are required */
#define ALL_PAGES_MASK 0x1FU

/**
 * Decodes GAL pages 1-5.
 * Reference: GAL OS SIS ICD, Issue 1.3, December 2016
 * @param data context data
 * @param prn transmitter's PRN
 * @param words the array of full page words (32 bit words)
 * @param sz must be >=8 (number of 32 bit words per one GAL page)
 */
void gal_decode_page(struct ubx_sbp_state *data,
                     int prn,
                     const u32 words[],
                     int sz) {
  assert(data);
  assert(words);
  if (prn < GAL_FIRST_PRN || prn >= (GAL_FIRST_PRN + NUM_SATS_GAL) || sz < 8) {
    return;
  }

  int ptype = (words[0] >> 30) & 1U;
  if (1 == ptype) {
    return; /* skip alert messages */
  }

  if (((words[0] >> 31) & 1U) != 0) {
    return; /* must start with even */
  }
  if (((words[4] >> 31) & 1U) != 1) {
    return; /* must end with odd */
  }

  /* word type identification */
  int wtype = (words[0] >> 24U) & 0x3FU;
  if ((wtype < 1) || (wtype > 5)) {
    return; /* only word types 1,2,3,4,5 contain ephemeris data, WN and TOW */
  }

  struct gal_sat_data *sat = &data->eph_data.gal_sat_data[prn - 1];
  sat->vmask |= 1U << (wtype - 1);
  assert(wtype <= (int)ARRAY_SIZE(sat->pg));
  memcpy(&sat->pg[wtype - 1].words, words, sizeof(sat->pg[wtype - 1].words));

  // Get the stored IODnavs for the first four subframes - the fifth has no
  // IODnav
  int IODnav[4];
  for (int i = 0; i < (int)ARRAY_SIZE(IODnav); i++) {
    IODnav[i] = (sat->pg[i].words[0] >> 14U) & 0x3FFU;
  }

  // If we haven't just read in the fifth subframe, check to see if this
  // subframe is newer than the old ones
  if (wtype != 5) {
    int new_IODnav = (words[0] >> 14U) & 0x3FFU;
    for (int i = 0; i < (int)ARRAY_SIZE(IODnav); i++) {
      if (IODnav[i] != new_IODnav) {
        invalidate_pages(sat, /*mask=*/(1U << i));
      }
    }
  }

  if (ALL_PAGES_MASK != (sat->vmask & ALL_PAGES_MASK)) {
    return; /* not all word types 1,2,3,4,5 are available yet */
  }

  /* Now let's actually decode the ephemeris... */

  u8 page[5][GAL_INAV_CONTENT_BYTE];
  assert(ARRAY_SIZE(page) == ARRAY_SIZE(sat->pg));

  for (int i = 0; i < (int)ARRAY_SIZE(page); i++) {
    u8 *p = &page[i][0];
    u32 *w = &sat->pg[i].words[0];
    int k = 0;

    /* extract Word type and Data(122-17) from from the event part */
    for (int j = 0; j < 4; j++) {
      p[k++] = (w[j] >> 24U) & 0xFFU;
      p[k++] = (w[j] >> 16U) & 0xFFU;
      p[k++] = (w[j] >> 8U) & 0xFFU;
      p[k++] = w[j] & 0xFFU;
    }
    /* shift Even/odd and Page type out */
    bitshl(p, ARRAY_SIZE(page[i]), /*shift=*/2);

    /* extract Data(16-1) from the odd part */
    u16 tmp = (w[4] >> 14U) & 0xFFFFU;
    p[14] = (tmp >> 8U) & 0xFFU;
    p[15] = tmp & 0xFFU;
  }

  ephemeris_t e;
  memset(&e, 0, sizeof(e));
  if (!decode_gal_ephemeris_safe((const u8(*)[GAL_INAV_CONTENT_BYTE])page,
                                 &e)) {
    return;
  }

  e.sid.code = CODE_GAL_E1B;

  sbp_msg_t sbp_msg;
  sbp_msg_ephemeris_gal_t *msg = &sbp_msg.ephemeris_gal;
  /* I/NAV message as per UBX-13003221-R17
     u-blox8 / u-blox M8 Receiver Description Manual */
  pack_ephemeris_gal(&e, msg, /*src=*/0);

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
      data->sender_id, SbpMsgEphemerisGal, &sbp_msg, data->context);
  invalidate_pages(sat, /*mask=*/ALL_PAGES_MASK);
}
