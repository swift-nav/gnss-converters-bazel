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
#include <gnss-converters-extra/sbp_conv.h>
#include <gnss-converters/sbp_rtcm3.h>
#include <libsbp/sbp.h>
#include <swiftnav/fifo_byte.h>
#include <swiftnav/gnss_time.h>
#include <time.h>

struct sbp_conv_s {
  struct rtcm3_out_state state;
  fifo_t fifo;
  uint8_t buf[2048];
};

static s32 sbp_conv_cb(uint8_t *buf, uint16_t len, void *context) {
  fifo_t *fifo = context;
  assert(fifo != NULL);
  return (s32)fifo_write(fifo, buf, len);
}

sbp_conv_t sbp_conv_new(msm_enum msm_output_type,
                        const char *ant_descriptor,
                        const char *rcv_descriptor) {
  assert(ant_descriptor);
  assert(rcv_descriptor);
  sbp_conv_t conv = malloc(sizeof(struct sbp_conv_s));
  if (conv != NULL) {
    fifo_init(&conv->fifo, conv->buf, sizeof(conv->buf));
    sbp2rtcm_init(&conv->state, sbp_conv_cb, &conv->fifo);
    gps_time_t gps_time = time2gps_t(time(NULL));
    int8_t leap_seconds = rint(get_gps_utc_offset(&gps_time, NULL));
    sbp2rtcm_set_leap_second(&leap_seconds, &conv->state);
    sbp2rtcm_set_rcv_ant_descriptors(
        ant_descriptor, rcv_descriptor, &conv->state);
    sbp2rtcm_set_rtcm_out_mode(msm_output_type, &conv->state);
  }
  return conv;
}

void sbp_conv_delete(sbp_conv_t conv) { free(conv); }

size_t sbp_conv(sbp_conv_t conv,
                uint16_t sender,
                uint16_t type,
                uint8_t *rbuf,
                size_t rlen,
                uint8_t *wbuf,
                size_t wlen) {
  sbp_msg_t msg;
  if (SBP_OK == sbp_message_decode(rbuf, (uint8_t)rlen, NULL, type, &msg)) {
    sbp2rtcm_sbp_cb(sender, type, &msg, &conv->state);
  }
  return fifo_read(&conv->fifo, wbuf, (u32)wlen);
}

void sbp_conv_set_legacy_on(sbp_conv_t conv) {
  assert(conv);
  sbp2rtcm_set_rtcm_out_mode(MSM_UNKNOWN, &conv->state);
}

void sbp_conv_set_stn_description_parameters(sbp_conv_t conv,
                                             uint8_t glo_ind,
                                             uint8_t ref_stn_ind,
                                             uint8_t quart_cycle_ind) {
  assert(conv);
  sbp2rtcm_set_stn_description_parameters(
      &conv->state, glo_ind, ref_stn_ind, quart_cycle_ind);
}
