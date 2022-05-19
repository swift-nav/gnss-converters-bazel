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

#ifndef GNSS_CONVERTERS_SBP_CONV_INTERFACE_H
#define GNSS_CONVERTERS_SBP_CONV_INTERFACE_H

#include <rtcm3/messages.h>
#include <stdint.h>
#include <stdlib.h>

typedef struct sbp_conv_s *sbp_conv_t;

#ifdef __cplusplus
extern "C" {
#endif

sbp_conv_t sbp_conv_new(msm_enum msm_output_type,
                        const char *ant_descriptor,
                        const char *rcv_descriptor);

void sbp_conv_delete(sbp_conv_t conv);

size_t sbp_conv(sbp_conv_t conv,
                uint16_t sender,
                uint16_t type,
                uint8_t *rbuf,
                size_t rlen,
                uint8_t *wbuf,
                size_t wlen);

/** Enable RTCM legacy output and disable MSM output */
void sbp_conv_set_legacy_on(sbp_conv_t conv);

/**
 * Function allows users to customize some parameters/attributes of Classic
 * context (specify in RTCM message 1005), including GLO_ind, ref_stn_ind,
 * quart_cycle_ind.
 * It will wrap sbp2rtcm_set_stn_description_parameters() in gnss_converters.
 */
void sbp_conv_set_stn_description_parameters(sbp_conv_t conv,
                                             uint8_t glo_ind,
                                             uint8_t ref_stn_ind,
                                             uint8_t quart_cycle_ind);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_SBP_CONV_INTERFACE_H */
