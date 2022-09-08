/*
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef SBP2RTCM_INTERNAL_SBP2RTCM_H
#define SBP2RTCM_INTERNAL_SBP2RTCM_H

#include <libsbp/sbp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

typedef int32_t (*readfn_ptr)(uint8_t *, uint32_t, void *);
typedef int32_t (*read_eof_fn_ptr)(void *);
typedef int32_t (*writefn_ptr)(uint8_t *, uint16_t, void *);

int sbp2rtcm(int argc,
             char **argv,
             const char *additional_opts_help,
             readfn_ptr readfn,
             read_eof_fn_ptr read_eof_fn,
             writefn_ptr writefn,
             void *context);

#endif  // SBP2RTCM_INTERNAL_SBP2RTCM_H
