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

#ifndef RTCM3TOSBP_INTERNAL_RTCM3TOSBP_H
#define RTCM3TOSBP_INTERNAL_RTCM3TOSBP_H

#include <libsbp/sbp.h>
#include <stdint.h>
#include <stdlib.h>

typedef int32_t (*readfn_ptr)(uint8_t *, size_t, void *);
typedef int32_t (*writefn_ptr)(const uint8_t *, size_t, void *);

int rtcm3tosbp(int argc,
               char **argv,
               const char *additional_opts_help,
               readfn_ptr readfn,
               sbp_write_fn_t writefn,
               void *context);

#endif  // RTCM3TOSBP_INTERNAL_RTCM3TOSBP_H
