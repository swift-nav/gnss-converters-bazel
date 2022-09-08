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

#ifndef NOV2SBP_INTERNAL_NOV2SBP_H
#define NOV2SBP_INTERNAL_NOV2SBP_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*readfn_ptr)(uint8_t *, uint32_t, void *);
typedef int (*writefn_ptr)(uint8_t *, uint32_t, void *);

int nov2sbp(int argc,
            char **argv,
            const char *additional_opts_help,
            readfn_ptr,
            writefn_ptr,
            void *);
}

#endif  // NOV2SBP_INTERNAL_NOV2SBP_H
