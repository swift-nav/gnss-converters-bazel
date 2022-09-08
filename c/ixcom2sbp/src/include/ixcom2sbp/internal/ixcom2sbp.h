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

#ifndef IXCOM2SBP_INTERNAL_IXCOM2SBP_H
#define IXCOM2SBP_INTERNAL_IXCOM2SBP_H

#include <stdint.h>
#include <stdlib.h>

typedef int (*readfn_ptr)(uint8_t *, size_t, void *);
typedef int (*writefn_ptr)(uint8_t *, uint32_t, void *);

int ixcom2sbp(int argc,
              char **argv,
              const char *additional_opts_help,
              readfn_ptr readfn,
              writefn_ptr writefn,
              void *context);

#endif  // IXCOM2SBP_INTERNAL_IXCOM2SBP_H
