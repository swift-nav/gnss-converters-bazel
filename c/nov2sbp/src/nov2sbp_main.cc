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

#include <nov2sbp/internal/nov2sbp.h>
#include <unistd.h>

#include <cstdint>
#include <cstdio>

static int readfn(uint8_t *bytes, uint32_t n_bytes, void *context) {
  (void)context;
  if (feof(stdin) != 0) {
    return -1;
  }
  return static_cast<int>(fread(bytes, sizeof(*bytes), n_bytes, stdin));
}

static int32_t nov2sbp_writefn(uint8_t *bytes,
                               uint32_t n_bytes,
                               void *context) {
  (void)context;
  return static_cast<int>(fwrite(bytes, sizeof(*bytes), n_bytes, stdout));
}

int main(int argc, char **argv) {
  return nov2sbp(argc, argv, "", readfn, nov2sbp_writefn, nullptr);
}
