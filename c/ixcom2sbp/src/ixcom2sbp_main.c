/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <ixcom2sbp/internal/ixcom2sbp.h>
#include <stdint.h>
#include <unistd.h>

static inline int read_fn(uint8_t *buf, size_t len, void *context) {
  (void)context;
  return (int)read(STDIN_FILENO, buf, len);
}

static inline int write_fn(uint8_t *buff, uint32_t n, void *context) {
  (void)context;
  return (int)write(STDOUT_FILENO, buff, sizeof(uint8_t) * n);
}

int main(int argc, char **argv) {
  return ixcom2sbp(argc, argv, "", read_fn, write_fn, NULL);
}
