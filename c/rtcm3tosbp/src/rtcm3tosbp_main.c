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

#include <rtcm3tosbp/internal/rtcm3tosbp.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

static inline int32_t read_fn(uint8_t *buf, size_t len, void *context) {
  (void)context;
  return (int32_t)read(STDIN_FILENO, buf, len);
}

static inline int32_t write_fn(uint8_t *buff, uint32_t n, void *context) {
  (void)context;
  return (int32_t)write(STDOUT_FILENO, buff, n);
}

int main(int argc, char **argv) {
  return rtcm3tosbp(argc, argv, "", read_fn, write_fn, NULL);
}
