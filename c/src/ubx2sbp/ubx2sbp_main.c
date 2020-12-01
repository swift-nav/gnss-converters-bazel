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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>

static int readfn(uint8_t *buf, size_t len, void *context) {
  (void)context;
  return read(STDIN_FILENO, buf, len);
}

static int writefn(uint8_t *buff, uint32_t n, void *context) {
  (void)context;
  return write(STDOUT_FILENO, buff, sizeof(uint8_t) * n);
}

typedef int (*readfn_ptr)(uint8_t *, size_t, void *);
typedef int (*writefn_ptr)(uint8_t *, uint32_t, void *);

extern int ubx2sbp_main(int argc, char **argv, readfn_ptr, writefn_ptr, void *);

int main(int argc, char **argv) {
  return ubx2sbp_main(argc, argv, readfn, writefn, 0);
}
