#ifndef SBP2SBP_INTERNAL_SBP2SBP_IO_FUNCTION_H
#define SBP2SBP_INTERNAL_SBP2SBP_IO_FUNCTION_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

typedef int32_t (*readfn_ptr)(uint8_t *, uint32_t, void *);
typedef int32_t (*readfn_eof_ptr)(void *);
typedef int32_t (*writefn_ptr)(uint8_t *, uint32_t, void *);

static bool at_eof = false;

static inline int32_t read_fn(uint8_t *buf, uint32_t len, void *context) {
  (void)context;
  int32_t ret = (int32_t)read(STDIN_FILENO, buf, len);

  if ((len > 0) && (ret == 0)) {
    at_eof = true;
  }
  return ret;
}

static inline int32_t write_fn(uint8_t *buff, uint32_t n, void *context) {
  (void)context;
  return (int32_t)write(STDOUT_FILENO, buff, n);
}

static inline int32_t read_fn_eof(void *context) {
  (void)context;
  return at_eof ? 1 : 0;
}

#endif  // SBP2SBP_INTERNAL_SBP2SBP_IO_FUNCTION_H
