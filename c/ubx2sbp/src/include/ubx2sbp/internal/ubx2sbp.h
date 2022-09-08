#ifndef UBX2SBP_INTERNAL_UBX2SBP_H
#define UBX2SBP_INTERNAL_UBX2SBP_H

#include <stdint.h>

typedef int (*readfn_ptr)(uint8_t *, size_t, void *);
typedef int (*writefn_ptr)(uint8_t *, uint32_t, void *);

int ubx2sbp(int argc,
            char **argv,
            const char *additional_opts_help,
            readfn_ptr readfn,
            writefn_ptr writefn,
            void *context);

#endif  // UBX2SBP_INTERNAL_UBX2SBP_H
