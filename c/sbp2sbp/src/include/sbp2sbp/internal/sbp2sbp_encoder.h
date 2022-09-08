#ifndef SBP2SBP_INTERNAL_SBP2SBP_ENCODER_H
#define SBP2SBP_INTERNAL_SBP2SBP_ENCODER_H

#include <libsbp/sbp.h>
#include <sbp2sbp/internal/sbp2sbp_io_function.h>
#include <stdint.h>
#include <stdlib.h>

int sbp2sbp_encoder(int argc,
                    char **argv,
                    const char *additional_opts_help,
                    readfn_ptr readfn,
                    readfn_eof_ptr readfn_eof,
                    writefn_ptr writefn,
                    void *context);

#endif  // SBP2SBP_INTERNAL_SBP2SBP_ENCODER_H
