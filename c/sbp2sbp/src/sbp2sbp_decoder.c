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

/* This is a stand-alone tool that takes Swift Binary Protocol (SBP) binary on
 * stdin and writes Swift Binary Protocol (SBP) struct on stdout. */

#include <getopt.h>
#include <libsbp/edc.h>
#include <libsbp/sbp.h>
#include <sbp2sbp/internal/sbp2sbp_decoder.h>
#include <stdint.h>
#include <stdio.h>

static sbp_write_fn_t sbp2sbp_writefn;
static sbp_state_t state;

/* Write the SBP packet to STDOUT. */
static void sbp_write(uint16_t sender_id,
                      sbp_msg_type_t msg_type,
                      const sbp_msg_t *msg,
                      void *context) {
  (void)context;
  sbp_message_send(&state, msg_type, sender_id, msg, sbp2sbp_writefn);
  fprintf(stderr, "Finish msg type: %d\n", msg_type);
}

static void help(char *arg, const char *additional_opts_help) {
  fprintf(stderr, "Usage: %s [options]%s\n", arg, additional_opts_help);
  fprintf(stderr, "  -h this message\n");
}

int sbp2sbp_decoder(int argc,
                    char **argv,
                    const char *additional_opts_help,
                    readfn_ptr readfn,
                    readfn_eof_ptr readfn_eof,
                    writefn_ptr writefn,
                    void *context) {
  sbp_state_init(&state);
  sbp_state_set_io_context(&state, context);

  sbp2sbp_writefn = writefn;

  sbp_msg_callbacks_node_t node;
  sbp_all_message_callback_register(&state, sbp_write, NULL, &node);

  int opt = -1;
  while ((opt = getopt(argc, argv, "h")) != -1) {
    switch (opt) {
      case 'h':
        help(argv[0], additional_opts_help);
        return 0;
      default:
        break;
    }
  }

  int8_t ret = SBP_OK;
  do {
    ret = sbp_process(&state, readfn);
  } while ((ret >= 0) && (readfn_eof(context) == 0));
  return 0;
}
