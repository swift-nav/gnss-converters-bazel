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

#include <getopt.h>
#include <gnss-converters/ubx_sbp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ubx2sbp/internal/time_truth.h>
#include <ubx2sbp/internal/ubx2sbp.h>

static sbp_state_t sbp_state;
static writefn_ptr ubx2sbp_writefn;

static void sbp_write(uint16_t sender_id,
                      sbp_msg_type_t msg_type,
                      const sbp_msg_t *msg,
                      void *context) {
  (void)context;

  sbp_message_send(&sbp_state, msg_type, sender_id, msg, ubx2sbp_writefn);
}

static void help(char *arg, const char *additional_opts_help) {
  fprintf(stderr, "Usage: %s [options]%s\n", arg, additional_opts_help);
  fprintf(stderr, "  -h this message\n");
  fprintf(stderr, "\n");
  fprintf(
      stderr,
      "  -s, --sender_id use provided sender id. Can be hex format if prefixed "
      "with '0x'. Defaults to %d\n",
      DEFAULT_UBX_SENDER_ID);
  fprintf(stderr,
          "  --hnr output POS_LLH (522) on HNR-PVT instead of NAV-PVT. NAV-PVT "
          "messages are still necessary.\n");
  fprintf(stderr,
          "  --time_truth requests that the converter upload any timing "
          "information to the time truth module. Option has no practical "
          "purposes other than to run fuzz testing on it.\n");
}

int ubx2sbp(int argc,
            char **argv,
            const char *additional_opts_help,
            readfn_ptr readfn,
            writefn_ptr writefn,
            void *context) {
  /* TODO(STAR-917) accept sender id as a cmdline argument */

  sbp_state_init(&sbp_state);
  sbp_state_set_io_context(&sbp_state, context);

  ubx2sbp_writefn = writefn;

  struct ubx_sbp_state state;
  ubx_sbp_init(&state, &sbp_write, context);

  int opt;
  int option_index = 0;
  static struct option long_options[] = {
      {"hnr", no_argument, 0, 0},
      {"sender_id", required_argument, 0, 's'},
      {"time_truth", no_argument, 0, 't'},
      {0, 0, 0, 0}};

  while ((opt = getopt_long(argc, argv, "hs:", long_options, &option_index)) !=
         -1) {
    switch (opt) {
      case 0:
        if (strcmp("hnr", long_options[option_index].name) == 0) {
          ubx_set_hnr_flag(&state, true);
        }
        break;

      case 's':
        ubx_set_sender_id(&state, (u16)strtol(optarg, NULL, 0));
        break;

      case 'h':
        help(argv[0], additional_opts_help);
        return 0;

      case 't':
        if (strcmp("time_truth", long_options[option_index].name) == 0) {
          ObservationTimeEstimator *observation_time_estimator = NULL;
          EphemerisTimeEstimator *ephemeris_time_estimator = NULL;
          UbxLeapTimeEstimator *ubx_leap_time_estimator = NULL;

          time_truth_reset(ubx_time_truth);
          time_truth_request_observation_time_estimator(
              ubx_time_truth,
              TIME_TRUTH_SOURCE_LOCAL,
              &observation_time_estimator);
          time_truth_request_ephemeris_time_estimator(
              ubx_time_truth,
              TIME_TRUTH_SOURCE_LOCAL,
              &ephemeris_time_estimator);
          time_truth_request_ubx_leap_time_estimator(ubx_time_truth,
                                                     TIME_TRUTH_SOURCE_LOCAL,
                                                     &ubx_leap_time_estimator);

          ubx_sbp_set_time_truth_estimators(&state,
                                            observation_time_estimator,
                                            ephemeris_time_estimator,
                                            ubx_leap_time_estimator);
        }
        break;

      default:
        break;
    }
  }

  int ret;
  do {
    ret = ubx_sbp_process(&state, readfn);
  } while (ret >= 0);

  return 0;
}
