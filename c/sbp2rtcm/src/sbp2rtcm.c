/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/* This is a stand-alone tool that takes Swift Binary Protocol (SBP) on stdin
 * and writes RTCM3 on stdout. */

#include <assert.h>
#include <gnss-converters/sbp_rtcm3.h>
#include <libsbp/sbp.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <swiftnav/gnss_time.h>
#include <unistd.h>

typedef int32_t (*readfn_ptr)(uint8_t *, uint32_t, void *);
typedef int32_t (*read_eof_fn_ptr)(void *);
typedef int32_t (*writefn_ptr)(uint8_t *, uint16_t, void *);

static writefn_ptr sbp2rtcm_writefn;

typedef struct {
  sbp_msg_callbacks_node_t base_pos;
  sbp_msg_callbacks_node_t glo_biases;
  sbp_msg_callbacks_node_t obs;
  sbp_msg_callbacks_node_t osr;
  sbp_msg_callbacks_node_t ssr_orbit_clock;
  sbp_msg_callbacks_node_t ssr_orbit_clock_bounds;
  sbp_msg_callbacks_node_t ssr_orbit_clock_bounds_degradation;
  sbp_msg_callbacks_node_t ssr_phase_biases;
  sbp_msg_callbacks_node_t ssr_code_biases;
  sbp_msg_callbacks_node_t ssr_code_phase_biases_bounds;
  sbp_msg_callbacks_node_t ssr_gridded_correction_dep_a;
  sbp_msg_callbacks_node_t ssr_gridded_correction_bounds;
  sbp_msg_callbacks_node_t ssr_grid_definition_dep_a;
  sbp_msg_callbacks_node_t ssr_stec_correction_dep_a;
  sbp_msg_callbacks_node_t ssr_stec_correction;
  sbp_msg_callbacks_node_t ssr_tile_definition;
  sbp_msg_callbacks_node_t ephemeris_gps;
  sbp_msg_callbacks_node_t ephemeris_gal;
  sbp_msg_callbacks_node_t ephemeris_bds;
  sbp_msg_callbacks_node_t ephemeris_qzss;
  sbp_msg_callbacks_node_t ephemeris_glo;
  sbp_msg_callbacks_node_t ssr_flag_high_level;
  sbp_msg_callbacks_node_t ssr_flag_satellites;
  sbp_msg_callbacks_node_t ssr_flag_tropo_grid_points;
  sbp_msg_callbacks_node_t ssr_flag_iono_grid_points;
  sbp_msg_callbacks_node_t ssr_flag_iono_tile_sat_los;
  sbp_msg_callbacks_node_t ssr_flag_iono_grid_point_sat_los;
  sbp_msg_callbacks_node_t utc_leap_second;
  sbp_msg_callbacks_node_t reference_frame_parameters;
  sbp_msg_callbacks_node_t log;
} sbp_nodes_t;

static void help(char *arg, const char *additional_opts_help) {
  fprintf(stderr, "Usage: %s [options]%s\n", arg, additional_opts_help);
  fprintf(stderr, "  -h this message\n");
}

static void sbp2rtcm_sbp_void_cb(uint16_t sender_id,
                                 sbp_msg_type_t msg_type,
                                 const sbp_msg_t *msg,
                                 void *context) {
  sbp2rtcm_sbp_cb(sender_id, msg_type, msg, (struct rtcm3_out_state *)context);
}

int sbp2rtcm_main(int argc,
                  char **argv,
                  const char *additional_opts_help,
                  readfn_ptr readfn,
                  read_eof_fn_ptr read_eof_fn,
                  writefn_ptr writefn,
                  void *context) {
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

  sbp2rtcm_writefn = writefn;

  /* set time from systime, account for UTC<->GPS leap second difference */
  time_t ct_utc_unix = time(NULL);
  gps_time_t noleapsec = time2gps_t(ct_utc_unix);
  double gps_utc_offset = get_gps_utc_offset(&noleapsec, NULL);

  struct rtcm3_out_state state;
  int8_t leap_seconds = lrint(gps_utc_offset);
  sbp2rtcm_init(&state, sbp2rtcm_writefn, context);
  sbp2rtcm_set_leap_second(&leap_seconds, &state);

  sbp_nodes_t sbp_nodes = {};
  sbp_state_t sbp_state;
  sbp_state_init(&sbp_state);
  sbp_state_set_io_context(&sbp_state, context);

  struct {
    sbp_msg_type_t msg_type;
    sbp_msg_callbacks_node_t node;
  } cb[29] = {
      {SbpMsgBasePosEcef, sbp_nodes.base_pos},
      {SbpMsgGloBiases, sbp_nodes.glo_biases},
      {SbpMsgObs, sbp_nodes.obs},
      {SbpMsgOsr, sbp_nodes.osr},
      {SbpMsgSsrOrbitClock, sbp_nodes.ssr_orbit_clock},
      {SbpMsgSsrOrbitClockBounds, sbp_nodes.ssr_orbit_clock_bounds},
      {SbpMsgSsrOrbitClockBoundsDegradation,
       sbp_nodes.ssr_orbit_clock_bounds_degradation},
      {SbpMsgSsrPhaseBiases, sbp_nodes.ssr_phase_biases},
      {SbpMsgSsrCodeBiases, sbp_nodes.ssr_code_biases},
      {SbpMsgSsrCodePhaseBiasesBounds, sbp_nodes.ssr_code_phase_biases_bounds},
      {SbpMsgSsrGriddedCorrectionDepA, sbp_nodes.ssr_gridded_correction_dep_a},
      {SbpMsgSsrGriddedCorrectionBounds,
       sbp_nodes.ssr_gridded_correction_bounds},
      {SbpMsgSsrGridDefinitionDepA, sbp_nodes.ssr_grid_definition_dep_a},
      {SbpMsgSsrStecCorrectionDepA, sbp_nodes.ssr_stec_correction_dep_a},
      {SbpMsgSsrStecCorrection, sbp_nodes.ssr_stec_correction},
      {SbpMsgSsrTileDefinition, sbp_nodes.ssr_tile_definition},
      {SbpMsgEphemerisGps, sbp_nodes.ephemeris_gps},
      {SbpMsgEphemerisGlo, sbp_nodes.ephemeris_glo},
      {SbpMsgEphemerisBds, sbp_nodes.ephemeris_bds},
      {SbpMsgEphemerisGal, sbp_nodes.ephemeris_gal},
      {SbpMsgSsrFlagHighLevel, sbp_nodes.ssr_flag_high_level},
      {SbpMsgSsrFlagSatellites, sbp_nodes.ssr_flag_satellites},
      {SbpMsgSsrFlagTropoGridPoints, sbp_nodes.ssr_flag_tropo_grid_points},
      {SbpMsgSsrFlagIonoGridPoints, sbp_nodes.ssr_flag_iono_grid_points},
      {SbpMsgSsrFlagIonoTileSatLos, sbp_nodes.ssr_flag_iono_tile_sat_los},
      {SbpMsgSsrFlagIonoGridPointSatLos,
       sbp_nodes.ssr_flag_iono_grid_point_sat_los},
      {SbpMsgUtcLeapSecond, sbp_nodes.utc_leap_second},
      {SbpMsgReferenceFrameParam, sbp_nodes.reference_frame_parameters},
      {SbpMsgLog, sbp_nodes.log}};

  for (size_t i = 0; i < ARRAY_SIZE(cb); i++) {
    sbp_callback_register(
        &sbp_state, cb[i].msg_type, &sbp2rtcm_sbp_void_cb, &state, &cb[i].node);
  }

  while (!read_eof_fn(context)) {
    sbp_process(&sbp_state, readfn);
  }
  return 0;
}
