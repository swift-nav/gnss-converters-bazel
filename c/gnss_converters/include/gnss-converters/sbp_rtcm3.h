/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_EXTRA_SBP_RTCM3_INTERFACE_H
#define GNSS_CONVERTERS_EXTRA_SBP_RTCM3_INTERFACE_H

#include <gnss-converters/rtcm3_sbp.h>
#include <libsbp/sbp.h>
#include <rtcm3/messages.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*sbp_rtcm_unix_time_callback_t)(int64_t *now);

struct rtcm3_out_state {
  int8_t cached_leap_seconds;
  bool cached_leap_seconds_known;

  int8_t user_leap_seconds;
  bool user_leap_seconds_known;
  sbp_rtcm_unix_time_callback_t unix_time_callback;
  TimeTruth *time_truth;
  TimeTruthCache *time_truth_cache;

  bool ant_known;
  s32 (*cb_sbp_to_rtcm)(u8 *buffer, u16 length, void *context);
  u16 sender_id;
  sbp_observation_header_t sbp_header;
  sbp_packed_obs_content_t sbp_obs_buffer[MAX_OBS_PER_EPOCH];
  u16 n_sbp_obs;
  void *context;
  /* GLO FCN map, indexed by 1-based PRN */
  u8 glo_sv_id_fcn_map[NUM_SATS_GLO + 1];

  /** RTCM OUT format options. */

  /* Note that while the specification does not forbid sending both the legacy
   * and MSM observations, it does not recommend it. */
  bool send_legacy_obs;
  bool send_msm_obs;
  msm_enum msm_type;

  double ant_height; /* Antenna height above ARP, meters */
  char ant_descriptor[RTCM_MAX_STRING_LEN];
  char rcv_descriptor[RTCM_MAX_STRING_LEN];
};

void sbp2rtcm_init(struct rtcm3_out_state *state,
                   s32 (*cb_sbp_to_rtcm)(u8 *buffer, u16 length, void *context),
                   void *context);

/**
 * Function allows users to specify the leap second explicitly rather than
 * having the converter use the unix time callback function or the time truth
 * object to query for it.
 *
 * @param leap_seconds current UTC->GPS leap second offset (value can be NULL at
 * which point function will mark leap second as unknown)
 * @param state pointer to converter object
 */
void sbp2rtcm_set_leap_second(const int8_t *leap_seconds,
                              struct rtcm3_out_state *state);

/**
 * Function allows users to specify a callback function which the converter may
 * invoke anytime it wishes to inquire about the current time. It uses the time
 * to determine the current leap second.
 *
 * The callback function signature takes in a numeric value which it will use to
 * store the unix time provided that it is able to successfully acquire a unix
 * time, if it is able to do so, it will return true otherwise it returns false.
 *
 * @param callback callback function which the converter should call. value can
 * be NULL, at which point it will remove any prior registered function.
 * @param state pointer to converter object
 */
void sbp2rtcm_set_unix_time_callback_function(
    rtcm_sbp_unix_time_callback_t callback, struct rtcm3_out_state *state);

/**
 * Allows user to specify a time truth object and its cache object which is used
 * internally to query the current leap second data. The time truth cache is
 * useful to avoid missing data during time truth queries.
 *
 * @param time_truth pointer to the time truth object (value can be NULL)
 * @param time_truth_cache pointer to the time truth cache (value can be NULL)
 * @param state pointer to converter object
 */
void sbp2rtcm_set_time_truth(TimeTruth *time_truth,
                             TimeTruthCache *time_truth_cache,
                             struct rtcm3_out_state *state);

void sbp2rtcm_set_rtcm_out_mode(msm_enum value, struct rtcm3_out_state *state);

void sbp2rtcm_set_glo_fcn(sbp_v4_gnss_signal_t sid,
                          u8 sbp_fcn,
                          struct rtcm3_out_state *state);

bool sbp2rtcm_set_ant_height(double ant_height, struct rtcm3_out_state *state);

void sbp2rtcm_set_rcv_ant_descriptors(const char *ant_descriptor,
                                      const char *rcv_descriptor,
                                      struct rtcm3_out_state *state);

void sbp2rtcm_base_pos_ecef_cb(u16 sender_id,
                               const sbp_msg_base_pos_ecef_t *msg,
                               struct rtcm3_out_state *state);

void sbp2rtcm_glo_biases_cb(u16 sender_id,
                            const sbp_msg_glo_biases_t *msg,
                            struct rtcm3_out_state *state);

void sbp2rtcm_sbp_obs_cb(u16 sender_id,
                         const sbp_msg_obs_t *msg,
                         struct rtcm3_out_state *state);

void sbp2rtcm_sbp_osr_cb(u16 sender_id,
                         const sbp_msg_osr_t *msg,
                         struct rtcm3_out_state *state);

void sbp2rtcm_sbp_ssr_orbit_clock_cb(u16 sender_id,
                                     const sbp_msg_ssr_orbit_clock_t *msg,
                                     struct rtcm3_out_state *state);

void sbp2rtcm_sbp_ssr_phase_biases_cb(u16 sender_id,
                                      const sbp_msg_ssr_phase_biases_t *msg,
                                      struct rtcm3_out_state *state);

void sbp2rtcm_sbp_ssr_code_biases_cb(u16 sender_id,
                                     const sbp_msg_ssr_code_biases_t *msg,
                                     struct rtcm3_out_state *state);

void sbp2rtcm_sbp_ssr_gridded_correction_cb(
    u16 sender_id,
    const sbp_msg_ssr_gridded_correction_dep_a_t *msg,
    struct rtcm3_out_state *state);

void sbp2rtcm_sbp_ssr_grid_definition_cb(
    u16 sender_id,
    const sbp_msg_ssr_grid_definition_dep_a_t *msg,
    struct rtcm3_out_state *state);

void sbp2rtcm_sbp_ssr_stec_correction_cb(
    u16 sender_id,
    const sbp_msg_ssr_stec_correction_dep_a_t *msg,
    struct rtcm3_out_state *state);

void sbp2rtcm_sbp_gps_eph_cb(u16 sender_id,
                             const sbp_msg_ephemeris_gps_t *msg,
                             struct rtcm3_out_state *state);

void sbp2rtcm_sbp_glo_eph_cb(u16 sender_id,
                             const sbp_msg_ephemeris_glo_t *msg,
                             struct rtcm3_out_state *state);

void sbp2rtcm_sbp_bds_eph_cb(u16 sender_id,
                             const sbp_msg_ephemeris_bds_t *msg,
                             struct rtcm3_out_state *state);

void sbp2rtcm_sbp_gal_eph_cb(u16 sender_id,
                             const sbp_msg_ephemeris_gal_t *msg,
                             struct rtcm3_out_state *state);

void sbp2rtcm_sbp_log_cb(u16 sender_id,
                         const sbp_msg_log_t *msg,
                         struct rtcm3_out_state *state);

void sbp2rtcm_sbp_cb(uint16_t sender_id,
                     sbp_msg_type_t msg_type,
                     const sbp_msg_t *msg,
                     struct rtcm3_out_state *state);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_EXTRA_SBP_RTCM3_INTERFACE_H */
