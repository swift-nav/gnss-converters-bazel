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

#ifndef GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H
#define GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H

#include <gnss-converters/eph_sat_data.h>
#include <gnss-converters/time_truth_v2.h>
#include <libsbp/sbp.h>
#include <libsbp/v4/observation.h>
#include <rtcm3/messages.h>
#include <swiftnav/fifo_byte.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif

/* This is the maximum number of SBP observations possible per epoch:
   - Max number of observation messages comes from the 4 bits assigned to the
     sequence count in header.n_obs
   - The number of observations per message comes from the max 255 byte
     message length
*/
#define SBP_OBS_SIZE (sizeof(sbp_packed_obs_content_t))
#define SBP_MAX_OBS_SEQ (15u)
#define MAX_OBS_PER_EPOCH (SBP_MAX_OBS_SEQ * SBP_MSG_OBS_OBS_MAX)
#define OBS_BUFFER_SIZE (MAX_OBS_PER_EPOCH * SBP_OBS_SIZE)

#define INVALID_TIME 0xFFFF
#define MAX_WN (INT16_MAX)

#define IS_POWER_OF_TWO(x) (0 == ((x) & ((x)-1)))

#define RTCM_MAX_SINGLE_SAT_SIGNAL 3

typedef enum {
  UNSUPPORTED_CODE_UNKNOWN = 0u,
  UNSUPPORTED_CODE_GLO_L1P,
  UNSUPPORTED_CODE_GLO_L2P,
  UNSUPPORTED_CODE_MAX
} unsupported_code_t;

/* A struct for storing either an SSR orbit correction message
or an SSR clock correction message. Used for combining the
separate messages into a combined message */
typedef struct {
  union {
    rtcm_msg_clock clock;
    rtcm_msg_orbit orbit;
  } data;
  /* Only one of these can be true at once (or neither) */
  bool contains_clock; /* True if clock contains data */
  bool contains_orbit; /* True if orbit contains data */
} ssr_orbit_clock_cache;

typedef bool (*rtcm_sbp_unix_time_callback_t)(int64_t *now);

/** Represent measurement info of single satellite. Satellite info is
 * transmitted under a range of signal (eg L1/L2/L5). Each signal has a code and
 * CNO. */
struct rtcm_meas_sat_signal {
  struct {
    uint8_t cn0;  // GNSS signal CNR [dB-Hz / 4]
    code_t code;
  } sig_data[RTCM_MAX_SINGLE_SAT_SIGNAL];
  uint8_t n_signal;
};

/** Represent info of single constellation, including a list of gnss satellites
and maximum number of satellites. */
struct rtcm_gnss_signal {
  uint8_t max_n_sat;
  struct rtcm_meas_sat_signal sat_data[MAX_NUM_SATS];
};

struct rtcm3_sbp_state {
  bool has_cached_time;
  bool cached_gps_time_known;
  gps_time_t cached_gps_time;
  bool cached_leap_seconds_known;
  int8_t cached_leap_seconds;

  bool user_gps_time_known;
  gps_time_t user_gps_time;
  bool user_leap_seconds_known;
  int8_t user_leap_seconds;

  rtcm_sbp_unix_time_callback_t unix_time_callback;

  bool rtcm_1013_time_known;
  gps_time_t rtcm_1013_gps_time;
  int8_t rtcm_1013_leap_seconds;

  const char *name;
  uint16_t time_truth_last_wn;
  TimeTruthState time_truth_last_wn_state;
  uint32_t time_truth_last_tow_ms;
  TimeTruthState time_truth_last_tow_ms_state;
  int8_t time_truth_last_leap_seconds;
  TimeTruthState time_truth_last_leap_seconds_state;

  u16 sender_id;
  gps_time_t last_gps_time;
  gps_time_t last_glo_time;
  gps_time_t last_1230_received;
  gps_time_t last_msm_received;
  TimeTruth *time_truth;
  TimeTruthCache *time_truth_cache;
  uint16_t gps_week_reference;
  void (*cb_rtcm_to_sbp)(uint16_t sender_id,
                         sbp_msg_type_t msg_type,
                         const sbp_msg_t *msg,
                         void *context);
  void (*cb_base_obs_invalid)(double time_diff, void *context);
  void *context;

  /* Track multi obs msg. SBP msgs are sent once this flag is true */
  bool send_observation_flag;

  ObservationTimeEstimator *observation_time_estimator;
  EphemerisTimeEstimator *ephemeris_time_estimator;
  Rtcm1013TimeEstimator *rtcm_1013_time_estimator;

  uint8_t obs_to_send;
  sbp_packed_obs_content_t obs_buffer[MAX_OBS_PER_EPOCH];
  sbp_v4_gps_time_t obs_time;
  bool sent_msm_warning;
  bool sent_code_warning[UNSUPPORTED_CODE_MAX];
  /* GLO FCN map, indexed by 1-based PRN */
  u8 glo_sv_id_fcn_map[NUM_SATS_GLO + 1];
  /* The cache for storing the first message before combining the separate
  orbit and clock messages into a combined SBP message */
  ssr_orbit_clock_cache orbit_clock_cache[CONSTELLATION_COUNT];
  uint8_t fifo_buf[RTCM3_FIFO_SIZE];
  fifo_t fifo;
  struct eph_sat_data eph_data;
  sbp_state_t sbp_state;

  /* Track azel and meas full msg when receiving stgsv msg */
  uint32_t tow_ms_azel;
  uint32_t tow_ms_meas;
  sbp_msg_t msg_azel_full;
  sbp_msg_t msg_meas_full;

  bool msg_azel_full_sent;
  bool msg_meas_full_sent;

  /* A "map" of measurement info of constellations. Dimension = #constellation.
   * Each constellation is represented in gnss_signal struct. For convenience
   * the array is stored in the order of constellation enum defined in
   * rtcm_constellation_t. Info is extracted from the observation msgs. */
  struct rtcm_gnss_signal cons_meas_map[RTCM_CONSTELLATION_COUNT];
};

/**
 * This is the combination of decoder & converter. The whole RTCM frame is
 * decoded, before output of the decoder is converted into SBP message.
 * @param frame RTCM frame
 * @param frame_length Length of RTCM frame
 * @param state Pointer to converter state
 */
void rtcm2sbp_decode_frame(const uint8_t *frame,
                           uint32_t frame_length,
                           struct rtcm3_sbp_state *state);

/**
 * This is the combination of decoder & converter. The RTCM message/payload is
 * decoded, before output of the decoder is converted into SBP message.
 * @param payload RTCM message/payload
 * @param payload_length Length of RTCM message
 * @param state Pointer to converter state
 */
void rtcm2sbp_decode_payload(const uint8_t *payload,
                             uint32_t payload_length,
                             struct rtcm3_sbp_state *state);

/**
 * The converter converts RTCM message in form of RTCM data struct to SBP msg.
 * @param rtcm_msg Pointer to RTCM message (struct)
 * @param state Pointer to converter state
 */
void rtcm2sbp_convert(const rtcm_msg_data_t *rtcm_msg,
                      struct rtcm3_sbp_state *state);

void rtcm2sbp_set_glo_fcn(sbp_v4_gnss_signal_t sid,
                          u8 sbp_fcn,
                          struct rtcm3_sbp_state *state);

/**
 * Checks whether converter is using user specified time.
 *
 * @param state pointer to converter object
 * @return true if converter is using used provided time (via
 * #rtcm2sbp_set_time), false if it is using time truth instance.
 */
bool rtcm2sbp_is_using_user_provided_time(struct rtcm3_sbp_state *state);

/**
 * Set a name for the converter to assist with some of the log outputs.
 *
 * @param name name of the converter (converter will not take a copy of the
 * name, so the lifetime of the name must exceed that of the converter)
 * @param state pointer to converter object
 */
void rtcm2sbp_set_name(const char *name, struct rtcm3_sbp_state *state);

/**
 * Function allows users to specify the current time explicitly rather than
 * using the time truth instance.
 *
 * If the user specifies a GPS time which is within the knowledge of the
 * system's GPS->leap second lookup table (#utc_leaps), than the converter will
 * ignore any user input to the leap seconds argument and use the value from the
 * lookup table. If the user specifies a GPS time that is outside the knowledge
 * of the system' GPS->leap second lookup table, than it will use the leap
 * second value provided.
 *
 * @param gps_time current GPS time (value can be NULL or invalid, at which
 * point the converter reverts back to using the time truth if its provided)
 * @param leap_seconds current UTC->GPS leap second offset (value can be NULL)
 * @param state pointer to converter object
 */
void rtcm2sbp_set_time(const gps_time_t *gps_time,
                       const int8_t *leap_seconds,
                       struct rtcm3_sbp_state *state);

/**
 * Function allows users to specify a callback function which the converter may
 * invoke anytime it wishes to inquire about the current time.
 *
 * The callback function signature takes in a numeric value which it will use to
 * store the unix time provided that it is able to successfully acquire a unix
 * time, if it is able to do so, it will return true otherwise it returns false.
 *
 * @param callback callback function which the converter should call. value can
 * be NULL, at which point it will remove any prior registered function.
 * @param state pointer to converter object
 */
void rtcm2sbp_set_unix_time_callback_function(
    rtcm_sbp_unix_time_callback_t callback, struct rtcm3_sbp_state *state);

/**
 * Allows user to specify a cache object to its time truth instance. This is
 * useful to avoid missing data during time truth queries.
 *
 * @param time_truth_cache pointer to the time truth cache
 * @param state pointer to converter object
 */
void rtcm2sbp_set_time_truth_cache(TimeTruthCache *time_truth_cache,
                                   struct rtcm3_sbp_state *state);

/**
 * Allows users to specify a last known GPS week number to deal with the ~20
 * year GPS rollover period. Value passed in should certainly be far enough in
 * the past that any data flowing through the converter will refer to a time
 * equal to or further into the future than the specified reference week.
 * Failing to do this will mean the converter will incorrectly assume that the
 * time is ~20 years into the future.
 *
 * @param gps_week_reference known week number in the past
 * @param state pointer to converter object
 */
void rtcm2sbp_set_gps_week_reference(uint16_t gps_week_reference,
                                     struct rtcm3_sbp_state *state);

/**
 * Offers the means for users to enlist time estimator instance which the
 * converter will call upon when timing information is available.
 *
 * @param observation_time_estimator pointer to the observation time estimator
 * @param ephemeris_time_estimator pointer to the ephemeris time estimator
 * @param rtcm_1013_time_estimator pointer to the RTCM 1013 time estimator
 * @param state pointer to converter object
 */
void rtcm2sbp_set_time_truth_estimators(
    ObservationTimeEstimator *observation_time_estimator,
    EphemerisTimeEstimator *ephemeris_time_estimator,
    Rtcm1013TimeEstimator *rtcm_1013_time_estimator,
    struct rtcm3_sbp_state *state);

/**
 * Initializes the RTCM to SBP converter object.
 *
 * @param state pointer to converter object
 * @param time_truth pointer to time truth instance, value can be NULL at which
 * point users are required to specify time explicitly via #rtcm2sbp_set_time
 * prior to running the converter
 * @param cb_rtcm_to_sbp callback function which is invoked when conversion from
 * RTCM to SBP is made
 * @param cb_base_obs_invalid callback function which is invoked when
 * observation time greatly mismatches the specified time
 * @param context context object which is passed to the callback functions
 */
void rtcm2sbp_init(struct rtcm3_sbp_state *state,
                   TimeTruth *time_truth,
                   void (*cb_rtcm_to_sbp)(uint16_t sender_id,
                                          sbp_msg_type_t msg_type,
                                          const sbp_msg_t *msg,
                                          void *context),
                   void (*cb_base_obs_invalid)(double time_diff, void *context),
                   void *context);

int rtcm2sbp_process(struct rtcm3_sbp_state *state,
                     int (*read_stream_func)(uint8_t *buf,
                                             size_t len,
                                             void *context));

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_RTCM3_SBP_INTERFACE_H */
