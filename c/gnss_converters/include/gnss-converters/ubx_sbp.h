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

#ifndef GNSS_CONVERTERS_UBX_SBP_INTERFACE_H
#define GNSS_CONVERTERS_UBX_SBP_INTERFACE_H

#include <gnss-converters/eph_sat_data.h>
#include <gnss-converters/time_truth_v2.h>
#include <libsbp/sbp.h>
#include <libsbp/v4/gnss.h>
#include <libsbp/v4/navigation.h>
#include <libsbp/v4/observation.h>
#include <libsbp/v4/orientation.h>
#include <swiftnav/bytestream.h>
#include <unistd.h>

#include "swiftnav/gnss_time.h"
#include "swiftnav/signal.h"

#ifdef __cplusplus
extern "C" {
#endif

/** UBX Frame:
 * SYNC_CHAR_1 | SYNC_CHAR_2 | CLASS | MSG_ID | 2-byte Length | Payload |
 * CHCKSUM_BYTE_1 | CHCKSUM_BYTE_2
 */
#define UBX_SYNC_BYTE_COUNT (2)
#define UBX_CLASS_BYTE_COUNT (1)
#define UBX_MSG_ID_BYTE_COUNT (1)
#define UBX_LENGTH_BYTE_COUNT (2)
#define UBX_CHECKSUM_BYTE_COUNT (2)

#define UBX_BUFFER_SIZE 4096
#define UBX_FRAME_SIZE 4096

#define DEFAULT_UBX_SENDER_ID 61568

#define UBX_UNKNOWN_PRN 255

/* Stores state for ESF-* messages */
struct ubx_esf_state {
  gps_time_t last_obs_time_gnss;
  gps_time_t last_imu_time_msss;
  s64 running_imu_msss;
  s64 running_odo_msss;
  u32 last_imu_msss;
  u32 last_odo_msss;
  u32 imu_raw_msgs_sent;
  u32 last_input_tick_count;
  s32 output_tick_count;
  double last_imu_temp;
  bool temperature_set;
  bool weeknumber_set;
};

struct ubx_sbp_state {
  u8 read_buffer[UBX_BUFFER_SIZE];
  size_t index;
  size_t bytes_in_buffer;
  u16 sender_id;
  int (*read_stream_func)(u8 *buf, size_t len, void *ctx);
  void (*cb_ubx_to_sbp)(uint16_t sender_id,
                        sbp_msg_type_t msg_type,
                        const sbp_msg_t *msg,
                        void *context);
  void *context;
  bool use_hnr;
  struct ubx_esf_state esf_state;

  struct eph_sat_data eph_data;
  u32 last_tow_ms;
  sbp_msg_orient_euler_t last_orient_euler;

  bool leap_second_known;
  utc_params_t utc_params;

  ObservationTimeEstimator *observation_time_estimator;
  EphemerisTimeEstimator *ephemeris_time_estimator;
  UbxLeapTimeEstimator *ubx_leap_time_estimator;
};

int16_t ubx_convert_temperature_to_bmi160(double temperature_degrees);

void ubx_sbp_init(struct ubx_sbp_state *state,
                  void (*cb_ubx_to_sbp)(uint16_t sender_id,
                                        sbp_msg_type_t msg_type,
                                        const sbp_msg_t *msg,
                                        void *context),
                  void *context);
void ubx_handle_frame(swiftnav_bytestream_t *frame,
                      struct ubx_sbp_state *state);
void ubx_set_sender_id(struct ubx_sbp_state *state, u16 sender_id);
void ubx_set_hnr_flag(struct ubx_sbp_state *state, bool use_hnr);
int ubx_sbp_process(struct ubx_sbp_state *state,
                    int (*read_stream_func)(u8 *buff, size_t len, void *ctx));

/**
 * Offers the means for users to enlist time estimator instance which the
 * converter will call upon when timing information is available.
 *
 * @param state pointer to converter object
 * @param observation_time_estimator pointer to the observation time estimator
 * @param ephemeris_time_estimator pointer to the ephemeris time estimator
 * @param ubx_leap_time_estimator pointer to the UBX leap time estimator
 */
void ubx_sbp_set_time_truth_estimators(
    struct ubx_sbp_state *state,
    ObservationTimeEstimator *observation_time_estimator,
    EphemerisTimeEstimator *ephemeris_time_estimator,
    UbxLeapTimeEstimator *ubx_leap_time_estimator);

void invalidate_subframes(struct sat_data *sat, unsigned mask);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_CONVERTERS_UBX_SBP_INTERFACE_H */
