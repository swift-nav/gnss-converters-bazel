/**
 * Copyright (C) 2021 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_TIME_TRUTH_H
#define GNSS_CONVERTERS_TIME_TRUTH_H

#include <swiftnav/gnss_time.h>

/**
 * Set of "time truth" states
 */
enum time_truth_state {
  /** No confidence in its ability to determine absolute GPS time */
  TIME_TRUTH_UNKNOWN,

  /** High confidence in its ability to determine absolute GPS time */
  TIME_TRUTH_SOLVED
};

/**
 * Set of time sources for the "time truth" state machine
 */
enum time_truth_source {
  /** GPS ephemeride */
  TIME_TRUTH_EPH_GPS,

  /** Galileo ephemeride */
  TIME_TRUTH_EPH_GAL,

  /** BeiDou ephemeride */
  TIME_TRUTH_EPH_BDS,

  /** GLONASS ephemeride */
  TIME_TRUTH_EPH_GLO,
};

/**
 * "Time truth" state machine structure
 */
struct time_truth {
  float tow;
  s16 wn;
  enum time_truth_state state;
};

/** Atomic time truth state machine type */
typedef _Atomic struct time_truth time_truth_t;

/**
 * Initializes the "time truth" state machine
 *
 * @param instance pointer to the state machine struct
 */
void time_truth_init(time_truth_t* instance);

/**
 * Passes in new time information into the "time truth" state machine
 *
 * @param instance pointer to the state machine struct
 * @param source source in which the time information is based off of
 * @param time relative time represented in GPS time format
 * @return true if the time truth time and/or state has been updated, otherwise
 * false
 *
 * @note "relative time" refers a direct representation of the information
 * coming from the data source. for instance, a time value for a source of
 * "EPH_GAL" is expected to be capped at {.tow = 604799, .wn = 4095}.
 */
bool time_truth_update(time_truth_t* instance,
                       enum time_truth_source source,
                       gps_time_t time);

/**
 * Retrieves the time truth's current time and state
 *
 * @param instance pointer to the state machine struct
 * @param state pointer to state object to fill in (NULL is valid)
 * @param time pointer to absolute gps time object to fill in (NULL is valid)
 */
void time_truth_get(time_truth_t* instance,
                    enum time_truth_state* state,
                    gps_time_t* time);

#endif  // GNSS_CONVERTERS_TIME_TRUTH_H
