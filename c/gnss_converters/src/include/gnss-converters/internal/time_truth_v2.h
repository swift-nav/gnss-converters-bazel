/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef GNSS_CONVERTERS_INTERNAL_TIME_TRUTH_V2_H
#define GNSS_CONVERTERS_INTERNAL_TIME_TRUTH_V2_H

#include <gnss-converters/internal/static_vector.h>
#include <gnss-converters/time_truth_v2.h>
#include <swiftnav/gnss_time.h>

#include <cinttypes>
#include <cstddef>

namespace gnss_converters {

/**
 * Total TOW (millisecond) estimations per source
 *
 * ONLY the observation time estimator contributes to the TOW (millisecond)
 * estimations.
 */
constexpr size_t kTotalTowMsEstimationPerSource = 1;

/**
 * Total leap second estimations per source
 *
 * Both RTCM 1013 and UBX leap estimators contribute to the leap second
 * estimations.
 */
constexpr size_t kTotalLeapSecondEstimationPerSource = 2;

/**
 * Total week number estimations per source
 *
 * The ephemeris, RTCM 1013 and UBX leap estimators contribute to the week
 * number estimations.
 */
constexpr size_t kTotalWeekNumberEstimationPerSource = 3;

/**
 * Maximum time difference between two TOW (time of week) values before it is
 * considered a mismatch in time. Value is expressed as milliseconds.
 *
 * @see kObservationEstimatorMaximumTowMsTimeMismatchCount
 */
constexpr uint32_t kObservationEstimatorMaximumTowMsTimeDifference =
    5 * SECS_MS;

/**
 * Number of times that an estimator will accept a mismatch in TOW before it
 * assumes that TOW has jumped.
 */
constexpr uint32_t kObservationEstimatorMaximumTowMsTimeMismatchCount = 5;

/**
 * Minimum number of ephemeris signals that must come from a constellation
 * before it gets included into the pile of all accepted ephemeris signals.
 */
constexpr size_t kEphemerisEstimatorMinimumSignalsToQualify = 6;

/**
 * Maximum inter-quartile difference (milliseconds) before the ephemeris time
 * estimator discontinues attempt so analyze data.
 */
constexpr uint64_t kEphemerisEstimatorMaximumInterQuartileDifferenceMs =
    8 * HOUR_SECS * SECS_MS;

/**
 * Additional tolerance offset (milliseconds) applied to the inter quartile
 * range to try and include passed in TOW best estimate
 */
constexpr uint64_t kEphemerisEstimatorInterQuartileOffsetMs =
    2.5 * HOUR_SECS * SECS_MS;

/**
 * RTCM 1013's time estimator tolerance between best TOW (millisecond) estimate
 * and GPS time.
 */
constexpr double kRtcm1013EstimatorGpsTimeTowMsTolerance =
    1 * MINUTE_SECS * SECS_MS;

/**
 * UBX leap time estimator tolerance between best TOW (millisecond) estimate and
 * GPS time.
 */
constexpr double kUbxLeapEstimatorGpsTimeTowMsTolerance =
    1 * MINUTE_SECS * SECS_MS;

/**
 * Maximum range which is acceptable from a list of best guesses for TOW
 * (millisecond) estimates
 */
constexpr uint32_t kGetLatestTimeMaximumTowMsRange = 10 * SECS_MS;

/**
 * Maximum range which is acceptable from a list of best guesses for ephemeris
 * estimates
 */
constexpr uint32_t kGetLatestTimeMaximumEphemerisRange = 6 * HOUR_SECS;

struct MatchCounter {
  size_t same_source_count;
  size_t cross_source_count;
};

struct QuartileIndices {
  std::pair<size_t, size_t> lower_quartile;
  std::pair<size_t, size_t> middle_quartile;
  std::pair<size_t, size_t> upper_quartile;

  bool operator==(const QuartileIndices &other) const {
    return lower_quartile == other.lower_quartile &&
           middle_quartile == other.middle_quartile &&
           upper_quartile == other.upper_quartile;
  }
};

/**
 * Given a number of sample points in a sorted list of values, this function
 * will return the indices for the lower/middle/upper quartiles. For example
 * if we had the fibonacci sequence of:
 *
 *   [  1  1  2  3  5  8  13  21  34  ]
 *
 * The quartile values would be:
 *
 *   middle quartile: 5 (index 4)
 *   lower quartile: 1.5 (between index 1 and 2)
 *   upper quartile: 17  (between index 6 and 7)
 *
 * @param size number of the samples in a sorted list
 * @return quartile indices location
 */
QuartileIndices get_quartile_indices(size_t size);

/**
 * Validates that the TOW (millisecond) value is within the range [0,
 * 604,800,000).
 *
 * @param tow_ms input TOW (millisecond) value
 * @return true if its a valid value
 */
bool is_tow_ms_valid(uint32_t tow_ms);

/**
 * Given two TOW (millisecond) values, this function will determine if the two
 * values are at most "tow_ms_tolerance" values apart. This function deals with
 * TOW rollover.
 *
 * NOTE: function will assume that all the TOW (millisecond) values are valid.
 *
 * @param this_tow_ms first TOW (millisecond) operand
 * @param other_tow_ms second TOW (millisecond) operand
 * @param tow_ms_tolerance maximum distance apart that the two operands are
 * allowed to be before they are considered outside the range
 * @return true if the two operands are within the "tow_ms_tolerance" distance
 */
bool is_tow_ms_within_tolerance(uint32_t this_tow_ms,
                                uint32_t other_tow_ms,
                                uint32_t tow_ms_tolerance);

/**
 * Given a list of TOW (millisecond) values, the function will determine if all
 * the values fall under the range of "tolerance". If they all do, than the
 * function will return true and set the "latest_tow_ms" to the latest TOW
 * (millisecond) value within that range.
 *
 * NOTE: this function will deal with rollover period of TOW values.
 *
 * @param tows_ms point to the list of TOW (millisecond) values
 * @param tows_ms_count number of values in the "tow_ms" list
 * @param tolerance maximum distance between the earliest/latest "tow_ms" list
 * @param latest_tow_ms pointer to save the latest TOW (millisecond) value
 * @return true if the values are all within the tolerance value, otherwise
 * return false. if no values are provided, than it returns false. if only one
 * value is returned than it will always return true.
 */
bool is_tow_ms_within_tolerance(const uint32_t *tows_ms,
                                size_t tows_ms_count,
                                uint32_t tolerance,
                                uint32_t *latest_tow_ms);

/**
 * Given a course estimate of GPS time and an accurate TOW (millisecond) value,
 * the function will adjust the GPS time value to align with the accurate TOW
 * value. The function will only do this if the GPS time is off from the TOW
 * (millisecond) value by a tolerance value expressed in milliseconds.
 *
 * @param gps_time course time
 * @param tow_ms accurate TOW value
 * @param tolerance_ms tolerance expressed in milliseconds
 * @return true if the GPS time is within tolerance from the TOW value, if so
 * the gps_time parameter is adjusted, otherwise false is returned
 */
bool match_gps_time_with_tow_ms(gps_time_t *gps_time,
                                uint32_t tow_ms,
                                uint32_t tolerance_ms);

/**
 * Given an estimators, this function will try and extract the states for and
 * save it into the estimator states output parameter. If the state isn't
 * extractable via the estimator, users can pass in a cache object which it will
 * try and use the last obtained state.
 *
 * NOTE: function will update the cache with the latest state.
 *
 * @tparam T time estimator type
 * @param estimators estimator objects
 * @param estimator_states_available location to store whether state has been
 * successfully extracted
 * @param estimator_states location to store the estimator's state if it was
 * able to obtain either from estimator directly or cache
 * @param cache_states_available pointer to the member variable that holds the
 * state availability in TimeTruthCache
 * @param cache_states pointer to the member variable that holds the state in
 * TimeTruthCache
 * @param cache pointer to the cache object (value can be nullptr)
 */
template <typename T>
void extract_estimator_states(
    bool (&estimators_used)[TIME_TRUTH_SOURCE_COUNT],
    const T (&estimators)[TIME_TRUTH_SOURCE_COUNT],
    bool (&estimator_states_available)[TIME_TRUTH_SOURCE_COUNT],
    typename T::State (&estimator_states)[TIME_TRUTH_SOURCE_COUNT],
    bool (TimeTruthCache::*cache_states_available)[TIME_TRUTH_SOURCE_COUNT],
    typename T::State (TimeTruthCache::*cache_states)[TIME_TRUTH_SOURCE_COUNT],
    TimeTruthCache *cache = nullptr) {
  for (auto source = 0; source < TIME_TRUTH_SOURCE_COUNT; ++source) {
    if (estimators_used[source] &&
        estimators[source].load_state(&estimator_states[source])) {
      if (cache) {
        (cache->*cache_states_available)[source] = true;
        (cache->*cache_states)[source] = estimator_states[source];
      }
      estimator_states_available[source] = true;
    } else if (cache && (cache->*cache_states_available)[source]) {
      estimator_states[source] = (cache->*cache_states)[source];
      estimator_states_available[source] = true;
    } else {
      estimator_states_available[source] = false;
    }
  }
}

/**
 * Given a list of TOW (millisecond) estimates and their associated data
 * sources, this function will return the best estimate of the TOW (millisecond)
 * and its state confidence.
 *
 * @param estimates list of all the TOW (millisecond) estimates
 * @param tolerance maximum tolerance allowed between matches
 * @param tow_ms best calculated tow estimate
 * @param state best calculated tow estimate's confidence state
 */
void calculate_tow_ms(
    const StaticVector<std::pair<uint32_t, TimeTruthSource>,
                       kTotalTowMsEstimationPerSource * TIME_TRUTH_SOURCE_COUNT>
        &estimates,
    uint32_t tolerance,
    uint32_t *tow_ms,
    TimeTruthState *state);

/**
 * Given a list of leap second estimates and their associated data sources, this
 * function will return the best estimate of the leap second and its state
 * confidence.
 *
 * @param estimates list of all the leap second estimates
 * @param leap_seconds best calculated leap second estimate
 * @param state best calculated leap second estimate's confidence state
 */
void calculate_leap_seconds(
    const StaticVector<std::pair<int8_t, TimeTruthSource>,
                       kTotalLeapSecondEstimationPerSource *
                           TIME_TRUTH_SOURCE_COUNT> &estimates,
    int8_t *leap_seconds,
    TimeTruthState *state);

/**
 * Given a list of week number estimates and their associated data sources, this
 * function will return the best estimate of the week number and its state
 * confidence.
 *
 * @param estimates list of all the week number estimates
 * @param leap_seconds best calculated week number estimate
 * @param state best calculated week number estimate's confidence state
 */
void calculate_week_number(
    const StaticVector<std::pair<uint16_t, TimeTruthSource>,
                       kTotalWeekNumberEstimationPerSource *
                           TIME_TRUTH_SOURCE_COUNT> &estimates,
    uint16_t *wn,
    TimeTruthState *state);

}  // namespace gnss_converters

#endif  // GNSS_CONVERTERS_INTERNAL_TIME_TRUTH_V2_H
