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

#ifndef GNSS_CONVERTERS_TIME_TRUTH_V2_H
#define GNSS_CONVERTERS_TIME_TRUTH_V2_H

#include <swiftnav/gnss_time.h>
#include <swiftnav/signal.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Enumerator that expresses where timing information is being obtained from.
 */
typedef enum {
  /**
   * Timing information is coming from a local source, like a GNSS antenna.
   */
  TIME_TRUTH_SOURCE_LOCAL,

  /**
   * Timing information is coming from an external source, like a NTRIP
   * broadcaster.
   */
  TIME_TRUTH_SOURCE_REMOTE,

  /**
   * Number of different sources in the numerator type. This enum value must
   * always be at the end of the list.
   */
  TIME_TRUTH_SOURCE_COUNT
} TimeTruthSource;

/**
 * Enumerator that expresses the level of confidence in the estimate
 */
typedef enum {
  /**
   * No confidence, this indicates that no estimate was provided as there is no
   * timing information to go on.
   */
  TIME_TRUTH_STATE_NONE,

  /**
   * Low level of confidence. This means that there were no other source of
   * information that were able to confirm the estimate. For example in a
   * system where there is a local antenna reporting timing information and a
   * correction service, if both sources can't agree on a value, this will be
   * a low confidence value estimate (most likely the value will be coming
   * from one of the two sources).
   *
   * NOTE that in a system where there are no other possible sources (ie. only
   * has a local antenna source and no correction service) than the system
   * doesn't consider this a BAD estimate since there was never any way to
   * cross validate the estimate againsted another source.
   *
   * @see #TIME_TRUTH_STATE_BEST
   */
  TIME_TRUTH_STATE_BAD,

  /**
   * Confidence level between #TIME_TRUTH_STATE_BAD and #TIME_TRUTH_STATE_BEST.
   * This means that there was some timing information that was able to confirm
   * that the estimate was correct, but that timing information came from the
   * same source in a system where there is timing information available from
   * another sources.
   */
  TIME_TRUTH_STATE_GOOD,

  /**
   * Best possible level of confidence. This means that either:
   *
   *  1. there was some timing information that was able to confirm that the
   *     estimate was correct, and these information were from different
   *     sources
   *  2. there was some timing information that was able to confirm that the
   *     estimate was correct, and these information were from sames sources
   *     in a system where there were no other sources available
   *  3. there was only one timing information available in a system where
   *     there were no other sources
   */
  TIME_TRUTH_STATE_BEST
} TimeTruthState;

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace gnss_converters {

/**
 * Internal state object for #ObservationTimeEstimator
 */
struct ObservationTimeEstimatorState final {
  bool has_data_ = false;
  uint32_t latest_tow_;
  size_t latest_tow_mismatch_count_;
};

/**
 * Internal state object for #EphemerisTimeEstimator
 */
struct EphemerisTimeEstimatorState final {  // NOLINT
  /**
   * Total number of satellites which the time estimator tracks, should equate
   * to the sum of all the satellites that we track.
   */
  static constexpr size_t kSatellites =
      NUM_SATS_GPS + NUM_SATS_GAL + NUM_SATS_BDS;

  /**
   * Less than comparator for a std::pair<uint16_t, uint64_t> first's element.
   * This is used by "*_entries_" to organize the elements according to the
   * first element.
   *
   * @param left left operand
   * @param right right operand
   * @return if the left operand's first element is less than the right
   * operand's first element
   */
  static bool entries_key_compare(const std::pair<uint16_t, uint64_t> &left,
                                  const std::pair<uint16_t, uint64_t> &right) {
    return left.first < right.first;
  }

  size_t gps_entries_count_ = 0;
  size_t gal_entries_count_ = 0;
  size_t bds_entries_count_ = 0;

  std::pair<uint16_t, uint64_t> gps_entries_[NUM_SATS_GPS];
  std::pair<uint16_t, uint64_t> gal_entries_[NUM_SATS_GAL];
  std::pair<uint16_t, uint64_t> bds_entries_[NUM_SATS_BDS];
};

/**
 * Internal state object for #Rtcm1013TimeEstimator
 */
struct Rtcm1013TimeEstimatorState final {
  bool has_data_ = false;
  gps_time_t gps_time_;
  int8_t leap_seconds_;
};

/**
 * Internal state object for #UbxLeapTimeEstimator
 */
struct UbxLeapTimeEstimatorState final {
  bool has_data_ = false;
  gps_time_t gps_time_;
  int8_t leap_seconds_;
};

/**
 * Base class for all the "time estimator" types.
 *
 * Class offers derived classes a way to publically publish and retrieve state
 * information from time estimators. Since each derived class will have a
 * concept of an internal state which is not thread save, this class offers a
 * way to share that information in a thread safe manner through the
 * "save_state" (publish state) and "load_state" (retrieve state) function.
 *
 * @tparam StateType the derived class's internal state type
 */
template <typename StateType>
class TimeEstimator {
 public:
  /**
   * Derived class's internal state type
   */
  using State = StateType;

 public:
  /**
   * Allows caller to obtain a copy of the time estimator's latest state
   * information.
   *
   * NOTE: function may not guarantee that the latest state information is
   * obtainable all the time, it is largely determined by the implementations of
   * the thread safety mechanism used.
   *
   * @param state pointer to the state object to save the information onto
   * @return true if the function was able o obtain a copy of the latest
   * published state information, false otherwise
   */
  bool load_state(State *state) const {
    if (state_flag_.test_and_set()) {
      return false;
    }

    *state = state_;
    state_flag_.clear();

    return true;
  }

  /**
   * Resets the time estimator with a default state value;
   */
  virtual void reset() = 0;

 protected:
  /**
   * Offers derived classes the means to publish their internal state
   * information for others to obtain.
   *
   * NOTE: function may not guarantee that the state information is saved all
   * the time, it is largely determined by the implementations of the thread
   * safety mechanism used.
   *
   * @param state pointer to the state object to publish
   * @return true if the function was able o save the passed in state object for
   * public distribution
   */
  bool save_state(const State &state) {
    if (state_flag_.test_and_set()) {
      return false;
    }

    state_ = state;
    state_flag_.clear();

    return true;
  }

 private:
  mutable std::atomic_flag state_flag_ = ATOMIC_FLAG_INIT;
  State state_;
};

/**
 * GNSS observation time estimator
 *
 * Class will be responsible for providing callers the most likely TOW
 * (millisecond) value based on the timing information coming through a single
 * source stream.
 */
class ObservationTimeEstimator final
    : public TimeEstimator<ObservationTimeEstimatorState> {
 public:
  /**
   * Provide the estimator with the latest TOW (millisecond) value which has
   * been streamed from the GNSS source.
   *
   * @param tow_ms current GPS time of week expressed in milliseconds
   */
  void push(uint32_t tow_ms);

  /**
   * Obtain the most likely TOW (millisecond) estimate for the GNSS source.
   *
   * @param state copy of the class's latest state information, value is
   * obtainable through #TimeEstimator::load_state
   * @param tow_ms pointer to the object where to save the TOW (millisecond)
   * data
   * @return true of the function was able to provide caller with an estimate,
   * false other.
   */
  static bool get_estimate(const State &state, uint32_t *tow_ms);

  void reset() override;

 private:
  State state_;
};

/**
 * GNSS ephemeris time estimator
 *
 * Class will be responsible for providing callers the most likely GPS WN/TOW
 * value based on the timing information coming through a single source stream.
 */
class EphemerisTimeEstimator final
    : public TimeEstimator<EphemerisTimeEstimatorState> {
 public:
  /**
   * Provide the estimator with the latest GPS WN/TOW value which has been
   * streamed from the GNSS source. The function will need to know from which
   * signal this timing information came from.
   *
   * @param gnss_signal gnss signal from which this timing information
   * originated from
   * @param gps_time current GPS WN/TOW reported for the GNSS signal
   */
  void push(gnss_signal_t gnss_signal, gps_time_t gps_time);

  /**
   * Obtain the most likely WN/TOW estimate for the GNSS source.
   *
   * @param state copy of the class's latest state information, value is
   * obtainable through #TimeEstimator::load_state
   * @param gps_time pointer to the object to save the WN/TOW
   * @param tow_ms the current best estimate for the TOW (millisecond) value,
   * the function uses this to identify the most likely "gps_time"
   * @return true of the function was able to provide caller with an estimate,
   * false otherwise
   */
  static bool get_estimate(const State &state,
                           gps_time_t *gps_time,
                           uint32_t tow_ms);

  void reset() override;

 private:
  State state_;
};

/**
 * RTCM 1013 (System Parameter) time estimator
 */
class Rtcm1013TimeEstimator final
    : public TimeEstimator<Rtcm1013TimeEstimatorState> {
 public:
  /**
   * Provide the estimator with the latest WN/TOW and leap second values which
   * has been streamed through the GNSS source.
   *
   * @param gps_time current WN/TOW values reported by the stream
   * @param leap_seconds current leap second value reported by the stream
   */
  void push(gps_time_t gps_time, int8_t leap_seconds);

  /**
   * Obtain the most likely WN/TOW and leap second (GPS->UTC) estimate for the
   * GNSS source.
   *
   * @param state copy of the class's latest state information, value is
   * obtainable through #TimeEstimator::load_state
   * @param gps_time pointer to the object to save the WN/TOW
   * @param leap_seconds pointer to the object to save the leap second
   * information
   * @param tow_ms the current best estimate for the TOW (millisecond) value,
   * the function uses this to validated integrety of data
   * @return true of the function was able to provide caller with an estimate,
   * false
   */
  static bool get_estimate(const State &state,
                           gps_time_t *gps_time,
                           int8_t *leap_seconds,
                           uint32_t tow_ms);

  void reset() override;

 private:
  State state_;
};

/**
 * UBX-NAV-TIMELS time estimator
 */
class UbxLeapTimeEstimator final
    : public TimeEstimator<UbxLeapTimeEstimatorState> {
 public:
  /**
   * Provide the estimator with the latest WN/TOW and leap second values which
   * has been streamed through the GNSS source.
   *
   * @param gps_time current WN/TOW values reported by the stream
   * @param leap_seconds current leap second value reported by the stream
   */
  void push(gps_time_t gps_time, int8_t leap_seconds);

  /**
   * Obtain the most likely WN/TOW and leap second (GPS->UTC) estimate for the
   * GNSS source.
   *
   * @param state copy of the class's latest state information, value is
   * obtainable through #TimeEstimator::load_state
   * @param gps_time pointer to the object to save the WN/TOW
   * @param leap_seconds pointer to the object to save the leap second
   * information
   * @param tow_ms the current best estimate for the TOW (millisecond) value,
   * the function uses this to validated integrety of data
   * @return true of the function was able to provide caller with an estimate,
   * false
   */
  static bool get_estimate(const State &state,
                           gps_time_t *gps_time,
                           int8_t *leap_seconds,
                           uint32_t tow_ms);

  void reset() override;

 private:
  State state_;
};

struct TimeTruthCache;

/**
 * Central hub where all system timing information is published and queryable.
 *
 * The class houses a pool of time estimator classes which anyone can request
 * for where they can publish timing information for any single stream of data.
 * Users can query for the latest timing information in a thread safe manner
 * through this class.
 *
 * By design, this class is intended (but not limited to) being used as a
 * singleton.
 */
class TimeTruth final {
 public:
  TimeTruth() = default;

  TimeTruth(const TimeTruth &other) = delete;
  TimeTruth(TimeTruth &&other) = delete;

  TimeTruth &operator=(const TimeTruth &other) = delete;
  TimeTruth &operator=(TimeTruth &&other) = delete;

 public:
  /**
   * Requests an ObservationTimeEstimator instance from the estimator pool for a
   * specific source.
   *
   * @param source source where the estimator is collecting information from
   * @param estimator reference to the pointer where the estimator will be save
   * to
   * @return true if class was able to allocate an estimator from its pool,
   * otherwise false
   */
  bool request_estimator(TimeTruthSource source,
                         ObservationTimeEstimator **estimator);

  /**
   * Requests an EphemerisTimeEstimator instance from the estimator pool for a
   * specific source.
   *
   * @param source source where the estimator is collecting information from
   * @param estimator reference to the pointer where the estimator will be save
   * to
   * @return true if class was able to allocate an estimator from its pool,
   * otherwise false
   */
  bool request_estimator(TimeTruthSource source,
                         EphemerisTimeEstimator **estimator);

  /**
   * Requests an Rtcm1013TimeEstimator instance from the estimator pool for a
   * specific source.
   *
   * @param source source where the estimator is collecting information from
   * @param estimator reference to the pointer where the estimator will be save
   * to
   * @return true if class was able to allocate an estimator from its pool,
   * otherwise false
   */
  bool request_estimator(TimeTruthSource source,
                         Rtcm1013TimeEstimator **estimator);

  /**
   * Requests an UbxLeapTimeEstimator instance from the estimator pool for a
   * specific source.
   *
   * @param source source where the estimator is collecting information from
   * @param estimator reference to the pointer where the estimator will be save
   * to
   * @return true if class was able to allocate an estimator from its pool,
   * otherwise false
   */
  bool request_estimator(TimeTruthSource source,
                         UbxLeapTimeEstimator **estimator);

  /**
   * Give the caller the best estimate given all the timing information that all
   * elements in the system have provided.
   *
   * @param wn best estimate of the current GPS week number
   * @param wn_state level of confidence of our GPS week number estimate
   * @param tow_ms best estimate of the current TOW (milliseconds)
   * @param tow_ms_state level of confidence of our TOW (millisecond) estimate
   * @param leap_seconds best estimate of the current leap second (GPS->UTC)
   * @param leap_seconds_state level of confidence of our leap seconds estimate
   * @param cache pointer to the cache object to use (value can be nullptr)
   */
  void get_latest_time(uint16_t *wn,
                       TimeTruthState *wn_state,
                       uint32_t *tow_ms,
                       TimeTruthState *tow_ms_state,
                       int8_t *leap_seconds,
                       TimeTruthState *leap_seconds_state,
                       TimeTruthCache *cache = nullptr);

  /**
   * Resets the time truth to its default state
   */
  void reset();

 private:
  inline ObservationTimeEstimator (
      &observation_estimator_pool())[TIME_TRUTH_SOURCE_COUNT] {
    return *static_cast<ObservationTimeEstimator(*)[TIME_TRUTH_SOURCE_COUNT]>(
        static_cast<void *>(&observation_estimator_pool_));
  }

  inline EphemerisTimeEstimator (
      &ephemeris_estimator_pool())[TIME_TRUTH_SOURCE_COUNT] {
    return *static_cast<EphemerisTimeEstimator(*)[TIME_TRUTH_SOURCE_COUNT]>(
        static_cast<void *>(&ephemeris_estimator_pool_));
  }

  inline Rtcm1013TimeEstimator (
      &rtcm_1013_estimator_pool())[TIME_TRUTH_SOURCE_COUNT] {
    return *static_cast<Rtcm1013TimeEstimator(*)[TIME_TRUTH_SOURCE_COUNT]>(
        static_cast<void *>(&rtcm_1013_estimator_pool_));
  }

  inline UbxLeapTimeEstimator (
      &ubx_leap_estimator_pool())[TIME_TRUTH_SOURCE_COUNT] {
    return *static_cast<UbxLeapTimeEstimator(*)[TIME_TRUTH_SOURCE_COUNT]>(
        static_cast<void *>(&ubx_leap_estimator_pool_));
  }

 private:
  bool observation_estimator_pool_used_[TIME_TRUTH_SOURCE_COUNT]{};
  bool ephemeris_estimator_pool_used_[TIME_TRUTH_SOURCE_COUNT]{};
  bool rtcm_1013_estimator_pool_used_[TIME_TRUTH_SOURCE_COUNT]{};
  bool ubx_leap_estimator_pool_used_[TIME_TRUTH_SOURCE_COUNT]{};

  std::aligned_storage<sizeof(EphemerisTimeEstimator),
                       alignof(ObservationTimeEstimator)>::type
      observation_estimator_pool_[TIME_TRUTH_SOURCE_COUNT];

  std::aligned_storage<sizeof(EphemerisTimeEstimator),
                       alignof(EphemerisTimeEstimator)>::type
      ephemeris_estimator_pool_[TIME_TRUTH_SOURCE_COUNT];

  std::aligned_storage<sizeof(Rtcm1013TimeEstimator),
                       alignof(Rtcm1013TimeEstimator)>::type
      rtcm_1013_estimator_pool_[TIME_TRUTH_SOURCE_COUNT];

  std::aligned_storage<sizeof(UbxLeapTimeEstimator),
                       alignof(UbxLeapTimeEstimator)>::type
      ubx_leap_estimator_pool_[TIME_TRUTH_SOURCE_COUNT];
};

/**
 * Struct is used by TimeTruth to store the cached state information for each of
 * its estimators.
 */
struct TimeTruthCache final {
  TimeTruthCache() = default;

  TimeTruthCache(const TimeTruthCache &) = delete;
  TimeTruthCache(TimeTruthCache &&) = delete;

  TimeTruthCache &operator=(const TimeTruthCache &other) = delete;
  TimeTruthCache &operator=(TimeTruthCache &&other) = delete;

  /**
   * Will reset the cache data
   */
  void reset();

  bool observation_estimator_states_available_[TIME_TRUTH_SOURCE_COUNT]{};
  bool ephemeris_estimator_states_available_[TIME_TRUTH_SOURCE_COUNT]{};
  bool rtcm_1013_estimator_states_available_[TIME_TRUTH_SOURCE_COUNT]{};
  bool ubx_leap_estimator_states_available_[TIME_TRUTH_SOURCE_COUNT]{};

  ObservationTimeEstimator::State
      observation_estimator_states_[TIME_TRUTH_SOURCE_COUNT];
  EphemerisTimeEstimator::State
      ephemeris_estimator_states_[TIME_TRUTH_SOURCE_COUNT];
  Rtcm1013TimeEstimator::State
      rtcm_1013_estimator_states_[TIME_TRUTH_SOURCE_COUNT];
  UbxLeapTimeEstimator::State
      ubx_leap_estimator_states_[TIME_TRUTH_SOURCE_COUNT];
};

}  // namespace gnss_converters

using TimeTruth = gnss_converters::TimeTruth;
using TimeTruthCache = gnss_converters::TimeTruthCache;
using ObservationTimeEstimator = gnss_converters::ObservationTimeEstimator;
using EphemerisTimeEstimator = gnss_converters::EphemerisTimeEstimator;
using Rtcm1013TimeEstimator = gnss_converters::Rtcm1013TimeEstimator;
using UbxLeapTimeEstimator = gnss_converters::UbxLeapTimeEstimator;

#else

typedef struct TimeTruth TimeTruth;
typedef struct TimeTruthCache TimeTruthCache;
typedef struct ObservationTimeEstimator ObservationTimeEstimator;
typedef struct EphemerisTimeEstimator EphemerisTimeEstimator;
typedef struct Rtcm1013TimeEstimator Rtcm1013TimeEstimator;
typedef struct UbxLeapTimeEstimator UbxLeapTimeEstimator;

#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * This is a C binding for TimeTruth::request_estimator(TimeTruthSource,
 * ObservationTimeEstimator**).
 */
bool time_truth_request_observation_time_estimator(
    TimeTruth *time_truth,
    TimeTruthSource source,
    ObservationTimeEstimator **estimator);

/**
 * This is a C binding for TimeTruth::request_estimator(TimeTruthSource,
 * EphemerisTimeEstimator**).
 */
bool time_truth_request_ephemeris_time_estimator(
    TimeTruth *time_truth,
    TimeTruthSource source,
    EphemerisTimeEstimator **estimator);

/**
 * This is a C binding for TimeTruth::request_estimator(TimeTruthSource,
 * Rtcm1013TimeEstimator**).
 */
bool time_truth_request_rtcm_1013_time_estimator(
    TimeTruth *time_truth,
    TimeTruthSource source,
    Rtcm1013TimeEstimator **estimator);

/**
 * This is a C binding for TimeTruth::request_estimator(TimeTruthSource,
 * UbxLeapTimeEstimator**).
 */
bool time_truth_request_ubx_leap_time_estimator(
    TimeTruth *time_truth,
    TimeTruthSource source,
    UbxLeapTimeEstimator **estimator);

/**
 * This is a C binding for ObservationTimeEstimator::push.
 */
void time_truth_observation_estimator_push(ObservationTimeEstimator *estimator,
                                           uint32_t tow_ms);

/**
 * This is a C binding for EphemerisTimeEstimator::push.
 */
void time_truth_ephemeris_estimator_push(EphemerisTimeEstimator *estimator,
                                         gnss_signal_t gnss_signal,
                                         gps_time_t gps_time);

/**
 * This is a C binding for Rtcm1013TimeEstimator::push.
 */
void time_truth_rtcm_1013_estimator_push(Rtcm1013TimeEstimator *estimator,
                                         gps_time_t gps_time,
                                         int8_t leap_seconds);

/**
 * This is a C binding for UbxLeapTimeEstimator::push.
 */
void time_truth_ubx_leap_estimator_push(UbxLeapTimeEstimator *estimator,
                                        gps_time_t gps_time,
                                        int8_t leap_seconds);

/**
 * This is a C binding for TimeTruth::get_latest_time
 */
void time_truth_get_latest_time(TimeTruth *time_truth,
                                uint16_t *wn,
                                TimeTruthState *wn_state,
                                uint32_t *tow_ms,
                                TimeTruthState *tow_ms_state,
                                int8_t *leap_seconds,
                                TimeTruthState *leap_seconds_state,
                                TimeTruthCache *cache);

/**
 * This is a C binding for TimeTruth::reset
 */
void time_truth_reset(TimeTruth *time_truth);

/**
 * This is a C binding for TimeTruthCache::reset
 */
void time_truth_cache_reset(TimeTruthCache *time_truth_cache);

#ifdef __cplusplus
}
#endif

#endif  // GNSS_CONVERTERS_TIME_TRUTH_V2_H
