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

#ifndef GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_MESSAGE_MATCHER_H
#define GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_MESSAGE_MATCHER_H

#include <observation_adjuster/internal/observation_adjuster_constants.h>
#include <observation_adjuster/internal/sbp_obs_array.h>

#include <cstdint>
#include <map>
#include <tuple>

namespace obs_adjuster {

static constexpr std::size_t cMessageMatcherHistorySize = 30;

class MatchingTriplet {
 public:
  MatchingTriplet() = default;
  virtual ~MatchingTriplet() = default;

  SbpObsArray<MAX_OBS_PER_EPOCH> station_obs_;
  SbpObsArray<MAX_OBS_PER_EPOCH> station_corr_;
  SbpObsArray<MAX_OBS_PER_EPOCH> vrs_corr_;
};

class MessageMatcherSample {
 public:
  MessageMatcherSample() = default;
  virtual ~MessageMatcherSample() = default;

  std::map<StreamType, SbpObsArray<MAX_OBS_PER_EPOCH>> sample_;
};

class MessageMatcher {
 public:
  explicit MessageMatcher(
      const std::size_t history_size = cMessageMatcherHistorySize);
  virtual ~MessageMatcher() = default;

  void add_obs(const StreamType stream_type,
               const TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> &obs_array);

  bool find_match(MatchingTriplet *match, gps_time_t *timestamp);

 private:
  void enforce_max_size();
  bool find_match(MessageMatcherSample *match, gps_time_t *timestamp);

  std::size_t history_size_;
  std::map<uint64_t, MessageMatcherSample> sample_map_;
};

}  // namespace obs_adjuster

#endif  // GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_MESSAGE_MATCHER_H
