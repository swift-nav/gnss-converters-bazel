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

#include <observation_adjuster/message_matcher.h>
#include <swiftnav/logging.h>

#include <algorithm>
#include <iostream>

namespace obs_adjuster {

MessageMatcher::MessageMatcher(const std::size_t history_size)
    : history_size_(history_size) {}

uint64_t to_milliseconds(const gps_time_t &gps_time) {
  if (!gps_time_valid(&gps_time)) {
    return 0;
  }

  return static_cast<uint64_t>(round(gps_time.tow * SECS_MS)) +
         static_cast<uint64_t>(gps_time.wn) * static_cast<uint64_t>(WEEK_MS);
}

void MessageMatcher::add_obs(
    const obs_adjuster::StreamType stream_type,
    const obs_adjuster::TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> &obs_array) {
  if (!gps_time_valid(&obs_array.timestamp_)) {
    return;
  }

  const uint64_t timestamp_ms = to_milliseconds(obs_array.timestamp_);
  if (sample_map_.count(timestamp_ms) > 0 &&
      sample_map_[timestamp_ms].sample_.count(stream_type) > 0) {
    // Sample already exists, don't overwrite
    log_warn("Added sample at time %" PRIu64
             "more than once, dropping. (for stream type %d)",
             timestamp_ms,
             static_cast<int>(stream_type));
    return;
  }

  MessageMatcherSample &samples = sample_map_[timestamp_ms];
  SbpObsArray<MAX_OBS_PER_EPOCH> &obs_element = samples.sample_[stream_type];
  obs_element = obs_array.obs_;
  enforce_max_size();
}

bool MessageMatcher::find_match(MatchingTriplet *match, gps_time_t *timestamp) {
  if (match == nullptr) {
    return false;
  }

  MessageMatcherSample sample;
  if (find_match(&sample, timestamp)) {
    if (sample.sample_.count(StreamType::STATION_OBS) > 0 &&
        sample.sample_.count(StreamType::STATION_CORR) > 0 &&
        sample.sample_.count(StreamType::VRS_CORR) > 0) {
      match->station_obs_ = sample.sample_.at(StreamType::STATION_OBS);
      match->station_corr_ = sample.sample_.at(StreamType::STATION_CORR);
      match->vrs_corr_ = sample.sample_.at(StreamType::VRS_CORR);
      log_info("Found matching obs at tow: %fs", timestamp->tow);
      return true;
    }
  }

  return false;
}

bool MessageMatcher::find_match(MessageMatcherSample *match,
                                gps_time_t *timestamp) {
  if (match == nullptr || timestamp == nullptr) {
    return false;
  }

  for (std::map<uint64_t, MessageMatcherSample>::const_iterator it =
           sample_map_.begin();
       it != sample_map_.end();
       ++it) {
    const uint64_t ms = it->first;
    const MessageMatcherSample &matcher_sample = it->second;

    if (matcher_sample.sample_.size() == STREAM_TYPE_COUNT) {
      *match = matcher_sample;
      timestamp->wn = static_cast<s16>((ms / SECS_MS) / WEEK_SECS);
      timestamp->tow =
          fmod(ms, WEEK_MS) / static_cast<double>(SECS_MS);  // ms -> seconds

      sample_map_.erase(sample_map_.begin(), ++it);
      return true;
    }
  }

  return false;
}

void MessageMatcher::enforce_max_size() {
  while (sample_map_.size() > history_size_) {
    sample_map_.erase(sample_map_.begin());
  }
}

}  // namespace obs_adjuster
