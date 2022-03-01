/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.
 * >
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <gnss-converters/internal/static_vector.h>
#include <gnss-converters/internal/time_truth_v2.h>
#include <gnss-converters/time_truth_v2.h>

#include <algorithm>
#include <cstring>

namespace gnss_converters {

QuartileIndices get_quartile_indices(size_t size) {
  assert(size >= 3);

  QuartileIndices quartiles;
  quartiles.lower_quartile.first = (size - 2) / 4;
  quartiles.lower_quartile.second = size / 4;
  quartiles.middle_quartile.first = (size - 1) / 2;
  quartiles.middle_quartile.second = size / 2;
  quartiles.upper_quartile.first += size - 1 - quartiles.lower_quartile.second;
  quartiles.upper_quartile.second += size - 1 - quartiles.lower_quartile.first;

  return quartiles;
}

bool is_tow_ms_valid(uint32_t tow_ms) { return tow_ms < WEEK_SECS * SECS_MS; }

bool is_tow_ms_within_tolerance(uint32_t this_tow_ms,
                                uint32_t other_tow_ms,
                                uint32_t tow_ms_tolerance) {
  if (std::max(other_tow_ms, this_tow_ms) -
          std::min(other_tow_ms, this_tow_ms) <=
      tow_ms_tolerance) {
    return true;
  }

  if (this_tow_ms + tow_ms_tolerance >= WEEK_SECS * SECS_MS) {
    return other_tow_ms <=
           (this_tow_ms + tow_ms_tolerance) % (WEEK_SECS * SECS_MS);
  }

  if (this_tow_ms < tow_ms_tolerance) {
    return other_tow_ms >=
           (WEEK_SECS * SECS_MS + this_tow_ms - tow_ms_tolerance) %
               (WEEK_SECS * SECS_MS);
  }

  return false;
}

bool is_tow_ms_within_tolerance(const uint32_t *tows_ms,
                                size_t tows_ms_count,
                                uint32_t tolerance,
                                uint32_t *latest_tow_ms) {
  if (tows_ms == nullptr || tows_ms_count == 0) {
    return false;
  }

  if (tows_ms_count == 1) {
    return true;
  }

  uint32_t minimum_tow_ms = WEEK_SECS * SECS_MS;
  uint32_t maximum_tow_ms = 0;

  for (size_t i = 0; i < tows_ms_count; ++i) {
    minimum_tow_ms = std::min(minimum_tow_ms, tows_ms[i]);
    maximum_tow_ms = std::max(maximum_tow_ms, tows_ms[i]);
  }

  if (maximum_tow_ms - minimum_tow_ms <= tolerance) {
    *latest_tow_ms = maximum_tow_ms;
    return true;
  }

  uint32_t minimum_upper_tolerance = WEEK_SECS * SECS_MS;
  uint32_t maximum_lower_tolerance = 0;

  for (size_t i = 0; i < tows_ms_count; ++i) {
    if (tows_ms[i] >= WEEK_SECS * SECS_MS - tolerance) {
      minimum_upper_tolerance = std::min(minimum_upper_tolerance, tows_ms[i]);
    } else if (tows_ms[i] < tolerance) {
      maximum_lower_tolerance = std::max(maximum_lower_tolerance, tows_ms[i]);
    } else {
      return false;
    }
  }

  bool ret = is_tow_ms_within_tolerance(
      minimum_upper_tolerance, maximum_lower_tolerance, tolerance);

  if (ret) {
    *latest_tow_ms = maximum_lower_tolerance;
  }

  return ret;
}

bool match_gps_time_with_tow_ms(gps_time_t *gps_time,
                                uint32_t tow_ms,
                                uint32_t tolerance_ms) {
  if (!is_tow_ms_within_tolerance(
          gps_time->tow * SECS_MS, tow_ms, tolerance_ms)) {
    return false;
  }

  gps_time_t base_time = *gps_time;

  gps_time->tow = static_cast<double>(tow_ms) / SECS_MS;
  gps_time->wn = base_time.wn;

  const double timediff = gpsdifftime(gps_time, &base_time);

  if (timediff < -WEEK_SECS / 2.0 && base_time.wn != INT16_MAX) {
    gps_time->wn = base_time.wn + 1;
  } else if (timediff > WEEK_SECS / 2.0 && base_time.wn != 0) {
    gps_time->wn = base_time.wn - 1;
  }

  return true;
}

void calculate_tow_ms(
    const StaticVector<std::pair<uint32_t, TimeTruthSource>,
                       kTotalTowMsEstimationPerSource * TIME_TRUTH_SOURCE_COUNT>
        &estimates,
    uint32_t tolerance,
    uint32_t *tow_ms,
    TimeTruthState *state) {
  static constexpr size_t N =
      kTotalTowMsEstimationPerSource * TIME_TRUTH_SOURCE_COUNT;

  *state = TIME_TRUTH_STATE_NONE;

  MatchCounter match_counter[N] = {};

  for (size_t i = 0; i < estimates.size(); ++i) {
    for (size_t j = 0; j < estimates.size(); ++j) {
      if (i == j) {
        continue;
      }
      if (is_tow_ms_within_tolerance(
              estimates[i].first, estimates[j].first, tolerance)) {
        if (estimates[i].second == estimates[j].second) {
          ++(match_counter[i].same_source_count);
        } else {
          ++(match_counter[i].cross_source_count);
        }
      }
    }
  }

  StaticVector<uint32_t, N> candidates;

  size_t local_estimates = 0;
  size_t remote_estimates = 0;
  size_t cross_source_matches = 0;

  for (size_t i = 0; i < estimates.size(); ++i) {
    const bool is_local_estimate =
        estimates[i].second == TIME_TRUTH_SOURCE_LOCAL;
    const bool is_remote_estimate =
        estimates[i].second == TIME_TRUTH_SOURCE_REMOTE;

    local_estimates += static_cast<size_t>(is_local_estimate);
    remote_estimates += static_cast<size_t>(is_remote_estimate);
    cross_source_matches +=
        static_cast<size_t>(match_counter[i].cross_source_count > 0);
  }

  do {
    size_t max_matches = 0;

    if (cross_source_matches > 0) {
      for (size_t i = 0; i < estimates.size(); ++i) {
        size_t matches = match_counter[i].cross_source_count;

        if (matches < max_matches) {
          continue;
        }

        if (matches > max_matches) {
          max_matches = matches;
          candidates.clear();
        }

        candidates.emplace_back(estimates[i].first);
      }

      if (is_tow_ms_within_tolerance(
              candidates.data(), candidates.size(), tolerance, tow_ms)) {
        *state = TIME_TRUTH_STATE_BEST;
        break;
      }

      candidates.clear();
    }

    if (local_estimates > 0) {
      for (size_t i = 0; i < estimates.size(); ++i) {
        if (estimates[i].second != TIME_TRUTH_SOURCE_LOCAL) {
          continue;
        }

        size_t matches = match_counter[i].same_source_count;

        if (matches < max_matches) {
          continue;
        }

        if (matches > max_matches) {
          max_matches = matches;
          candidates.clear();
        }

        candidates.emplace_back(estimates[i].first);
      }

      if (max_matches > 0 &&
          is_tow_ms_within_tolerance(
              candidates.data(), candidates.size(), tolerance, tow_ms)) {
        *state = remote_estimates > 0 ? TIME_TRUTH_STATE_GOOD
                                      : TIME_TRUTH_STATE_BEST;
        break;
      }

      if (candidates.size() == 1 && *state == TIME_TRUTH_STATE_NONE) {
        *tow_ms = candidates[0];
        *state =
            remote_estimates > 0 ? TIME_TRUTH_STATE_BAD : TIME_TRUTH_STATE_BEST;
      }

      candidates.clear();
    }

    if (remote_estimates > 0) {
      for (size_t i = 0; i < estimates.size(); ++i) {
        if (estimates[i].second != TIME_TRUTH_SOURCE_REMOTE) {
          continue;
        }

        size_t matches = match_counter[i].same_source_count;

        if (matches < max_matches) {
          continue;
        }

        if (matches > max_matches) {
          max_matches = matches;
          candidates.clear();
        }

        candidates.emplace_back(estimates[i].first);
      }

      if (max_matches > 0 &&
          is_tow_ms_within_tolerance(
              candidates.data(), candidates.size(), tolerance, tow_ms)) {
        *state =
            local_estimates > 0 ? TIME_TRUTH_STATE_GOOD : TIME_TRUTH_STATE_BEST;
        break;
      }

      if (candidates.size() == 1 && *state == TIME_TRUTH_STATE_NONE) {
        *tow_ms = candidates[0];
        *state =
            local_estimates > 0 ? TIME_TRUTH_STATE_BAD : TIME_TRUTH_STATE_BEST;
      }

      candidates.clear();
    }
  } while (false);
}

void calculate_leap_seconds(
    const StaticVector<std::pair<int8_t, TimeTruthSource>,
                       kTotalLeapSecondEstimationPerSource *
                           TIME_TRUTH_SOURCE_COUNT> &estimates,
    int8_t *leap_seconds,
    TimeTruthState *state) {
  static constexpr size_t N =
      kTotalLeapSecondEstimationPerSource * TIME_TRUTH_SOURCE_COUNT;

  const auto all_leap_seconds_same = [](int8_t *values, size_t count) -> bool {
    if (count == 0) {
      return false;
    }

    if (count == 1) {
      return true;
    }

    int8_t value = values[0];
    return std::all_of(values + 1, values + count, [value](int8_t elem) {
      return elem == value;
    });
  };

  *state = TIME_TRUTH_STATE_NONE;

  MatchCounter match_counter[N] = {};

  for (size_t i = 0; i < estimates.size(); ++i) {
    for (size_t j = 0; j < estimates.size(); ++j) {
      if (i == j) {
        continue;
      }
      if (estimates[i].first == estimates[j].first) {
        if (estimates[i].second == estimates[j].second) {
          ++(match_counter[i].same_source_count);
        } else {
          ++(match_counter[i].cross_source_count);
        }
      }
    }
  }

  StaticVector<int8_t, N> candidates;

  size_t local_estimates = 0;
  size_t remote_estimates = 0;
  size_t cross_source_matches = 0;

  for (size_t i = 0; i < estimates.size(); ++i) {
    const bool is_local_estimate =
        estimates[i].second == TIME_TRUTH_SOURCE_LOCAL;
    const bool is_remote_estimate =
        estimates[i].second == TIME_TRUTH_SOURCE_REMOTE;

    local_estimates += static_cast<size_t>(is_local_estimate);
    remote_estimates += static_cast<size_t>(is_remote_estimate);
    cross_source_matches +=
        static_cast<size_t>(match_counter[i].cross_source_count > 0);
  }

  do {
    size_t max_matches = 0;

    if (cross_source_matches > 0) {
      for (size_t i = 0; i < estimates.size(); ++i) {
        size_t matches = match_counter[i].cross_source_count;

        if (matches < max_matches) {
          continue;
        }

        if (matches > max_matches) {
          max_matches = matches;
          candidates.clear();
        }

        candidates.emplace_back(estimates[i].first);
      }

      if (all_leap_seconds_same(candidates.data(), candidates.size())) {
        *leap_seconds = candidates[0];
        *state = TIME_TRUTH_STATE_BEST;
        break;
      }

      candidates.clear();
    }

    if (remote_estimates > 0) {
      for (size_t i = 0; i < estimates.size(); ++i) {
        if (estimates[i].second != TIME_TRUTH_SOURCE_REMOTE) {
          continue;
        }

        size_t matches = match_counter[i].same_source_count;

        if (matches < max_matches) {
          continue;
        }

        if (matches > max_matches) {
          max_matches = matches;
          candidates.clear();
        }

        candidates.emplace_back(estimates[i].first);
      }

      if (max_matches > 0 &&
          all_leap_seconds_same(candidates.data(), candidates.size())) {
        *leap_seconds = candidates[0];
        *state =
            local_estimates > 0 ? TIME_TRUTH_STATE_GOOD : TIME_TRUTH_STATE_BEST;
        break;
      }

      if (candidates.size() == 1 && *state == TIME_TRUTH_STATE_NONE) {
        *leap_seconds = candidates[0];
        *state =
            local_estimates > 0 ? TIME_TRUTH_STATE_BAD : TIME_TRUTH_STATE_BEST;
      }

      candidates.clear();
    }

    if (local_estimates > 0) {
      for (size_t i = 0; i < estimates.size(); ++i) {
        if (estimates[i].second != TIME_TRUTH_SOURCE_LOCAL) {
          continue;
        }

        size_t matches = match_counter[i].same_source_count;

        if (matches < max_matches) {
          continue;
        }

        if (matches > max_matches) {
          max_matches = matches;
          candidates.clear();
        }

        candidates.emplace_back(estimates[i].first);
      }

      if (max_matches > 0 &&
          all_leap_seconds_same(candidates.data(), candidates.size())) {
        *leap_seconds = candidates[0];
        *state = remote_estimates > 0 ? TIME_TRUTH_STATE_GOOD
                                      : TIME_TRUTH_STATE_BEST;
        break;
      }

      if (candidates.size() == 1 && *state == TIME_TRUTH_STATE_NONE) {
        *leap_seconds = candidates[0];
        *state =
            remote_estimates > 0 ? TIME_TRUTH_STATE_BAD : TIME_TRUTH_STATE_BEST;
      }

      candidates.clear();
    }
  } while (false);
}

void calculate_week_number(
    const StaticVector<std::pair<uint16_t, TimeTruthSource>,
                       kTotalWeekNumberEstimationPerSource *
                           TIME_TRUTH_SOURCE_COUNT> &estimates,
    uint16_t *wn,
    TimeTruthState *state) {
  static constexpr size_t N =
      kTotalWeekNumberEstimationPerSource * TIME_TRUTH_SOURCE_COUNT;

  const auto all_week_number_same = [](uint16_t *values, size_t count) -> bool {
    if (count == 0) {
      return false;
    }

    if (count == 1) {
      return true;
    }

    uint16_t value = values[0];
    return std::all_of(values + 1, values + count, [value](uint16_t elem) {
      return elem == value;
    });
  };

  *state = TIME_TRUTH_STATE_NONE;

  MatchCounter match_counter[N] = {};

  for (size_t i = 0; i < estimates.size(); ++i) {
    for (size_t j = 0; j < estimates.size(); ++j) {
      if (i == j) {
        continue;
      }
      if (estimates[i].first == estimates[j].first) {
        if (estimates[i].second == estimates[j].second) {
          ++(match_counter[i].same_source_count);
        } else {
          ++(match_counter[i].cross_source_count);
        }
      }
    }
  }

  StaticVector<uint16_t, N> candidates;

  size_t local_estimates = 0;
  size_t remote_estimates = 0;
  size_t cross_source_matches = 0;

  for (size_t i = 0; i < estimates.size(); ++i) {
    const bool is_local_estimate =
        estimates[i].second == TIME_TRUTH_SOURCE_LOCAL;
    const bool is_remote_estimate =
        estimates[i].second == TIME_TRUTH_SOURCE_REMOTE;

    local_estimates += static_cast<size_t>(is_local_estimate);
    remote_estimates += static_cast<size_t>(is_remote_estimate);
    cross_source_matches +=
        static_cast<size_t>(match_counter[i].cross_source_count > 0);
  }

  do {
    size_t max_matches = 0;

    if (cross_source_matches > 0) {
      for (size_t i = 0; i < estimates.size(); ++i) {
        size_t matches = match_counter[i].cross_source_count;

        if (matches < max_matches) {
          continue;
        }

        if (matches > max_matches) {
          max_matches = matches;
          candidates.clear();
        }

        candidates.emplace_back(estimates[i].first);
      }

      if (all_week_number_same(candidates.data(), candidates.size())) {
        *wn = candidates[0];
        *state = TIME_TRUTH_STATE_BEST;
        break;
      }

      candidates.clear();
    }

    if (remote_estimates > 0) {
      for (size_t i = 0; i < estimates.size(); ++i) {
        if (estimates[i].second != TIME_TRUTH_SOURCE_REMOTE) {
          continue;
        }

        size_t matches = match_counter[i].same_source_count;

        if (matches < max_matches) {
          continue;
        }

        if (matches > max_matches) {
          max_matches = matches;
          candidates.clear();
        }

        candidates.emplace_back(estimates[i].first);
      }

      if (max_matches > 0 &&
          all_week_number_same(candidates.data(), candidates.size())) {
        *wn = candidates[0];
        *state =
            local_estimates > 0 ? TIME_TRUTH_STATE_GOOD : TIME_TRUTH_STATE_BEST;
        break;
      }

      if (candidates.size() == 1 && *state == TIME_TRUTH_STATE_NONE) {
        *wn = candidates[0];
        *state =
            local_estimates > 0 ? TIME_TRUTH_STATE_BAD : TIME_TRUTH_STATE_BEST;
      }

      candidates.clear();
    }

    if (local_estimates > 0) {
      for (size_t i = 0; i < estimates.size(); ++i) {
        if (estimates[i].second != TIME_TRUTH_SOURCE_LOCAL) {
          continue;
        }

        size_t matches = match_counter[i].same_source_count;

        if (matches < max_matches) {
          continue;
        }

        if (matches > max_matches) {
          max_matches = matches;
          candidates.clear();
        }

        candidates.emplace_back(estimates[i].first);
      }

      if (max_matches > 0 &&
          all_week_number_same(candidates.data(), candidates.size())) {
        *wn = candidates[0];
        *state = remote_estimates > 0 ? TIME_TRUTH_STATE_GOOD
                                      : TIME_TRUTH_STATE_BEST;
        break;
      }

      if (candidates.size() == 1 && *state == TIME_TRUTH_STATE_NONE) {
        *wn = candidates[0];
        *state =
            remote_estimates > 0 ? TIME_TRUTH_STATE_BAD : TIME_TRUTH_STATE_BEST;
      }

      candidates.clear();
    }
  } while (false);
}

void ObservationTimeEstimator::push(uint32_t tow_ms) {
  if (!is_tow_ms_valid(tow_ms)) {
    return;
  }

  do {
    if (!state_.has_data_) {
      state_.has_data_ = true;
      state_.latest_tow_ = tow_ms;
      state_.latest_tow_mismatch_count_ = 0;
      break;
    }

    if (is_tow_ms_within_tolerance(
            state_.latest_tow_,
            tow_ms,
            kObservationEstimatorMaximumTowMsTimeDifference)) {
      state_.latest_tow_ = tow_ms;
      state_.latest_tow_mismatch_count_ = 0;
      break;
    }

    if (state_.latest_tow_mismatch_count_ >=
        kObservationEstimatorMaximumTowMsTimeMismatchCount) {
      state_.latest_tow_ = tow_ms;
      state_.latest_tow_mismatch_count_ = 0;
    } else {
      ++state_.latest_tow_mismatch_count_;
    }
  } while (false);

  save_state(state_);
}

bool ObservationTimeEstimator::get_estimate(const State &state,
                                            uint32_t *tow_ms) {
  if (!state.has_data_) {
    return false;
  }

  *tow_ms = state.latest_tow_;
  return true;
}

void ObservationTimeEstimator::reset() {
  state_ = State{};
  save_state(state_);
}

void EphemerisTimeEstimator::push(gnss_signal_t gnss_signal,
                                  gps_time_t gps_time) {
  if (!gps_time_valid(&gps_time)) {
    return;
  }

  const constellation_t constellation = sid_to_constellation(gnss_signal);

  std::pair<uint16_t, uint64_t> *entries;
  size_t *entries_count;
  size_t entries_capacity;

  switch (constellation) {
    case CONSTELLATION_BDS:
      entries = state_.bds_entries_;
      entries_count = &state_.bds_entries_count_;
      entries_capacity = ARRAY_SIZE(state_.bds_entries_);
      break;
    case CONSTELLATION_GAL:
      entries = state_.gal_entries_;
      entries_count = &state_.gal_entries_count_;
      entries_capacity = ARRAY_SIZE(state_.gal_entries_);
      break;
    case CONSTELLATION_GPS:
      entries = state_.gps_entries_;
      entries_count = &state_.gps_entries_count_;
      entries_capacity = ARRAY_SIZE(state_.gps_entries_);
      break;
    case CONSTELLATION_COUNT:
    case CONSTELLATION_GLO:
    case CONSTELLATION_INVALID:
    case CONSTELLATION_QZS:
    case CONSTELLATION_SBAS:
    default:
      return;
  }

  const uint64_t gps_time_ms =
      (static_cast<uint64_t>(gps_time.wn) * WEEK_SECS * SECS_MS +
       static_cast<uint64_t>(gps_time.tow * SECS_MS));
  const auto entry = std::make_pair(gnss_signal.sat, gps_time_ms);

  auto *begin = entries;
  auto *end = entries + *entries_count;

  auto *itr = std::lower_bound(begin, end, entry, State::entries_key_compare);
  bool found = itr != end && itr->first == gnss_signal.sat;

  if (found) {
    *itr = entry;
    save_state(state_);
    return;
  }

  if (*entries_count >= entries_capacity) {
    return;
  }

  std::copy_backward(itr, end, end + 1);
  *itr = entry;
  ++(*entries_count);
  save_state(state_);
}

bool EphemerisTimeEstimator::get_estimate(const State &state,
                                          gps_time_t *gps_time,
                                          uint32_t tow_ms) {
  struct {
    const std::pair<uint16_t, uint64_t> *entries;
    size_t entries_count;
  } const constellation_data[] = {
      {state.gps_entries_, state.gps_entries_count_},
      {state.gal_entries_, state.gal_entries_count_},
      {state.bds_entries_, state.bds_entries_count_}};

  uint64_t all_entries[State::kSatellites];
  size_t all_entries_count = 0;

  for (const auto &data : constellation_data) {
    if (data.entries_count >= kEphemerisEstimatorMinimumSignalsToQualify) {
      for (size_t i = 0; i < data.entries_count; ++i) {
        if (is_tow_ms_within_tolerance(
                tow_ms,
                data.entries[i].second % (WEEK_SECS * SECS_MS),
                kEphemerisEstimatorMaximumInterQuartileDifferenceMs / 2)) {
          all_entries[all_entries_count++] = data.entries[i].second;
        }
      }
    }
  }

  if (all_entries_count < kEphemerisEstimatorMinimumSignalsToQualify) {
    return false;
  }

  std::sort(all_entries, all_entries + all_entries_count);
  QuartileIndices quartile_indices = get_quartile_indices(all_entries_count);

  uint64_t lower_quartile_ms =
      (all_entries[quartile_indices.lower_quartile.first] +
       all_entries[quartile_indices.lower_quartile.second]) /
      2;
  uint64_t upper_quartile_ms =
      (all_entries[quartile_indices.upper_quartile.first] +
       all_entries[quartile_indices.upper_quartile.second]) /
      2;

  if (upper_quartile_ms - lower_quartile_ms >
      kEphemerisEstimatorMaximumInterQuartileDifferenceMs) {
    return false;
  }

  lower_quartile_ms =
      lower_quartile_ms > kEphemerisEstimatorInterQuartileOffsetMs
          ? lower_quartile_ms - kEphemerisEstimatorInterQuartileOffsetMs
          : 0;
  upper_quartile_ms =
      upper_quartile_ms + kEphemerisEstimatorInterQuartileOffsetMs;

  uint64_t lower_quartile_wn = lower_quartile_ms / (WEEK_SECS * SECS_MS);
  uint64_t upper_quartile_wn = upper_quartile_ms / (WEEK_SECS * SECS_MS);

  uint64_t option_a = lower_quartile_wn * WEEK_SECS * SECS_MS + tow_ms;
  if (option_a >= lower_quartile_ms && option_a <= upper_quartile_ms) {
    gps_time->wn = lower_quartile_wn;
    gps_time->tow = tow_ms / SECS_MS;
    return true;
  }

  uint64_t option_b = upper_quartile_wn * WEEK_SECS * SECS_MS + tow_ms;
  if (option_b >= lower_quartile_ms && option_b <= upper_quartile_ms) {
    gps_time->wn = upper_quartile_wn;
    gps_time->tow = tow_ms / SECS_MS;
    return true;
  }

  return false;
}

void EphemerisTimeEstimator::reset() {
  state_ = State{};
  save_state(state_);
}

void Rtcm1013TimeEstimator::push(gps_time_t gps_time, int8_t leap_seconds) {
  if (!gps_time_valid(&gps_time)) {
    return;
  }

  state_.has_data_ = true;
  state_.gps_time_ = gps_time;
  state_.leap_seconds_ = leap_seconds;
  save_state(state_);
}

bool Rtcm1013TimeEstimator::get_estimate(const State &state,
                                         gps_time_t *gps_time,
                                         int8_t *leap_seconds,
                                         uint32_t tow_ms) {
  if (!state.has_data_) {
    return false;
  }

  if (!is_tow_ms_within_tolerance(state.gps_time_.tow * SECS_MS,
                                  tow_ms,
                                  kRtcm1013EstimatorGpsTimeTowMsTolerance)) {
    return false;
  }

  *gps_time = state.gps_time_;
  *leap_seconds = state.leap_seconds_;
  return true;
}

void Rtcm1013TimeEstimator::reset() {
  state_ = State{};
  save_state(state_);
}

void UbxLeapTimeEstimator::push(gps_time_t gps_time, int8_t leap_seconds) {
  if (!gps_time_valid(&gps_time)) {
    return;
  }

  state_.has_data_ = true;
  state_.gps_time_ = gps_time;
  state_.leap_seconds_ = leap_seconds;
  save_state(state_);
}

bool UbxLeapTimeEstimator::get_estimate(const State &state,
                                        gps_time_t *gps_time,
                                        int8_t *leap_seconds,
                                        uint32_t tow_ms) {
  if (!state.has_data_) {
    return false;
  }

  if (!is_tow_ms_within_tolerance(state.gps_time_.tow * SECS_MS,
                                  tow_ms,
                                  kUbxLeapEstimatorGpsTimeTowMsTolerance)) {
    return false;
  }

  *gps_time = state.gps_time_;
  *leap_seconds = state.leap_seconds_;
  return true;
}

void UbxLeapTimeEstimator::reset() {
  state_ = State{};
  save_state(state_);
}

bool TimeTruth::request_estimator(TimeTruthSource source,
                                  ObservationTimeEstimator **estimator) {
  if (observation_estimator_pool_used_[source]) {
    return false;
  }
  observation_estimator_pool_used_[source] = true;
  new (&observation_estimator_pool_[source]) ObservationTimeEstimator();
  *estimator = &observation_estimator_pool()[source];
  return true;
}

bool TimeTruth::request_estimator(TimeTruthSource source,
                                  EphemerisTimeEstimator **estimator) {
  if (ephemeris_estimator_pool_used_[source]) {
    return false;
  }
  ephemeris_estimator_pool_used_[source] = true;
  new (&ephemeris_estimator_pool_[source]) EphemerisTimeEstimator();
  *estimator = &ephemeris_estimator_pool()[source];
  return true;
}

bool TimeTruth::request_estimator(TimeTruthSource source,
                                  Rtcm1013TimeEstimator **estimator) {
  if (rtcm_1013_estimator_pool_used_[source]) {
    return false;
  }
  rtcm_1013_estimator_pool_used_[source] = true;
  new (&rtcm_1013_estimator_pool_[source]) Rtcm1013TimeEstimator();
  *estimator = &rtcm_1013_estimator_pool()[source];
  return true;
}

bool TimeTruth::request_estimator(TimeTruthSource source,
                                  UbxLeapTimeEstimator **estimator) {
  if (ubx_leap_estimator_pool_used_[source]) {
    return false;
  }
  ubx_leap_estimator_pool_used_[source] = true;
  new (&ubx_leap_estimator_pool_[source]) UbxLeapTimeEstimator();
  *estimator = &ubx_leap_estimator_pool()[source];
  return true;
}

void TimeTruth::get_latest_time(uint16_t *wn,
                                TimeTruthState *wn_state,
                                uint32_t *tow_ms,
                                TimeTruthState *tow_ms_state,
                                int8_t *leap_seconds,
                                TimeTruthState *leap_seconds_state,
                                TimeTruthCache *cache) {
  *wn_state = TIME_TRUTH_STATE_NONE;
  *tow_ms_state = TIME_TRUTH_STATE_NONE;
  *leap_seconds_state = TIME_TRUTH_STATE_NONE;

  bool observation_estimator_states_available[TIME_TRUTH_SOURCE_COUNT]{};
  bool ephemeris_estimator_states_available[TIME_TRUTH_SOURCE_COUNT]{};
  bool rtcm_1013_estimator_states_available[TIME_TRUTH_SOURCE_COUNT]{};
  bool ubx_leap_estimator_states_available[TIME_TRUTH_SOURCE_COUNT]{};

  ObservationTimeEstimator::State
      observation_estimator_states[TIME_TRUTH_SOURCE_COUNT];
  EphemerisTimeEstimator::State
      ephemeris_estimator_states[TIME_TRUTH_SOURCE_COUNT];
  Rtcm1013TimeEstimator::State
      rtcm_1013_estimator_states[TIME_TRUTH_SOURCE_COUNT];
  UbxLeapTimeEstimator::State
      ubx_leap_estimator_states[TIME_TRUTH_SOURCE_COUNT];

  extract_estimator_states(
      observation_estimator_pool_used_,
      observation_estimator_pool(),
      observation_estimator_states_available,
      observation_estimator_states,
      &TimeTruthCache::observation_estimator_states_available_,
      &TimeTruthCache::observation_estimator_states_,
      cache);
  extract_estimator_states(
      ephemeris_estimator_pool_used_,
      ephemeris_estimator_pool(),
      ephemeris_estimator_states_available,
      ephemeris_estimator_states,
      &TimeTruthCache::ephemeris_estimator_states_available_,
      &TimeTruthCache::ephemeris_estimator_states_,
      cache);
  extract_estimator_states(
      rtcm_1013_estimator_pool_used_,
      rtcm_1013_estimator_pool(),
      rtcm_1013_estimator_states_available,
      rtcm_1013_estimator_states,
      &TimeTruthCache::rtcm_1013_estimator_states_available_,
      &TimeTruthCache::rtcm_1013_estimator_states_,
      cache);
  extract_estimator_states(
      ubx_leap_estimator_pool_used_,
      ubx_leap_estimator_pool(),
      ubx_leap_estimator_states_available,
      ubx_leap_estimator_states,
      &TimeTruthCache::ubx_leap_estimator_states_available_,
      &TimeTruthCache::ubx_leap_estimator_states_,
      cache);

  StaticVector<std::pair<uint32_t, TimeTruthSource>,
               kTotalTowMsEstimationPerSource * TIME_TRUTH_SOURCE_COUNT>
      tow_ms_estimates;
  StaticVector<std::pair<int8_t, TimeTruthSource>,
               kTotalLeapSecondEstimationPerSource * TIME_TRUTH_SOURCE_COUNT>
      leap_second_estimates;
  StaticVector<std::pair<gps_time_t, TimeTruthSource>,
               kTotalWeekNumberEstimationPerSource * TIME_TRUTH_SOURCE_COUNT>
      gps_time_estimates;

  for (auto source_index = 0; source_index < TIME_TRUTH_SOURCE_COUNT;
       ++source_index) {
    const TimeTruthSource source = static_cast<TimeTruthSource>(source_index);

    uint32_t tow_ms_estimate;

    if (observation_estimator_states_available[source] &&
        ObservationTimeEstimator::get_estimate(
            observation_estimator_states[source], &tow_ms_estimate)) {
      tow_ms_estimates.emplace_back(tow_ms_estimate, source);
    }
  }

  calculate_tow_ms(
      tow_ms_estimates, kGetLatestTimeMaximumTowMsRange, tow_ms, tow_ms_state);

  if (*tow_ms_state == TIME_TRUTH_STATE_NONE) {
    return;
  }

  for (auto source_index = 0; source_index < TIME_TRUTH_SOURCE_COUNT;
       ++source_index) {
    const TimeTruthSource source = static_cast<TimeTruthSource>(source_index);

    gps_time_t wn_estimate;
    int8_t leap_seconds_estimate;

    if (ephemeris_estimator_states_available[source] &&
        EphemerisTimeEstimator::get_estimate(
            ephemeris_estimator_states[source], &wn_estimate, *tow_ms)) {
      gps_time_estimates.emplace_back(wn_estimate, source);
    }

    if (rtcm_1013_estimator_states_available[source] &&
        Rtcm1013TimeEstimator::get_estimate(rtcm_1013_estimator_states[source],
                                            &wn_estimate,
                                            &leap_seconds_estimate,
                                            *tow_ms)) {
      leap_second_estimates.emplace_back(leap_seconds_estimate, source);
      gps_time_estimates.emplace_back(wn_estimate, source);
    }

    if (ubx_leap_estimator_states_available[source] &&
        UbxLeapTimeEstimator::get_estimate(ubx_leap_estimator_states[source],
                                           &wn_estimate,
                                           &leap_seconds_estimate,
                                           *tow_ms)) {
      leap_second_estimates.emplace_back(leap_seconds_estimate, source);
      gps_time_estimates.emplace_back(wn_estimate, source);
    }
  }

  StaticVector<std::pair<uint16_t, TimeTruthSource>,
               3 * TIME_TRUTH_SOURCE_COUNT>
      wn_estimates;
  for (const auto &gps_time_estimate : gps_time_estimates) {
    gps_time_t gps_time = gps_time_estimate.first;
    TimeTruthSource source = gps_time_estimate.second;

    match_gps_time_with_tow_ms(
        &gps_time, *tow_ms, kGetLatestTimeMaximumEphemerisRange);

    wn_estimates.push_back(
        std::make_pair(static_cast<uint16_t>(gps_time.wn), source));
  }

  calculate_week_number(wn_estimates, wn, wn_state);

  if (*tow_ms_state != TIME_TRUTH_STATE_NONE &&
      *wn_state != TIME_TRUTH_STATE_NONE) {
    gps_time_t leap_second_expiry_time = {
        static_cast<double>(gps_time_utc_leaps_expiry[1]),
        static_cast<int16_t>(gps_time_utc_leaps_expiry[0])};

    gps_time_t gps_time = {static_cast<double>(*tow_ms) / SECS_MS,
                           static_cast<int16_t>(*wn)};

    if (gpsdifftime(&leap_second_expiry_time, &gps_time) > 0) {
      *leap_seconds =
          static_cast<int8_t>(get_gps_utc_offset(&gps_time, nullptr));
      *leap_seconds_state = TIME_TRUTH_STATE_BEST;
    }
  }

  if (*leap_seconds_state == TIME_TRUTH_STATE_NONE) {
    calculate_leap_seconds(
        leap_second_estimates, leap_seconds, leap_seconds_state);
  }
}

void TimeTruth::reset() {
  for (auto source = 0; source < TIME_TRUTH_SOURCE_COUNT; ++source) {
    if (observation_estimator_pool_used_[source]) {
      observation_estimator_pool()[source].~ObservationTimeEstimator();
    }
    if (ephemeris_estimator_pool_used_[source]) {
      ephemeris_estimator_pool()[source].~EphemerisTimeEstimator();
    }
    if (rtcm_1013_estimator_pool_used_[source]) {
      rtcm_1013_estimator_pool()[source].~Rtcm1013TimeEstimator();
    }
    if (ubx_leap_estimator_pool_used_[source]) {
      ubx_leap_estimator_pool()[source].~UbxLeapTimeEstimator();
    }
  }

  memset(observation_estimator_pool_used_,
         0,
         sizeof(observation_estimator_pool_used_));
  memset(ephemeris_estimator_pool_used_,
         0,
         sizeof(ephemeris_estimator_pool_used_));
  memset(rtcm_1013_estimator_pool_used_,
         0,
         sizeof(rtcm_1013_estimator_pool_used_));
  memset(
      ubx_leap_estimator_pool_used_, 0, sizeof(ubx_leap_estimator_pool_used_));
}

void TimeTruthCache::reset() {
  for (auto source = 0; source < TIME_TRUTH_SOURCE_COUNT; ++source) {
    if (observation_estimator_states_available_[source]) {
      observation_estimator_states_[source] = ObservationTimeEstimatorState();
    }
    if (ephemeris_estimator_states_available_[source]) {
      ephemeris_estimator_states_[source] = EphemerisTimeEstimatorState();
    }
    if (rtcm_1013_estimator_states_available_[source]) {
      rtcm_1013_estimator_states_[source] = Rtcm1013TimeEstimatorState();
    }
    if (ubx_leap_estimator_states_available_[source]) {
      ubx_leap_estimator_states_[source] = UbxLeapTimeEstimatorState();
    }
  }

  memset(observation_estimator_states_available_,
         0,
         sizeof(observation_estimator_states_available_));
  memset(ephemeris_estimator_states_available_,
         0,
         sizeof(ephemeris_estimator_states_available_));
  memset(rtcm_1013_estimator_states_available_,
         0,
         sizeof(rtcm_1013_estimator_states_available_));
  memset(ubx_leap_estimator_states_available_,
         0,
         sizeof(ubx_leap_estimator_states_available_));
}
}  // namespace gnss_converters

bool time_truth_request_observation_time_estimator(
    TimeTruth *time_truth,
    TimeTruthSource source,
    ObservationTimeEstimator **estimator) {
  return time_truth->request_estimator(source, estimator);
}

bool time_truth_request_ephemeris_time_estimator(
    TimeTruth *time_truth,
    TimeTruthSource source,
    EphemerisTimeEstimator **estimator) {
  return time_truth->request_estimator(source, estimator);
}

bool time_truth_request_rtcm_1013_time_estimator(
    TimeTruth *time_truth,
    TimeTruthSource source,
    Rtcm1013TimeEstimator **estimator) {
  return time_truth->request_estimator(source, estimator);
}

bool time_truth_request_ubx_leap_time_estimator(
    TimeTruth *time_truth,
    TimeTruthSource source,
    UbxLeapTimeEstimator **estimator) {
  return time_truth->request_estimator(source, estimator);
}

void time_truth_observation_estimator_push(ObservationTimeEstimator *estimator,
                                           uint32_t tow_ms) {
  estimator->push(tow_ms);
}

void time_truth_ephemeris_estimator_push(EphemerisTimeEstimator *estimator,
                                         gnss_signal_t gnss_signal,
                                         gps_time_t gps_time) {
  estimator->push(gnss_signal, gps_time);
}

void time_truth_rtcm_1013_estimator_push(Rtcm1013TimeEstimator *estimator,
                                         gps_time_t gps_time,
                                         int8_t leap_seconds) {
  estimator->push(gps_time, leap_seconds);
}

void time_truth_ubx_leap_estimator_push(UbxLeapTimeEstimator *estimator,
                                        gps_time_t gps_time,
                                        int8_t leap_seconds) {
  estimator->push(gps_time, leap_seconds);
}

void time_truth_get_latest_time(TimeTruth *time_truth,
                                uint16_t *wn,
                                TimeTruthState *wn_state,
                                uint32_t *tow_ms,
                                TimeTruthState *tow_ms_state,
                                int8_t *leap_seconds,
                                TimeTruthState *leap_seconds_state,
                                TimeTruthCache *cache) {
  time_truth->get_latest_time(wn,
                              wn_state,
                              tow_ms,
                              tow_ms_state,
                              leap_seconds,
                              leap_seconds_state,
                              cache);
}

void time_truth_reset(TimeTruth *time_truth) { time_truth->reset(); }

void time_truth_cache_reset(TimeTruthCache *time_truth_cache) {
  time_truth_cache->reset();
}
