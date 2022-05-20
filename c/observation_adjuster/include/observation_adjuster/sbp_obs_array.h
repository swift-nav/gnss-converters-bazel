/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OBSERVATION_ADJUSTER_SBP_OBS_ARRAY_H
#define OBSERVATION_ADJUSTER_SBP_OBS_ARRAY_H

#include <libsbp/v4/observation/PackedObsContent.h>
#include <swiftnav/gnss_time.h>

#include <array>

namespace obs_adjuster {

template <std::size_t N>
class SbpObsArray final {
 public:
  SbpObsArray();
  ~SbpObsArray() = default;

  // Add a new entry at the end of the array
  bool add(const sbp_packed_obs_content_t &sbp_obs);
  // Set a sample at a given index, the array must already have a sample at this
  // index
  bool set(const std::size_t index, const sbp_packed_obs_content_t &sbp_obs);
  // Get a sample at the given index
  bool get(const std::size_t index,
           sbp_packed_obs_content_t *sbp_obs_out) const;

  // Erase all elements from the array (size() will be zero)
  void clear();

  // Returns how many elements are present in the array
  size_t size() const;
  // Returns how many elements can maximally fit in the array
  size_t max_size() const;

 private:
  std::array<sbp_packed_obs_content_t, N> arr_;
  size_t num_entries_;
};

template <std::size_t N>
class TimestampedSbpObsArray {
 public:
  TimestampedSbpObsArray();
  virtual ~TimestampedSbpObsArray() = default;

  void reset();

  gps_time_t timestamp_;
  SbpObsArray<N> obs_;
};

template <std::size_t N>
TimestampedSbpObsArray<N>::TimestampedSbpObsArray()
    : timestamp_(GPS_TIME_UNKNOWN) {}

template <std::size_t N>
void TimestampedSbpObsArray<N>::reset() {
  timestamp_ = GPS_TIME_UNKNOWN;
  obs_.clear();
}

template <std::size_t N>
SbpObsArray<N>::SbpObsArray() : num_entries_(0) {}

template <std::size_t N>
bool SbpObsArray<N>::add(const sbp_packed_obs_content_t &sbp_obs) {
  if (num_entries_ < N) {
    arr_[num_entries_] = sbp_obs;
    num_entries_++;
    return true;
  }

  return false;
}

template <std::size_t N>
bool SbpObsArray<N>::set(const std::size_t index,
                         const sbp_packed_obs_content_t &sbp_obs) {
  if (index < num_entries_) {
    arr_[index] = sbp_obs;
    return true;
  }

  return false;
}

template <std::size_t N>
bool SbpObsArray<N>::get(const std::size_t index,
                         sbp_packed_obs_content_t *sbp_obs_out) const {
  if (index < num_entries_ && sbp_obs_out != nullptr) {
    *sbp_obs_out = arr_[index];
    return true;
  }

  return false;
}

template <std::size_t N>
void SbpObsArray<N>::clear() {
  num_entries_ = 0;
  arr_ = std::array<sbp_packed_obs_content_t, N>{};
}

template <std::size_t N>
size_t SbpObsArray<N>::size() const {
  return num_entries_;
}
template <std::size_t N>
size_t SbpObsArray<N>::max_size() const {
  return arr_.max_size();
}

}  // namespace obs_adjuster

#endif  // GNSS_CONVERTERS_EXTRA_OBSERVATION_ADJUSTER_HELPERS_SBP_OBS_ARRAY_H
