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

#ifndef OBSERVATION_ADJUSTER_EPOCH_ADJUSTER_H
#define OBSERVATION_ADJUSTER_EPOCH_ADJUSTER_H

#include <observation_adjuster/observation_adjuster_constants.h>
#include <observation_adjuster/sbp_obs_array.h>

namespace obs_adjuster {

class EpochAdjuster {
 public:
  EpochAdjuster() = default;
  virtual ~EpochAdjuster() = default;

  bool adjust(const SbpObsArray<MAX_OBS_PER_EPOCH> &base_obs,
              const SbpObsArray<MAX_OBS_PER_EPOCH> &base_corr,
              const SbpObsArray<MAX_OBS_PER_EPOCH> &vrs_corr,
              SbpObsArray<MAX_OBS_PER_EPOCH> *adjusted_vrs_obs);
};

}  // namespace obs_adjuster

#endif  // OBSERVATION_ADJUSTER_EPOCH_ADJUSTER_H
