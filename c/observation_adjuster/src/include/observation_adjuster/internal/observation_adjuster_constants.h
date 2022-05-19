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

#ifndef GNSS_CONVERTERS_OBSERVATION_ADJUSTER_CONSTANTS_H
#define GNSS_CONVERTERS_OBSERVATION_ADJUSTER_CONSTANTS_H

#include <cstddef>

#include <libsbp/v4/observation/MSG_OBS.h>

namespace obs_adjuster {

enum class StreamType { STATION_OBS, STATION_CORR, VRS_CORR };
static constexpr std::size_t STREAM_TYPE_COUNT = 3;

/* This is the maximum number of SBP observations possible per epoch:
   - Max number of observation messages comes from the 4 bits assigned to the
     sequence count in header.n_obs
   - The number of observations per message comes from the max 255 byte
     message length
*/
static constexpr std::size_t SBP_OBS_SIZE(sizeof(sbp_packed_obs_content_t));
static constexpr std::size_t SBP_MAX_OBS_SEQ(15u);
static constexpr std::size_t MAX_OBS_PER_EPOCH(
    SBP_MAX_OBS_SEQ* SBP_MSG_OBS_OBS_MAX);
static constexpr std::size_t OBS_BUFFER_SIZE(MAX_OBS_PER_EPOCH* SBP_OBS_SIZE);

}  // namespace obs_adjuster

#endif  // GNSS_CONVERTERS_OBSERVATION_ADJUSTER_CONSTANTS_H
