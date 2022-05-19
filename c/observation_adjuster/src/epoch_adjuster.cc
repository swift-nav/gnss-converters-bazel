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

#include <observation_adjuster/epoch_adjuster.h>
#include <swiftnav/logging.h>

#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <vector>

namespace obs_adjuster {

static constexpr double cSbpFractionScaling = 256.0;
using SatSigId = uint16_t;

SatSigId sid_to_map_key(const uint8_t sat, const uint8_t code) {
  static_assert(sizeof(SatSigId) == 2, "Map key must be two bytes");
  SatSigId map_key = static_cast<SatSigId>(sat << 8);
  map_key = static_cast<SatSigId>(map_key | code);
  return map_key;
}

SatSigId sid_to_map_key(const sbp_v4_gnss_signal_t& sid) {
  return sid_to_map_key(sid.sat, sid.code);
}

void to_obs_map(const obs_adjuster::SbpObsArray<MAX_OBS_PER_EPOCH>& obs_array,
                std::map<SatSigId, sbp_packed_obs_content_t>* obs_map) {
  if (obs_map == nullptr) {
    return;
  }

  std::vector<SatSigId> duplicate_keys;

  for (std::size_t i = 0; i < obs_array.size(); ++i) {
    sbp_packed_obs_content_t obs_content{};
    if (!obs_array.get(i, &obs_content)) {
      continue;
    }
    const SatSigId map_key = sid_to_map_key(obs_content.sid);
    if (obs_map->count(map_key) < 1) {
      obs_map->operator[](map_key) = obs_content;
    } else {
      duplicate_keys.push_back(map_key);
    }
  }

  for (const SatSigId& dup_key : duplicate_keys) {
    obs_map->erase(dup_key);
  }
}

double unpack_pseudorange(const sbp_packed_obs_content_t& signal) {
  return signal.P * 0.02;
}

double unpack_carrier_phase(const sbp_packed_obs_content_t& signal) {
  return static_cast<double>(signal.L.i) +
         static_cast<double>(signal.L.f) / cSbpFractionScaling;
}

double unpack_doppler(const sbp_packed_obs_content_t& signal) {
  return static_cast<double>(signal.D.i) +
         static_cast<double>(signal.D.f) / cSbpFractionScaling;
}

uint8_t lowest_value(const uint8_t v1, const uint8_t v2, const uint8_t v3) {
  uint8_t lowest = v1;
  if (v2 < lowest) {
    lowest = v2;
  }
  if (v3 < lowest) {
    lowest = v3;
  }
  return lowest;
}

uint8_t combine_flags(const uint8_t v1, const uint8_t v2, const uint8_t v3) {
  uint8_t flags = 0;

  // Each of pseudorange, carrier phase, half cycle ambiguity and doppler
  // need to be valid on all three components to be valid on the combined signal
  std::array<uint8_t, 4> and_flags{{0x01, 0x02, 0x04, 0x08}};

  // RAIM flag on any means the RAIM flag should be set on the combined signal
  std::array<uint8_t, 4> or_flags{{0x80}};

  for (const uint8_t and_flag : and_flags) {
    flags |= ((v1 & and_flag) & (v2 & and_flag) & (v3 & and_flag));
  }
  for (const uint8_t or_flag : or_flags) {
    flags |= ((v1 & or_flag) | (v2 & or_flag) | (v3 & or_flag));
  }

  return flags;
}

sbp_packed_obs_content_t adjust_sample(
    const sbp_packed_obs_content_t& base_obs,
    const sbp_packed_obs_content_t& base_corr,
    const sbp_packed_obs_content_t& vrs_corr) {
  sbp_packed_obs_content_t vrs_adjusted{};
  vrs_adjusted.sid = base_obs.sid;

  // The general formula is: vrs = base_obs + (vrs_corr - base_corr)

  // Pseudorange
  const double final_full_psr =
      unpack_pseudorange(base_obs) +
      (unpack_pseudorange(vrs_corr) - unpack_pseudorange(base_corr));
  vrs_adjusted.P = static_cast<uint32_t>(std::round(final_full_psr / 0.02));

  // Carrier phase
  const double final_full_cp =
      unpack_carrier_phase(base_obs) +
      (unpack_carrier_phase(vrs_corr) - unpack_carrier_phase(base_corr));
  vrs_adjusted.L.i = static_cast<s32>(std::floor(final_full_cp));
  vrs_adjusted.L.f = static_cast<u8>(
      std::round(fmod(final_full_cp, 1.0) * cSbpFractionScaling));

  // Doppler
  const double final_full_doppler =
      unpack_doppler(base_obs) +
      (unpack_doppler(vrs_corr) - unpack_doppler(base_corr));
  vrs_adjusted.D.i = static_cast<s16>(std::floor(final_full_doppler));
  vrs_adjusted.D.f = static_cast<u8>(
      std::round(fmod(final_full_doppler, 1.0) * cSbpFractionScaling));

  // Lock time, take the lowest of the three
  vrs_adjusted.lock =
      lowest_value(base_obs.lock, base_corr.lock, vrs_corr.lock);

  // CN0, take the lowest of the three
  vrs_adjusted.cn0 = lowest_value(base_obs.cn0, base_corr.cn0, vrs_corr.cn0);

  // Flags
  vrs_adjusted.flags =
      combine_flags(base_obs.flags, base_corr.flags, vrs_corr.flags);

  return vrs_adjusted;
}

bool obs_adjuster::EpochAdjuster::adjust(
    const obs_adjuster::SbpObsArray<MAX_OBS_PER_EPOCH>& base_obs,
    const obs_adjuster::SbpObsArray<MAX_OBS_PER_EPOCH>& base_corr,
    const obs_adjuster::SbpObsArray<MAX_OBS_PER_EPOCH>& vrs_corr,
    SbpObsArray<MAX_OBS_PER_EPOCH>* adjusted_vrs_obs) {
  if (adjusted_vrs_obs == nullptr) {
    return false;
  }

  // Store base obs and base corr in maps for quicker lookup later
  std::map<SatSigId, sbp_packed_obs_content_t> base_obs_map;
  to_obs_map(base_obs, &base_obs_map);

  std::map<SatSigId, sbp_packed_obs_content_t> base_corr_map;
  to_obs_map(base_corr, &base_corr_map);

  // For each element in vrs_corr, look for corresponding signals in base
  // and apply the adjustment
  bool something_adjusted = false;
  for (std::size_t i = 0; i < vrs_corr.size(); ++i) {
    sbp_packed_obs_content_t vrs_sample{};
    if (!vrs_corr.get(i, &vrs_sample)) {
      continue;
    }

    const SatSigId map_key = sid_to_map_key(vrs_sample.sid);
    if (base_obs_map.count(map_key) < 1 || base_corr_map.count(map_key) < 1) {
      continue;
    }
    const sbp_packed_obs_content_t base_obs_sample = base_obs_map[map_key];
    const sbp_packed_obs_content_t base_corr_sample = base_corr_map[map_key];

    // We have the same signal for all three sources, do the adjustment
    const sbp_packed_obs_content_t adjusted_vrs =
        adjust_sample(base_obs_sample, base_corr_sample, vrs_sample);
    adjusted_vrs_obs->add(adjusted_vrs);
    something_adjusted = true;
  }

  // Count unique sat codes
  std::set<uint8_t> codes;

  for (std::size_t i = 0; i < adjusted_vrs_obs->size(); i++) {
    sbp_packed_obs_content_t obs{};
    if (adjusted_vrs_obs->get(i, &obs)) {
      codes.insert(obs.sid.code);
    }
  }

  std::stringstream stream;
  for (const uint8_t code : codes) {
    stream << " " << static_cast<uint32_t>(code);
  }

  log_info("Adjusted %zu obs (codes: [ %s ])",
           adjusted_vrs_obs->size(),
           stream.str().c_str());

  return something_adjusted;
}

}  // namespace obs_adjuster
