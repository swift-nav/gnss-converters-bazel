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

#include <common.h>
#include <observation_adjuster/internal/obs_unpacker.h>
#include <swiftnav/gnss_time.h>

code_t to_supported_code_t(const code_t code);

obs_adjuster::ObsUnpacker::ObsUnpacker()
    : sender_id_set_(false), sender_id_(0) {}

double sbp_diff_time(const sbp_v4_gps_time_t& end,
                     const sbp_v4_gps_time_t& begin) {
  const double end_tow_s =
      end.tow / 1000.0 + static_cast<double>(end.ns_residual) / SECS_NS;
  const double begin_tow_s =
      begin.tow / 1000.0 + static_cast<double>(begin.ns_residual) / SECS_NS;

  gps_time_t gps_time_end{end_tow_s, static_cast<int16_t>(end.wn)};
  gps_time_t gps_time_begin{begin_tow_s, static_cast<int16_t>(begin.wn)};

  return gpsdifftime(&gps_time_end, &gps_time_begin);
}

bool obs_adjuster::ObsUnpacker::unpack_obs(const uint16_t sender,
                                           const sbp_msg_obs_t& msg) {
  if (sender_id_set_ && (sender != sender_id_)) {
    send_out_obs();
  }
  sender_id_ = sender;
  sender_id_set_ = true;

  u8 seq_counter = static_cast<u8>((msg.header.n_obs & 0x0F) + 1);
  u8 seq_size = static_cast<u8>(msg.header.n_obs >> 4);

  /* if sbp buffer is not empty, check that this observations belongs to the
   * sequence */
  if (sbp_obs_array_.obs_.size() > 0) {
    double dt = sbp_diff_time(msg.header.t, sbp_header_.t);

    if (dt < 0) {
      printf("Discarding SBP obs with timestamp %.1f seconds in the past\n",
             dt);
      return false;
    }
    if (dt > 0) {
      /* observations belong to the next epoch, send out the current buffer
       * before processing this message */
      printf(
          "SBP obs sequence ended prematurely, starting new one at dt=%.1f\n",
          dt);

      send_out_obs();
    } else {
      u8 current_seq_counter = static_cast<u8>((sbp_header_.n_obs & 0x0F) + 1);
      u8 current_seq_size = static_cast<u8>(sbp_header_.n_obs >> 4);

      if (seq_size != current_seq_size || seq_counter <= current_seq_counter) {
        /* sequence broken, send out current buffer and ignore this message */
        printf(
            "Ignoring SBP obs with invalid obs sequence: expected %d/%d "
            "but got %d/%d\n",
            current_seq_counter + 1,
            current_seq_size,
            seq_counter,
            seq_size);
        send_out_obs();
        return true;
      }
      if (seq_counter != current_seq_counter + 1) {
        /* missed a packet, emit warning but still process this message */
        printf("Missed an SBP obs packet, expected seq %d/%d but got %d/%d\n",
               current_seq_counter + 1,
               current_seq_size,
               seq_counter,
               seq_size);
      }
    }
  }

  /* number of observations in the incoming message */
  u8 n_meas = msg.n_obs;

  // timestamp
  sbp_obs_array_.timestamp_.wn = static_cast<s16>(msg.header.t.wn);
  sbp_obs_array_.timestamp_.tow =
      static_cast<double>(msg.header.t.tow) / 1000.0;

  for (u8 i = 0; i < n_meas; i++) {
    sbp_packed_obs_content_t obs = msg.obs[i];
    if ((0 == (obs.flags & MSG_OBS_FLAGS_CODE_VALID)) ||
        (0 != (obs.flags & MSG_OBS_FLAGS_RAIM_EXCLUSION)) ||
        !code_valid(static_cast<code_t>(obs.sid.code))) {
      /* skip observations that do not have valid code measurement or that are
       * flagged by RAIM */
      continue;
    }

    // Transform the more detailed codes to supported codes
    obs.sid.code =
        static_cast<u8>(to_supported_code_t(static_cast<code_t>(obs.sid.code)));

    /* add incoming sbp observation into sbp array */
    sbp_obs_array_.obs_.add(obs);
    if (sbp_obs_array_.obs_.size() == sbp_obs_array_.obs_.max_size()) {
      printf(
          "Reached max observations per epoch %u, ignoring the remaining %u "
          "obs\n",
          static_cast<uint16_t>(MAX_OBS_PER_EPOCH),
          n_meas - i - 1);
      break;
    }
  }
  sbp_header_ = msg.header;

  /* if sequence is complete, convert into RTCM */
  if (seq_counter == seq_size) {
    send_out_obs();
  }

  return true;
}

void obs_adjuster::ObsUnpacker::set_obs_callback(
    const obs_adjuster::ObsUnpacker::ObsCallbackFn& callback) {
  obs_callback_ = callback;
}

void obs_adjuster::ObsUnpacker::unset_callback() {
  obs_callback_ = ObsCallbackFn();
}

void obs_adjuster::ObsUnpacker::send_out_obs() {
  if (obs_callback_) {
    obs_callback_(sbp_obs_array_);
  }
  sbp_obs_array_.reset();
}

/** Contains the mapping of codes into supported codes.
 *
 * Primary codes are the ones supported
 * Primary codes are mapped to themselves.
 *
 * Secondary codes are the codes that can be found from remote observations.
 * Secondary codes have a mapping to the corresponding primary code.
 *
 */
/* clang-format off */
static code_t code_replacement(code_t code) {
  switch (code) {
    case CODE_GPS_L1CA:return CODE_GPS_L1CA;
    case CODE_GPS_L2CM:return CODE_GPS_L2CM;
    case CODE_SBAS_L1CA:return CODE_SBAS_L1CA;
    case CODE_GLO_L1OF:return CODE_GLO_L1OF;
    case CODE_GLO_L2OF:return CODE_GLO_L2OF;
    case CODE_GPS_L1P:return CODE_GPS_L1P;
    case CODE_GPS_L2P:return CODE_GPS_L2P;
    case CODE_GPS_L2CL:return CODE_GPS_L2CM;
    case CODE_GPS_L2CX:return CODE_GPS_L2CM;
    case CODE_GPS_L5I:return CODE_GPS_L5I;
    case CODE_GPS_L5Q:return CODE_GPS_L5I;
    case CODE_GPS_L5X:return CODE_GPS_L5I;
    case CODE_BDS2_B1:return CODE_BDS2_B1;
    case CODE_BDS2_B2:return CODE_BDS2_B2;
    case CODE_GAL_E1B:return CODE_GAL_E1B;
    case CODE_GAL_E1C:return CODE_GAL_E1B;
    case CODE_GAL_E1X:return CODE_GAL_E1B;
    case CODE_GAL_E6B:return CODE_GAL_E6B;
    case CODE_GAL_E6C:return CODE_GAL_E6B;
    case CODE_GAL_E6X:return CODE_GAL_E6B;
    case CODE_GAL_E7I:return CODE_GAL_E7I;
    case CODE_GAL_E7Q:return CODE_GAL_E7I;
    case CODE_GAL_E7X:return CODE_GAL_E7I;
    case CODE_GAL_E8I:return CODE_GAL_E8I;
    case CODE_GAL_E8Q:return CODE_GAL_E8I;
    case CODE_GAL_E8X:return CODE_GAL_E8I;
    case CODE_GAL_E5I:return CODE_GAL_E5I;
    case CODE_GAL_E5Q:return CODE_GAL_E5I;
    case CODE_GAL_E5X:return CODE_GAL_E5I;
    case CODE_GLO_L1P:return CODE_GLO_L1P;
    case CODE_GLO_L2P:return CODE_GLO_L2P;
    case CODE_QZS_L1CA:return CODE_QZS_L1CA;
    case CODE_QZS_L1CI:return CODE_QZS_L1CI;
    case CODE_QZS_L1CQ:return CODE_QZS_L1CI;
    case CODE_QZS_L1CX:return CODE_QZS_L1CI;
    case CODE_QZS_L2CM:return CODE_QZS_L2CM;
    case CODE_QZS_L2CL:return CODE_QZS_L2CM;
    case CODE_QZS_L2CX:return CODE_QZS_L2CM;
    case CODE_QZS_L5I:return CODE_QZS_L5I;
    case CODE_QZS_L5Q:return CODE_QZS_L5I;
    case CODE_QZS_L5X:return CODE_QZS_L5I;
    case CODE_SBAS_L5I:return CODE_SBAS_L5I;
    case CODE_SBAS_L5Q:return CODE_SBAS_L5I;
    case CODE_SBAS_L5X:return CODE_SBAS_L5I;
    case CODE_BDS3_B1CI:return CODE_BDS3_B1CI;
    case CODE_BDS3_B1CQ:return CODE_BDS3_B1CI;
    case CODE_BDS3_B1CX:return CODE_BDS3_B1CI;
    case CODE_BDS3_B5I:return CODE_BDS3_B5I;
    case CODE_BDS3_B5Q:return CODE_BDS3_B5I;
    case CODE_BDS3_B5X:return CODE_BDS3_B5I;
    case CODE_BDS3_B7I:return CODE_BDS3_B7I;
    case CODE_BDS3_B7Q:return CODE_BDS3_B7I;
    case CODE_BDS3_B7X:return CODE_BDS3_B7I;
    case CODE_BDS3_B3I:return CODE_BDS3_B3I;
    case CODE_BDS3_B3Q:return CODE_BDS3_B3I;
    case CODE_BDS3_B3X:return CODE_BDS3_B3I;
    case CODE_GPS_L1CI:return CODE_GPS_L1CI;
    case CODE_GPS_L1CQ:return CODE_GPS_L1CI;
    case CODE_GPS_L1CX:return CODE_GPS_L1CI;
    case CODE_AUX_GPS:return CODE_AUX_GPS;
    case CODE_AUX_SBAS:return CODE_AUX_SBAS;
    case CODE_AUX_GAL:return CODE_AUX_GAL;
    case CODE_AUX_QZS:return CODE_AUX_QZS;
    case CODE_AUX_BDS:return CODE_AUX_BDS;
    case CODE_INVALID:
    case CODE_COUNT:
    default: return CODE_INVALID;
  }
}
/* clang-format on */

code_t to_supported_code_t(const code_t code) { return code_replacement(code); }
