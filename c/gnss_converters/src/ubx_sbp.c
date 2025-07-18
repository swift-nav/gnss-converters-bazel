/*
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <ephemeris/sbas.h>
#include <gnss-converters/ubx_sbp.h>
#include <libsbp/sbp_msg_type.h>
#include <libsbp/v4/imu.h>
#include <libsbp/v4/logging.h>
#include <libsbp/v4/orientation.h>
#include <libsbp/v4/system.h>
#include <libsbp/v4/tracking.h>
#include <libsbp/v4/vehicle.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <swiftnav/common.h>
#include <swiftnav/logging.h>
#include <swiftnav/nav_meas.h>
#include <swiftnav/signal.h>
#include <ubx/decode.h>
#include <ubx/ubx_messages.h>
#include <ubx_ephemeris/bds.h>
#include <ubx_ephemeris/gal.h>
#include <ubx_ephemeris/glo.h>
#include <ubx_ephemeris/gps.h>

#include "swiftnav/bytestream.h"

/* TODO(STAR-918) should probably consolidate these into central .h file */
#define SBP_OBS_LF_MULTIPLIER 256
#define SBP_OBS_DF_MULTIPLIER 256

#define UBX_SBP_LAT_LON_SCALING (1e-7)
#define UBX_SBP_HEIGHT_SCALING (1e-3)

#define UBX_HNR_PVT_FIX_TYPE_NONE (0x00)
#define UBX_HNR_PVT_FIX_TYPE_DEAD_RECKONING (0x01)
#define UBX_HNR_PVT_FIX_TYPE_2D (0x02)
#define UBX_HNR_PVT_FIX_TYPE_3D (0x03)
#define UBX_HNR_PVT_FIX_TYPE_COMBINED (0x04)
#define UBX_HNR_PVT_FIX_TYPE_TIME (0x05)

#define UBX_HNR_PVT_DGNSS_MASK (0x02)

#define UBX_NAV_ATT_SCALING (10)       /* 1e-5->1e-6 degrees */
#define UBX_NAV_ATT_ACC_SCALING (1e-5) /* 1e-5->1 degrees */

#define SBP_ORIENT_EULER_INS_MASK (0x01)

#define UBX_NAV_PVT_FIX_TYPE_NONE (0x00)
#define UBX_NAV_PVT_FIX_TYPE_DEAD_RECKONING (0x01)
#define UBX_NAV_PVT_FIX_TYPE_2D (0x02)
#define UBX_NAV_PVT_FIX_TYPE_3D (0x03)
#define UBX_NAV_PVT_FIX_TYPE_COMBINED (0x04)
#define UBX_NAV_PVT_FIX_TYPE_TIME (0x05)

#define UBX_NAV_PVT_DGNSS_MASK (0x02)
#define UBX_NAV_PVT_INS_MASK (0x04)

#define UBX_NAV_PVT_CARR_SOLN_MASK (0xC0)
#define UBX_NAV_PVT_SOLN_NO_PHASE (0x00)
#define UBX_NAV_PVT_SOLN_FLOAT_RTK (0x40)
#define UBX_NAV_PVT_SOLN_FIXED_RTK (0x80)

#define UBX_NAV_VELECEF_SCALING (10) /* cm->mm */

#define SBP_LLH_INVALID_SOLN_MASK (0x00)
#define SBP_LLH_FLOAT_RTK_MASK (0x03)
#define SBP_LLH_FIXED_RTK_MASK (0x04)
#define SBP_LLH_DEAD_RECKONING_MASK (0x05)
#define SBP_LLH_DGNSS_MASK (0x02)
#define SBP_LLH_SPP_MASK (0x01)
#define SBP_LLH_INS_MASK (0x08)

#define UBX_SBP_TOW_MS_SCALING (1e3)
#define UBX_SBP_TOW_NS_SCALING (1e6)

#define UBX_SBP_PSEUDORANGE_SCALING (50)

#define SBP_OBS_DOPPLER_MASK (1 << 3)
#define SBP_OBS_TRACK_STATE_MASK (0x07)

/* Scale factor to go from ESF-RAW sensor time tags to seconds.
 * This value is not specified in the UBX protocol spec and has been derived
 * from the Bosch BMI 160 datasheet, so it might only be valid for the M8L,
 * which we know uses this IMU.
 */
static const double ubx_sensortime_scale = 39.0625e-6;

/* A single SBP message can only fit a maximum number of observations in it
 * floor((0xFF - sizeof(observation_header_t)) / sizeof(packed_obs_content_t))
 */
#define SBP_MAX_NUM_OBS 14

/* Syncs data between HNR_PVT and NAV_PVT */
struct ubx_pvt_state {
  u8 num_sats;
  u8 flags;
  u8 fix_type;
};

static struct ubx_pvt_state pvt_state;

static int reload_ubx_buffer(struct ubx_sbp_state *state) {
  state->index = 0;
  int bytes_read = state->read_stream_func(
      state->read_buffer, UBX_BUFFER_SIZE, state->context);
  state->bytes_in_buffer = bytes_read > 0 ? bytes_read : 0;
  return bytes_read;
}

static void reset_esf_state(struct ubx_esf_state *state) {
  state->running_imu_msss = -1;
  state->running_odo_msss = -1;
}

/** Attempts to read `length` bytes into `buf`, implemented as a memcpy from
 * state->read_buffer. If there are not enough bytes in the read_buffer, we
 * attempt to reload the read_buffer. If we still do not have sufficient bytes,
 * we assume EOF reached and return -1.
 */
static int read_ubx_bytes(u8 *buf, size_t length, struct ubx_sbp_state *state) {
  int bytes_read = 0;
  while (state->index + length > state->bytes_in_buffer) {
    if (state->bytes_in_buffer == 0) {
      /* Try again in case we perfectly aligned read */
      int reload_bytes = reload_ubx_buffer(state);
      if (reload_bytes <= 0) {
        return reload_bytes;
      }
      continue;
    }

    size_t remaining_bytes = state->bytes_in_buffer - state->index;
    memcpy(
        buf + bytes_read, &state->read_buffer[state->index], remaining_bytes);
    length -= remaining_bytes;
    bytes_read += remaining_bytes;
    int reload_bytes = reload_ubx_buffer(state);
    if (reload_bytes <= 0) {
      return reload_bytes;
    }
  }

  memcpy(buf + bytes_read, &state->read_buffer[state->index], length);
  state->index += length;
  bytes_read += length;

  return bytes_read;
}

/** UBX Frame:
 * SYNC_CHAR_1 | SYNC_CHAR_2 | CLASS | MSG_ID | 2-byte Length | Payload |
 * CHCKSUM_BYTE_1 | CHCKSUM_BYTE_2
 */
static int read_ubx_frame(u8 *frame, struct ubx_sbp_state *state) {
  int ret;
  do {
    ret = read_ubx_bytes(frame, 1, state);
    if (ret <= 0) {
      return ret == 0 ? -1 : ret;
    }

    if (frame[0] != UBX_SYNC_CHAR_1) {
      continue;
    }

    while (frame[0] == UBX_SYNC_CHAR_1) {
      ret = read_ubx_bytes(frame, 1, state);
      if (ret <= 0) {
        return ret == 0 ? -1 : ret;
      }
    }

    if (frame[0] != UBX_SYNC_CHAR_2) {
      continue;
    }

    ret = read_ubx_bytes(frame, 4, state);
    if (ret <= 0) {
      return ret == 0 ? -1 : ret;
    }

    /* First two bytes are class and msg ID */
    u16 payload_length = frame[2] + (frame[3] << 8);
    /* Assume massive payload is due to corrupted message. 2 bytes for header, 2
     * bytes for length, 2 bytes for checksum not counted in payload_length
     */
    if (payload_length > UBX_FRAME_SIZE - 6) {
      log_warn(
          "UBX payload_length for class 0x%X and ID 0x%X too large: %d; "
          "possible corrupted frame",
          frame[0],
          frame[1],
          payload_length);
      continue;
    }

    /* +2 for checksum bytes */
    ret = read_ubx_bytes(frame + 4, payload_length + 2, state);
    if (ret <= 0) {
      return ret == 0 ? -1 : ret;
    }

#ifndef GNSS_CONVERTERS_DISABLE_CRC_VALIDATION
    u8 checksum[2];
    ubx_checksum(frame, 4 + payload_length, (u8 *)&checksum);
    if (memcmp(&checksum, frame + 4 + payload_length, 2) != 0) {
      continue;
    }
#endif
    return payload_length + 4;

  } while (state->bytes_in_buffer != 0);

  return -1;
}

static code_t convert_ubx_gnssid_sigid(u8 gnss_id, u8 sig_id) {
  /* GPS L1C/A */
  if ((gnss_id == 0) && (sig_id == 0)) {
    return CODE_GPS_L1CA;
  }
  /* GPS L2 CL */
  if ((gnss_id == UBX_GNSS_ID_GPS) && (sig_id == 3)) {
    return CODE_GPS_L2CL;
  }
  /* GPS L2 CM */
  if ((gnss_id == UBX_GNSS_ID_GPS) && (sig_id == 4)) {
    return CODE_GPS_L2CM;
  }
  /* SBAS L1 CA */
  if ((gnss_id == UBX_GNSS_ID_SBAS) && (sig_id == 0)) {
    return CODE_SBAS_L1CA;
  }
  /* Galileo E1 C */
  if ((gnss_id == UBX_GNSS_ID_GAL) && (sig_id == 0)) {
    return CODE_GAL_E1C;
  }
  /* Galileo E1 B */
  if ((gnss_id == UBX_GNSS_ID_GAL) && (sig_id == 1)) {
    return CODE_GAL_E1B;
  }
  /* Galileo E5 bI */
  if ((gnss_id == UBX_GNSS_ID_GAL) && (sig_id == 5)) {
    return CODE_GAL_E7I;
  }
  /* Galileo E5 bQ */
  if ((gnss_id == UBX_GNSS_ID_GAL) && (sig_id == 6)) {
    return CODE_GAL_E7Q;
  }
  /* BeiDou B1I D1 */
  if ((gnss_id == UBX_GNSS_ID_BDS) && (sig_id == 0)) {
    return CODE_BDS2_B1;
  }
  /* BeiDou B1I D2 */
  if ((gnss_id == UBX_GNSS_ID_BDS) && (sig_id == 1)) {
    return CODE_BDS2_B1;
  }
  /* BeiDou B2I D1 */
  if ((gnss_id == UBX_GNSS_ID_BDS) && (sig_id == 2)) {
    return CODE_BDS2_B2;
  }
  /* BeiDou B2I D2 */
  if ((gnss_id == UBX_GNSS_ID_BDS) && (sig_id == 3)) {
    return CODE_BDS2_B2;
  }
  /* QZSS L1C/A */
  if ((gnss_id == UBX_GNSS_ID_QZSS) && (sig_id == 0)) {
    return CODE_QZS_L1CA;
  }
  /* QZSS L2 CM */
  if ((gnss_id == UBX_GNSS_ID_QZSS) && (sig_id == 4)) {
    return CODE_QZS_L2CM;
  }
  /* QZSS L2 CL */
  if ((gnss_id == UBX_GNSS_ID_QZSS) && (sig_id == 5)) {
    return CODE_QZS_L2CL;
  }
  /* GLONASS L1 OF */
  if ((gnss_id == UBX_GNSS_ID_GLO) && (sig_id == 0)) {
    return CODE_GLO_L1OF;
  }
  /* GLONASS L2 OF */
  if ((gnss_id == UBX_GNSS_ID_GLO) && (sig_id == 2)) {
    return CODE_GLO_L2OF;
  }

  return CODE_INVALID;
}

static bool pack_carrier_phase(double L_in, sbp_carrier_phase_t *L_out) {
  double Li = floor(L_in);
  if (Li < INT32_MIN || Li > INT32_MAX) {
    return false;
  }

  double Lf = L_in - Li;

  L_out->i = (s32)Li;
  u16 frac_part_cp = (u16)round(Lf * SBP_OBS_LF_MULTIPLIER);
  if (frac_part_cp > UINT8_MAX) {
    frac_part_cp = 0;
    L_out->i += 1;
  }
  L_out->f = frac_part_cp;

  return true;
}

/* TODO(STAR-920) Move pack_* functions into libsbp or libswitfnav */
static bool pack_doppler(double D_in, sbp_doppler_t *D_out) {
  double Di = floor(D_in);
  if (Di < INT16_MIN || Di > INT16_MAX) {
    return false;
  }

  double Df = D_in - Di;

  D_out->i = (s16)Di;
  u16 frac_part_d = (u16)round(Df * SBP_OBS_DF_MULTIPLIER);
  if (frac_part_d > UINT8_MAX) {
    frac_part_d = 0;
    D_out->i += 1;
  }
  D_out->f = frac_part_d;

  return true;
}

s32 convert24b(u32 value) {
  const int MODULO = 1 << 24;
  const int MAX_VALUE = (1 << 23) - 1;
  s32 result = *(s32 *)&(value);
  if (result > MAX_VALUE) {
    result -= MODULO;
  }
  return result;
}

void check_overflow(s32 *value) {
  if (*value > INT16_MAX) {
    *value = INT16_MAX;
  }
  if (*value < INT16_MIN) {
    *value = INT16_MIN;
  }
}

static int fill_msg_obs(const ubx_rxm_rawx *rxm_rawx,
                        sbp_v4_gps_time_t *obs_time,
                        uint8_t *n_obs,
                        sbp_packed_obs_content_t *obs) {
  /* convert from sec to ms */
  double ms = UBX_SBP_TOW_MS_SCALING * rxm_rawx->rcv_tow;
  double ns = UBX_SBP_TOW_NS_SCALING * modf(ms, &ms);
  obs_time->tow = (u32)ms;
  obs_time->ns_residual = (s32)ns;
  if (obs_time->ns_residual > 500000) {
    obs_time->ns_residual -= 1000000;
    obs_time->tow += 1;
  }

  if (rxm_rawx->rcv_wn == 0) {
    obs_time->wn = -1;
  } else {
    obs_time->wn = rxm_rawx->rcv_wn;
  }
  /* Not proper SBP n_obs. Needed to pass total number of measurements
   * to handle_rxm_rawx
   */
  *n_obs = rxm_rawx->num_meas;

  for (int i = 0; i < *n_obs; i++) {
    /* convert units: meter -> 2 cm */
    obs[i].P = (u32)(UBX_SBP_PSEUDORANGE_SCALING * rxm_rawx->pseudorange_m[i]);
    pack_carrier_phase(rxm_rawx->carrier_phase_cycles[i], &obs[i].L);
    pack_doppler(rxm_rawx->doppler_hz[i], &obs[i].D);
    /* check for overflow */
    if (rxm_rawx->cno_dbhz[i] > 0x3F) {
      obs[i].cn0 = 0xFF;
    } else {
      obs[i].cn0 = 4 * rxm_rawx->cno_dbhz[i];
    }
    /* TODO(STAR-919) converts from u32 ms, to double s; encode_lock_time
     * internally converts back to u32 ms.
     */
    obs[i].lock = encode_lock_time((double)rxm_rawx->lock_time[i] / SECS_MS);
    obs[i].flags = 0;
    /* currently assumes all doppler are valid */
    obs[i].flags |= SBP_OBS_DOPPLER_MASK;
    obs[i].flags |= rxm_rawx->track_state[i] & SBP_OBS_TRACK_STATE_MASK;
    obs[i].sid.sat = rxm_rawx->sat_id[i];
    obs[i].sid.code =
        convert_ubx_gnssid_sigid(rxm_rawx->gnss_id[i], rxm_rawx->sig_id[i]);
  }

  return 0;
}

static int fill_msg_orient_euler(swiftnav_bytestream_t *buf,
                                 sbp_msg_orient_euler_t *msg) {
  ubx_nav_att nav_att;
  if (ubx_decode_nav_att_bytestream(buf, &nav_att) != RC_OK) {
    return -1;
  }

  msg->tow = nav_att.i_tow;

  msg->roll = nav_att.roll * UBX_NAV_ATT_SCALING;
  msg->pitch = nav_att.pitch * UBX_NAV_ATT_SCALING;
  msg->yaw = nav_att.heading * UBX_NAV_ATT_SCALING;
  msg->roll_accuracy = (float)(nav_att.acc_roll * UBX_NAV_ATT_ACC_SCALING);
  msg->pitch_accuracy = (float)(nav_att.acc_pitch * UBX_NAV_ATT_ACC_SCALING);
  msg->yaw_accuracy = (float)(nav_att.acc_heading * UBX_NAV_ATT_ACC_SCALING);

  msg->flags = 0;
  if ((pvt_state.fix_type == UBX_NAV_PVT_FIX_TYPE_DEAD_RECKONING) ||
      (pvt_state.fix_type == UBX_NAV_PVT_FIX_TYPE_COMBINED)) {
    msg->flags |= SBP_ORIENT_EULER_INS_MASK;
  }

  return 0;
}

static int fill_msg_pos_llh(swiftnav_bytestream_t *buf,
                            sbp_msg_pos_llh_t *msg) {
  ubx_nav_pvt nav_pvt;
  if (ubx_decode_nav_pvt_bytestream(buf, &nav_pvt) != RC_OK) {
    return -1;
  }

  msg->tow = nav_pvt.i_tow;
  msg->lat = UBX_SBP_LAT_LON_SCALING * nav_pvt.lat;
  msg->lon = UBX_SBP_LAT_LON_SCALING * nav_pvt.lon;
  /* convert from mm to m */
  msg->height = UBX_SBP_HEIGHT_SCALING * nav_pvt.height;
  /* bounding u32 -> u16 conversion */
  u16 max_accuracy = UINT16_MAX;
  msg->h_accuracy = (u16)(nav_pvt.horizontal_accuracy > max_accuracy
                              ? max_accuracy
                              : nav_pvt.horizontal_accuracy);
  msg->v_accuracy = (u16)(nav_pvt.vertical_accuracy > max_accuracy
                              ? max_accuracy
                              : nav_pvt.vertical_accuracy);
  msg->n_sats = nav_pvt.num_sats;
  pvt_state.num_sats = nav_pvt.num_sats;

  msg->flags = 0;
  if ((nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_NONE) ||
      (nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_TIME)) {
    msg->flags |= SBP_LLH_INVALID_SOLN_MASK;
  } else if (nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_DEAD_RECKONING) {
    msg->flags |= SBP_LLH_DEAD_RECKONING_MASK;
  } else if ((nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_2D) ||
             (nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_3D) ||
             (nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_COMBINED)) {
    if ((nav_pvt.flags & UBX_NAV_PVT_DGNSS_MASK) == 0) {
      msg->flags |= SBP_LLH_SPP_MASK;
    } else {
      u8 carr_soln = nav_pvt.flags & UBX_NAV_PVT_CARR_SOLN_MASK;
      switch (carr_soln) {
        case UBX_NAV_PVT_SOLN_NO_PHASE:
          msg->flags |= SBP_LLH_DGNSS_MASK;
          break;
        case UBX_NAV_PVT_SOLN_FLOAT_RTK:
          msg->flags |= SBP_LLH_FLOAT_RTK_MASK;
          break;
        case UBX_NAV_PVT_SOLN_FIXED_RTK:
          msg->flags |= SBP_LLH_FIXED_RTK_MASK;
          break;
        default:
          log_warn("Invalid NAV_PVT.flags carrier solution value: %d",
                   carr_soln);
          break;
      }
    }
  }

  if ((nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_DEAD_RECKONING) ||
      (nav_pvt.fix_type == UBX_NAV_PVT_FIX_TYPE_COMBINED)) {
    msg->flags |= SBP_LLH_INS_MASK;
  }

  pvt_state.flags = msg->flags;
  pvt_state.fix_type = nav_pvt.fix_type;

  return 0;
}

static int fill_msg_vel_ecef(swiftnav_bytestream_t *buf,
                             sbp_msg_vel_ecef_t *msg) {
  ubx_nav_velecef nav_velecef;
  if (ubx_decode_nav_velecef_bytestream(buf, &nav_velecef) != RC_OK) {
    return -1;
  }

  msg->tow = nav_velecef.i_tow;
  msg->x = nav_velecef.ecefVX * UBX_NAV_VELECEF_SCALING;
  msg->y = nav_velecef.ecefVY * UBX_NAV_VELECEF_SCALING;
  msg->z = nav_velecef.ecefVZ * UBX_NAV_VELECEF_SCALING;
  msg->accuracy = nav_velecef.speed_acc;
  msg->n_sats = pvt_state.num_sats;
  msg->flags = 0;
  /* Assume either invalid or dead reckoning. Temp. hack */
  if (pvt_state.fix_type > 0) {
    msg->flags |= SBP_LLH_DEAD_RECKONING_MASK;
  } else {
    msg->flags = 0;
  }
  /* Assume INS always used */
  msg->flags |= SBP_LLH_INS_MASK;

  return 0;
}

static void fill_msg_pos_llh_hnr(const ubx_hnr_pvt *hnr_pvt,
                                 sbp_msg_pos_llh_t *msg) {
  msg->tow = hnr_pvt->i_tow;
  msg->lat = UBX_SBP_LAT_LON_SCALING * hnr_pvt->lat;
  msg->lon = UBX_SBP_LAT_LON_SCALING * hnr_pvt->lon;
  /* convert from mm to m */
  msg->height = UBX_SBP_HEIGHT_SCALING * hnr_pvt->height;
  /* bounding u32 -> u16 conversion */
  u16 max_accuracy = UINT16_MAX;
  msg->h_accuracy = (u16)(hnr_pvt->horizontal_accuracy > max_accuracy
                              ? max_accuracy
                              : hnr_pvt->horizontal_accuracy);
  msg->v_accuracy = (u16)(hnr_pvt->vertical_accuracy > max_accuracy
                              ? max_accuracy
                              : hnr_pvt->vertical_accuracy);

  msg->n_sats = pvt_state.num_sats;
  msg->flags = pvt_state.flags;
}

static int fill_msg_fwd(swiftnav_bytestream_t *buf, sbp_msg_fwd_t *msg) {
  size_t copy_len = swiftnav_bytestream_remaining(buf);
  size_t max_copy_len = sizeof(msg->fwd_payload);
  if (copy_len > max_copy_len) {
    log_warn("UBX frame for class 0x%X and ID 0x%X too large to forward: %d; ",
             buf->data[buf->offset + 0],
             buf->data[buf->offset + 1],
             (int)copy_len);
    return -1;
  }
#define SBP_MSG_FWD_PROTOCOL_UBX 0x1
  msg->source = 0; /* unused */
  msg->protocol = SBP_MSG_FWD_PROTOCOL_UBX;
  swiftnav_bytestream_get_bytes(buf, 0, copy_len, (uint8_t *)msg->fwd_payload);
#undef SBP_MSG_FWD_PROTOCOL_UBX
  msg->n_fwd_payload = copy_len;
  return copy_len;
}

static bool is_odo(u8 data_type) {
  return (ESF_REAR_LEFT_WHEEL_TICKS == data_type) ||
         (ESF_REAR_RIGHT_WHEEL_TICKS == data_type) ||
         (ESF_FRONT_LEFT_WHEEL_TICKS == data_type) ||
         (ESF_FRONT_RIGHT_WHEEL_TICKS == data_type) ||
         (ESF_SINGLE_TICK == data_type) || (ESF_SPEED == data_type);
}

static u8 get_wheeltick_velocity_source(u8 data_type) {
  switch (data_type) {
    case ESF_REAR_LEFT_WHEEL_TICKS:
      return 1;
    case ESF_REAR_RIGHT_WHEEL_TICKS:
      return 0;
    case ESF_FRONT_LEFT_WHEEL_TICKS:
      return 2;
    case ESF_FRONT_RIGHT_WHEEL_TICKS:
      return 3;
    case ESF_SINGLE_TICK:
      return 0;
    default:
      assert(false && "Unsupported data type");
      return 0;
  }
}

static u8 get_odo_velocity_source(u8 data_type) {
  switch (data_type) {
    case ESF_SPEED:
      return 3;
    default:
      assert(false && "Unsupported data type");
      return 0;
  }
}

static void set_wheeltick_time(s64 msss, sbp_msg_wheeltick_t *msg_wheeltick) {
  const u8 time_source_local = 2;
  msg_wheeltick->flags = time_source_local;
  // The SBP protocol expects CPU local timestamps to wrap around after one
  // week. We thus wrap the time since startup into gps_time_t struct and
  // normalize it.
  const double kMilliseconds = 0.001;
  gps_time_t tmp = {.tow = kMilliseconds * msss, .wn = 0};
  normalize_gps_time(&tmp);
  msg_wheeltick->time = (u64)(tmp.tow * SECS_US);
}

static void set_odo_time(s64 msss, sbp_msg_odometry_t *msg_odo) {
  const u8 time_source_processor = 2;
  msg_odo->flags = time_source_processor;
  // The SBP protocol expects CPU local timestamps to wrap around after one
  // week. We thus wrap the time since startup into gps_time_t struct and
  // normalize it.
  const double kMilliseconds = 0.001;
  gps_time_t tmp = {.tow = kMilliseconds * msss, .wn = 0};
  normalize_gps_time(&tmp);
  msg_odo->tow = (u32)(tmp.tow * SECS_MS);
}

static void handle_esf_meas(struct ubx_sbp_state *state,
                            swiftnav_bytestream_t *inbuf) {
  ubx_esf_meas esf_meas;
  if (ubx_decode_esf_meas_bytestream(inbuf, &esf_meas) != RC_OK) {
    return;
  }
  u8 num_meas = (esf_meas.flags >> 11) & 0x1F;
  bool calib_time_tag_valid = (esf_meas.flags & 0x8) > 3;
  if (!calib_time_tag_valid) {
    return;
  }

  int idx;
  for (idx = 0; idx < num_meas; idx++) {
    u32 data_type = ((u32)esf_meas.data[idx] & 0x3F000000) >> 24;
    u32 data_value = ((u32)esf_meas.data[idx] & 0xFFFFFF);

    if (is_odo(data_type)) {
      const uint8_t kCalibTtagValid = 0x8;

      if (!(esf_meas.flags & kCalibTtagValid)) {
        return;
      }

      // A gap of more than 1 second in odo samples may indicate the ME has
      // reset. In that case msss will likely be less than the previous value
      // cached in the state object and would cause wrap around and
      // errors down the processing chain
      if ((state->esf_state.running_odo_msss != -1) &&
          ((uint64_t)esf_meas.calib_tag -
           (uint64_t)state->esf_state.running_odo_msss) > 1000) {
        reset_esf_state(&state->esf_state);
        // The first packet after a reset appears to contain bad data so just
        // drop it, the next packet to arrive should be valid
        return;
      }
      if (state->esf_state.running_odo_msss == -1) {
        state->esf_state.running_odo_msss = esf_meas.calib_tag;
      } else {
        state->esf_state.running_odo_msss +=
            esf_meas.calib_tag - state->esf_state.last_odo_msss;
      }
      state->esf_state.last_odo_msss = esf_meas.calib_tag;

      sbp_msg_t msg;
      if (data_type == ESF_SPEED) {
        sbp_msg_odometry_t *msg_odo = &msg.odometry;
        set_odo_time(state->esf_state.running_odo_msss, msg_odo);
        u8 velocity_source = get_odo_velocity_source(data_type);
        u32 direction = (data_value & 0x800000) >> 23;
        msg_odo->flags |= direction << 5;
        msg_odo->flags |= velocity_source << 3;
        msg_odo->velocity = convert24b(data_value & 0x00FFFFFF);
        state->cb_ubx_to_sbp(
            state->sender_id, SbpMsgOdometry, &msg, state->context);
      } else {
        u32 tick_count = data_value & 0x3FFFFF;
        bool reverse = (data_value & (1 << 23)) >> 23;
        if (state->esf_state.last_input_tick_count == UINT32_MAX) {
          state->esf_state.last_input_tick_count = tick_count;
          state->esf_state.output_tick_count = tick_count;
        } else {
          u32 increment = tick_count - state->esf_state.last_input_tick_count;
          state->esf_state.last_input_tick_count = tick_count;
          if (reverse) {
            state->esf_state.output_tick_count -= increment;
          } else {
            state->esf_state.output_tick_count += increment;
          }
        }
        sbp_msg_wheeltick_t *msg_wheeltick = &msg.wheeltick;
        set_wheeltick_time(state->esf_state.running_odo_msss, msg_wheeltick);
        msg_wheeltick->source = get_wheeltick_velocity_source(data_type);
        msg_wheeltick->ticks = state->esf_state.output_tick_count;
        state->cb_ubx_to_sbp(
            state->sender_id, SbpMsgWheeltick, &msg, state->context);
      }
    }
  }
}

int16_t ubx_convert_temperature_to_bmi160(double temperature_degrees) {
  if (temperature_degrees >= 87.0 - 1.0 / 512.0) {
    return INT16_MAX;
  }

  if (temperature_degrees <= -41.0 + 1.0 / 512.0) {
    /* INT16_MIN denotes an invalid value according to the BMI160 spec */
    return INT16_MIN + 1;
  }

  double imu_temp_bmi160_scaled = (temperature_degrees - 23.0) * 512.0;
  return (int16_t)imu_temp_bmi160_scaled;
}

struct sbp_imuraw_timespec {
  u32 tow;
  u8 tow_f;
};

static struct sbp_imuraw_timespec convert_tow_to_imuraw_time(double tow) {
  struct sbp_imuraw_timespec ret;
  double tow_in_ms = tow * SECS_MS;
  ret.tow = (u32)tow_in_ms;
  ret.tow_f = (u32)((tow_in_ms - ret.tow) * 256.0);
  return ret;
}

static s16 float_to_s16_clamped(float val) {
  s16 ret;
  if (val < INT16_MIN) {
    ret = INT16_MIN;
  } else if (val > INT16_MAX) {
    ret = INT16_MAX;
  } else {
    ret = lroundf(val);
  }
  return ret;
}

static void maybe_parse_imu_data(s32 parsed_value,
                                 u8 data_type,
                                 sbp_msg_imu_raw_t *msg) {
  if (data_type == ESF_X_AXIS_GYRO_ANG_RATE ||
      data_type == ESF_Y_AXIS_GYRO_ANG_RATE ||
      data_type == ESF_Z_AXIS_GYRO_ANG_RATE) {
    const float ubx_scale_angular_rate = ldexpf(1, -12);
    const float angular_rate_degs = ubx_scale_angular_rate * parsed_value;
    const float sbp_scale_rate_125degs = (float)(125.0 / 32768.0);
    s16 sbp_scaled_rate =
        float_to_s16_clamped(angular_rate_degs / sbp_scale_rate_125degs);
    switch (data_type) {
      case ESF_X_AXIS_GYRO_ANG_RATE:
        msg->gyr_x = sbp_scaled_rate;
        break;
      case ESF_Y_AXIS_GYRO_ANG_RATE:
        msg->gyr_y = sbp_scaled_rate;
        break;
      case ESF_Z_AXIS_GYRO_ANG_RATE:
        msg->gyr_z = sbp_scaled_rate;
        break;
      default:
        break;
    }
  }

  if (data_type == ESF_X_AXIS_ACCEL_SPECIFIC_FORCE ||
      data_type == ESF_Y_AXIS_ACCEL_SPECIFIC_FORCE ||
      data_type == ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE) {
    const float ubx_scale_acc = ldexpf(1, -10);
    const float specforce_ms2 = ubx_scale_acc * parsed_value;
    const float sbp_scale_acc_4g = (float)(4.0 * 9.80665 / 32768.0);
    s16 sbp_scaled_specforce =
        float_to_s16_clamped(specforce_ms2 / sbp_scale_acc_4g);
    switch (data_type) {
      case ESF_X_AXIS_ACCEL_SPECIFIC_FORCE:
        msg->acc_x = sbp_scaled_specforce;
        break;
      case ESF_Y_AXIS_ACCEL_SPECIFIC_FORCE:
        msg->acc_y = sbp_scaled_specforce;
        break;
      case ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE:
        msg->acc_z = sbp_scaled_specforce;
        break;
      default:
        break;
    }
  }
}

static bool set_sbp_imu_time(u32 sensortime_this_message,
                             u32 sensortime_first_message,
                             s64 msss,
                             sbp_msg_imu_raw_t *msg,
                             struct ubx_esf_state *esf_state) {
  // The SBP protocol expects CPU local timestamps to wrap around after one
  // week. We thus wrap the time into gps_time_t struct and
  // normalize it.
  const double first_msg_tss = 0.001 * msss;
  gps_time_t cpu_local_time_first_message = {.tow = first_msg_tss, .wn = 0};
  if (!normalize_gps_time_safe(&cpu_local_time_first_message)) {
    return false;
  }

  // The msss field contains only the time for the first IMU data packet of this
  // burst - we derive subsequent timestamps by diffs on the sensor time. The
  // sensor time has a wraparound at 24 bits that we need to handle.
  s32 sensor_time_diff = sensortime_this_message - sensortime_first_message;
  while (sensor_time_diff < 0) {
    sensor_time_diff += (1 << 24); /* unwrap 24 bit overflow */
  }

  double sensor_tss_s = ubx_sensortime_scale * sensor_time_diff + first_msg_tss;
  // This constant has been found empirically by comparing the M8L IMU angular
  // rate with a properly time stamped reference.
  const double ubx_imu_gnss_time_offset = 0.05;
  gps_time_t imu_time_msss;
  imu_time_msss.tow = sensor_tss_s - ubx_imu_gnss_time_offset;
  imu_time_msss.wn = 0;
  if (!normalize_gps_time_safe(&imu_time_msss)) {
    return false;
  }
  if (!gps_time_match_weeks_safe(&imu_time_msss,
                                 &cpu_local_time_first_message)) {
    return false;
  }
  esf_state->last_imu_time_msss = imu_time_msss;

  struct sbp_imuraw_timespec timespec =
      convert_tow_to_imuraw_time(imu_time_msss.tow);
  msg->tow = timespec.tow;
  msg->tow_f = timespec.tow_f;

  const u32 reference_tss_flags = (1 << 30);
  msg->tow |= reference_tss_flags;
  return true;
}

static bool check_imu_message_complete(const s16 *received_number_msgs,
                                       s16 current_msg_number) {
  return (received_number_msgs[ESF_X_AXIS_GYRO_ANG_RATE] ==
              current_msg_number + 1 &&
          received_number_msgs[ESF_Y_AXIS_GYRO_ANG_RATE] ==
              current_msg_number + 1 &&
          received_number_msgs[ESF_Z_AXIS_GYRO_ANG_RATE] ==
              current_msg_number + 1 &&
          received_number_msgs[ESF_X_AXIS_ACCEL_SPECIFIC_FORCE] ==
              current_msg_number + 1 &&
          received_number_msgs[ESF_Y_AXIS_ACCEL_SPECIFIC_FORCE] ==
              current_msg_number + 1 &&
          received_number_msgs[ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE] ==
              current_msg_number + 1);
}

static void send_imu_aux(struct ubx_sbp_state *state) {
  sbp_msg_t sbp_msg;
  sbp_msg_imu_aux_t *msg = &sbp_msg.imu_aux;
  memset(msg, 0, sizeof(*msg));
  const u8 imu_type_bmi160 = 0;
  msg->imu_type = imu_type_bmi160;
  const u8 gyro_range_125degs = 4;
  const u8 acc_range_4g = 1;
  msg->imu_conf = (gyro_range_125degs << 4) | acc_range_4g;
  /* Convert the IMU temperature to BMI160 format, see
   * https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI160-DS000.pdf,
   * section 2.11.8 */
  if (!state->esf_state.temperature_set) {
    const int16_t kTemperatureInvalid = (int16_t)0x8000;
    msg->temp = kTemperatureInvalid;
  } else {
    msg->temp =
        ubx_convert_temperature_to_bmi160(state->esf_state.last_imu_temp);
  }

  state->cb_ubx_to_sbp(
      state->sender_id, SbpMsgImuAux, &sbp_msg, state->context);
}

static bool is_imu_data(u8 data_type) {
  return (ESF_X_AXIS_GYRO_ANG_RATE == data_type) ||
         (ESF_Y_AXIS_GYRO_ANG_RATE == data_type) ||
         (ESF_Z_AXIS_GYRO_ANG_RATE == data_type) ||
         (ESF_X_AXIS_ACCEL_SPECIFIC_FORCE == data_type) ||
         (ESF_Y_AXIS_ACCEL_SPECIFIC_FORCE == data_type) ||
         (ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE == data_type);
}

static u8 get_esf_raw_num_msgs(const ubx_esf_raw *esf_raw) {
  return (esf_raw->length - 4) / 8;
}

static u8 get_esf_raw_data_type(u32 data_field) {
  return (data_field >> 24) & 0xFF;
}

static s32 get_esf_raw_parsed_data(u32 data_field) {
  u32 data = data_field & 0xFFFFFF;
  return convert24b(data);
}

static bool are_imu_timestamps_consistent(const ubx_esf_raw *esf_raw) {
  u8 num_raw = get_esf_raw_num_msgs(esf_raw);
  for (int i = 0; i < num_raw; i++) {
    u8 data_type = get_esf_raw_data_type(esf_raw->data[i]);
    if (!is_imu_data(data_type)) {
      continue;
    }

    s32 time_tag_diff_to_first_sample =
        esf_raw->sensor_time_tag[i] - esf_raw->sensor_time_tag[0];
    // Sensor time tags have a 24 bit rollover. Here we check whether the first
    // sensor sample contained in the current ESF-RAW message is close to a 24
    // bit overflow (e.g. larger than 0x00EFFFFF). If we detect a decreasing
    // timestamp but the first sample of the message was not close to the
    // overflow, we claim the timestamps to be inconsistent.
    const u32 time_tag_near_overflow = 0x00EFFFFF;
    bool first_sample_near_overflow =
        (esf_raw->sensor_time_tag[0] >= time_tag_near_overflow);
    if (time_tag_diff_to_first_sample < 0 && !first_sample_near_overflow) {
      return false;
    }
  }
  return true;
}

static void handle_esf_raw(struct ubx_sbp_state *state,
                           swiftnav_bytestream_t *inbuf) {
  ubx_esf_raw esf_raw;
  sbp_msg_t sbp_msg;
  sbp_msg_imu_raw_t *msg = &sbp_msg.imu_raw;
  memset(msg, 0, sizeof(*msg));
  if (ubx_decode_esf_raw_bytestream(inbuf, &esf_raw) != RC_OK) {
    return;
  }

  // If timestamps in this message are inconsistent, it's better not to convert
  // this message at all
  if (!are_imu_timestamps_consistent(&esf_raw)) {
    return;
  }

  u8 num_raw = get_esf_raw_num_msgs(&esf_raw);

  const size_t MAX_MESSAGES = ESF_Z_AXIS_ACCEL_SPECIFIC_FORCE + 1;
  s16 received_number_msgs[MAX_MESSAGES];
  memset(received_number_msgs, 0, sizeof(s16) * MAX_MESSAGES);
  s16 current_msg_number = 0;
  for (int i = 0; i < num_raw; i++) {
    u8 data_type = get_esf_raw_data_type(esf_raw.data[i]);
    s32 parsed_value = get_esf_raw_parsed_data(esf_raw.data[i]);
    if (data_type == ESF_GYRO_TEMP) {
      const double ubx_scale_temp = 0.01;
      state->esf_state.last_imu_temp = ubx_scale_temp * parsed_value;
      state->esf_state.temperature_set = true;
    }

    if (!is_imu_data(data_type)) {
      continue;
    }

    // A gap of more than 1 second in IMU samples may indicate the ME has reset.
    // In that case msss will likely be less than the previous value cached in
    // the state object and would cause wrap around and errors down
    // the processing chain
    if ((state->esf_state.running_imu_msss != -1) &&
        (esf_raw.msss - state->esf_state.last_imu_msss) > 1000) {
      reset_esf_state(&state->esf_state);
      // The first packet after a reset appears to contain bad data so just drop
      // it, the next packet to arrive should be valid
      return;
    }

    if (state->esf_state.running_imu_msss == -1) {
      state->esf_state.running_imu_msss = esf_raw.msss;
    } else {
      state->esf_state.running_imu_msss +=
          esf_raw.msss - state->esf_state.last_imu_msss;
    }
    state->esf_state.last_imu_msss = esf_raw.msss;

    if (!set_sbp_imu_time(esf_raw.sensor_time_tag[i],
                          esf_raw.sensor_time_tag[0],
                          state->esf_state.running_imu_msss,
                          msg,
                          &state->esf_state)) {
      return;
    }
    maybe_parse_imu_data(parsed_value, data_type, msg);

    received_number_msgs[data_type]++;
    // Before getting the next sample for an axis, we expect to receive all
    // other axes for the same timestamp.
    if (!(data_type == ESF_GYRO_TEMP ||
          received_number_msgs[data_type] == current_msg_number + 1)) {
      log_warn("Lost IMU data, possibly corrupted ESF-RAW frame");
      return;
    }

    // Check if the current IMU message is complete (all axes received) and
    // output an SBP message if it is.
    if (check_imu_message_complete(received_number_msgs, current_msg_number)) {
      if (state->esf_state.imu_raw_msgs_sent % 20 == 0) {
        send_imu_aux(state);
        state->esf_state.imu_raw_msgs_sent = 0;
      }
      state->cb_ubx_to_sbp(
          state->sender_id, SbpMsgImuRaw, &sbp_msg, state->context);
      current_msg_number++;
      state->esf_state.imu_raw_msgs_sent++;
    }
  }
}

static void handle_hnr_pvt(struct ubx_sbp_state *state,
                           swiftnav_bytestream_t *inbuf) {
  ubx_hnr_pvt hnr_pvt;
  sbp_msg_t msg, orient_msg;
  sbp_msg_pos_llh_t *sbp_pos_llh = &msg.pos_llh;
  orient_msg.orient_euler = state->last_orient_euler;
  sbp_msg_orient_euler_t *sbp_orient_euler = &orient_msg.orient_euler;
  if (ubx_decode_hnr_pvt_bytestream(inbuf, &hnr_pvt) != RC_OK) {
    return;
  }

  fill_msg_pos_llh_hnr(&hnr_pvt, sbp_pos_llh);
  state->last_tow_ms = sbp_pos_llh->tow;

  sbp_orient_euler->tow = hnr_pvt.i_tow;
  sbp_orient_euler->yaw = hnr_pvt.heading_vehicle * UBX_NAV_ATT_SCALING;
  sbp_orient_euler->yaw_accuracy =
      (float)(hnr_pvt.heading_acc * UBX_NAV_ATT_ACC_SCALING);

  if (state->use_hnr) {
    state->cb_ubx_to_sbp(state->sender_id, SbpMsgPosLlh, &msg, state->context);
    state->cb_ubx_to_sbp(
        state->sender_id, SbpMsgOrientEuler, &orient_msg, state->context);
  }
}

static void handle_nav_att(struct ubx_sbp_state *state,
                           swiftnav_bytestream_t *inbuf) {
  sbp_msg_t msg;
  sbp_msg_orient_euler_t *sbp_orient_euler = &msg.orient_euler;
  if (fill_msg_orient_euler(inbuf, sbp_orient_euler) == 0) {
    memcpy(&state->last_orient_euler,
           sbp_orient_euler,
           sizeof(sbp_msg_orient_euler_t));
    if (!state->use_hnr) {
      state->cb_ubx_to_sbp(
          state->sender_id, SbpMsgOrientEuler, &msg, state->context);
    }
  }
}

static void handle_nav_pvt(struct ubx_sbp_state *state,
                           swiftnav_bytestream_t *inbuf) {
  sbp_msg_t msg;
  sbp_msg_pos_llh_t *sbp_pos_llh = &msg.pos_llh;
  if (fill_msg_pos_llh(inbuf, sbp_pos_llh) == 0) {
    state->last_tow_ms = sbp_pos_llh->tow;
    if (!state->use_hnr) {
      state->cb_ubx_to_sbp(
          state->sender_id, SbpMsgPosLlh, &msg, state->context);
    }
  }
}

static void handle_nav_velecef(struct ubx_sbp_state *state,
                               swiftnav_bytestream_t *inbuf) {
  sbp_msg_t msg;
  sbp_msg_vel_ecef_t *sbp_vel_ecef = &msg.vel_ecef;
  if (fill_msg_vel_ecef(inbuf, sbp_vel_ecef) == 0) {
    state->cb_ubx_to_sbp(state->sender_id, SbpMsgVelEcef, &msg, state->context);
  }
}

static void handle_nav_sat(struct ubx_sbp_state *state,
                           swiftnav_bytestream_t *inbuf) {
  ubx_nav_sat nav_sat; /* 1218 bytes (ish) ! */
  if (ubx_decode_nav_sat_bytestream(inbuf, &nav_sat) != RC_OK) {
    return;
  }

  /* prep SBP_MSG_SV_AZ_EL */
  sbp_msg_t sbp_msg_az;
  sbp_msg_sv_az_el_t *msg_az_el = &sbp_msg_az.sv_az_el;
  memset(msg_az_el, 0, sizeof(*msg_az_el));
  const u8 max_az = SBP_MSG_SV_AZ_EL_AZEL_MAX;

  /* prep SBP_MSG_MEASUREMENT_STATE */
  sbp_msg_t sbp_msg_state;
  sbp_msg_measurement_state_t *msg_meas_state =
      &sbp_msg_state.measurement_state;
  memset(msg_meas_state, 0, sizeof(*msg_meas_state));
  const u8 max_state = SBP_MSG_MEASUREMENT_STATE_STATES_MAX;

  for (int i = 0; i < nav_sat.num_svs; i++) {
    if (i >= NAV_DATA_MAX_COUNT) {
      break;
    }

    ubx_nav_sat_data *data = &nav_sat.data[i];

    u8 sat_id = data->sv_id;
    u8 code = convert_ubx_gnssid_sigid(data->gnss_id,
                                       0); /* always zero for UBX-NAV-SAT */

    msg_az_el->azel[msg_az_el->n_azel].sid.sat = sat_id;
    msg_az_el->azel[msg_az_el->n_azel].sid.code = code;
    msg_az_el->azel[msg_az_el->n_azel].az = lround((double)data->azim / 2.0);
    msg_az_el->azel[msg_az_el->n_azel].el = data->elev;
    msg_az_el->n_azel++;

    msg_meas_state->states[msg_meas_state->n_states].mesid.sat = sat_id;
    msg_meas_state->states[msg_meas_state->n_states].mesid.code = code;
    msg_meas_state->states[msg_meas_state->n_states].cn0 =
        lround((double)data->cno * 4.0);
    msg_meas_state->n_states++;

    /* send SBP_MSG_SV_AZ_EL if full */
    if ((msg_az_el->n_azel + 1) > max_az) {
      state->cb_ubx_to_sbp(
          state->sender_id, SbpMsgSvAzEl, &sbp_msg_az, state->context);
      memset(msg_az_el, 0, sizeof(*msg_az_el));
    }

    /* send SBP_MSG_MEASUREMENT_STATE if full */
    if ((msg_meas_state->n_states + 1) > max_state) {
      state->cb_ubx_to_sbp(state->sender_id,
                           SbpMsgMeasurementState,
                           &sbp_msg_state,
                           state->context);
      memset(msg_meas_state, 0, sizeof(*msg_meas_state));
    }
  }

  /* send SBP_MSG_SV_AZ_EL if not empty */
  if (msg_az_el->n_azel > 0) {
    state->cb_ubx_to_sbp(
        state->sender_id, SbpMsgSvAzEl, &sbp_msg_az, state->context);
  }

  /* send SBP_MSG_MEASUREMENT_STATE if not empty */
  if (msg_meas_state->n_states > 0) {
    state->cb_ubx_to_sbp(state->sender_id,
                         SbpMsgMeasurementState,
                         &sbp_msg_state,
                         state->context);
  }
}

static void handle_nav_status(struct ubx_sbp_state *state,
                              swiftnav_bytestream_t *inbuf) {
  ubx_nav_status nav_status;
  if (ubx_decode_nav_status_bytestream(inbuf, &nav_status) != RC_OK) {
    return;
  }
  const u8 fix_type_3d = 0x03;
  const u8 fix_type_gnss_dr = 0x04;
  const u8 gps_fix_ok = 0x1;
  bool gnss_fix_good = ((nav_status.fix_type == fix_type_3d) ||
                        (nav_status.fix_type == fix_type_gnss_dr)) &&
                       (nav_status.status_flags & gps_fix_ok);

  const u8 weeknumber_ok = 0x8;
  const u8 tow_ok = 0x4;
  bool time_good = (nav_status.status_flags & weeknumber_ok) &&
                   (nav_status.status_flags & tow_ok);

  if (time_good) {
    state->esf_state.weeknumber_set = true;
  }

  if (gnss_fix_good && time_good &&
      gps_time_valid(&state->esf_state.last_obs_time_gnss)) {
    gps_time_t nav_status_time_gnss;
    nav_status_time_gnss.wn = 0;
    nav_status_time_gnss.tow = 0.001 * nav_status.i_tow;
    if (!normalize_gps_time_safe(&nav_status_time_gnss)) {
      return;
    }
    if (!gps_time_match_weeks_safe(&nav_status_time_gnss,
                                   &state->esf_state.last_obs_time_gnss)) {
      return;
    }

    if (gps_time_valid(&state->esf_state.last_imu_time_msss)) {
      gps_time_t nav_status_time_local;
      nav_status_time_local.tow = 0.001 * nav_status.msss;
      nav_status_time_local.wn = 0;
      if (!normalize_gps_time_safe(&nav_status_time_local)) {
        return;
      }
      if (!gps_time_match_weeks_safe(&nav_status_time_local,
                                     &state->esf_state.last_imu_time_msss)) {
        return;
      }

      gps_time_t offset;
      offset.wn = nav_status_time_gnss.wn - nav_status_time_local.wn;
      offset.tow = nav_status_time_gnss.tow - nav_status_time_local.tow;

      state->last_tow_ms = nav_status.i_tow;
      sbp_msg_t sbp_msg;
      sbp_msg_gnss_time_offset_t *msg = &sbp_msg.gnss_time_offset;
      msg->weeks = offset.wn;
      msg->milliseconds = lround(1000.0 * offset.tow);
      msg->microseconds = 0;
      msg->flags = 0;
      state->cb_ubx_to_sbp(
          state->sender_id, SbpMsgGnssTimeOffset, &sbp_msg, state->context);
    }
  }
}

static void update_utc_params(struct ubx_sbp_state *state,
                              const ubx_rxm_rawx *rxm_rawx) {
  if ((rxm_rawx->rec_status & 1U) == 1) {
    state->leap_second_known = true;
    /* create a dummy utc_params struct for glo2gps time conversion */
    state->utc_params.dt_ls = rxm_rawx->leap_second;
    state->utc_params.dt_lsf = rxm_rawx->leap_second;
    /* set the dummy time stamp of this message and the last leap second event
     * to beginning of the week. these values do not matter in the glo2gps
     * conversion since the current and next leap second values above are set to
     * be equal and the polynomial corrections are all zeros */
    state->utc_params.tot.wn = rxm_rawx->rcv_wn;
    state->utc_params.t_lse.wn = rxm_rawx->rcv_wn;

    if (state->ubx_leap_time_estimator != NULL) {
      time_truth_ubx_leap_estimator_push(
          state->ubx_leap_time_estimator,
          (gps_time_t){.wn = rxm_rawx->rcv_wn, .tow = rxm_rawx->rcv_tow},
          rxm_rawx->leap_second);
    }
  } else {
    state->leap_second_known = false;
  }
}

static void handle_rxm_rawx(struct ubx_sbp_state *state,
                            swiftnav_bytestream_t *inbuf) {
  ubx_rxm_rawx rxm_rawx;
  if (ubx_decode_rxm_rawx_bytestream(inbuf, &rxm_rawx) != RC_OK) {
    return;
  }

  if (rxm_rawx.num_meas != 0 && rxm_rawx.rcv_wn != 0 &&
      state->esf_state.weeknumber_set) {
    state->esf_state.last_obs_time_gnss.tow = rxm_rawx.rcv_tow;
    state->esf_state.last_obs_time_gnss.wn = rxm_rawx.rcv_wn;
  }
  update_utc_params(state, &rxm_rawx);

  sbp_v4_gps_time_t obs_time;
  uint8_t n_obs;
  sbp_packed_obs_content_t sbp_obs[UBX_MAX_NUM_OBS];
  if (fill_msg_obs(&rxm_rawx, &obs_time, &n_obs, sbp_obs) == 0) {
    u8 total_messages;
    if (n_obs > 0) {
      total_messages = 1 + ((n_obs - 1) / SBP_MAX_NUM_OBS);
    } else {
      total_messages = 1;
    }

    sbp_msg_t msg;
    sbp_msg_obs_t *sbp_obs_to_send = &msg.obs;
    sbp_obs_to_send->header.t = obs_time;

    if (state->observation_time_estimator != NULL) {
      time_truth_observation_estimator_push(state->observation_time_estimator,
                                            obs_time.tow);
    }

    u8 obs_index;
    for (u8 msg_num = 0; msg_num < total_messages; msg_num++) {
      sbp_obs_to_send->header.n_obs = (total_messages << 4) + msg_num;
      for (obs_index = 0; obs_index < SBP_MAX_NUM_OBS &&
                          (obs_index + msg_num * SBP_MAX_NUM_OBS) < n_obs;
           obs_index++) {
        sbp_obs_to_send->obs[obs_index] =
            sbp_obs[obs_index + msg_num * SBP_MAX_NUM_OBS];
      }

      sbp_obs_to_send->n_obs = obs_index;

      state->cb_ubx_to_sbp(state->sender_id, SbpMsgObs, &msg, state->context);
    }
  }
}

static void handle_rxm_sfrbx(struct ubx_sbp_state *state,
                             swiftnav_bytestream_t *buf,
                             int sz) {
  (void)sz;
  assert(state);
  assert(buf);
  assert(sz > 0);
  ubx_rxm_sfrbx sfrbx;
  if (ubx_decode_rxm_sfrbx_bytestream(buf, &sfrbx) != RC_OK) {
    return;
  }

  u8 prn = sfrbx.sat_id;
  if (UBX_GNSS_ID_GPS == sfrbx.gnss_id) {
    gps_decode_subframe(state, prn, sfrbx.data_words, sfrbx.num_words);
  } else if (UBX_GNSS_ID_BDS == sfrbx.gnss_id) {
    bds_decode_subframe(state, prn, sfrbx.data_words, sfrbx.num_words);
  } else if (UBX_GNSS_ID_GAL == sfrbx.gnss_id) {
    gal_decode_page(state, prn, sfrbx.data_words, sfrbx.num_words);
  } else if (UBX_GNSS_ID_GLO == sfrbx.gnss_id) {
    glo_decode_string(
        state, prn, sfrbx.freq_id, sfrbx.data_words, sfrbx.num_words);
  } else if (UBX_GNSS_ID_SBAS == sfrbx.gnss_id) {
    u32 tow_ms = state->last_tow_ms;
    if (tow_ms > WEEK_MS) {
      /* TOW not yet set */
      return;
    }

    /* Fill in the approximate time of transmission of the start of the SBAS
     * message. The best guess for it we have is the current solution time
     * minus 1 s (length of the message) minus 120 ms (~= 36000km / 3e8m/s). */
    tow_ms -= 1120;
    if (tow_ms < 0.0) {
      tow_ms += WEEK_MS;
    }

    sbas_decode_subframe(&state->eph_data,
                         tow_ms,
                         prn,
                         sfrbx.data_words,
                         sfrbx.num_words,
                         state->sender_id,
                         state->context,
                         state->cb_ubx_to_sbp);
  }
}

static void handle_mon_hw(struct ubx_sbp_state *state,
                          swiftnav_bytestream_t *inbuf) {
  sbp_msg_t sbp_msg;
  sbp_msg_fwd_t *msg = &sbp_msg.fwd;
  int written = fill_msg_fwd(inbuf, msg);
  if (written > 0) {
    state->cb_ubx_to_sbp(state->sender_id, SbpMsgFwd, &sbp_msg, state->context);
  }
}

void ubx_handle_frame(swiftnav_bytestream_t *frame,
                      struct ubx_sbp_state *state) {
  if (frame->len < 2) {
    return;
  }
  u8 class_id = frame->data[0];
  u8 msg_id = frame->data[1];

  switch (class_id) {
    case UBX_CLASS_ESF:
      switch (msg_id) {
        case UBX_MSG_ESF_RAW:
          handle_esf_raw(state, frame);
          break;
        case UBX_MSG_ESF_MEAS:
          handle_esf_meas(state, frame);
          break;
        default:
          break;
      }
      break;

    case UBX_CLASS_HNR:
      if (msg_id == UBX_MSG_HNR_PVT) {
        handle_hnr_pvt(state, frame);
      }
      break;

    case UBX_CLASS_NAV:
      switch (msg_id) {
        case UBX_MSG_NAV_ATT:
          handle_nav_att(state, frame);
          break;
        case UBX_MSG_NAV_PVT:
          handle_nav_pvt(state, frame);
          break;
        case UBX_MSG_NAV_VELECEF:
          handle_nav_velecef(state, frame);
          break;
        case UBX_MSG_NAV_SAT:
          handle_nav_sat(state, frame);
          break;
        case UBX_MSG_NAV_STATUS:
          handle_nav_status(state, frame);
          break;
        default:
          break;
      }
      break;

    case UBX_CLASS_RXM:
      if (msg_id == UBX_MSG_RXM_RAWX) {
        handle_rxm_rawx(state, frame);
      } else if (msg_id == UBX_MSG_RXM_SFRBX) {
        handle_rxm_sfrbx(state, frame, UBX_FRAME_SIZE);
      }
      break;

    case UBX_CLASS_MON:
      switch (msg_id) {
        case UBX_MSG_MON_HW:
          handle_mon_hw(state, frame);
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

void ubx_sbp_init(struct ubx_sbp_state *state,
                  void (*cb_ubx_to_sbp)(uint16_t sender_id,
                                        sbp_msg_type_t msg_type,
                                        const sbp_msg_t *msg,
                                        void *context),
                  void *context) {
  memset(state, 0, sizeof(*state));
  state->index = 0;
  state->bytes_in_buffer = 0;
  state->sender_id = DEFAULT_UBX_SENDER_ID;
  state->cb_ubx_to_sbp = cb_ubx_to_sbp;
  state->context = context;
  state->use_hnr = false;
  state->last_tow_ms = -1;
  state->esf_state.last_obs_time_gnss = GPS_TIME_UNKNOWN;
  state->esf_state.last_imu_time_msss = GPS_TIME_UNKNOWN;
  state->esf_state.last_imu_msss = 0;
  state->esf_state.last_odo_msss = 0;
  state->esf_state.imu_raw_msgs_sent = 0;
  state->esf_state.last_input_tick_count = UINT32_MAX;
  state->esf_state.output_tick_count = 0;
  state->esf_state.last_imu_temp = 0;
  state->esf_state.temperature_set = false;
  state->esf_state.weeknumber_set = false;
  reset_esf_state(&state->esf_state);

  state->leap_second_known = false;

  state->observation_time_estimator = NULL;
  state->ephemeris_time_estimator = NULL;
  state->ubx_leap_time_estimator = NULL;
}

void ubx_set_sender_id(struct ubx_sbp_state *state, u16 sender_id) {
  state->sender_id = sender_id;
}

void ubx_set_hnr_flag(struct ubx_sbp_state *state, bool use_hnr) {
  state->use_hnr = use_hnr;
}

/**
 * Processes a stream of data, converting UBX messages into SBP and outputting
 * via the callback functions in the `state` parameter. This function assumes
 * that the `ubx_sbp_state` object has already been fully populated.
 * `read_stream_func` will be called repeatedly until it returns a zero or
 * negative value.
 *
 * @param state An already populated state object
 * @param read_stream_func The function to call to read from the stream. `buf`
 * is the buffer to write data into, `len` is the size of the buffer in bytes,
 * `ctx` is the context pointer from the `state` argument, the return value is
 * the number of bytes written into `buf` with negative values indicating an
 * error.
 * @return the last value of read_stream_func, either 0 or a negative value
 * indicating an error.
 */
int ubx_sbp_process(struct ubx_sbp_state *state,
                    int (*read_stream_func)(u8 *buff, size_t len, void *ctx)) {
  state->read_stream_func = read_stream_func;
  u8 inbuf[UBX_FRAME_SIZE];

  ssize_t ret = read_ubx_frame(inbuf, state);
  if (ret <= 0) {
    return ret;
  }

  swiftnav_bytestream_t bytestream;
  swiftnav_bytestream_init(&bytestream, inbuf, ret);

  ubx_handle_frame(&bytestream, state);

  return ret;
}

void ubx_sbp_set_time_truth_estimators(
    struct ubx_sbp_state *state,
    ObservationTimeEstimator *observation_time_estimator,
    EphemerisTimeEstimator *ephemeris_time_estimator,
    UbxLeapTimeEstimator *ubx_leap_time_estimator) {
  state->observation_time_estimator = observation_time_estimator;
  state->ephemeris_time_estimator = ephemeris_time_estimator;
  state->ubx_leap_time_estimator = ubx_leap_time_estimator;
}

void invalidate_subframes(struct sat_data *sat, unsigned mask) {
  assert(sat);
  sat->vmask &= ~mask;
}
