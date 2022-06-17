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

#include <libsbp/sbp.h>
#include <observation_adjuster/epoch_adjuster.h>
#include <observation_adjuster/observation_adjuster.h>
#include <rtcm3/messages.h>
#include <swiftnav/logging.h>

uint32_t type_to_idx(const obs_adjuster::StreamType type) {
  return static_cast<uint32_t>(type);
}

ObservationAdjuster::ObservationAdjuster(const double vrs_ecef[3],
                                         const uint8_t msm_output_type) {
  vrs_ecef_.at(0) = vrs_ecef[0];
  vrs_ecef_.at(1) = vrs_ecef[1];
  vrs_ecef_.at(2) = vrs_ecef[2];

  for (const auto stream_type : {obs_adjuster::StreamType::STATION_OBS,
                                 obs_adjuster::StreamType::STATION_CORR,
                                 obs_adjuster::StreamType::VRS_CORR}) {
    auto unpacker = std::make_unique<obs_adjuster::SbpUnpacker>(stream_type);
    unpacker->set_obs_callback(std::bind(&ObservationAdjuster::obs_decoded,
                                         this,
                                         std::placeholders::_1,
                                         std::placeholders::_2));
    unpackers_.at(type_to_idx(stream_type)) = std::move(unpacker);
  }

  msm_type_e msm_type = static_cast<msm_type_e>(msm_output_type);
  sbp2rtcm_converter_ = sbp_conv_new(msm_type, "ADVNULLANTENNA  NONE", "");
  // Activate Skylark RTK specific station parameters
  // (no Glonass, VRS like reference station, no quarter cycle bias)
  sbp_conv_set_stn_description_parameters(sbp2rtcm_converter_, 0, 1, 1);
}

ObservationAdjuster::~ObservationAdjuster() {
  sbp_conv_delete(sbp2rtcm_converter_);
}

void ObservationAdjuster::read_station_observations(uint16_t sender,
                                                    uint16_t type,
                                                    uint8_t *buf,
                                                    size_t size) {
  std::lock_guard<std::mutex> lock(mutex_);
  unpackers_.at(type_to_idx(obs_adjuster::StreamType::STATION_OBS))
      ->process(sender, type, buf, size);
}

void ObservationAdjuster::read_station_corrections(uint16_t sender,
                                                   uint16_t type,
                                                   uint8_t *buf,
                                                   size_t size) {
  std::lock_guard<std::mutex> lock(mutex_);
  unpackers_.at(type_to_idx(obs_adjuster::StreamType::STATION_CORR))
      ->process(sender, type, buf, size);
}

void ObservationAdjuster::read_receiver_corrections(uint16_t sender,
                                                    uint16_t type,
                                                    uint8_t *buf,
                                                    size_t size) {
  (void)sender;
  (void)type;

  std::lock_guard<std::mutex> lock(mutex_);
  if (type != SBP_MSG_OBS && type != SBP_MSG_BASE_POS_ECEF) {
    sbp_packer_.pack(sender, type, buf, size);
    return;
  }

  unpackers_.at(type_to_idx(obs_adjuster::StreamType::VRS_CORR))
      ->process(sender, type, buf, size);
}

size_t ObservationAdjuster::write_VRS_corrections_SBP(uint8_t *buf,
                                                      size_t size) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Do we have left over data from previous calls, still to be sent out?
  std::unique_ptr<std::vector<u8>> bytes = sbp_packer_.pop_packed_bytes(size);
  if (!bytes->empty()) {
    memcpy(buf, bytes->data(), bytes->size());
    return bytes->size();
  }

  obs_adjuster::MatchingTriplet triplet;
  gps_time_t timestamp = GPS_TIME_UNKNOWN;

  if (message_matcher_.find_match(&triplet, &timestamp)) {
    obs_adjuster::EpochAdjuster adjuster;
    obs_adjuster::SbpObsArray<obs_adjuster::MAX_OBS_PER_EPOCH> adjusted_vrs_obs;
    if (adjuster.adjust(triplet.station_obs_,
                        triplet.station_corr_,
                        triplet.vrs_corr_,
                        &adjusted_vrs_obs)) {
      // We have a new adjusted epoch, for each epoch we should send
      // out the VRS location and the observation bytes, so pack them in order
      sbp_packer_.pack(vrs_ecef_);
      sbp_packer_.pack(timestamp, adjusted_vrs_obs);
      bytes = sbp_packer_.pop_packed_bytes(size);
      memcpy(buf, bytes->data(), bytes->size());
      return bytes->size();
    }
  }

  return 0;
}

size_t ObservationAdjuster::write_VRS_corrections_RTCM(uint8_t *buf,
                                                       size_t size) {
  std::vector<u8> sbp_buffer(size, 0);
  const size_t num_sbp_bytes =
      write_VRS_corrections_SBP(sbp_buffer.data(), sbp_buffer.size());

  std::lock_guard<std::mutex> lock(mutex_);

  // For each SBP message in the output, convert it to RTCM
  size_t sbp_message_start = 0;
  size_t nr_rtcm_bytes_written = 0;
  while ((num_sbp_bytes - sbp_message_start) >=
         (SBP_HEADER_LEN + SBP_CRC_LEN)) {
    // If start of buffer is not the SBP preamble, try at the next byte
    if (sbp_buffer[sbp_message_start] != 0x55) {
      sbp_message_start += 1;
      continue;
    }

    uint16_t msg_type = 0;
    memcpy(&msg_type,
           &sbp_buffer[sbp_message_start + SBP_FRAME_OFFSET_MSGTYPE],
           sizeof(msg_type));
    uint16_t sender_id = 0;
    memcpy(&sender_id,
           &sbp_buffer[sbp_message_start + SBP_FRAME_OFFSET_SENDERID],
           sizeof(sender_id));
    uint8_t payload_len = 0;
    memcpy(&payload_len,
           &sbp_buffer[sbp_message_start + SBP_FRAME_OFFSET_MSGLEN],
           sizeof(payload_len));

    if ((num_sbp_bytes - sbp_message_start) <
        static_cast<size_t>((SBP_HEADER_LEN + SBP_CRC_LEN) + payload_len)) {
      log_warn("When converting SBP to RTCM: payload_len is %" PRIu8
               " but msg buffer only %zu, which is too small",
               payload_len,
               num_sbp_bytes - sbp_message_start);
      break;
    }

    std::size_t rtcm_written =
        sbp_conv(sbp2rtcm_converter_,
                 sender_id,
                 msg_type,
                 &sbp_buffer[sbp_message_start + SBP_HEADER_LEN],
                 payload_len,
                 &buf[nr_rtcm_bytes_written],
                 size - nr_rtcm_bytes_written);
    nr_rtcm_bytes_written += rtcm_written;
    sbp_message_start += payload_len + (SBP_HEADER_LEN + SBP_CRC_LEN);
  }

  return nr_rtcm_bytes_written;
}

void ObservationAdjuster::obs_decoded(
    const obs_adjuster::StreamType stream_type,
    const obs_adjuster::TimestampedSbpObsArray<obs_adjuster::MAX_OBS_PER_EPOCH>
        &obs_array) {
  message_matcher_.add_obs(stream_type, obs_array);
}
