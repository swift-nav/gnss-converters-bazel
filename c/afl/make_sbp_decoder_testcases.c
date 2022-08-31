#include <helpers/sbp/sbp_data.h>
#include <libsbp/edc.h>
#include <libsbp/sbp.h>
#include <stdio.h>
#include <string.h>
#include <swiftnav/bits.h>
#include <swiftnav/edc.h>

#include "make_afl_testcases.h"

#define SBP_PREAMBLE 0x55

static uint8_t output_buf[4096];
static uint16_t output_buf_len = 0;

/// Reset the out buffer and length tracker
static inline void reset_output_buf(void) {
  memset(output_buf, 0, sizeof(output_buf));
  output_buf_len = 0;
}

/// Write `buff` to `output_buf`
static int32_t sbp_output_buf_write(uint8_t *buff,
                                    uint32_t buf_len,
                                    void *context) {
  (void)context;

  if ((ARRAY_SIZE(output_buf) - output_buf_len) < buf_len) {
    return 0;
  }
  memcpy(output_buf + output_buf_len, buff, buf_len);
  output_buf_len += buf_len;
  return buf_len;
}

/// Encode sbp message into a complete frame, and Add the frame to out buffer
static bool sbp_msg_encode_to_buffer(void (*get_sbp_msg)(sbp_msg_t *,
                                                         sbp_msg_type_t *,
                                                         uint16_t *)) {
  // Get sbp message
  sbp_msg_t msg;
  sbp_msg_type_t msg_type = SbpMsgUnknown;
  uint16_t sender_id = 0;
  get_sbp_msg(&msg, &msg_type, &sender_id);

  // Encode sbp frame (not payload)
  sbp_state_t state;
  sbp_state_init(&state);
  sbp_state_set_io_context(&state, NULL);

  if (SBP_OK != sbp_message_send(
                    &state, msg_type, sender_id, &msg, sbp_output_buf_write)) {
    fprintf(stderr, "Error encoding sbp frame of message %d.\n", msg_type);
    return false;
  }

  return true;
}

static void write_output(const char *name) {
  write_file(name, output_buf, output_buf_len);
  reset_output_buf();
}

static void make_testcase_single_msg(const char *tc_name,
                                     void (*get_sbp_msg)(sbp_msg_t *,
                                                         sbp_msg_type_t *,
                                                         uint16_t *)) {
  reset_output_buf();

  if (!sbp_msg_encode_to_buffer(get_sbp_msg)) {
    return;
  }
  write_output(tc_name);
  fprintf(stderr, "Testcase generated: %s\n", tc_name);
}

/// Make testcases for multi message
static void make_testcase_multi_msg_obs(const char *tc_name) {
  reset_output_buf();

  if (!sbp_msg_encode_to_buffer(get_SbpMsgObs)) {
    return;
  }
  if (!sbp_msg_encode_to_buffer(get_SbpMsgObs_alt)) {
    return;
  }
  write_output(tc_name);
  fprintf(stderr, "Testcase generated (multi msg): %s\n", tc_name);
}

static void make_testcase_multi_msg_EphemerisGps(const char *tc_name) {
  reset_output_buf();

  if (!sbp_msg_encode_to_buffer(get_SbpMsgEphemerisGps)) {
    return;
  }
  if (!sbp_msg_encode_to_buffer(get_SbpMsgEphemerisGps_alt)) {
    return;
  }
  write_output(tc_name);
  fprintf(stderr, "Testcase generated (multi msg): %s\n", tc_name);
}

static void make_testcase_multi_msg_EphemerisBds(const char *tc_name) {
  reset_output_buf();

  if (!sbp_msg_encode_to_buffer(get_SbpMsgEphemerisBds)) {
    return;
  }
  if (!sbp_msg_encode_to_buffer(get_SbpMsgEphemerisBds_alt)) {
    return;
  }
  write_output(tc_name);
  fprintf(stderr, "Testcase generated (multi msg): %s\n", tc_name);
}

static void make_testcase_multi_msg_EphemerisGal(const char *tc_name) {
  reset_output_buf();

  if (!sbp_msg_encode_to_buffer(get_SbpMsgEphemerisGal)) {
    return;
  }
  if (!sbp_msg_encode_to_buffer(get_SbpMsgEphemerisGal_alt)) {
    return;
  }
  write_output(tc_name);
  fprintf(stderr, "Testcase generated (multi msg): %s\n", tc_name);
}

static void make_testcase_multi_msg_EphemerisGlo(const char *tc_name) {
  reset_output_buf();

  if (!sbp_msg_encode_to_buffer(get_SbpMsgEphemerisGlo)) {
    return;
  }
  if (!sbp_msg_encode_to_buffer(get_SbpMsgEphemerisGlo_alt)) {
    return;
  }
  write_output(tc_name);
  fprintf(stderr, "Testcase generated (multi msg): %s\n", tc_name);
}

static void make_testcase_multi_msg_EphemerisQzss(const char *tc_name) {
  reset_output_buf();

  if (!sbp_msg_encode_to_buffer(get_SbpMsgEphemerisQzss)) {
    return;
  }
  if (!sbp_msg_encode_to_buffer(get_SbpMsgEphemerisQzss_alt)) {
    return;
  }
  write_output(tc_name);
  fprintf(stderr, "Testcase generated (multi msg): %s\n", tc_name);
}

void make_sbp_decoder_testcases(void) {
  struct sbp_single_msg_tc {
    const char *tc_name;
    void (*get_sbp_msg)(sbp_msg_t *, sbp_msg_type_t *, uint16_t *);
  } testcases[] = {
      // Acquisition msgs
      {"SbpMsgAcqResultDepA.sbp", get_SbpMsgAcqResultDepA},
      {"SbpMsgAcqResultDepB.sbp", get_SbpMsgAcqResultDepB},
      {"SbpMsgAcqResultDepC.sbp", get_SbpMsgAcqResultDepC},

      {"SbpMsgBootloaderHandshakeResp.sbp", get_SbpMsgBootloaderHandshakeResp},
      {"SbpMsgExtEvent.sbp", get_SbpMsgExtEvent},
      {"SbpMsgFileioWriteResp.sbp", get_SbpMsgFileioWriteResp},

      // IMU msgs
      {"SbpMsgImuAux.sbp", get_SbpMsgImuAux},
      {"SbpMsgImuRaw.sbp", get_SbpMsgImuRaw},

      // Integrity msgs
      {"SbpMsgSsrFlagHighLevel.sbp", get_SbpMsgSsrFlagHighLevel},
      {"SbpMsgSsrFlagIonoGridPoints.sbp", get_SbpMsgSsrFlagIonoGridPoints},
      {"SbpMsgSsrFlagIonoGridPointSatLos.sbp",
       get_SbpMsgSsrFlagIonoGridPointSatLos},
      {"SbpMsgSsrFlagIonoTileSatLos.sbp", get_SbpMsgSsrFlagIonoTileSatLos},
      {"SbpMsgSsrFlagSatellites.sbp", get_SbpMsgSsrFlagSatellites},
      {"SbpMsgSsrFlagTropoGridPoints.sbp", get_SbpMsgSsrFlagTropoGridPoints},

      // Logging msgs
      {"SbpMsgFwd.sbp", get_SbpMsgFwd},
      {"SbpMsgLog.sbp", get_SbpMsgLog},
      {"SbpMsgPrintDep.sbp", get_SbpMsgPrintDep},

      // Navigation msgs
      {"SbpMsgAgeCorrections.sbp", get_SbpMsgAgeCorrections},
      {"SbpMsgBaselineEcef.sbp", get_SbpMsgBaselineEcef},
      {"SbpMsgBaselineEcefDepA.sbp", get_SbpMsgBaselineEcefDepA},
      {"SbpMsgBaselineNed.sbp", get_SbpMsgBaselineNed},
      {"SbpMsgBaselineNedDepA.sbp", get_SbpMsgBaselineNedDepA},
      {"SbpMsgDops.sbp", get_SbpMsgDops},
      {"SbpMsgDopsDepA.sbp", get_SbpMsgDopsDepA},
      {"SbpMsgGpsTime.sbp", get_SbpMsgGpsTime},
      {"SbpMsgGpsTimeDepA.sbp", get_SbpMsgGpsTimeDepA},
      {"SbpMsgGpsTimeGnss.sbp", get_SbpMsgGpsTimeGnss},
      {"SbpMsgPosEcef.sbp", get_SbpMsgPosEcef},
      {"SbpMsgPosEcefCov.sbp", get_SbpMsgPosEcefCov},
      {"SbpMsgPosEcefCovGnss.sbp", get_SbpMsgPosEcefCovGnss},
      {"SbpMsgPosEcefDepA.sbp", get_SbpMsgPosEcefDepA},
      {"SbpMsgPosEcefGnss.sbp", get_SbpMsgPosEcefGnss},
      {"SbpMsgPosLlh.sbp", get_SbpMsgPosLlh},
      {"SbpMsgPosLlhCov.sbp", get_SbpMsgPosLlhCov},
      {"SbpMsgPosLlhCovGnss.sbp", get_SbpMsgPosLlhCovGnss},
      {"SbpMsgPosLlhDepA.sbp", get_SbpMsgPosLlhDepA},
      {"SbpMsgPosLlhGnss.sbp", get_SbpMsgPosLlhGnss},
      {"SbpMsgProtectionLevelDepA.sbp", get_SbpMsgProtectionLevelDepA},
      {"SbpMsgReferenceFrameParam.sbp", get_SbpMsgReferenceFrameParam},
      {"SbpMsgUtcLeapSecond.sbp", get_SbpMsgUtcLeapSecond},
      {"SbpMsgUtcTime.sbp", get_SbpMsgUtcTime},
      {"SbpMsgUtcTimeGnss.sbp", get_SbpMsgUtcTimeGnss},
      {"SbpMsgVelBody.sbp", get_SbpMsgVelBody},
      {"SbpMsgVelCog.sbp", get_SbpMsgVelCog},
      {"SbpMsgVelEcef.sbp", get_SbpMsgVelEcef},
      {"SbpMsgVelEcefCov.sbp", get_SbpMsgVelEcefCov},
      {"SbpMsgVelEcefCovGnss.sbp", get_SbpMsgVelEcefCovGnss},
      {"SbpMsgVelEcefDepA.sbp", get_SbpMsgVelEcefDepA},
      {"SbpMsgVelEcefGnss.sbp", get_SbpMsgVelEcefGnss},
      {"SbpMsgVelNed.sbp", get_SbpMsgVelNed},
      {"SbpMsgVelNedCov.sbp", get_SbpMsgVelNedCov},
      {"SbpMsgVelNedCovGnss.sbp", get_SbpMsgVelNedCovGnss},
      {"SbpMsgVelNedDepA.sbp", get_SbpMsgVelNedDepA},
      {"SbpMsgVelNedGnss.sbp", get_SbpMsgVelNedGnss},

      // Observation msgs
      {"SbpMsgBasePosEcef.sbp", get_SbpMsgBasePosEcef},
      {"SbpMsgEphemerisBds.sbp", get_SbpMsgEphemerisBds},
      {"SbpMsgEphemerisGal.sbp", get_SbpMsgEphemerisGal},
      {"SbpMsgEphemerisGlo.sbp", get_SbpMsgEphemerisGlo},
      {"SbpMsgEphemerisGps.sbp", get_SbpMsgEphemerisGps},
      {"SbpMsgEphemerisQzss.sbp", get_SbpMsgEphemerisQzss},
      {"SbpMsgGloBiases.sbp", get_SbpMsgGloBiases},
      {"SbpMsgObs.sbp", get_SbpMsgObs},
      {"SbpMsgObsDepA.sbp", get_SbpMsgObsDepA},
      {"SbpMsgObsDepB.sbp", get_SbpMsgObsDepB},
      {"SbpMsgObsDepC.sbp", get_SbpMsgObsDepC},
      {"SbpMsgOsr.sbp", get_SbpMsgOsr},
      {"SbpMsgSvAzEl.sbp", get_SbpMsgSvAzEl},

      // Orientation msgs
      {"SbpMsgAngularRate.sbp", get_SbpMsgAngularRate},
      {"SbpMsgOrientEuler.sbp", get_SbpMsgOrientEuler},
      {"SbpMsgOrientQuat.sbp", get_SbpMsgOrientQuat},

      // Piksi msgs
      {"SbpMsgDeviceMonitor.sbp", get_SbpMsgDeviceMonitor},
      {"SbpMsgIarState.sbp", get_SbpMsgIarState},
      {"SbpMsgNetworkBandwidthUsage.sbp", get_SbpMsgNetworkBandwidthUsage},
      {"SbpMsgThreadState.sbp", get_SbpMsgThreadState},
      {"SbpMsgUartStateDepa.sbp", get_SbpMsgUartStateDepa},

      // sbas & setting msgs
      {"SbpMsgSbasRaw.sbp", get_SbpMsgSbasRaw},
      {"SbpMsgSettingsReadByIndexDone.sbp", get_SbpMsgSettingsReadByIndexDone},
      {"SbpMsgSettingsReadByIndexResp.sbp", get_SbpMsgSettingsReadByIndexResp},

      // Ssr msgs
      {"SbpMsgSsrCodePhaseBiasesBounds.sbp",
       get_SbpMsgSsrCodePhaseBiasesBounds},
      {"SbpMsgSsrGriddedCorrectionBounds.sbp",
       get_SbpMsgSsrGriddedCorrectionBounds},
      {"SbpMsgSsrOrbitClockBounds.sbp", get_SbpMsgSsrOrbitClockBounds},
      {"SbpMsgSsrOrbitClockBoundsDegradation.sbp",
       get_SbpMsgSsrOrbitClockBoundsDegradation},
      {"SbpMsgSsrStecCorrection.sbp", get_SbpMsgSsrStecCorrection},
      {"SbpMsgSsrTileDefinition.sbp", get_SbpMsgSsrTileDefinition},

      // System msgs
      {"SbpMsgDgnssStatus.sbp", get_SbpMsgDgnssStatus},
      {"SbpMsgGroupMeta.sbp", get_SbpMsgGroupMeta},
      {"SbpMsgHeartbeat.sbp", get_SbpMsgHeartbeat},
      {"SbpMsgInsStatus.sbp", get_SbpMsgInsStatus},
      {"SbpMsgInsUpdates.sbp", get_SbpMsgInsUpdates},
      {"SbpMsgSensorAidEvent.sbp", get_SbpMsgSensorAidEvent},
      {"SbpMsgStartup.sbp", get_SbpMsgStartup},
      {"SbpMsgStatusJournal.sbp", get_SbpMsgStatusJournal},

      // Tracking
      {"SbpMsgMeasurementState.sbp", get_SbpMsgMeasurementState},
      {"SbpMsgTrackingStateDepB.sbp", get_SbpMsgTrackingStateDepB},
      {"SbpMsgTrackingStateDepA.sbp", get_SbpMsgTrackingStateDepA},
      {"SbpMsgTrackingStateDetailedDep.sbp",
       get_SbpMsgTrackingStateDetailedDep},

      // Vehicle msgs
      {"SbpMsgOdometry.sbp", get_SbpMsgOdometry},
  };

  for (size_t i = 0; i < ARRAY_SIZE(testcases); i++) {
    make_testcase_single_msg(testcases[i].tc_name, testcases[i].get_sbp_msg);
  }

  make_testcase_multi_msg_obs("SbpMsgObs_multi_msg.sbp");
  make_testcase_multi_msg_EphemerisGps("SbpMsgEphemerisGps_multi_msg.sbp");
  make_testcase_multi_msg_EphemerisBds("SbpMsgEphemerisBds_multi_msg.sbp");
  make_testcase_multi_msg_EphemerisGal("SbpMsgEphemerisGal_multi_msg.sbp");
  make_testcase_multi_msg_EphemerisGlo("SbpMsgEphemerisGlo_multi_msg.sbp");
  make_testcase_multi_msg_EphemerisQzss("SbpMsgEphemerisQzss_multi_msg.sbp");
}
