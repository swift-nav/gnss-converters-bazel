#include <helpers/sbp/sbp_data.h>
#include <libsbp/sbp.h>
#include <stdio.h>
#include <string.h>
#include <swiftnav/bits.h>

#include "make_afl_testcases.h"

static void make_testcase_single_msg(const char *tc_name,
                                     void (*get_sbp_msg)(sbp_msg_t *,
                                                         sbp_msg_type_t *,
                                                         uint16_t *)) {
  FILE *fp;
  fp = fopen(tc_name, "w");

  // Get sbp message
  sbp_msg_t msg = {0};
  sbp_msg_type_t msg_type = SbpMsgUnknown;
  uint16_t sender_id = 0;
  get_sbp_msg(&msg, &msg_type, &sender_id);

  if (fwrite(&msg_type, 1, sizeof(msg_type), fp) != sizeof(msg_type)) {
    fprintf(stderr, "Error encoding message type %d.\n", msg_type);
    return;
  }
  if (fwrite(&sender_id, 1, sizeof(sender_id), fp) != sizeof(sender_id)) {
    fprintf(stderr, "Error encoding sender_id of msg_type %d.\n", msg_type);
    return;
  }
  if (fwrite(&msg, 1, sizeof(msg), fp) != sizeof(msg)) {
    fprintf(stderr, "Error encoding message of msg_type %d.\n", msg_type);
    return;
  }
  fclose(fp);

  fprintf(stderr, "Testcase generated: %s\n", tc_name);
}

void make_sbp_encoder_testcases(void) {
  struct sbp_single_msg_tc {
    const char *tc_name;
    void (*get_sbp_msg)(sbp_msg_t *, sbp_msg_type_t *, uint16_t *);
  } testcases[] = {
      // Acquisition msgs
      {"SbpMsgAcqResultDepA_cstruct.sbp", get_SbpMsgAcqResultDepA},
      {"SbpMsgAcqResultDepB_cstruct.sbp", get_SbpMsgAcqResultDepB},
      {"SbpMsgAcqResultDepC_cstruct.sbp", get_SbpMsgAcqResultDepC},

      {"SbpMsgBootloaderHandshakeResp_cstruct.sbp",
       get_SbpMsgBootloaderHandshakeResp},
      {"SbpMsgExtEvent_cstruct.sbp", get_SbpMsgExtEvent},
      {"SbpMsgFileioWriteResp_cstruct.sbp", get_SbpMsgFileioWriteResp},

      // IMU msgs
      {"SbpMsgImuAux_cstruct.sbp", get_SbpMsgImuAux},
      {"SbpMsgImuRaw_cstruct.sbp", get_SbpMsgImuRaw},

      // Integrity msgs
      {"SbpMsgSsrFlagHighLevel_cstruct.sbp", get_SbpMsgSsrFlagHighLevel},
      {"SbpMsgSsrFlagIonoGridPoints_cstruct.sbp",
       get_SbpMsgSsrFlagIonoGridPoints},
      {"SbpMsgSsrFlagIonoGridPointSatLos_cstruct.sbp",
       get_SbpMsgSsrFlagIonoGridPointSatLos},
      {"SbpMsgSsrFlagIonoTileSatLos_cstruct.sbp",
       get_SbpMsgSsrFlagIonoTileSatLos},
      {"SbpMsgSsrFlagSatellites_cstruct.sbp", get_SbpMsgSsrFlagSatellites},
      {"SbpMsgSsrFlagTropoGridPoints_cstruct.sbp",
       get_SbpMsgSsrFlagTropoGridPoints},

      // Logging msgs
      {"SbpMsgFwd_cstruct.sbp", get_SbpMsgFwd},
      {"SbpMsgLog_cstruct.sbp", get_SbpMsgLog},
      {"SbpMsgPrintDep_cstruct.sbp", get_SbpMsgPrintDep},

      // Navigation msgs
      {"SbpMsgAgeCorrections_cstruct.sbp", get_SbpMsgAgeCorrections},
      {"SbpMsgBaselineEcef_cstruct.sbp", get_SbpMsgBaselineEcef},
      {"SbpMsgBaselineEcefDepA_cstruct.sbp", get_SbpMsgBaselineEcefDepA},
      {"SbpMsgBaselineNed_cstruct.sbp", get_SbpMsgBaselineNed},
      {"SbpMsgBaselineNedDepA_cstruct.sbp", get_SbpMsgBaselineNedDepA},
      {"SbpMsgDops_cstruct.sbp", get_SbpMsgDops},
      {"SbpMsgDopsDepA_cstruct.sbp", get_SbpMsgDopsDepA},
      {"SbpMsgGpsTime_cstruct.sbp", get_SbpMsgGpsTime},
      {"SbpMsgGpsTimeDepA_cstruct.sbp", get_SbpMsgGpsTimeDepA},
      {"SbpMsgGpsTimeGnss_cstruct.sbp", get_SbpMsgGpsTimeGnss},
      {"SbpMsgPosEcef_cstruct.sbp", get_SbpMsgPosEcef},
      {"SbpMsgPosEcefCov_cstruct.sbp", get_SbpMsgPosEcefCov},
      {"SbpMsgPosEcefCovGnss_cstruct.sbp", get_SbpMsgPosEcefCovGnss},
      {"SbpMsgPosEcefDepA_cstruct.sbp", get_SbpMsgPosEcefDepA},
      {"SbpMsgPosEcefGnss_cstruct.sbp", get_SbpMsgPosEcefGnss},
      {"SbpMsgPosLlh_cstruct.sbp", get_SbpMsgPosLlh},
      {"SbpMsgPosLlhCov_cstruct.sbp", get_SbpMsgPosLlhCov},
      {"SbpMsgPosLlhCovGnss_cstruct.sbp", get_SbpMsgPosLlhCovGnss},
      {"SbpMsgPosLlhDepA_cstruct.sbp", get_SbpMsgPosLlhDepA},
      {"SbpMsgPosLlhGnss_cstruct.sbp", get_SbpMsgPosLlhGnss},
      {"SbpMsgProtectionLevelDepA_cstruct.sbp", get_SbpMsgProtectionLevelDepA},
      {"SbpMsgReferenceFrameParam_cstruct.sbp", get_SbpMsgReferenceFrameParam},
      {"SbpMsgUtcLeapSecond_cstruct.sbp", get_SbpMsgUtcLeapSecond},
      {"SbpMsgUtcTime_cstruct.sbp", get_SbpMsgUtcTime},
      {"SbpMsgUtcTimeGnss_cstruct.sbp", get_SbpMsgUtcTimeGnss},
      {"SbpMsgVelBody_cstruct.sbp", get_SbpMsgVelBody},
      {"SbpMsgVelCog_cstruct.sbp", get_SbpMsgVelCog},
      {"SbpMsgVelEcef_cstruct.sbp", get_SbpMsgVelEcef},
      {"SbpMsgVelEcefCov_cstruct.sbp", get_SbpMsgVelEcefCov},
      {"SbpMsgVelEcefCovGnss_cstruct.sbp", get_SbpMsgVelEcefCovGnss},
      {"SbpMsgVelEcefDepA_cstruct.sbp", get_SbpMsgVelEcefDepA},
      {"SbpMsgVelEcefGnss_cstruct.sbp", get_SbpMsgVelEcefGnss},
      {"SbpMsgVelNed_cstruct.sbp", get_SbpMsgVelNed},
      {"SbpMsgVelNedCov_cstruct.sbp", get_SbpMsgVelNedCov},
      {"SbpMsgVelNedCovGnss_cstruct.sbp", get_SbpMsgVelNedCovGnss},
      {"SbpMsgVelNedDepA_cstruct.sbp", get_SbpMsgVelNedDepA},
      {"SbpMsgVelNedGnss_cstruct.sbp", get_SbpMsgVelNedGnss},

      // Observation msgs
      {"SbpMsgBasePosEcef_cstruct.sbp", get_SbpMsgBasePosEcef},
      {"SbpMsgEphemerisBds_cstruct.sbp", get_SbpMsgEphemerisBds},
      {"SbpMsgEphemerisGal_cstruct.sbp", get_SbpMsgEphemerisGal},
      {"SbpMsgEphemerisGlo_cstruct.sbp", get_SbpMsgEphemerisGlo},
      {"SbpMsgEphemerisGps_cstruct.sbp", get_SbpMsgEphemerisGps},
      {"SbpMsgEphemerisQzss_cstruct.sbp", get_SbpMsgEphemerisQzss},
      {"SbpMsgGloBiases_cstruct.sbp", get_SbpMsgGloBiases},
      {"SbpMsgObs_cstruct.sbp", get_SbpMsgObs},
      {"SbpMsgObs_alt_cstruct.sbp", get_SbpMsgObs_alt},
      {"SbpMsgObsDepA_cstruct.sbp", get_SbpMsgObsDepA},
      {"SbpMsgObsDepB_cstruct.sbp", get_SbpMsgObsDepB},
      {"SbpMsgObsDepC_cstruct.sbp", get_SbpMsgObsDepC},
      {"SbpMsgOsr_cstruct.sbp", get_SbpMsgOsr},
      {"SbpMsgSvAzEl_cstruct.sbp", get_SbpMsgSvAzEl},

      // Orientation msgs
      {"SbpMsgAngularRate_cstruct.sbp", get_SbpMsgAngularRate},
      {"SbpMsgOrientEuler_cstruct.sbp", get_SbpMsgOrientEuler},
      {"SbpMsgOrientQuat_cstruct.sbp", get_SbpMsgOrientQuat},

      // Piksi msgs
      {"SbpMsgDeviceMonitor_cstruct.sbp", get_SbpMsgDeviceMonitor},
      {"SbpMsgIarState_cstruct.sbp", get_SbpMsgIarState},
      {"SbpMsgNetworkBandwidthUsage_cstruct.sbp",
       get_SbpMsgNetworkBandwidthUsage},
      {"SbpMsgThreadState_cstruct.sbp", get_SbpMsgThreadState},
      {"SbpMsgUartStateDepa_cstruct.sbp", get_SbpMsgUartStateDepa},

      // sbas & setting msgs
      {"SbpMsgSbasRaw_cstruct.sbp", get_SbpMsgSbasRaw},
      {"SbpMsgSettingsReadByIndexDone_cstruct.sbp",
       get_SbpMsgSettingsReadByIndexDone},
      {"SbpMsgSettingsReadByIndexResp_cstruct.sbp",
       get_SbpMsgSettingsReadByIndexResp},

      // Ssr msgs
      {"SbpMsgSsrCodePhaseBiasesBounds_cstruct.sbp",
       get_SbpMsgSsrCodePhaseBiasesBounds},
      {"SbpMsgSsrGriddedCorrectionBounds_cstruct.sbp",
       get_SbpMsgSsrGriddedCorrectionBounds},
      {"SbpMsgSsrOrbitClockBounds_cstruct.sbp", get_SbpMsgSsrOrbitClockBounds},
      {"SbpMsgSsrOrbitClockBoundsDegradation_cstruct.sbp",
       get_SbpMsgSsrOrbitClockBoundsDegradation},
      {"SbpMsgSsrStecCorrection_cstruct.sbp", get_SbpMsgSsrStecCorrection},
      {"SbpMsgSsrTileDefinition_cstruct.sbp", get_SbpMsgSsrTileDefinition},

      // System msgs
      {"SbpMsgDgnssStatus_cstruct.sbp", get_SbpMsgDgnssStatus},
      {"SbpMsgGroupMeta_cstruct.sbp", get_SbpMsgGroupMeta},
      {"SbpMsgHeartbeat_cstruct.sbp", get_SbpMsgHeartbeat},
      {"SbpMsgInsStatus_cstruct.sbp", get_SbpMsgInsStatus},
      {"SbpMsgInsUpdates_cstruct.sbp", get_SbpMsgInsUpdates},
      {"SbpMsgSensorAidEvent_cstruct.sbp", get_SbpMsgSensorAidEvent},
      {"SbpMsgStartup_cstruct.sbp", get_SbpMsgStartup},
      {"SbpMsgStatusJournal_cstruct.sbp", get_SbpMsgStatusJournal},

      // Tracking
      {"SbpMsgMeasurementState_cstruct.sbp", get_SbpMsgMeasurementState},
      {"SbpMsgTrackingStateDepB_cstruct.sbp", get_SbpMsgTrackingStateDepB},
      {"SbpMsgTrackingStateDepA_cstruct.sbp", get_SbpMsgTrackingStateDepA},
      {"SbpMsgTrackingStateDetailedDep_cstruct.sbp",
       get_SbpMsgTrackingStateDetailedDep},

      // Vehicle msgs
      {"SbpMsgOdometry_cstruct.sbp", get_SbpMsgOdometry},
  };

  for (size_t i = 0; i < ARRAY_SIZE(testcases); i++) {
    make_testcase_single_msg(testcases[i].tc_name, testcases[i].get_sbp_msg);
  }
}
