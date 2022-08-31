#ifndef AFL_HELPERS_SBP_SBP_DATA_H
#define AFL_HELPERS_SBP_SBP_DATA_H

#include <libsbp/edc.h>
#include <libsbp/sbp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <swiftnav/bits.h>
#include <swiftnav/edc.h>

/// Functions to create sbp testcases
void get_SbpMsgBasePosEcef(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgObs(sbp_msg_t *msg,
                   sbp_msg_type_t *msg_type,
                   uint16_t *sender_id);

void get_SbpMsgObs_alt(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id);

void get_SbpMsgAcqResultDepA(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id);

void get_SbpMsgAcqResultDepB(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id);

void get_SbpMsgAcqResultDepC(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id);

void get_SbpMsgBootloaderHandshakeResp(sbp_msg_t *msg,
                                       sbp_msg_type_t *msg_type,
                                       uint16_t *sender_id);

void get_SbpMsgExtEvent(sbp_msg_t *msg,
                        sbp_msg_type_t *msg_type,
                        uint16_t *sender_id);

void get_SbpMsgFileioWriteResp(sbp_msg_t *msg,
                               sbp_msg_type_t *msg_type,
                               uint16_t *sender_id);

void get_SbpMsgImuAux(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id);

void get_SbpMsgImuRaw(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id);

void get_SbpMsgSsrFlagHighLevel(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id);

void get_SbpMsgSsrFlagIonoGridPoints(sbp_msg_t *msg,
                                     sbp_msg_type_t *msg_type,
                                     uint16_t *sender_id);

void get_SbpMsgSsrFlagIonoGridPointSatLos(sbp_msg_t *msg,
                                          sbp_msg_type_t *msg_type,
                                          uint16_t *sender_id);

void get_SbpMsgSsrFlagIonoTileSatLos(sbp_msg_t *msg,
                                     sbp_msg_type_t *msg_type,
                                     uint16_t *sender_id);

void get_SbpMsgSsrFlagSatellites(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id);

void get_SbpMsgSsrFlagTropoGridPoints(sbp_msg_t *msg,
                                      sbp_msg_type_t *msg_type,
                                      uint16_t *sender_id);

void get_SbpMsgFwd(sbp_msg_t *msg,
                   sbp_msg_type_t *msg_type,
                   uint16_t *sender_id);

void get_SbpMsgLog(sbp_msg_t *msg,
                   sbp_msg_type_t *msg_type,
                   uint16_t *sender_id);

void get_SbpMsgPrintDep(sbp_msg_t *msg,
                        sbp_msg_type_t *msg_type,
                        uint16_t *sender_id);

void get_SbpMsgAgeCorrections(sbp_msg_t *msg,
                              sbp_msg_type_t *msg_type,
                              uint16_t *sender_id);

void get_SbpMsgBaselineEcef(sbp_msg_t *msg,
                            sbp_msg_type_t *msg_type,
                            uint16_t *sender_id);

void get_SbpMsgBaselineEcefDepA(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id);

void get_SbpMsgBaselineNed(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgBaselineNedDepA(sbp_msg_t *msg,
                               sbp_msg_type_t *msg_type,
                               uint16_t *sender_id);

void get_SbpMsgDops(sbp_msg_t *msg,
                    sbp_msg_type_t *msg_type,
                    uint16_t *sender_id);

void get_SbpMsgDopsDepA(sbp_msg_t *msg,
                        sbp_msg_type_t *msg_type,
                        uint16_t *sender_id);

void get_SbpMsgGpsTime(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id);

void get_SbpMsgGpsTimeDepA(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgGpsTimeGnss(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgPosEcef(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id);

void get_SbpMsgPosEcefCov(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id);

void get_SbpMsgPosEcefCovGnss(sbp_msg_t *msg,
                              sbp_msg_type_t *msg_type,
                              uint16_t *sender_id);

void get_SbpMsgPosEcefDepA(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgPosEcefGnss(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgPosLlh(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id);

void get_SbpMsgPosLlhCov(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id);

void get_SbpMsgPosLlhCovGnss(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id);

void get_SbpMsgPosLlhDepA(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id);

void get_SbpMsgPosLlhGnss(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id);

void get_SbpMsgProtectionLevelDepA(sbp_msg_t *msg,
                                   sbp_msg_type_t *msg_type,
                                   uint16_t *sender_id);

void get_SbpMsgReferenceFrameParam(sbp_msg_t *msg,
                                   sbp_msg_type_t *msg_type,
                                   uint16_t *sender_id);

void get_SbpMsgUtcLeapSecond(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id);

void get_SbpMsgUtcTime(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id);

void get_SbpMsgUtcTimeGnss(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgVelBody(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id);

void get_SbpMsgVelCog(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id);

void get_SbpMsgVelEcef(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id);

void get_SbpMsgVelEcefCov(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id);

void get_SbpMsgVelEcefCovGnss(sbp_msg_t *msg,
                              sbp_msg_type_t *msg_type,
                              uint16_t *sender_id);

void get_SbpMsgVelEcefDepA(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgVelEcefGnss(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgVelNed(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id);

void get_SbpMsgVelNedCov(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id);

void get_SbpMsgVelNedCovGnss(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id);

void get_SbpMsgVelNedDepA(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id);

void get_SbpMsgVelNedGnss(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id);

void get_SbpMsgEphemerisBds(sbp_msg_t *msg,
                            sbp_msg_type_t *msg_type,
                            uint16_t *sender_id);

void get_SbpMsgEphemerisBds_alt(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id);

void get_SbpMsgEphemerisGal(sbp_msg_t *msg,
                            sbp_msg_type_t *msg_type,
                            uint16_t *sender_id);

void get_SbpMsgEphemerisGal_alt(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id);

void get_SbpMsgEphemerisGlo(sbp_msg_t *msg,
                            sbp_msg_type_t *msg_type,
                            uint16_t *sender_id);

void get_SbpMsgEphemerisGlo_alt(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id);

void get_SbpMsgEphemerisGps(sbp_msg_t *msg,
                            sbp_msg_type_t *msg_type,
                            uint16_t *sender_id);

void get_SbpMsgEphemerisGps_alt(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id);

void get_SbpMsgEphemerisQzss(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id);

void get_SbpMsgEphemerisQzss_alt(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id);

void get_SbpMsgGloBiases(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id);

void get_SbpMsgObsDepA(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id);

void get_SbpMsgObsDepB(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id);

void get_SbpMsgObsDepC(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id);

void get_SbpMsgOsr(sbp_msg_t *msg,
                   sbp_msg_type_t *msg_type,
                   uint16_t *sender_id);

void get_SbpMsgSvAzEl(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id);

void get_SbpMsgAngularRate(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgOrientEuler(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgOrientQuat(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id);

void get_SbpMsgDeviceMonitor(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id);

void get_SbpMsgIarState(sbp_msg_t *msg,
                        sbp_msg_type_t *msg_type,
                        uint16_t *sender_id);

void get_SbpMsgNetworkBandwidthUsage(sbp_msg_t *msg,
                                     sbp_msg_type_t *msg_type,
                                     uint16_t *sender_id);

void get_SbpMsgThreadState(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgUartStateDepa(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id);

void get_SbpMsgSbasRaw(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id);

void get_SbpMsgSettingsReadByIndexDone(sbp_msg_t *msg,
                                       sbp_msg_type_t *msg_type,
                                       uint16_t *sender_id);

void get_SbpMsgSettingsReadByIndexResp(sbp_msg_t *msg,
                                       sbp_msg_type_t *msg_type,
                                       uint16_t *sender_id);

void get_SbpMsgSsrCodePhaseBiasesBounds(sbp_msg_t *msg,
                                        sbp_msg_type_t *msg_type,
                                        uint16_t *sender_id);

void get_SbpMsgSsrGriddedCorrectionBounds(sbp_msg_t *msg,
                                          sbp_msg_type_t *msg_type,
                                          uint16_t *sender_id);

void get_SbpMsgSsrOrbitClockBounds(sbp_msg_t *msg,
                                   sbp_msg_type_t *msg_type,
                                   uint16_t *sender_id);

void get_SbpMsgSsrOrbitClockBoundsDegradation(sbp_msg_t *msg,
                                              sbp_msg_type_t *msg_type,
                                              uint16_t *sender_id);

void get_SbpMsgSsrStecCorrection(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id);

void get_SbpMsgSsrTileDefinition(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id);

void get_SbpMsgDgnssStatus(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id);

void get_SbpMsgGroupMeta(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id);

void get_SbpMsgHeartbeat(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id);

void get_SbpMsgInsStatus(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id);

void get_SbpMsgInsUpdates(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id);

void get_SbpMsgSensorAidEvent(sbp_msg_t *msg,
                              sbp_msg_type_t *msg_type,
                              uint16_t *sender_id);

void get_SbpMsgStartup(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id);

void get_SbpMsgStatusJournal(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id);

void get_SbpMsgMeasurementState(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id);

void get_SbpMsgTrackingStateDepB(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id);

void get_SbpMsgTrackingStateDepA(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id);

void get_SbpMsgTrackingStateDetailedDep(sbp_msg_t *msg,
                                        sbp_msg_type_t *msg_type,
                                        uint16_t *sender_id);

void get_SbpMsgOdometry(sbp_msg_t *msg,
                        sbp_msg_type_t *msg_type,
                        uint16_t *sender_id);

#endif  // AFL_HELPERS_SBP_SBP_DATA_H
