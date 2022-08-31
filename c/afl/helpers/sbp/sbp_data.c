#include "sbp_data.h"

#include <libsbp/edc.h>
#include <libsbp/sbp.h>
#include <stdio.h>
#include <string.h>
#include <swiftnav/bits.h>
#include <swiftnav/edc.h>

/// Functions to create sbp testcases
void get_SbpMsgBasePosEcef(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 51228;
  *msg_type = SbpMsgBasePosEcef;

  msg->base_pos_ecef.x = -4130728.7735;
  msg->base_pos_ecef.y = 2898701.207;
  msg->base_pos_ecef.z = -3887601.0391;
}

void get_SbpMsgObs(sbp_msg_t *msg,
                   sbp_msg_type_t *msg_type,
                   uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 51228;
  *msg_type = SbpMsgObs;

  msg->obs.header.t.tow = 180063200;
  msg->obs.header.t.ns_residual = 0;
  msg->obs.header.t.wn = 2222;
  msg->obs.header.n_obs = 32;

  sbp_packed_obs_content_t obs_data[] = {
      {1201891447, {126319649, 16}, {-612, 76}, 102, 6, 15, {5, 0}},
      {1053365747, {110709569, 79}, {-1741, 82}, 135, 12, 15, {7, 0}},
      {1147751694, {120629454, 196}, {-1754, 135}, 116, 9, 15, {8, 0}},
      {1180171219, {124036812, 140}, {-3526, 146}, 177, 15, 15, {9, 0}},
      {1180169822, {92624761, 71}, {-2633, 33}, 146, 15, 15, {9, 10}},
      {1153924003, {0, 0}, {3118, 163}, 88, 0, 9, {13, 0}},
      {1028299222, {0, 0}, {1066, 76}, 86, 0, 9, {14, 0}},
      {1189240153, {124990017, 213}, {3872, 149}, 164, 15, 15, {17, 0}},
      {1028988831, {108147581, 44}, {235, 52}, 100, 6, 11, {30, 0}},
      {1176153384, {0, 0}, {-16, 8}, 78, 0, 9, {1, 16}},
      {1204761822, {0, 0}, {405, 175}, 96, 0, 9, {4, 16}},
      {1366357841, {0, 0}, {2585, 76}, 75, 0, 9, {9, 16}},
      {1401365369, {147284447, 129}, {1739, 199}, 173, 15, 15, {13, 16}},
      {1318549500, {138580606, 174}, {-2465, 34}, 134, 10, 11, {19, 16}}};

  msg->obs.n_obs = ARRAY_SIZE(obs_data);
  for (size_t i = 0; i < msg->obs.n_obs; i++) {
    msg->obs.obs[i].P = obs_data[i].P;
    msg->obs.obs[i].L.i = obs_data[i].L.i;
    msg->obs.obs[i].L.f = obs_data[i].L.f;
    msg->obs.obs[i].D.i = obs_data[i].D.i;
    msg->obs.obs[i].D.f = obs_data[i].D.f;
    msg->obs.obs[i].cn0 = obs_data[i].cn0;
    msg->obs.obs[i].lock = obs_data[i].lock;
    msg->obs.obs[i].flags = obs_data[i].flags;
    msg->obs.obs[i].sid.sat = obs_data[i].sid.sat;
    msg->obs.obs[i].sid.code = obs_data[i].sid.code;
  }
}

void get_SbpMsgObs_alt(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 51228;
  *msg_type = SbpMsgObs;

  msg->obs.header.t.tow = 180063200;
  msg->obs.header.t.ns_residual = 0;
  msg->obs.header.t.wn = 2222;
  msg->obs.header.n_obs = 33;

  sbp_packed_obs_content_t obs_data[] = {
      {1318547419, {103485267, 204}, {-1841, 175}, 98, 7, 15, {19, 27}},
      {1402034134, {147354741, 149}, {-2958, 174}, 130, 10, 11, {20, 16}},
      {1260644859, {0, 0}, {-1936, 52}, 90, 0, 9, {21, 16}},
      {1316512769, {0, 0}, {2032, 159}, 89, 0, 9, {31, 16}},
      {1860807766, {193794283, 168}, {-11, 204}, 145, 7, 15, {1, 12}},
      {1880217950, {195815771, 102}, {6, 143}, 139, 6, 15, {4, 12}},
      {1806985653, {188188969, 202}, {413, 6}, 96, 4, 15, {16, 12}},
      {1232792378, {128389522, 63}, {-2115, 122}, 148, 14, 15, {25, 12}},
      {1232798444, {96755306, 243}, {-1594, 250}, 118, 13, 15, {25, 48}},
  };

  msg->obs.n_obs = ARRAY_SIZE(obs_data);
  for (size_t i = 0; i < msg->obs.n_obs; i++) {
    msg->obs.obs[i].P = obs_data[i].P;
    msg->obs.obs[i].L.i = obs_data[i].L.i;
    msg->obs.obs[i].L.f = obs_data[i].L.f;
    msg->obs.obs[i].D.i = obs_data[i].D.i;
    msg->obs.obs[i].D.f = obs_data[i].D.f;
    msg->obs.obs[i].cn0 = obs_data[i].cn0;
    msg->obs.obs[i].lock = obs_data[i].lock;
    msg->obs.obs[i].flags = obs_data[i].flags;
    msg->obs.obs[i].sid.sat = obs_data[i].sid.sat;
    msg->obs.obs[i].sid.code = obs_data[i].sid.code;
  }
}

void get_SbpMsgAcqResultDepA(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 51228;
  *msg_type = SbpMsgAcqResultDepA;

  msg->acq_result_dep_a.cf = 8241.943359375;
  msg->acq_result_dep_a.cp = 727.0;
  msg->acq_result_dep_a.prn = 8;
  msg->acq_result_dep_a.snr = 14.5;
}

void get_SbpMsgAcqResultDepB(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgAcqResultDepB;

  msg->acq_result_dep_b.cf = 4995.1171875;
  msg->acq_result_dep_b.cp = 322.0;
  msg->acq_result_dep_b.sid.code = 0;
  msg->acq_result_dep_b.sid.reserved = 0;
  msg->acq_result_dep_b.sid.sat = 9;
  msg->acq_result_dep_b.snr = 36.66360855102539;
}

void get_SbpMsgAcqResultDepC(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 3112;
  *msg_type = SbpMsgAcqResultDepC;

  msg->acq_result_dep_c.cf = 1769.06591796875;
  msg->acq_result_dep_c.cn0 = 40.509063720703125;
  msg->acq_result_dep_c.cp = 457.1922302246094;
  msg->acq_result_dep_c.sid.code = 0;
  msg->acq_result_dep_c.sid.reserved = 0;
  msg->acq_result_dep_c.sid.sat = 10;
}

void get_SbpMsgBootloaderHandshakeResp(sbp_msg_t *msg,
                                       sbp_msg_type_t *msg_type,
                                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 0;
  *msg_type = SbpMsgBootloaderHandshakeResp;

  sbp_msg_bootloader_handshake_resp_version_set(
      &msg->bootloader_handshake_resp, "v1.2\n", false, NULL);
}

void get_SbpMsgExtEvent(sbp_msg_t *msg,
                        sbp_msg_type_t *msg_type,
                        uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 1781;
  *msg_type = SbpMsgExtEvent;

  msg->ext_event.flags = 3;
  msg->ext_event.ns_residual = 999882;
  msg->ext_event.pin = 0;
  msg->ext_event.tow = 254924999;
  msg->ext_event.wn = 1840;
}

void get_SbpMsgFileioWriteResp(sbp_msg_t *msg,
                               sbp_msg_type_t *msg_type,
                               uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgFileioWriteResp;

  msg->fileio_write_resp.sequence = 202;
}

void get_SbpMsgImuAux(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 4660;
  *msg_type = SbpMsgImuAux;

  msg->imu_aux.imu_conf = 66;
  msg->imu_aux.imu_type = 1;
  msg->imu_aux.temp = 2804;
}

void get_SbpMsgImuRaw(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 4660;
  *msg_type = SbpMsgImuRaw;

  msg->imu_raw.acc_x = 96;
  msg->imu_raw.acc_y = -33;
  msg->imu_raw.acc_z = 4140;
  msg->imu_raw.gyr_x = 60;
  msg->imu_raw.gyr_y = -304;
  msg->imu_raw.gyr_z = -18;
  msg->imu_raw.tow = 3221225754;
  msg->imu_raw.tow_f = 206;
}

void get_SbpMsgSsrFlagHighLevel(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrFlagHighLevel;

  msg->ssr_flag_high_level.chain_id = 40;
  msg->ssr_flag_high_level.corr_time.tow = 360;
  msg->ssr_flag_high_level.corr_time.wn = 6;
  msg->ssr_flag_high_level.obs_time.tow = 180;
  msg->ssr_flag_high_level.obs_time.wn = 3;
  msg->ssr_flag_high_level.reserved[0] = 0;
  msg->ssr_flag_high_level.reserved[1] = 0;
  msg->ssr_flag_high_level.reserved[2] = 0;
  msg->ssr_flag_high_level.reserved[3] = 0;
  msg->ssr_flag_high_level.reserved[4] = 0;
  msg->ssr_flag_high_level.reserved[5] = 0;
  msg->ssr_flag_high_level.ssr_sol_id = 10;
  msg->ssr_flag_high_level.tile_id = 30;
  msg->ssr_flag_high_level.tile_set_id = 20;
  msg->ssr_flag_high_level.use_bds_sat = 3;
  msg->ssr_flag_high_level.use_gal_sat = 2;
  msg->ssr_flag_high_level.use_gps_sat = 1;
  msg->ssr_flag_high_level.use_iono_grid_point_sat_los = 7;
  msg->ssr_flag_high_level.use_iono_grid_points = 5;
  msg->ssr_flag_high_level.use_iono_tile_sat_los = 6;
  msg->ssr_flag_high_level.use_tropo_grid_points = 4;
}

void get_SbpMsgSsrFlagIonoGridPoints(sbp_msg_t *msg,
                                     sbp_msg_type_t *msg_type,
                                     uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrFlagIonoGridPoints;

  msg->ssr_flag_iono_grid_points.faulty_points[0] = 10;
  msg->ssr_flag_iono_grid_points.faulty_points[1] = 11;
  msg->ssr_flag_iono_grid_points.faulty_points[2] = 12;
  msg->ssr_flag_iono_grid_points.header.chain_id = 6;
  msg->ssr_flag_iono_grid_points.header.num_msgs = 1;
  msg->ssr_flag_iono_grid_points.header.obs_time.tow = 180;
  msg->ssr_flag_iono_grid_points.header.obs_time.wn = 3;
  msg->ssr_flag_iono_grid_points.header.seq_num = 2;
  msg->ssr_flag_iono_grid_points.header.ssr_sol_id = 3;
  msg->ssr_flag_iono_grid_points.header.tile_id = 5;
  msg->ssr_flag_iono_grid_points.header.tile_set_id = 4;
  msg->ssr_flag_iono_grid_points.n_faulty_points = 3;
}

void get_SbpMsgSsrFlagIonoGridPointSatLos(sbp_msg_t *msg,
                                          sbp_msg_type_t *msg_type,
                                          uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrFlagIonoGridPointSatLos;

  msg->ssr_flag_iono_grid_point_sat_los.faulty_los[0].constellation = 11;
  msg->ssr_flag_iono_grid_point_sat_los.faulty_los[0].satId = 10;
  msg->ssr_flag_iono_grid_point_sat_los.faulty_los[1].constellation = 14;
  msg->ssr_flag_iono_grid_point_sat_los.faulty_los[1].satId = 15;
  msg->ssr_flag_iono_grid_point_sat_los.grid_point_id = 30;
  msg->ssr_flag_iono_grid_point_sat_los.header.chain_id = 6;
  msg->ssr_flag_iono_grid_point_sat_los.header.num_msgs = 1;
  msg->ssr_flag_iono_grid_point_sat_los.header.obs_time.tow = 180;
  msg->ssr_flag_iono_grid_point_sat_los.header.obs_time.wn = 3;
  msg->ssr_flag_iono_grid_point_sat_los.header.seq_num = 2;
  msg->ssr_flag_iono_grid_point_sat_los.header.ssr_sol_id = 3;
  msg->ssr_flag_iono_grid_point_sat_los.header.tile_id = 5;
  msg->ssr_flag_iono_grid_point_sat_los.header.tile_set_id = 4;
  msg->ssr_flag_iono_grid_point_sat_los.n_faulty_los = 2;
}

void get_SbpMsgSsrFlagIonoTileSatLos(sbp_msg_t *msg,
                                     sbp_msg_type_t *msg_type,
                                     uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrFlagIonoTileSatLos;

  msg->ssr_flag_iono_tile_sat_los.faulty_los[0].constellation = 11;
  msg->ssr_flag_iono_tile_sat_los.faulty_los[0].satId = 10;
  msg->ssr_flag_iono_tile_sat_los.faulty_los[1].constellation = 14;
  msg->ssr_flag_iono_tile_sat_los.faulty_los[1].satId = 15;
  msg->ssr_flag_iono_tile_sat_los.header.chain_id = 6;
  msg->ssr_flag_iono_tile_sat_los.header.num_msgs = 1;
  msg->ssr_flag_iono_tile_sat_los.header.obs_time.tow = 180;
  msg->ssr_flag_iono_tile_sat_los.header.obs_time.wn = 3;
  msg->ssr_flag_iono_tile_sat_los.header.seq_num = 2;
  msg->ssr_flag_iono_tile_sat_los.header.ssr_sol_id = 3;
  msg->ssr_flag_iono_tile_sat_los.header.tile_id = 5;
  msg->ssr_flag_iono_tile_sat_los.header.tile_set_id = 4;
  msg->ssr_flag_iono_tile_sat_los.n_faulty_los = 2;
}

void get_SbpMsgSsrFlagSatellites(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrFlagSatellites;

  msg->ssr_flag_satellites.chain_id = 4;
  msg->ssr_flag_satellites.const_id = 5;
  msg->ssr_flag_satellites.faulty_sats[0] = 10;
  msg->ssr_flag_satellites.faulty_sats[1] = 11;
  msg->ssr_flag_satellites.faulty_sats[2] = 12;
  msg->ssr_flag_satellites.n_faulty_sats = 3;
  msg->ssr_flag_satellites.num_msgs = 1;
  msg->ssr_flag_satellites.obs_time.tow = 180;
  msg->ssr_flag_satellites.obs_time.wn = 3;
  msg->ssr_flag_satellites.seq_num = 2;
  msg->ssr_flag_satellites.ssr_sol_id = 3;
}

void get_SbpMsgSsrFlagTropoGridPoints(sbp_msg_t *msg,
                                      sbp_msg_type_t *msg_type,
                                      uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrFlagTropoGridPoints;

  msg->ssr_flag_tropo_grid_points.faulty_points[0] = 10;
  msg->ssr_flag_tropo_grid_points.faulty_points[1] = 11;
  msg->ssr_flag_tropo_grid_points.faulty_points[2] = 12;
  msg->ssr_flag_tropo_grid_points.header.chain_id = 6;
  msg->ssr_flag_tropo_grid_points.header.num_msgs = 1;
  msg->ssr_flag_tropo_grid_points.header.obs_time.tow = 180;
  msg->ssr_flag_tropo_grid_points.header.obs_time.wn = 3;
  msg->ssr_flag_tropo_grid_points.header.seq_num = 2;
  msg->ssr_flag_tropo_grid_points.header.ssr_sol_id = 3;
  msg->ssr_flag_tropo_grid_points.header.tile_id = 5;
  msg->ssr_flag_tropo_grid_points.header.tile_set_id = 4;
  msg->ssr_flag_tropo_grid_points.n_faulty_points = 3;
}

void get_SbpMsgFwd(sbp_msg_t *msg,
                   sbp_msg_type_t *msg_type,
                   uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgFwd;

  msg->fwd.fwd_payload[0] = 86;
  msg->fwd.fwd_payload[1] = 81;
  msg->fwd.fwd_payload[2] = 68;
  msg->fwd.fwd_payload[3] = 47;
  msg->fwd.fwd_payload[4] = 81;
  msg->fwd.fwd_payload[5] = 103;
  msg->fwd.fwd_payload[6] = 65;
  msg->fwd.fwd_payload[7] = 69;
  msg->fwd.fwd_payload[8] = 65;
  msg->fwd.fwd_payload[9] = 65;
  msg->fwd.fwd_payload[10] = 65;
  msg->fwd.fwd_payload[11] = 65;
  msg->fwd.fwd_payload[12] = 65;
  msg->fwd.fwd_payload[13] = 69;
  msg->fwd.fwd_payload[14] = 97;
  msg->fwd.fwd_payload[15] = 103;
  msg->fwd.n_fwd_payload = 16;
  msg->fwd.protocol = 0;
  msg->fwd.source = 0;
}

void get_SbpMsgLog(sbp_msg_t *msg,
                   sbp_msg_type_t *msg_type,
                   uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 2314;
  *msg_type = SbpMsgLog;

  msg->log.level = 6;
  sbp_msg_log_text_set(
      &msg->log, "Filtered all obs from 2314 at tow 83.539019", false, NULL);
}

void get_SbpMsgPrintDep(sbp_msg_t *msg,
                        sbp_msg_type_t *msg_type,
                        uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 8738;
  *msg_type = SbpMsgPrintDep;

  sbp_msg_print_dep_text_set(&msg->print_dep,
                             "INFO: acq: PRN 15 found @ -2497 Hz, 20 SNR\n",
                             false,
                             NULL);
}

void get_SbpMsgAgeCorrections(sbp_msg_t *msg,
                              sbp_msg_type_t *msg_type,
                              uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgAgeCorrections;

  msg->age_corrections.age = 30;
  msg->age_corrections.tow = 100;
}

void get_SbpMsgBaselineEcef(sbp_msg_t *msg,
                            sbp_msg_type_t *msg_type,
                            uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 35027;
  *msg_type = SbpMsgBaselineEcef;

  msg->baseline_ecef.accuracy = 0;
  msg->baseline_ecef.flags = 0;
  msg->baseline_ecef.n_sats = 14;
  msg->baseline_ecef.tow = 326825000;
  msg->baseline_ecef.x = -1154410;
  msg->baseline_ecef.y = 1327294;
  msg->baseline_ecef.z = 631798;
}

void get_SbpMsgBaselineEcefDepA(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgBaselineEcefDepA;

  msg->baseline_ecef_dep_a.accuracy = 0;
  msg->baseline_ecef_dep_a.flags = 1;
  msg->baseline_ecef_dep_a.n_sats = 9;
  msg->baseline_ecef_dep_a.tow = 2567700;
  msg->baseline_ecef_dep_a.x = -53227;
  msg->baseline_ecef_dep_a.y = -35532;
  msg->baseline_ecef_dep_a.z = -76840;
}

void get_SbpMsgBaselineNed(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 35027;
  *msg_type = SbpMsgBaselineNed;

  msg->baseline_ned.d = 32153;
  msg->baseline_ned.e = -1681229;
  msg->baseline_ned.flags = 0;
  msg->baseline_ned.h_accuracy = 0;
  msg->baseline_ned.n = 816073;
  msg->baseline_ned.n_sats = 14;
  msg->baseline_ned.tow = 326825000;
  msg->baseline_ned.v_accuracy = 0;
}

void get_SbpMsgBaselineNedDepA(sbp_msg_t *msg,
                               sbp_msg_type_t *msg_type,
                               uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgBaselineNedDepA;

  msg->baseline_ned_dep_a.d = 0;
  msg->baseline_ned_dep_a.e = -26134;
  msg->baseline_ned_dep_a.flags = 1;
  msg->baseline_ned_dep_a.h_accuracy = 0;
  msg->baseline_ned_dep_a.n = -96525;
  msg->baseline_ned_dep_a.n_sats = 9;
  msg->baseline_ned_dep_a.tow = 2567700;
  msg->baseline_ned_dep_a.v_accuracy = 0;
}

void get_SbpMsgDops(sbp_msg_t *msg,
                    sbp_msg_type_t *msg_type,
                    uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgDops;

  msg->dops.flags = 0;
  msg->dops.gdop = 2;
  msg->dops.hdop = 5;
  msg->dops.pdop = 6;
  msg->dops.tdop = 5;
  msg->dops.tow = 100;
  msg->dops.vdop = 5;
}

void get_SbpMsgDopsDepA(sbp_msg_t *msg,
                        sbp_msg_type_t *msg_type,
                        uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgDopsDepA;

  msg->dops_dep_a.gdop = 180;
  msg->dops_dep_a.hdop = 160;
  msg->dops_dep_a.pdop = 190;
  msg->dops_dep_a.tdop = 170;
  msg->dops_dep_a.tow = 2568200;
  msg->dops_dep_a.vdop = 150;
}

void get_SbpMsgGpsTime(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 35027;
  *msg_type = SbpMsgGpsTime;

  msg->gps_time.flags = 0;
  msg->gps_time.ns_residual = 166900;
  msg->gps_time.tow = 326825000;
  msg->gps_time.wn = 1920;
}

void get_SbpMsgGpsTimeDepA(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgGpsTimeDepA;

  msg->gps_time_dep_a.flags = 0;
  msg->gps_time_dep_a.ns_residual = 0;
  msg->gps_time_dep_a.tow = 2567800;
  msg->gps_time_dep_a.wn = 1787;
}

void get_SbpMsgGpsTimeGnss(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 35027;
  *msg_type = SbpMsgGpsTimeGnss;

  msg->gps_time_gnss.flags = 0;
  msg->gps_time_gnss.ns_residual = 166900;
  msg->gps_time_gnss.tow = 326825000;
  msg->gps_time_gnss.wn = 1920;
}

void get_SbpMsgPosEcef(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 35027;
  *msg_type = SbpMsgPosEcef;

  msg->pos_ecef.accuracy = 0;
  msg->pos_ecef.flags = 2;
  msg->pos_ecef.n_sats = 15;
  msg->pos_ecef.tow = 326826000;
  msg->pos_ecef.x = -2684269.0326572997;
  msg->pos_ecef.y = -4316646.751816;
  msg->pos_ecef.z = 3839646.7095350414;
}

void get_SbpMsgPosEcefCov(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgPosEcefCov;

  msg->pos_ecef_cov.cov_x_x = 8.0;
  msg->pos_ecef_cov.cov_x_y = 7.0;
  msg->pos_ecef_cov.cov_x_z = 2.0;
  msg->pos_ecef_cov.cov_y_y = 6.0;
  msg->pos_ecef_cov.cov_y_z = 8.0;
  msg->pos_ecef_cov.cov_z_z = 5.0;
  msg->pos_ecef_cov.flags = 5;
  msg->pos_ecef_cov.n_sats = 4;
  msg->pos_ecef_cov.tow = 7;
  msg->pos_ecef_cov.x = 6.0;
  msg->pos_ecef_cov.y = 1.0;
  msg->pos_ecef_cov.z = 4.0;
}

void get_SbpMsgPosEcefCovGnss(sbp_msg_t *msg,
                              sbp_msg_type_t *msg_type,
                              uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 4096;
  *msg_type = SbpMsgPosEcefCovGnss;

  msg->pos_ecef_cov_gnss.cov_x_x = 0.009699014946818352;
  msg->pos_ecef_cov_gnss.cov_x_y = 0.009086096659302711;
  msg->pos_ecef_cov_gnss.cov_x_z = -0.006058753002434969;
  msg->pos_ecef_cov_gnss.cov_y_y = 0.020321274176239967;
  msg->pos_ecef_cov_gnss.cov_y_z = -0.009988312609493732;
  msg->pos_ecef_cov_gnss.cov_z_z = 0.01487385667860508;
  msg->pos_ecef_cov_gnss.flags = 4;
  msg->pos_ecef_cov_gnss.n_sats = 18;
  msg->pos_ecef_cov_gnss.tow = 501867800;
  msg->pos_ecef_cov_gnss.x = -2694229.7079770807;
  msg->pos_ecef_cov_gnss.y = -4264073.427345817;
  msg->pos_ecef_cov_gnss.z = 3890655.013186158;
}

void get_SbpMsgPosEcefDepA(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgPosEcefDepA;

  msg->pos_ecef_dep_a.accuracy = 0;
  msg->pos_ecef_dep_a.flags = 0;
  msg->pos_ecef_dep_a.n_sats = 9;
  msg->pos_ecef_dep_a.tow = 2567700;
  msg->pos_ecef_dep_a.x = -2700354.5912927105;
  msg->pos_ecef_dep_a.y = -4292510.764041577;
  msg->pos_ecef_dep_a.z = 3855357.977260149;
}

void get_SbpMsgPosEcefGnss(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 4096;
  *msg_type = SbpMsgPosEcefGnss;

  msg->pos_ecef_gnss.accuracy = 182;
  msg->pos_ecef_gnss.flags = 4;
  msg->pos_ecef_gnss.n_sats = 18;
  msg->pos_ecef_gnss.tow = 501867800;
  msg->pos_ecef_gnss.x = -2694229.7079770807;
  msg->pos_ecef_gnss.y = -4264073.427345817;
  msg->pos_ecef_gnss.z = 3890655.013186158;
}

void get_SbpMsgPosLlh(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 35027;
  *msg_type = SbpMsgPosLlh;

  msg->pos_llh.flags = 2;
  msg->pos_llh.h_accuracy = 0;
  msg->pos_llh.height = 28.21160739227208;
  msg->pos_llh.lat = 37.25130398358085;
  msg->pos_llh.lon = -121.87505366879361;
  msg->pos_llh.n_sats = 14;
  msg->pos_llh.tow = 326825000;
  msg->pos_llh.v_accuracy = 0;
}

void get_SbpMsgPosLlhCov(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgPosLlhCov;

  msg->pos_llh_cov.cov_d_d = 2.0;
  msg->pos_llh_cov.cov_e_d = 1.0;
  msg->pos_llh_cov.cov_e_e = 6.0;
  msg->pos_llh_cov.cov_n_d = 8.0;
  msg->pos_llh_cov.cov_n_e = 5.0;
  msg->pos_llh_cov.cov_n_n = 7.0;
  msg->pos_llh_cov.flags = 5;
  msg->pos_llh_cov.height = 0.0;
  msg->pos_llh_cov.lat = 0.0;
  msg->pos_llh_cov.lon = 7.0;
  msg->pos_llh_cov.n_sats = 5;
  msg->pos_llh_cov.tow = 7;
}

void get_SbpMsgPosLlhCovGnss(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 4096;
  *msg_type = SbpMsgPosLlhCovGnss;

  msg->pos_llh_cov_gnss.cov_d_d = 0.03288137540221214;
  msg->pos_llh_cov_gnss.cov_e_d = -0.0008439270895905793;
  msg->pos_llh_cov_gnss.cov_e_e = 0.004523798823356628;
  msg->pos_llh_cov_gnss.cov_n_d = 0.0018563168123364449;
  msg->pos_llh_cov_gnss.cov_n_e = -0.00036755966721102595;
  msg->pos_llh_cov_gnss.cov_n_n = 0.007488971576094627;
  msg->pos_llh_cov_gnss.flags = 4;
  msg->pos_llh_cov_gnss.height = -17.39382124780135;
  msg->pos_llh_cov_gnss.lat = 37.83123196497633;
  msg->pos_llh_cov_gnss.lon = -122.28650381011681;
  msg->pos_llh_cov_gnss.n_sats = 18;
  msg->pos_llh_cov_gnss.tow = 501867800;
}

void get_SbpMsgPosLlhDepA(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgPosLlhDepA;

  msg->pos_llh_dep_a.flags = 0;
  msg->pos_llh_dep_a.h_accuracy = 0;
  msg->pos_llh_dep_a.height = 69.80437675175607;
  msg->pos_llh_dep_a.lat = 37.42906890908121;
  msg->pos_llh_dep_a.lon = -122.17338662202773;
  msg->pos_llh_dep_a.n_sats = 9;
  msg->pos_llh_dep_a.tow = 2567700;
  msg->pos_llh_dep_a.v_accuracy = 0;
}

void get_SbpMsgPosLlhGnss(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 4096;
  *msg_type = SbpMsgPosLlhGnss;

  msg->pos_llh_gnss.flags = 4;
  msg->pos_llh_gnss.h_accuracy = 87;
  msg->pos_llh_gnss.height = -17.39382124780135;
  msg->pos_llh_gnss.lat = 37.83123196497633;
  msg->pos_llh_gnss.lon = -122.28650381011681;
  msg->pos_llh_gnss.n_sats = 18;
  msg->pos_llh_gnss.tow = 501867800;
  msg->pos_llh_gnss.v_accuracy = 181;
}

void get_SbpMsgProtectionLevelDepA(sbp_msg_t *msg,
                                   sbp_msg_type_t *msg_type,
                                   uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 4096;
  *msg_type = SbpMsgProtectionLevelDepA;

  msg->protection_level_dep_a.flags = 0;
  msg->protection_level_dep_a.height = 0.0;
  msg->protection_level_dep_a.hpl = 0;
  msg->protection_level_dep_a.lat = 0.0;
  msg->protection_level_dep_a.lon = 0.0;
  msg->protection_level_dep_a.tow = 501867400;
  msg->protection_level_dep_a.vpl = 0;
}

void get_SbpMsgReferenceFrameParam(sbp_msg_t *msg,
                                   sbp_msg_type_t *msg_type,
                                   uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgReferenceFrameParam;

  msg->reference_frame_param.delta_X0 = 7;
  msg->reference_frame_param.delta_Y0 = 8;
  msg->reference_frame_param.delta_Z0 = 9;
  msg->reference_frame_param.dot_delta_X0 = 14;
  msg->reference_frame_param.dot_delta_Y0 = 15;
  msg->reference_frame_param.dot_delta_Z0 = 16;
  msg->reference_frame_param.dot_scale = 20;
  msg->reference_frame_param.dot_theta_01 = 17;
  msg->reference_frame_param.dot_theta_02 = 18;
  msg->reference_frame_param.dot_theta_03 = 19;
  msg->reference_frame_param.re_t0 = 6;
  msg->reference_frame_param.scale = 13;
  msg->reference_frame_param.sin = 4;
  {
    const char assign_string[] = {
        (char)102, (char)111, (char)111, (char)0, (char)0, (char)0, (char)0,
        (char)0,   (char)0,   (char)0,   (char)0, (char)0, (char)0, (char)0,
        (char)0,   (char)0,   (char)0,   (char)0, (char)0, (char)0, (char)0,
        (char)0,   (char)0,   (char)0,   (char)0, (char)0, (char)0, (char)0,
        (char)0,   (char)0,   (char)0,   (char)0};
    memcpy(msg->reference_frame_param.sn, assign_string, sizeof(assign_string));
  }
  msg->reference_frame_param.ssr_iod = 1;
  msg->reference_frame_param.theta_01 = 10;
  msg->reference_frame_param.theta_02 = 11;
  msg->reference_frame_param.theta_03 = 12;
  {
    const char assign_string[] = {
        (char)98, (char)97, (char)114, (char)0, (char)0, (char)0, (char)0,
        (char)0,  (char)0,  (char)0,   (char)0, (char)0, (char)0, (char)0,
        (char)0,  (char)0,  (char)0,   (char)0, (char)0, (char)0, (char)0,
        (char)0,  (char)0,  (char)0,   (char)0, (char)0, (char)0, (char)0,
        (char)0,  (char)0,  (char)0,   (char)0};
    memcpy(msg->reference_frame_param.tn, assign_string, sizeof(assign_string));
  }
  msg->reference_frame_param.utn = 5;
}

void get_SbpMsgUtcLeapSecond(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgUtcLeapSecond;

  msg->utc_leap_second.count_after = 9;
  msg->utc_leap_second.count_before = 4;
  msg->utc_leap_second.ref_dn = 8;
  msg->utc_leap_second.ref_wn = 7;
  msg->utc_leap_second.reserved_0 = 1;
  msg->utc_leap_second.reserved_1 = 2;
  msg->utc_leap_second.reserved_2 = 3;
  msg->utc_leap_second.reserved_3 = 5;
  msg->utc_leap_second.reserved_4 = 6;
}

void get_SbpMsgUtcTime(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 789;
  *msg_type = SbpMsgUtcTime;

  msg->utc_time.day = 9;
  msg->utc_time.flags = 1;
  msg->utc_time.hours = 19;
  msg->utc_time.minutes = 24;
  msg->utc_time.month = 4;
  msg->utc_time.ns = 800000000;
  msg->utc_time.seconds = 9;
  msg->utc_time.tow = 501867800;
  msg->utc_time.year = 2021;
}

void get_SbpMsgUtcTimeGnss(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 789;
  *msg_type = SbpMsgUtcTimeGnss;

  msg->utc_time_gnss.day = 9;
  msg->utc_time_gnss.flags = 1;
  msg->utc_time_gnss.hours = 19;
  msg->utc_time_gnss.minutes = 24;
  msg->utc_time_gnss.month = 4;
  msg->utc_time_gnss.ns = 800000000;
  msg->utc_time_gnss.seconds = 9;
  msg->utc_time_gnss.tow = 501867800;
  msg->utc_time_gnss.year = 2021;
}

void get_SbpMsgVelBody(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgVelBody;

  msg->vel_body.cov_x_x = 0.0;
  msg->vel_body.cov_x_y = 5.0;
  msg->vel_body.cov_x_z = 7.0;
  msg->vel_body.cov_y_y = 7.0;
  msg->vel_body.cov_y_z = 3.0;
  msg->vel_body.cov_z_z = 2.0;
  msg->vel_body.flags = 8;
  msg->vel_body.n_sats = 3;
  msg->vel_body.tow = 1;
  msg->vel_body.x = 4;
  msg->vel_body.y = 2;
  msg->vel_body.z = 1;
}

void get_SbpMsgVelCog(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 35027;
  *msg_type = SbpMsgVelCog;

  msg->vel_cog.cog = 1000;
  msg->vel_cog.cog_accuracy = 4000;
  msg->vel_cog.flags = 62;
  msg->vel_cog.sog = 2000;
  msg->vel_cog.sog_accuracy = 5000;
  msg->vel_cog.tow = 326825520;
  msg->vel_cog.v_up = 3000;
  msg->vel_cog.v_up_accuracy = 6000;
}

void get_SbpMsgVelEcef(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 35027;
  *msg_type = SbpMsgVelEcef;

  msg->vel_ecef.accuracy = 0;
  msg->vel_ecef.flags = 0;
  msg->vel_ecef.n_sats = 14;
  msg->vel_ecef.tow = 326825000;
  msg->vel_ecef.x = -8;
  msg->vel_ecef.y = -5;
  msg->vel_ecef.z = 10;
}

void get_SbpMsgVelEcefCov(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgVelEcefCov;

  msg->vel_ecef_cov.cov_x_x = 2.0;
  msg->vel_ecef_cov.cov_x_y = 2.0;
  msg->vel_ecef_cov.cov_x_z = 2.0;
  msg->vel_ecef_cov.cov_y_y = 2.0;
  msg->vel_ecef_cov.cov_y_z = 1.0;
  msg->vel_ecef_cov.cov_z_z = 3.0;
  msg->vel_ecef_cov.flags = 4;
  msg->vel_ecef_cov.n_sats = 3;
  msg->vel_ecef_cov.tow = 2;
  msg->vel_ecef_cov.x = 0;
  msg->vel_ecef_cov.y = 0;
  msg->vel_ecef_cov.z = 6;
}

void get_SbpMsgVelEcefCovGnss(sbp_msg_t *msg,
                              sbp_msg_type_t *msg_type,
                              uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 4096;
  *msg_type = SbpMsgVelEcefCovGnss;

  msg->vel_ecef_cov_gnss.cov_x_x = 0.0024547684006392956;
  msg->vel_ecef_cov_gnss.cov_x_y = 0.0021795108914375305;
  msg->vel_ecef_cov_gnss.cov_x_z = -0.0016828652005642653;
  msg->vel_ecef_cov_gnss.cov_y_y = 0.004218944814056158;
  msg->vel_ecef_cov_gnss.cov_y_z = -0.0024961293675005436;
  msg->vel_ecef_cov_gnss.cov_z_z = 0.0037804271560162306;
  msg->vel_ecef_cov_gnss.flags = 2;
  msg->vel_ecef_cov_gnss.n_sats = 21;
  msg->vel_ecef_cov_gnss.tow = 501868000;
  msg->vel_ecef_cov_gnss.x = -3;
  msg->vel_ecef_cov_gnss.y = 1;
  msg->vel_ecef_cov_gnss.z = 4;
}

void get_SbpMsgVelEcefDepA(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgVelEcefDepA;

  msg->vel_ecef_dep_a.accuracy = 0;
  msg->vel_ecef_dep_a.flags = 0;
  msg->vel_ecef_dep_a.n_sats = 9;
  msg->vel_ecef_dep_a.tow = 2567700;
  msg->vel_ecef_dep_a.x = 3034;
  msg->vel_ecef_dep_a.y = -2682;
  msg->vel_ecef_dep_a.z = -861;
}

void get_SbpMsgVelEcefGnss(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 4096;
  *msg_type = SbpMsgVelEcefGnss;

  msg->vel_ecef_gnss.accuracy = 89;
  msg->vel_ecef_gnss.flags = 2;
  msg->vel_ecef_gnss.n_sats = 21;
  msg->vel_ecef_gnss.tow = 501868000;
  msg->vel_ecef_gnss.x = -3;
  msg->vel_ecef_gnss.y = 1;
  msg->vel_ecef_gnss.z = 4;
}

void get_SbpMsgVelNed(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 35027;
  *msg_type = SbpMsgVelNed;

  msg->vel_ned.d = -13;
  msg->vel_ned.e = -4;
  msg->vel_ned.flags = 0;
  msg->vel_ned.h_accuracy = 0;
  msg->vel_ned.n = 3;
  msg->vel_ned.n_sats = 14;
  msg->vel_ned.tow = 326825000;
  msg->vel_ned.v_accuracy = 0;
}

void get_SbpMsgVelNedCov(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgVelNedCov;

  msg->vel_ned_cov.cov_d_d = 1.0;
  msg->vel_ned_cov.cov_e_d = 1.0;
  msg->vel_ned_cov.cov_e_e = 1.0;
  msg->vel_ned_cov.cov_n_d = 1.0;
  msg->vel_ned_cov.cov_n_e = 1.0;
  msg->vel_ned_cov.cov_n_n = 1.0;
  msg->vel_ned_cov.d = 1;
  msg->vel_ned_cov.e = 1;
  msg->vel_ned_cov.flags = 0;
  msg->vel_ned_cov.n = 1;
  msg->vel_ned_cov.n_sats = 10;
  msg->vel_ned_cov.tow = 100;
}

void get_SbpMsgVelNedCovGnss(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 4096;
  *msg_type = SbpMsgVelNedCovGnss;

  msg->vel_ned_cov_gnss.cov_d_d = 0.007882959209382534;
  msg->vel_ned_cov_gnss.cov_e_d = 0.00016467059322167188;
  msg->vel_ned_cov_gnss.cov_e_e = 0.0009897587588056922;
  msg->vel_ned_cov_gnss.cov_n_d = 0.00017716512957122177;
  msg->vel_ned_cov_gnss.cov_n_e = 1.457612233934924e-05;
  msg->vel_ned_cov_gnss.cov_n_n = 0.0015810149488970637;
  msg->vel_ned_cov_gnss.d = -10;
  msg->vel_ned_cov_gnss.e = 0;
  msg->vel_ned_cov_gnss.flags = 2;
  msg->vel_ned_cov_gnss.n = -5;
  msg->vel_ned_cov_gnss.n_sats = 21;
  msg->vel_ned_cov_gnss.tow = 501868200;
}

void get_SbpMsgVelNedDepA(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgVelNedDepA;

  msg->vel_ned_dep_a.d = 0;
  msg->vel_ned_dep_a.e = 3996;
  msg->vel_ned_dep_a.flags = 0;
  msg->vel_ned_dep_a.h_accuracy = 0;
  msg->vel_ned_dep_a.n = -1082;
  msg->vel_ned_dep_a.n_sats = 9;
  msg->vel_ned_dep_a.tow = 2567700;
  msg->vel_ned_dep_a.v_accuracy = 0;
}

void get_SbpMsgVelNedGnss(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 4096;
  *msg_type = SbpMsgVelNedGnss;

  msg->vel_ned_gnss.d = -10;
  msg->vel_ned_gnss.e = 0;
  msg->vel_ned_gnss.flags = 2;
  msg->vel_ned_gnss.h_accuracy = 40;
  msg->vel_ned_gnss.n = -5;
  msg->vel_ned_gnss.n_sats = 21;
  msg->vel_ned_gnss.tow = 501868200;
  msg->vel_ned_gnss.v_accuracy = 89;
}

void get_SbpMsgEphemerisBds(sbp_msg_t *msg,
                            sbp_msg_type_t *msg_type,
                            uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 61568;
  *msg_type = SbpMsgEphemerisBds;

  msg->ephemeris_bds.af0 = -0.0008911322802305222;
  msg->ephemeris_bds.af1 = 1.2398970739013748e-12;
  msg->ephemeris_bds.af2 = -7.318364664277155e-19;
  msg->ephemeris_bds.c_ic = -6.658956408500671e-08;
  msg->ephemeris_bds.c_is = 3.5529956221580505e-07;
  msg->ephemeris_bds.c_rc = 234.640625;
  msg->ephemeris_bds.c_rs = 224.4375;
  msg->ephemeris_bds.c_uc = 7.606577128171921e-06;
  msg->ephemeris_bds.c_us = 6.551854312419891e-07;
  msg->ephemeris_bds.common.fit_interval = 10800;
  msg->ephemeris_bds.common.health_bits = 0;
  msg->ephemeris_bds.common.sid.code = 12;
  msg->ephemeris_bds.common.sid.sat = 8;
  msg->ephemeris_bds.common.toe.tow = 439214;
  msg->ephemeris_bds.common.toe.wn = 2154;
  msg->ephemeris_bds.common.ura = 2.0;
  msg->ephemeris_bds.common.valid = 1;
  msg->ephemeris_bds.dn = 1.1296899132622133e-09;
  msg->ephemeris_bds.ecc = 0.005184737499803305;
  msg->ephemeris_bds.inc = 1.0421769543504915;
  msg->ephemeris_bds.inc_dot = 7.507455572801683e-10;
  msg->ephemeris_bds.iodc = 5;
  msg->ephemeris_bds.iode = 6;
  msg->ephemeris_bds.m0 = 1.6943958190727237;
  msg->ephemeris_bds.omega0 = -2.581073762870982;
  msg->ephemeris_bds.omegadot = -2.303310227830545e-09;
  msg->ephemeris_bds.sqrta = 6493.49845123291;
  msg->ephemeris_bds.tgd1 = 1.0499999980595476e-08;
  msg->ephemeris_bds.tgd2 = -1.0999999799921056e-09;
  msg->ephemeris_bds.toc.tow = 439214;
  msg->ephemeris_bds.toc.wn = 2154;
  msg->ephemeris_bds.w = -2.698603205735458;
}

void get_SbpMsgEphemerisBds_alt(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 51228;
  *msg_type = SbpMsgEphemerisBds;

  msg->ephemeris_bds.af0 = 0.00004738220013678074;
  msg->ephemeris_bds.af1 = 1.0662582e-11;
  msg->ephemeris_bds.af2 = 0.0;
  msg->ephemeris_bds.c_ic = -9.2200935e-8;
  msg->ephemeris_bds.c_is = 5.6810677e-8;
  msg->ephemeris_bds.c_rc = 389.32812;
  msg->ephemeris_bds.c_rs = -631.0;
  msg->ephemeris_bds.c_uc = -0.000020444393;
  msg->ephemeris_bds.c_us = -0.000004312489;
  msg->ephemeris_bds.common.fit_interval = 10800;
  msg->ephemeris_bds.common.health_bits = 0;
  msg->ephemeris_bds.common.sid.code = 12;
  msg->ephemeris_bds.common.sid.sat = 13;
  msg->ephemeris_bds.common.toe.tow = 176414;
  msg->ephemeris_bds.common.toe.wn = 2222;
  msg->ephemeris_bds.common.ura = 2.0;
  msg->ephemeris_bds.common.valid = 1;
  msg->ephemeris_bds.dn = 7.218157808102856e-10;
  msg->ephemeris_bds.ecc = 0.004354586941190064;
  msg->ephemeris_bds.inc = 1.0147900643651668;
  msg->ephemeris_bds.inc_dot = -2.6679682744447465e-10;
  msg->ephemeris_bds.iodc = 0;
  msg->ephemeris_bds.iode = 1;
  msg->ephemeris_bds.m0 = 2.8462748877337534;
  msg->ephemeris_bds.omega0 = 1.689829609946071;
  msg->ephemeris_bds.omegadot = -2.32473969188231e-9;
  msg->ephemeris_bds.sqrta = 6493.453912734985;
  msg->ephemeris_bds.tgd1 = -9.6e-9;
  msg->ephemeris_bds.tgd2 = 2.6e-9;
  msg->ephemeris_bds.toc.tow = 176414;
  msg->ephemeris_bds.toc.wn = 2222;
  msg->ephemeris_bds.w = -2.562373456613879;
}

void get_SbpMsgEphemerisGal(sbp_msg_t *msg,
                            sbp_msg_type_t *msg_type,
                            uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 61568;
  *msg_type = SbpMsgEphemerisGal;

  msg->ephemeris_gal.af0 = -1.7088896129280325e-05;
  msg->ephemeris_gal.af1 = -8.185452315956353e-12;
  msg->ephemeris_gal.af2 = 0.0;
  msg->ephemeris_gal.bgd_e1e5a = 2.0954757928848267e-09;
  msg->ephemeris_gal.bgd_e1e5b = 2.0954757928848267e-09;
  msg->ephemeris_gal.c_ic = -3.166496753692627e-08;
  msg->ephemeris_gal.c_is = -3.166496753692627e-08;
  msg->ephemeris_gal.c_rc = 265.4375;
  msg->ephemeris_gal.c_rs = 10.125;
  msg->ephemeris_gal.c_uc = 5.364418029785156e-07;
  msg->ephemeris_gal.c_us = 3.993511199951172e-06;
  msg->ephemeris_gal.common.fit_interval = 14400;
  msg->ephemeris_gal.common.health_bits = 0;
  msg->ephemeris_gal.common.sid.code = 14;
  msg->ephemeris_gal.common.sid.sat = 27;
  msg->ephemeris_gal.common.toe.tow = 448800;
  msg->ephemeris_gal.common.toe.wn = 2154;
  msg->ephemeris_gal.common.ura = 3.119999885559082;
  msg->ephemeris_gal.common.valid = 1;
  msg->ephemeris_gal.dn = 3.2262058129932258e-09;
  msg->ephemeris_gal.ecc = 0.00017060607206076384;
  msg->ephemeris_gal.inc = 0.9777456094977858;
  msg->ephemeris_gal.inc_dot = -3.1787038343451465e-10;
  msg->ephemeris_gal.iodc = 108;
  msg->ephemeris_gal.iode = 108;
  msg->ephemeris_gal.m0 = -1.8457115744155868;
  msg->ephemeris_gal.omega0 = 1.16967730598334;
  msg->ephemeris_gal.omegadot = -5.757382675240872e-09;
  msg->ephemeris_gal.source = 0;
  msg->ephemeris_gal.sqrta = 5440.602401733398;
  msg->ephemeris_gal.toc.tow = 448800;
  msg->ephemeris_gal.toc.wn = 2154;
  msg->ephemeris_gal.w = 0.12250912091662625;
}

void get_SbpMsgEphemerisGal_alt(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 51228;
  *msg_type = SbpMsgEphemerisGal;

  msg->ephemeris_gal.af0 = 0.0004301711451262235;
  msg->ephemeris_gal.af1 = 9.9475983006414e-14;
  msg->ephemeris_gal.af2 = 0.0;
  msg->ephemeris_gal.bgd_e1e5a = 6.9849193e-10;
  msg->ephemeris_gal.bgd_e1e5b = 4.656613e-10;
  msg->ephemeris_gal.c_ic = -2.2351742e-8;
  msg->ephemeris_gal.c_is = 1.2479722e-7;
  msg->ephemeris_gal.c_rc = 431.46875;
  msg->ephemeris_gal.c_rs = -23.90625;
  msg->ephemeris_gal.c_uc = -0.0000010300428;
  msg->ephemeris_gal.c_us = -0.0000034440309;
  msg->ephemeris_gal.common.fit_interval = 14400;
  msg->ephemeris_gal.common.health_bits = 0;
  msg->ephemeris_gal.common.sid.code = 14;
  msg->ephemeris_gal.common.sid.sat = 13;
  msg->ephemeris_gal.common.toe.tow = 178800;
  msg->ephemeris_gal.common.toe.wn = 2222;
  msg->ephemeris_gal.common.ura = 3.12;
  msg->ephemeris_gal.common.valid = 1;
  msg->ephemeris_gal.dn = 2.733685297536826e-9;
  msg->ephemeris_gal.ecc = 0.00013237446546554565;
  msg->ephemeris_gal.inc = 0.9979406332844445;
  msg->ephemeris_gal.inc_dot = -4.643050544549091e-12;
  msg->ephemeris_gal.iodc = 42;
  msg->ephemeris_gal.iode = 42;
  msg->ephemeris_gal.m0 = 1.6468149220288053;
  msg->ephemeris_gal.omega0 = 1.1227143601652847;
  msg->ephemeris_gal.omegadot = -5.940961750617659e-9;
  msg->ephemeris_gal.source = 0;
  msg->ephemeris_gal.sqrta = 5440.605186462402;
  msg->ephemeris_gal.toc.tow = 178800;
  msg->ephemeris_gal.toc.wn = 2222;
  msg->ephemeris_gal.w = 1.7916777837598907;
}

void get_SbpMsgEphemerisGlo(sbp_msg_t *msg,
                            sbp_msg_type_t *msg_type,
                            uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 2314;
  *msg_type = SbpMsgEphemerisGlo;

  msg->ephemeris_glo.acc[0] = 9.313225746154785e-07;
  msg->ephemeris_glo.acc[1] = 9.313225746154785e-07;
  msg->ephemeris_glo.acc[2] = 2.7939677238464355e-06;
  msg->ephemeris_glo.common.fit_interval = 2400;
  msg->ephemeris_glo.common.health_bits = 0;
  msg->ephemeris_glo.common.sid.code = 3;
  msg->ephemeris_glo.common.sid.sat = 4;
  msg->ephemeris_glo.common.toe.tow = 443718;
  msg->ephemeris_glo.common.toe.wn = 2154;
  msg->ephemeris_glo.common.ura = 5.0;
  msg->ephemeris_glo.common.valid = 1;
  msg->ephemeris_glo.d_tau = -2.7939677238464355e-09;
  msg->ephemeris_glo.fcn = 14;
  msg->ephemeris_glo.gamma = 9.094947017729282e-13;
  msg->ephemeris_glo.iod = 100;
  msg->ephemeris_glo.pos[0] = -12177330.078125;
  msg->ephemeris_glo.pos[1] = 599893.06640625;
  msg->ephemeris_glo.pos[2] = -22373708.49609375;
  msg->ephemeris_glo.tau = -8.36281105875969e-05;
  msg->ephemeris_glo.vel[0] = -1726.506233215332;
  msg->ephemeris_glo.vel[1] = -2542.6149368286133;
  msg->ephemeris_glo.vel[2] = 869.8177337646484;
}

void get_SbpMsgEphemerisGlo_alt(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 0;
  *msg_type = SbpMsgEphemerisGlo;

  msg->ephemeris_glo.acc[0] = -9.313226e-7;
  msg->ephemeris_glo.acc[1] = 0.000004656613;
  msg->ephemeris_glo.acc[2] = 0.0000018626451;
  msg->ephemeris_glo.common.fit_interval = 2400;
  msg->ephemeris_glo.common.health_bits = 0;
  msg->ephemeris_glo.common.sid.code = 3;
  msg->ephemeris_glo.common.sid.sat = 1;
  msg->ephemeris_glo.common.toe.tow = 180918;
  msg->ephemeris_glo.common.toe.wn = 2222;
  msg->ephemeris_glo.common.ura = 2.5;
  msg->ephemeris_glo.common.valid = 1;
  msg->ephemeris_glo.d_tau = 8.381903e-9;
  msg->ephemeris_glo.fcn = 9;
  msg->ephemeris_glo.gamma = 0.0;
  msg->ephemeris_glo.iod = 84;
  msg->ephemeris_glo.pos[0] = -17672041.9921875;
  msg->ephemeris_glo.pos[1] = 16744720.21484375;
  msg->ephemeris_glo.pos[2] = 7593349.12109375;
  msg->ephemeris_glo.tau = -0.000012818724;
  msg->ephemeris_glo.vel[0] = 751.5621185302734;
  msg->ephemeris_glo.vel[1] = -732.8472137451172;
  msg->ephemeris_glo.vel[2] = 3365.7236099243164;
}

void get_SbpMsgEphemerisGps(sbp_msg_t *msg,
                            sbp_msg_type_t *msg_type,
                            uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 2314;
  *msg_type = SbpMsgEphemerisGps;

  msg->ephemeris_gps.af0 = -0.0006315018981695175;
  msg->ephemeris_gps.af1 = 8.981260180007666e-12;
  msg->ephemeris_gps.af2 = 0.0;
  msg->ephemeris_gps.c_ic = 7.450580596923828e-09;
  msg->ephemeris_gps.c_is = -1.1548399925231934e-07;
  msg->ephemeris_gps.c_rc = 308.625;
  msg->ephemeris_gps.c_rs = -52.3125;
  msg->ephemeris_gps.c_uc = -2.7436763048171997e-06;
  msg->ephemeris_gps.c_us = 3.1366944313049316e-06;
  msg->ephemeris_gps.common.fit_interval = 14400;
  msg->ephemeris_gps.common.health_bits = 0;
  msg->ephemeris_gps.common.sid.code = 0;
  msg->ephemeris_gps.common.sid.sat = 22;
  msg->ephemeris_gps.common.toe.tow = 446384;
  msg->ephemeris_gps.common.toe.wn = 2154;
  msg->ephemeris_gps.common.ura = 2.0;
  msg->ephemeris_gps.common.valid = 1;
  msg->ephemeris_gps.dn = 5.694522914022375e-09;
  msg->ephemeris_gps.ecc = 0.007072207052260637;
  msg->ephemeris_gps.inc = 0.9341514480259797;
  msg->ephemeris_gps.inc_dot = -4.035882396415757e-11;
  msg->ephemeris_gps.iodc = 45;
  msg->ephemeris_gps.iode = 45;
  msg->ephemeris_gps.m0 = -0.02200078842114688;
  msg->ephemeris_gps.omega0 = -1.8731818448797617;
  msg->ephemeris_gps.omegadot = -8.903585155774196e-09;
  msg->ephemeris_gps.sqrta = 5153.550029754639;
  msg->ephemeris_gps.tgd = -1.7695128917694092e-08;
  msg->ephemeris_gps.toc.tow = 446384;
  msg->ephemeris_gps.toc.wn = 2154;
  msg->ephemeris_gps.w = -0.9893036629599647;
}

void get_SbpMsgEphemerisGps_alt(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 51228;
  *msg_type = SbpMsgEphemerisGps;

  msg->ephemeris_gps.af0 = -0.0003274544;
  msg->ephemeris_gps.af1 = -8.7538865e-12;
  msg->ephemeris_gps.af2 = 0.0;
  msg->ephemeris_gps.c_ic = 1.8626451e-8;
  msg->ephemeris_gps.c_is = 8.940697e-8;
  msg->ephemeris_gps.c_rc = 224.4375;
  msg->ephemeris_gps.c_rs = -122.875;
  msg->ephemeris_gps.c_uc = -0.0000064373016;
  msg->ephemeris_gps.c_us = 0.000008808449;
  msg->ephemeris_gps.common.fit_interval = 14400;
  msg->ephemeris_gps.common.health_bits = 0;
  msg->ephemeris_gps.common.sid.code = 0;
  msg->ephemeris_gps.common.sid.sat = 3;
  msg->ephemeris_gps.common.toe.tow = 172800;
  msg->ephemeris_gps.common.toe.wn = 2222;
  msg->ephemeris_gps.common.ura = 4.0;
  msg->ephemeris_gps.common.valid = 1;
  msg->ephemeris_gps.dn = 4.009095566351042e-9;
  msg->ephemeris_gps.ecc = 0.004445207072421908;
  msg->ephemeris_gps.inc = 0.9749349693604852;
  msg->ephemeris_gps.inc_dot = 1.4572035555200222e-10;
  msg->ephemeris_gps.iodc = 58;
  msg->ephemeris_gps.iode = 58;
  msg->ephemeris_gps.m0 = 0.19683191906806538;
  msg->ephemeris_gps.omega0 = 2.2768524903310077;
  msg->ephemeris_gps.omegadot = -7.80782522726059e-9;
  msg->ephemeris_gps.sqrta = 5153.581796646118;
  msg->ephemeris_gps.tgd = 1.8626451e-9;
  msg->ephemeris_gps.toc.tow = 172800;
  msg->ephemeris_gps.toc.wn = 2222;
  msg->ephemeris_gps.w = 1.0259259123550766;
}

void get_SbpMsgEphemerisQzss(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 61568;
  *msg_type = SbpMsgEphemerisQzss;
  msg->ephemeris_qzss.af0 = -0.00036908406764268875;
  msg->ephemeris_qzss.af1 = -4.774847184307873e-12;
  msg->ephemeris_qzss.af2 = 0.0;
  msg->ephemeris_qzss.c_ic = -1.1753290891647339e-06;
  msg->ephemeris_qzss.c_is = 1.6205012798309326e-07;
  msg->ephemeris_qzss.c_rc = -36.90625;
  msg->ephemeris_qzss.c_rs = -457.8125;
  msg->ephemeris_qzss.c_uc = -1.6197562217712402e-05;
  msg->ephemeris_qzss.c_us = 8.247792720794678e-06;
  msg->ephemeris_qzss.common.fit_interval = 14400;
  msg->ephemeris_qzss.common.health_bits = 0;
  msg->ephemeris_qzss.common.sid.code = 31;
  msg->ephemeris_qzss.common.sid.sat = 193;
  msg->ephemeris_qzss.common.toe.tow = 450000;
  msg->ephemeris_qzss.common.toe.wn = 2154;
  msg->ephemeris_qzss.common.ura = 2.0;
  msg->ephemeris_qzss.common.valid = 0;
  msg->ephemeris_qzss.dn = 2.678325848736433e-09;
  msg->ephemeris_qzss.ecc = 0.07550019584596157;
  msg->ephemeris_qzss.inc = 0.7309715119432375;
  msg->ephemeris_qzss.inc_dot = 2.0000833114980698e-11;
  msg->ephemeris_qzss.iodc = 817;
  msg->ephemeris_qzss.iode = 49;
  msg->ephemeris_qzss.m0 = 0.30694242158961144;
  msg->ephemeris_qzss.omega0 = -1.1693743795366662;
  msg->ephemeris_qzss.omegadot = -2.7936877968817684e-09;
  msg->ephemeris_qzss.sqrta = 6493.172393798828;
  msg->ephemeris_qzss.tgd = -5.587935447692871e-09;
  msg->ephemeris_qzss.toc.tow = 450000;
  msg->ephemeris_qzss.toc.wn = 2154;
  msg->ephemeris_qzss.w = -1.5662079690885238;
}

void get_SbpMsgEphemerisQzss_alt(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 51228;
  *msg_type = SbpMsgEphemerisQzss;

  msg->ephemeris_qzss.af0 = -5.234033e-7;
  msg->ephemeris_qzss.af1 = -2.2737368e-13;
  msg->ephemeris_qzss.af2 = 0.0;
  msg->ephemeris_qzss.c_ic = -0.0000054985285;
  msg->ephemeris_qzss.c_is = -0.0000033080578;
  msg->ephemeris_qzss.c_rc = 975.71875;
  msg->ephemeris_qzss.c_rs = 395.8125;
  msg->ephemeris_qzss.c_uc = 0.000013712794;
  msg->ephemeris_qzss.c_us = -0.000024672598;
  msg->ephemeris_qzss.common.fit_interval = 14400;
  msg->ephemeris_qzss.common.health_bits = 0;
  msg->ephemeris_qzss.common.sid.code = 31;
  msg->ephemeris_qzss.common.sid.sat = 195;
  msg->ephemeris_qzss.common.toe.tow = 183600;
  msg->ephemeris_qzss.common.toe.wn = 2222;
  msg->ephemeris_qzss.common.ura = 2.0;
  msg->ephemeris_qzss.common.valid = 0;
  msg->ephemeris_qzss.dn = -2.2715231894870936e-10;
  msg->ephemeris_qzss.ecc = 0.07455702591687441;
  msg->ephemeris_qzss.inc = 0.7113099602521185;
  msg->ephemeris_qzss.inc_dot = 2.000797626966462e-9;
  msg->ephemeris_qzss.iodc = 969;
  msg->ephemeris_qzss.iode = 201;
  msg->ephemeris_qzss.m0 = -2.2884232741063104;
  msg->ephemeris_qzss.omega0 = 0.8026393873464875;
  msg->ephemeris_qzss.omegadot = -4.285892810353007e-12;
  msg->ephemeris_qzss.sqrta = 6493.025869369507;
  msg->ephemeris_qzss.tgd = 4.656613e-10;
  msg->ephemeris_qzss.toc.tow = 183600;
  msg->ephemeris_qzss.toc.wn = 2222;
  msg->ephemeris_qzss.w = -1.58514017858247;
}

void get_SbpMsgGloBiases(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 0;
  *msg_type = SbpMsgGloBiases;

  msg->glo_biases.l1ca_bias = 0;
  msg->glo_biases.l1p_bias = 0;
  msg->glo_biases.l2ca_bias = 0;
  msg->glo_biases.l2p_bias = 0;
  msg->glo_biases.mask = 0;
}

void get_SbpMsgObsDepA(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 1219;
  *msg_type = SbpMsgObsDepA;

  msg->obs_dep_a.header.n_obs = 32;
  msg->obs_dep_a.header.t.tow = 407084600;
  msg->obs_dep_a.header.t.wn = 1838;
  msg->obs_dep_a.n_obs = 7;
  msg->obs_dep_a.obs[0].L.f = 33;
  msg->obs_dep_a.obs[0].L.i = -36108;
  msg->obs_dep_a.obs[0].P = 2046421816;
  msg->obs_dep_a.obs[0].cn0 = 46;
  msg->obs_dep_a.obs[0].lock = 55875;
  msg->obs_dep_a.obs[0].prn = 0;
  msg->obs_dep_a.obs[1].L.f = 98;
  msg->obs_dep_a.obs[1].L.i = 203030;
  msg->obs_dep_a.obs[1].P = 2085014510;
  msg->obs_dep_a.obs[1].cn0 = 43;
  msg->obs_dep_a.obs[1].lock = 40376;
  msg->obs_dep_a.obs[1].prn = 2;
  msg->obs_dep_a.obs[2].L.f = 185;
  msg->obs_dep_a.obs[2].L.i = -178306;
  msg->obs_dep_a.obs[2].P = 2110096816;
  msg->obs_dep_a.obs[2].cn0 = 39;
  msg->obs_dep_a.obs[2].lock = 14148;
  msg->obs_dep_a.obs[2].prn = 3;
  msg->obs_dep_a.obs[3].L.f = 139;
  msg->obs_dep_a.obs[3].L.i = -137374;
  msg->obs_dep_a.obs[3].P = 2208476476;
  msg->obs_dep_a.obs[3].cn0 = 30;
  msg->obs_dep_a.obs[3].lock = 4129;
  msg->obs_dep_a.obs[3].prn = 10;
  msg->obs_dep_a.obs[4].L.f = 40;
  msg->obs_dep_a.obs[4].L.i = -167638;
  msg->obs_dep_a.obs[4].P = 2298000000;
  msg->obs_dep_a.obs[4].cn0 = 20;
  msg->obs_dep_a.obs[4].lock = 18218;
  msg->obs_dep_a.obs[4].prn = 13;
  msg->obs_dep_a.obs[5].L.f = 64;
  msg->obs_dep_a.obs[5].L.i = 209919;
  msg->obs_dep_a.obs[5].P = 2266101494;
  msg->obs_dep_a.obs[5].cn0 = 27;
  msg->obs_dep_a.obs[5].lock = 63852;
  msg->obs_dep_a.obs[5].prn = 22;
  msg->obs_dep_a.obs[6].L.f = 31;
  msg->obs_dep_a.obs[6].L.i = -53117;
  msg->obs_dep_a.obs[6].P = 1987193298;
  msg->obs_dep_a.obs[6].cn0 = 52;
  msg->obs_dep_a.obs[6].lock = 15074;
  msg->obs_dep_a.obs[6].prn = 30;
}

void get_SbpMsgObsDepB(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgObsDepB;

  msg->obs_dep_b.header.n_obs = 32;
  msg->obs_dep_b.header.t.tow = 2567800;
  msg->obs_dep_b.header.t.wn = 1787;
  msg->obs_dep_b.n_obs = 6;
  msg->obs_dep_b.obs[0].L.f = 27;
  msg->obs_dep_b.obs[0].L.i = 117913055;
  msg->obs_dep_b.obs[0].P = 2243669940;
  msg->obs_dep_b.obs[0].cn0 = 157;
  msg->obs_dep_b.obs[0].lock = 0;
  msg->obs_dep_b.obs[0].sid.code = 0;
  msg->obs_dep_b.obs[0].sid.reserved = 0;
  msg->obs_dep_b.obs[0].sid.sat = 202;
  msg->obs_dep_b.obs[1].L.f = 175;
  msg->obs_dep_b.obs[1].L.i = 129899608;
  msg->obs_dep_b.obs[1].P = 2471857210;
  msg->obs_dep_b.obs[1].cn0 = 144;
  msg->obs_dep_b.obs[1].lock = 0;
  msg->obs_dep_b.obs[1].sid.code = 0;
  msg->obs_dep_b.obs[1].sid.reserved = 0;
  msg->obs_dep_b.obs[1].sid.sat = 203;
  msg->obs_dep_b.obs[2].L.f = 135;
  msg->obs_dep_b.obs[2].L.i = 122531024;
  msg->obs_dep_b.obs[2].P = 2331544796;
  msg->obs_dep_b.obs[2].cn0 = 151;
  msg->obs_dep_b.obs[2].lock = 0;
  msg->obs_dep_b.obs[2].sid.code = 0;
  msg->obs_dep_b.obs[2].sid.reserved = 0;
  msg->obs_dep_b.obs[2].sid.sat = 208;
  msg->obs_dep_b.obs[3].L.f = 242;
  msg->obs_dep_b.obs[3].L.i = 119280243;
  msg->obs_dep_b.obs[3].P = 2269692589;
  msg->obs_dep_b.obs[3].cn0 = 156;
  msg->obs_dep_b.obs[3].lock = 0;
  msg->obs_dep_b.obs[3].sid.code = 0;
  msg->obs_dep_b.obs[3].sid.reserved = 0;
  msg->obs_dep_b.obs[3].sid.sat = 212;
  msg->obs_dep_b.obs[4].L.f = 120;
  msg->obs_dep_b.obs[4].L.i = 109691922;
  msg->obs_dep_b.obs[4].P = 2087293092;
  msg->obs_dep_b.obs[4].cn0 = 168;
  msg->obs_dep_b.obs[4].lock = 0;
  msg->obs_dep_b.obs[4].sid.code = 0;
  msg->obs_dep_b.obs[4].sid.reserved = 0;
  msg->obs_dep_b.obs[4].sid.sat = 217;
  msg->obs_dep_b.obs[5].L.f = 87;
  msg->obs_dep_b.obs[5].L.i = 123340754;
  msg->obs_dep_b.obs[5].P = 2347034654;
  msg->obs_dep_b.obs[5].cn0 = 150;
  msg->obs_dep_b.obs[5].lock = 0;
  msg->obs_dep_b.obs[5].sid.code = 0;
  msg->obs_dep_b.obs[5].sid.reserved = 0;
  msg->obs_dep_b.obs[5].sid.sat = 218;
}

void get_SbpMsgObsDepC(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 38982;
  *msg_type = SbpMsgObsDepC;

  msg->obs_dep_c.header.n_obs = 32;
  msg->obs_dep_c.header.t.tow = 414670600;
  msg->obs_dep_c.header.t.wn = 1898;
  msg->obs_dep_c.n_obs = 5;
  msg->obs_dep_c.obs[0].L.f = 231;
  msg->obs_dep_c.obs[0].L.i = -565647;
  msg->obs_dep_c.obs[0].P = 1347025534;
  msg->obs_dep_c.obs[0].cn0 = 163;
  msg->obs_dep_c.obs[0].lock = 58853;
  msg->obs_dep_c.obs[0].sid.code = 0;
  msg->obs_dep_c.obs[0].sid.reserved = 0;
  msg->obs_dep_c.obs[0].sid.sat = 4;
  msg->obs_dep_c.obs[1].L.f = 196;
  msg->obs_dep_c.obs[1].L.i = -355503;
  msg->obs_dep_c.obs[1].P = 1180752956;
  msg->obs_dep_c.obs[1].cn0 = 208;
  msg->obs_dep_c.obs[1].lock = 7188;
  msg->obs_dep_c.obs[1].sid.code = 0;
  msg->obs_dep_c.obs[1].sid.reserved = 0;
  msg->obs_dep_c.obs[1].sid.sat = 6;
  msg->obs_dep_c.obs[2].L.f = 110;
  msg->obs_dep_c.obs[2].L.i = -902116;
  msg->obs_dep_c.obs[2].P = 1295924728;
  msg->obs_dep_c.obs[2].cn0 = 171;
  msg->obs_dep_c.obs[2].lock = 45748;
  msg->obs_dep_c.obs[2].sid.code = 0;
  msg->obs_dep_c.obs[2].sid.reserved = 0;
  msg->obs_dep_c.obs[2].sid.sat = 7;
  msg->obs_dep_c.obs[3].L.f = 41;
  msg->obs_dep_c.obs[3].L.i = 861612;
  msg->obs_dep_c.obs[3].P = 1304319213;
  msg->obs_dep_c.obs[3].cn0 = 170;
  msg->obs_dep_c.obs[3].lock = 42217;
  msg->obs_dep_c.obs[3].sid.code = 0;
  msg->obs_dep_c.obs[3].sid.reserved = 0;
  msg->obs_dep_c.obs[3].sid.sat = 10;
  msg->obs_dep_c.obs[4].L.f = 19;
  msg->obs_dep_c.obs[4].L.i = 1424624;
  msg->obs_dep_c.obs[4].P = 1258902820;
  msg->obs_dep_c.obs[4].cn0 = 182;
  msg->obs_dep_c.obs[4].lock = 53700;
  msg->obs_dep_c.obs[4].sid.code = 0;
  msg->obs_dep_c.obs[4].sid.reserved = 0;
  msg->obs_dep_c.obs[4].sid.sat = 12;
}

void get_SbpMsgOsr(sbp_msg_t *msg,
                   sbp_msg_type_t *msg_type,
                   uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 0;
  *msg_type = SbpMsgOsr;

  msg->osr.header.n_obs = 64;
  msg->osr.header.t.ns_residual = 0;
  msg->osr.header.t.tow = 501867000;
  msg->osr.header.t.wn = 2152;
  msg->osr.n_obs = 12;
  msg->osr.obs[0].L.f = 66;
  msg->osr.obs[0].L.i = 121567974;
  msg->osr.obs[0].P = 1156681547;
  msg->osr.obs[0].flags = 3;
  msg->osr.obs[0].iono_std = 13;
  msg->osr.obs[0].lock = 15;
  msg->osr.obs[0].range_std = 7;
  msg->osr.obs[0].sid.code = 0;
  msg->osr.obs[0].sid.sat = 1;
  msg->osr.obs[0].tropo_std = 7;
  msg->osr.obs[1].L.f = 75;
  msg->osr.obs[1].L.i = 111817196;
  msg->osr.obs[1].P = 1063905486;
  msg->osr.obs[1].flags = 3;
  msg->osr.obs[1].iono_std = 13;
  msg->osr.obs[1].lock = 15;
  msg->osr.obs[1].range_std = 3;
  msg->osr.obs[1].sid.code = 0;
  msg->osr.obs[1].sid.sat = 13;
  msg->osr.obs[1].tropo_std = 3;
  msg->osr.obs[2].L.f = 128;
  msg->osr.obs[2].L.i = 110692129;
  msg->osr.obs[2].P = 1053200685;
  msg->osr.obs[2].flags = 3;
  msg->osr.obs[2].iono_std = 13;
  msg->osr.obs[2].lock = 15;
  msg->osr.obs[2].range_std = 3;
  msg->osr.obs[2].sid.code = 0;
  msg->osr.obs[2].sid.sat = 14;
  msg->osr.obs[2].tropo_std = 3;
  msg->osr.obs[3].L.f = 127;
  msg->osr.obs[3].L.i = 119549583;
  msg->osr.obs[3].P = 1137476697;
  msg->osr.obs[3].flags = 3;
  msg->osr.obs[3].iono_std = 13;
  msg->osr.obs[3].lock = 15;
  msg->osr.obs[3].range_std = 5;
  msg->osr.obs[3].sid.code = 0;
  msg->osr.obs[3].sid.sat = 15;
  msg->osr.obs[3].tropo_std = 5;
  msg->osr.obs[4].L.f = 55;
  msg->osr.obs[4].L.i = 106934294;
  msg->osr.obs[4].P = 1017446132;
  msg->osr.obs[4].flags = 3;
  msg->osr.obs[4].iono_std = 0;
  msg->osr.obs[4].lock = 15;
  msg->osr.obs[4].range_std = 2;
  msg->osr.obs[4].sid.code = 0;
  msg->osr.obs[4].sid.sat = 17;
  msg->osr.obs[4].tropo_std = 2;
  msg->osr.obs[5].L.f = 108;
  msg->osr.obs[5].L.i = 110024343;
  msg->osr.obs[5].P = 1046846826;
  msg->osr.obs[5].flags = 3;
  msg->osr.obs[5].iono_std = 13;
  msg->osr.obs[5].lock = 15;
  msg->osr.obs[5].range_std = 3;
  msg->osr.obs[5].sid.code = 0;
  msg->osr.obs[5].sid.sat = 19;
  msg->osr.obs[5].tropo_std = 3;
  msg->osr.obs[6].L.f = 206;
  msg->osr.obs[6].L.i = 111507381;
  msg->osr.obs[6].P = 1060957521;
  msg->osr.obs[6].flags = 3;
  msg->osr.obs[6].iono_std = 13;
  msg->osr.obs[6].lock = 15;
  msg->osr.obs[6].range_std = 3;
  msg->osr.obs[6].sid.code = 0;
  msg->osr.obs[6].sid.sat = 28;
  msg->osr.obs[6].tropo_std = 3;
  msg->osr.obs[7].L.f = 200;
  msg->osr.obs[7].L.i = 113614775;
  msg->osr.obs[7].P = 1081009286;
  msg->osr.obs[7].flags = 3;
  msg->osr.obs[7].iono_std = 13;
  msg->osr.obs[7].lock = 15;
  msg->osr.obs[7].range_std = 3;
  msg->osr.obs[7].sid.code = 0;
  msg->osr.obs[7].sid.sat = 30;
  msg->osr.obs[7].tropo_std = 3;
  msg->osr.obs[8].L.f = 170;
  msg->osr.obs[8].L.i = 94728270;
  msg->osr.obs[8].P = 1156681781;
  msg->osr.obs[8].flags = 3;
  msg->osr.obs[8].iono_std = 21;
  msg->osr.obs[8].lock = 15;
  msg->osr.obs[8].range_std = 7;
  msg->osr.obs[8].sid.code = 6;
  msg->osr.obs[8].sid.sat = 1;
  msg->osr.obs[8].tropo_std = 7;
  msg->osr.obs[9].L.f = 129;
  msg->osr.obs[9].L.i = 87130275;
  msg->osr.obs[9].P = 1063905531;
  msg->osr.obs[9].flags = 3;
  msg->osr.obs[9].iono_std = 21;
  msg->osr.obs[9].lock = 15;
  msg->osr.obs[9].range_std = 3;
  msg->osr.obs[9].sid.code = 6;
  msg->osr.obs[9].sid.sat = 13;
  msg->osr.obs[9].tropo_std = 3;
  msg->osr.obs[10].L.f = 46;
  msg->osr.obs[10].L.i = 86253605;
  msg->osr.obs[10].P = 1053200752;
  msg->osr.obs[10].flags = 3;
  msg->osr.obs[10].iono_std = 21;
  msg->osr.obs[10].lock = 15;
  msg->osr.obs[10].range_std = 3;
  msg->osr.obs[10].sid.code = 6;
  msg->osr.obs[10].sid.sat = 14;
  msg->osr.obs[10].tropo_std = 3;
  msg->osr.obs[11].L.f = 95;
  msg->osr.obs[11].L.i = 93155512;
  msg->osr.obs[11].P = 1137476774;
  msg->osr.obs[11].flags = 3;
  msg->osr.obs[11].iono_std = 21;
  msg->osr.obs[11].lock = 15;
  msg->osr.obs[11].range_std = 5;
  msg->osr.obs[11].sid.code = 6;
  msg->osr.obs[11].sid.sat = 15;
  msg->osr.obs[11].tropo_std = 5;
}

void get_SbpMsgSvAzEl(sbp_msg_t *msg,
                      sbp_msg_type_t *msg_type,
                      uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 31183;
  *msg_type = SbpMsgSvAzEl;

  msg->sv_az_el.azel[0].az = 160;
  msg->sv_az_el.azel[0].el = 12;
  msg->sv_az_el.azel[0].sid.code = 0;
  msg->sv_az_el.azel[0].sid.sat = 8;
  msg->sv_az_el.azel[1].az = 139;
  msg->sv_az_el.azel[1].el = 66;
  msg->sv_az_el.azel[1].sid.code = 0;
  msg->sv_az_el.azel[1].sid.sat = 10;
  msg->sv_az_el.azel[2].az = 16;
  msg->sv_az_el.azel[2].el = 1;
  msg->sv_az_el.azel[2].sid.code = 0;
  msg->sv_az_el.azel[2].sid.sat = 13;
  msg->sv_az_el.azel[3].az = 24;
  msg->sv_az_el.azel[3].el = 25;
  msg->sv_az_el.azel[3].sid.code = 0;
  msg->sv_az_el.azel[3].sid.sat = 15;
  msg->sv_az_el.azel[4].az = 127;
  msg->sv_az_el.azel[4].el = 18;
  msg->sv_az_el.azel[4].sid.code = 0;
  msg->sv_az_el.azel[4].sid.sat = 16;
  msg->sv_az_el.azel[5].az = 42;
  msg->sv_az_el.azel[5].el = 53;
  msg->sv_az_el.azel[5].sid.code = 0;
  msg->sv_az_el.azel[5].sid.sat = 18;
  msg->sv_az_el.azel[6].az = 31;
  msg->sv_az_el.azel[6].el = 16;
  msg->sv_az_el.azel[6].sid.code = 0;
  msg->sv_az_el.azel[6].sid.sat = 20;
  msg->sv_az_el.azel[7].az = 12;
  msg->sv_az_el.azel[7].el = 67;
  msg->sv_az_el.azel[7].sid.code = 0;
  msg->sv_az_el.azel[7].sid.sat = 23;
  msg->sv_az_el.azel[8].az = 47;
  msg->sv_az_el.azel[8].el = 10;
  msg->sv_az_el.azel[8].sid.code = 0;
  msg->sv_az_el.azel[8].sid.sat = 24;
  msg->sv_az_el.azel[9].az = 116;
  msg->sv_az_el.azel[9].el = 8;
  msg->sv_az_el.azel[9].sid.code = 0;
  msg->sv_az_el.azel[9].sid.sat = 26;
  msg->sv_az_el.azel[10].az = 153;
  msg->sv_az_el.azel[10].el = 43;
  msg->sv_az_el.azel[10].sid.code = 0;
  msg->sv_az_el.azel[10].sid.sat = 27;
  msg->sv_az_el.azel[11].az = 77;
  msg->sv_az_el.azel[11].el = 10;
  msg->sv_az_el.azel[11].sid.code = 0;
  msg->sv_az_el.azel[11].sid.sat = 29;
  msg->sv_az_el.azel[12].az = 94;
  msg->sv_az_el.azel[12].el = 26;
  msg->sv_az_el.azel[12].sid.code = 0;
  msg->sv_az_el.azel[12].sid.sat = 32;
  msg->sv_az_el.azel[13].az = 16;
  msg->sv_az_el.azel[13].el = 58;
  msg->sv_az_el.azel[13].sid.code = 3;
  msg->sv_az_el.azel[13].sid.sat = 1;
  msg->sv_az_el.azel[14].az = 108;
  msg->sv_az_el.azel[14].el = 53;
  msg->sv_az_el.azel[14].sid.code = 3;
  msg->sv_az_el.azel[14].sid.sat = 2;
  msg->sv_az_el.azel[15].az = 17;
  msg->sv_az_el.azel[15].el = 13;
  msg->sv_az_el.azel[15].sid.code = 3;
  msg->sv_az_el.azel[15].sid.sat = 8;
  msg->sv_az_el.azel[16].az = 165;
  msg->sv_az_el.azel[16].el = 40;
  msg->sv_az_el.azel[16].sid.code = 3;
  msg->sv_az_el.azel[16].sid.sat = 17;
  msg->sv_az_el.azel[17].az = 63;
  msg->sv_az_el.azel[17].el = 35;
  msg->sv_az_el.azel[17].sid.code = 3;
  msg->sv_az_el.azel[17].sid.sat = 23;
  msg->sv_az_el.azel[18].az = 41;
  msg->sv_az_el.azel[18].el = 73;
  msg->sv_az_el.azel[18].sid.code = 3;
  msg->sv_az_el.azel[18].sid.sat = 24;
  msg->sv_az_el.azel[19].az = 114;
  msg->sv_az_el.azel[19].el = 26;
  msg->sv_az_el.azel[19].sid.code = 12;
  msg->sv_az_el.azel[19].sid.sat = 20;
  msg->sv_az_el.azel[20].az = 72;
  msg->sv_az_el.azel[20].el = 54;
  msg->sv_az_el.azel[20].sid.code = 12;
  msg->sv_az_el.azel[20].sid.sat = 27;
  msg->sv_az_el.azel[21].az = 69;
  msg->sv_az_el.azel[21].el = 3;
  msg->sv_az_el.azel[21].sid.code = 12;
  msg->sv_az_el.azel[21].sid.sat = 28;
  msg->sv_az_el.azel[22].az = 158;
  msg->sv_az_el.azel[22].el = 14;
  msg->sv_az_el.azel[22].sid.code = 12;
  msg->sv_az_el.azel[22].sid.sat = 29;
  msg->sv_az_el.azel[23].az = 152;
  msg->sv_az_el.azel[23].el = 68;
  msg->sv_az_el.azel[23].sid.code = 12;
  msg->sv_az_el.azel[23].sid.sat = 30;
  msg->sv_az_el.azel[24].az = 120;
  msg->sv_az_el.azel[24].el = 82;
  msg->sv_az_el.azel[24].sid.code = 12;
  msg->sv_az_el.azel[24].sid.sat = 32;
  msg->sv_az_el.azel[25].az = 131;
  msg->sv_az_el.azel[25].el = 6;
  msg->sv_az_el.azel[25].sid.code = 14;
  msg->sv_az_el.azel[25].sid.sat = 2;
  msg->sv_az_el.azel[26].az = 27;
  msg->sv_az_el.azel[26].el = 44;
  msg->sv_az_el.azel[26].sid.code = 14;
  msg->sv_az_el.azel[26].sid.sat = 4;
  msg->sv_az_el.azel[27].az = 101;
  msg->sv_az_el.azel[27].el = 21;
  msg->sv_az_el.azel[27].sid.code = 14;
  msg->sv_az_el.azel[27].sid.sat = 5;
  msg->sv_az_el.azel[28].az = 81;
  msg->sv_az_el.azel[28].el = 65;
  msg->sv_az_el.azel[28].sid.code = 14;
  msg->sv_az_el.azel[28].sid.sat = 9;
  msg->sv_az_el.azel[29].az = 49;
  msg->sv_az_el.azel[29].el = 56;
  msg->sv_az_el.azel[29].sid.code = 14;
  msg->sv_az_el.azel[29].sid.sat = 11;
  msg->sv_az_el.azel[30].az = 59;
  msg->sv_az_el.azel[30].el = 6;
  msg->sv_az_el.azel[30].sid.code = 14;
  msg->sv_az_el.azel[30].sid.sat = 12;
  msg->sv_az_el.azel[31].az = 154;
  msg->sv_az_el.azel[31].el = 4;
  msg->sv_az_el.azel[31].sid.code = 14;
  msg->sv_az_el.azel[31].sid.sat = 30;
  msg->sv_az_el.azel[32].az = 165;
  msg->sv_az_el.azel[32].el = 62;
  msg->sv_az_el.azel[32].sid.code = 14;
  msg->sv_az_el.azel[32].sid.sat = 36;
  msg->sv_az_el.n_azel = 33;
}

void get_SbpMsgAngularRate(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgAngularRate;

  msg->angular_rate.flags = 0;
  msg->angular_rate.tow = 2;
  msg->angular_rate.x = 2;
  msg->angular_rate.y = 5;
  msg->angular_rate.z = 2;
}

void get_SbpMsgOrientEuler(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgOrientEuler;

  msg->orient_euler.flags = 3;
  msg->orient_euler.pitch = 2;
  msg->orient_euler.pitch_accuracy = 3.0;
  msg->orient_euler.roll = 1;
  msg->orient_euler.roll_accuracy = 7.0;
  msg->orient_euler.tow = 1;
  msg->orient_euler.yaw = 8;
  msg->orient_euler.yaw_accuracy = 7.0;
}

void get_SbpMsgOrientQuat(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgOrientQuat;

  msg->orient_quat.flags = 1;
  msg->orient_quat.tow = 0;
  msg->orient_quat.w = 3;
  msg->orient_quat.w_accuracy = 3.0;
  msg->orient_quat.x = 7;
  msg->orient_quat.x_accuracy = 4.0;
  msg->orient_quat.y = 8;
  msg->orient_quat.y_accuracy = 8.0;
  msg->orient_quat.z = 4;
  msg->orient_quat.z_accuracy = 3.0;
}

void get_SbpMsgDeviceMonitor(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 16991;
  *msg_type = SbpMsgDeviceMonitor;

  msg->device_monitor.cpu_temperature = 6165;
  msg->device_monitor.cpu_vaux = 1789;
  msg->device_monitor.cpu_vint = 987;
  msg->device_monitor.dev_vin = -9999;
  msg->device_monitor.fe_temperature = 4776;
}

void get_SbpMsgIarState(sbp_msg_t *msg,
                        sbp_msg_type_t *msg_type,
                        uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgIarState;

  msg->iar_state.num_hyps = 1;
}

void get_SbpMsgNetworkBandwidthUsage(sbp_msg_t *msg,
                                     sbp_msg_type_t *msg_type,
                                     uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 31183;
  *msg_type = SbpMsgNetworkBandwidthUsage;

  msg->network_bandwidth_usage.interfaces[0].duration = 2159176030;
  {
    const char assign_string[] = {(char)99,
                                  (char)97,
                                  (char)110,
                                  (char)48,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0};
    memcpy(msg->network_bandwidth_usage.interfaces[0].interface_name,
           assign_string,
           sizeof(assign_string));
  }
  msg->network_bandwidth_usage.interfaces[0].rx_bytes = 0;
  msg->network_bandwidth_usage.interfaces[0].total_bytes = 0;
  msg->network_bandwidth_usage.interfaces[0].tx_bytes = 0;
  msg->network_bandwidth_usage.interfaces[1].duration = 2159176030;
  {
    const char assign_string[] = {(char)99,
                                  (char)97,
                                  (char)110,
                                  (char)49,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0};
    memcpy(msg->network_bandwidth_usage.interfaces[1].interface_name,
           assign_string,
           sizeof(assign_string));
  }
  msg->network_bandwidth_usage.interfaces[1].rx_bytes = 0;
  msg->network_bandwidth_usage.interfaces[1].total_bytes = 0;
  msg->network_bandwidth_usage.interfaces[1].tx_bytes = 0;
  msg->network_bandwidth_usage.interfaces[2].duration = 2159176030;
  {
    const char assign_string[] = {(char)101,
                                  (char)116,
                                  (char)104,
                                  (char)48,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0};
    memcpy(msg->network_bandwidth_usage.interfaces[2].interface_name,
           assign_string,
           sizeof(assign_string));
  }
  msg->network_bandwidth_usage.interfaces[2].rx_bytes = 4036234989;
  msg->network_bandwidth_usage.interfaces[2].total_bytes = 3411995557;
  msg->network_bandwidth_usage.interfaces[2].tx_bytes = 3670727864;
  msg->network_bandwidth_usage.interfaces[3].duration = 2159176030;
  {
    const char assign_string[] = {(char)108,
                                  (char)111,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0};
    memcpy(msg->network_bandwidth_usage.interfaces[3].interface_name,
           assign_string,
           sizeof(assign_string));
  }
  msg->network_bandwidth_usage.interfaces[3].rx_bytes = 0;
  msg->network_bandwidth_usage.interfaces[3].total_bytes = 0;
  msg->network_bandwidth_usage.interfaces[3].tx_bytes = 0;
  msg->network_bandwidth_usage.interfaces[4].duration = 2159176030;
  {
    const char assign_string[] = {(char)115,
                                  (char)105,
                                  (char)116,
                                  (char)48,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0,
                                  (char)0};
    memcpy(msg->network_bandwidth_usage.interfaces[4].interface_name,
           assign_string,
           sizeof(assign_string));
  }
  msg->network_bandwidth_usage.interfaces[4].rx_bytes = 0;
  msg->network_bandwidth_usage.interfaces[4].total_bytes = 0;
  msg->network_bandwidth_usage.interfaces[4].tx_bytes = 0;
  msg->network_bandwidth_usage.n_interfaces = 5;
}

void get_SbpMsgThreadState(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgThreadState;

  msg->thread_state.cpu = 0;
  {
    const char assign_string[] = {
        (char)109, (char)97, (char)105, (char)110, (char)0, (char)0, (char)0,
        (char)0,   (char)0,  (char)0,   (char)0,   (char)0, (char)0, (char)0,
        (char)0,   (char)0,  (char)0,   (char)0,   (char)0, (char)0};
    memcpy(msg->thread_state.name, assign_string, sizeof(assign_string));
  }
  msg->thread_state.stack_free = 2460;
}

void get_SbpMsgUartStateDepa(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 1219;
  *msg_type = SbpMsgUartStateDepa;

  msg->uart_state_depa.latency.avg = -1;
  msg->uart_state_depa.latency.current = -1;
  msg->uart_state_depa.latency.lmax = 0;
  msg->uart_state_depa.latency.lmin = 0;
  msg->uart_state_depa.uart_a.crc_error_count = 0;
  msg->uart_state_depa.uart_a.io_error_count = 0;
  msg->uart_state_depa.uart_a.rx_buffer_level = 0;
  msg->uart_state_depa.uart_a.rx_throughput = 0.0;
  msg->uart_state_depa.uart_a.tx_buffer_level = 0;
  msg->uart_state_depa.uart_a.tx_throughput = 0.0;
  msg->uart_state_depa.uart_b.crc_error_count = 0;
  msg->uart_state_depa.uart_b.io_error_count = 0;
  msg->uart_state_depa.uart_b.rx_buffer_level = 0;
  msg->uart_state_depa.uart_b.rx_throughput = 0.0;
  msg->uart_state_depa.uart_b.tx_buffer_level = 0;
  msg->uart_state_depa.uart_b.tx_throughput = 0.0;
  msg->uart_state_depa.uart_ftdi.crc_error_count = 0;
  msg->uart_state_depa.uart_ftdi.io_error_count = 0;
  msg->uart_state_depa.uart_ftdi.rx_buffer_level = 0;
  msg->uart_state_depa.uart_ftdi.rx_throughput = 0.0;
  msg->uart_state_depa.uart_ftdi.tx_buffer_level = 15;
  msg->uart_state_depa.uart_ftdi.tx_throughput = 11.600000381469727;
}

void get_SbpMsgSbasRaw(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 51228;
  *msg_type = SbpMsgSbasRaw;

  msg->sbas_raw.data[0] = 23;
  msg->sbas_raw.data[1] = 255;
  msg->sbas_raw.data[2] = 0;
  msg->sbas_raw.data[3] = 23;
  msg->sbas_raw.data[4] = 255;
  msg->sbas_raw.data[5] = 0;
  msg->sbas_raw.data[6] = 23;
  msg->sbas_raw.data[7] = 255;
  msg->sbas_raw.data[8] = 127;
  msg->sbas_raw.data[9] = 240;
  msg->sbas_raw.data[10] = 2;
  msg->sbas_raw.data[11] = 255;
  msg->sbas_raw.data[12] = 192;
  msg->sbas_raw.data[13] = 3;
  msg->sbas_raw.data[14] = 127;
  msg->sbas_raw.data[15] = 247;
  msg->sbas_raw.data[16] = 255;
  msg->sbas_raw.data[17] = 127;
  msg->sbas_raw.data[18] = 247;
  msg->sbas_raw.data[19] = 255;
  msg->sbas_raw.data[20] = 229;
  msg->sbas_raw.data[21] = 229;
  msg->sbas_raw.data[22] = 238;
  msg->sbas_raw.data[23] = 170;
  msg->sbas_raw.data[24] = 175;
  msg->sbas_raw.data[25] = 255;
  msg->sbas_raw.data[26] = 240;
  msg->sbas_raw.message_type = 4;
  msg->sbas_raw.sid.code = 2;
  msg->sbas_raw.sid.sat = 131;
  msg->sbas_raw.tow = 501867721;
}

void get_SbpMsgSettingsReadByIndexDone(sbp_msg_t *msg,
                                       sbp_msg_type_t *msg_type,
                                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgSettingsReadByIndexDone;

  (void)msg;
}

void get_SbpMsgSettingsReadByIndexResp(sbp_msg_t *msg,
                                       sbp_msg_type_t *msg_type,
                                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgSettingsReadByIndexResp;

  msg->settings_read_by_index_resp.index = 0;
  sbp_msg_settings_read_by_index_resp_setting_add_section(
      &msg->settings_read_by_index_resp,
      "AT&F,ATS1=115,ATS2=128,ATS5=0,AT&W,ATZ");
}

void get_SbpMsgSsrCodePhaseBiasesBounds(sbp_msg_t *msg,
                                        sbp_msg_type_t *msg_type,
                                        uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrCodePhaseBiasesBounds;

  msg->ssr_code_phase_biases_bounds.const_id = 1;
  msg->ssr_code_phase_biases_bounds.header.num_msgs = 1;
  msg->ssr_code_phase_biases_bounds.header.seq_num = 2;
  msg->ssr_code_phase_biases_bounds.header.sol_id = 14;
  msg->ssr_code_phase_biases_bounds.header.time.tow = 180;
  msg->ssr_code_phase_biases_bounds.header.time.wn = 3;
  msg->ssr_code_phase_biases_bounds.header.update_interval = 1;
  msg->ssr_code_phase_biases_bounds.n_sats_signals = 3;
  msg->ssr_code_phase_biases_bounds.satellites_signals[0].code_bias_bound_mu =
      39;
  msg->ssr_code_phase_biases_bounds.satellites_signals[0].code_bias_bound_sig =
      1;
  msg->ssr_code_phase_biases_bounds.satellites_signals[0].phase_bias_bound_mu =
      39;
  msg->ssr_code_phase_biases_bounds.satellites_signals[0].phase_bias_bound_sig =
      1;
  msg->ssr_code_phase_biases_bounds.satellites_signals[0].sat_id = 0;
  msg->ssr_code_phase_biases_bounds.satellites_signals[0].signal_id = 3;
  msg->ssr_code_phase_biases_bounds.satellites_signals[1].code_bias_bound_mu =
      39;
  msg->ssr_code_phase_biases_bounds.satellites_signals[1].code_bias_bound_sig =
      1;
  msg->ssr_code_phase_biases_bounds.satellites_signals[1].phase_bias_bound_mu =
      39;
  msg->ssr_code_phase_biases_bounds.satellites_signals[1].phase_bias_bound_sig =
      1;
  msg->ssr_code_phase_biases_bounds.satellites_signals[1].sat_id = 1;
  msg->ssr_code_phase_biases_bounds.satellites_signals[1].signal_id = 3;
  msg->ssr_code_phase_biases_bounds.satellites_signals[2].code_bias_bound_mu =
      39;
  msg->ssr_code_phase_biases_bounds.satellites_signals[2].code_bias_bound_sig =
      1;
  msg->ssr_code_phase_biases_bounds.satellites_signals[2].phase_bias_bound_mu =
      39;
  msg->ssr_code_phase_biases_bounds.satellites_signals[2].phase_bias_bound_sig =
      1;
  msg->ssr_code_phase_biases_bounds.satellites_signals[2].sat_id = 1;
  msg->ssr_code_phase_biases_bounds.satellites_signals[2].signal_id = 1;
  msg->ssr_code_phase_biases_bounds.ssr_iod = 15;
}

void get_SbpMsgSsrGriddedCorrectionBounds(sbp_msg_t *msg,
                                          sbp_msg_type_t *msg_type,
                                          uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrGriddedCorrectionBounds;

  msg->ssr_gridded_correction_bounds.grid_point_id = 1000;
  msg->ssr_gridded_correction_bounds.header.num_msgs = 1;
  msg->ssr_gridded_correction_bounds.header.seq_num = 1;
  msg->ssr_gridded_correction_bounds.header.sol_id = 0;
  msg->ssr_gridded_correction_bounds.header.time.tow = 180;
  msg->ssr_gridded_correction_bounds.header.time.wn = 3;
  msg->ssr_gridded_correction_bounds.header.update_interval = 10;
  msg->ssr_gridded_correction_bounds.n_sats = 2;
  msg->ssr_gridded_correction_bounds.ssr_iod_atmo = 15;
  msg->ssr_gridded_correction_bounds.stec_sat_list[0].stec_bound_mu = 18;
  msg->ssr_gridded_correction_bounds.stec_sat_list[0].stec_bound_mu_dot = 20;
  msg->ssr_gridded_correction_bounds.stec_sat_list[0].stec_bound_sig = 19;
  msg->ssr_gridded_correction_bounds.stec_sat_list[0].stec_bound_sig_dot = 21;
  msg->ssr_gridded_correction_bounds.stec_sat_list[0].stec_residual.residual =
      16;
  msg->ssr_gridded_correction_bounds.stec_sat_list[0].stec_residual.stddev = 17;
  msg->ssr_gridded_correction_bounds.stec_sat_list[0]
      .stec_residual.sv_id.constellation = 10;
  msg->ssr_gridded_correction_bounds.stec_sat_list[0]
      .stec_residual.sv_id.satId = 5;
  msg->ssr_gridded_correction_bounds.stec_sat_list[1].stec_bound_mu = 24;
  msg->ssr_gridded_correction_bounds.stec_sat_list[1].stec_bound_mu_dot = 26;
  msg->ssr_gridded_correction_bounds.stec_sat_list[1].stec_bound_sig = 25;
  msg->ssr_gridded_correction_bounds.stec_sat_list[1].stec_bound_sig_dot = 27;
  msg->ssr_gridded_correction_bounds.stec_sat_list[1].stec_residual.residual =
      22;
  msg->ssr_gridded_correction_bounds.stec_sat_list[1].stec_residual.stddev = 23;
  msg->ssr_gridded_correction_bounds.stec_sat_list[1]
      .stec_residual.sv_id.constellation = 10;
  msg->ssr_gridded_correction_bounds.stec_sat_list[1]
      .stec_residual.sv_id.satId = 6;
  msg->ssr_gridded_correction_bounds.tile_id = 10;
  msg->ssr_gridded_correction_bounds.tile_set_id = 1;
  msg->ssr_gridded_correction_bounds.tropo_delay_correction.hydro = 500;
  msg->ssr_gridded_correction_bounds.tropo_delay_correction.stddev = 200;
  msg->ssr_gridded_correction_bounds.tropo_delay_correction.wet = 100;
  msg->ssr_gridded_correction_bounds.tropo_qi = 39;
  msg->ssr_gridded_correction_bounds.tropo_v_hydro_bound_mu = 150;
  msg->ssr_gridded_correction_bounds.tropo_v_hydro_bound_sig = 100;
  msg->ssr_gridded_correction_bounds.tropo_v_wet_bound_mu = 150;
  msg->ssr_gridded_correction_bounds.tropo_v_wet_bound_sig = 100;
}

void get_SbpMsgSsrOrbitClockBounds(sbp_msg_t *msg,
                                   sbp_msg_type_t *msg_type,
                                   uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrOrbitClockBounds;

  msg->ssr_orbit_clock_bounds.const_id = 1;
  msg->ssr_orbit_clock_bounds.header.num_msgs = 1;
  msg->ssr_orbit_clock_bounds.header.seq_num = 2;
  msg->ssr_orbit_clock_bounds.header.sol_id = 48;
  msg->ssr_orbit_clock_bounds.header.time.tow = 180;
  msg->ssr_orbit_clock_bounds.header.time.wn = 3;
  msg->ssr_orbit_clock_bounds.header.update_interval = 3;
  msg->ssr_orbit_clock_bounds.n_sats = 2;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[0].clock_bound_mu = 39;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[0].clock_bound_sig = 1;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[0].orb_along_bound_mu = 38;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[0].orb_along_bound_sig = 2;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[0].orb_cross_bound_mu = 37;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[0].orb_cross_bound_sig = 3;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[0].orb_radial_bound_mu = 39;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[0].orb_radial_bound_sig = 1;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[0].sat_id = 24;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[1].clock_bound_mu = 39;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[1].clock_bound_sig = 1;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[1].orb_along_bound_mu = 38;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[1].orb_along_bound_sig = 2;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[1].orb_cross_bound_mu = 37;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[1].orb_cross_bound_sig = 3;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[1].orb_radial_bound_mu = 39;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[1].orb_radial_bound_sig = 1;
  msg->ssr_orbit_clock_bounds.orbit_clock_bounds[1].sat_id = 3;
  msg->ssr_orbit_clock_bounds.ssr_iod = 15;
}

void get_SbpMsgSsrOrbitClockBoundsDegradation(sbp_msg_t *msg,
                                              sbp_msg_type_t *msg_type,
                                              uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrOrbitClockBoundsDegradation;

  msg->ssr_orbit_clock_bounds_degradation.const_id = 1;
  msg->ssr_orbit_clock_bounds_degradation.header.num_msgs = 1;
  msg->ssr_orbit_clock_bounds_degradation.header.seq_num = 2;
  msg->ssr_orbit_clock_bounds_degradation.header.sol_id = 48;
  msg->ssr_orbit_clock_bounds_degradation.header.time.tow = 180;
  msg->ssr_orbit_clock_bounds_degradation.header.time.wn = 3;
  msg->ssr_orbit_clock_bounds_degradation.header.update_interval = 3;
  msg->ssr_orbit_clock_bounds_degradation.orbit_clock_bounds_degradation
      .clock_bound_mu_dot = 194;
  msg->ssr_orbit_clock_bounds_degradation.orbit_clock_bounds_degradation
      .clock_bound_sig_dot = 193;
  msg->ssr_orbit_clock_bounds_degradation.orbit_clock_bounds_degradation
      .orb_along_bound_mu_dot = 199;
  msg->ssr_orbit_clock_bounds_degradation.orbit_clock_bounds_degradation
      .orb_along_bound_sig_dot = 196;
  msg->ssr_orbit_clock_bounds_degradation.orbit_clock_bounds_degradation
      .orb_cross_bound_mu_dot = 198;
  msg->ssr_orbit_clock_bounds_degradation.orbit_clock_bounds_degradation
      .orb_cross_bound_sig_dot = 195;
  msg->ssr_orbit_clock_bounds_degradation.orbit_clock_bounds_degradation
      .orb_radial_bound_mu_dot = 200;
  msg->ssr_orbit_clock_bounds_degradation.orbit_clock_bounds_degradation
      .orb_radial_bound_sig_dot = 197;
  msg->ssr_orbit_clock_bounds_degradation.sat_bitmask = 10;
  msg->ssr_orbit_clock_bounds_degradation.ssr_iod = 15;
}

void get_SbpMsgSsrStecCorrection(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrStecCorrection;

  msg->ssr_stec_correction.header.num_msgs = 1;
  msg->ssr_stec_correction.header.seq_num = 1;
  msg->ssr_stec_correction.header.sol_id = 0;
  msg->ssr_stec_correction.header.time.tow = 180;
  msg->ssr_stec_correction.header.time.wn = 3;
  msg->ssr_stec_correction.header.update_interval = 10;
  msg->ssr_stec_correction.n_sats = 2;
  msg->ssr_stec_correction.ssr_iod_atmo = 15;
  msg->ssr_stec_correction.stec_sat_list[0].stec_coeff[0] = 63;
  msg->ssr_stec_correction.stec_sat_list[0].stec_coeff[1] = 62;
  msg->ssr_stec_correction.stec_sat_list[0].stec_coeff[2] = 61;
  msg->ssr_stec_correction.stec_sat_list[0].stec_coeff[3] = 60;
  msg->ssr_stec_correction.stec_sat_list[0].stec_quality_indicator = 1;
  msg->ssr_stec_correction.stec_sat_list[0].sv_id.constellation = 1;
  msg->ssr_stec_correction.stec_sat_list[0].sv_id.satId = 1;
  msg->ssr_stec_correction.stec_sat_list[1].stec_coeff[0] = 63;
  msg->ssr_stec_correction.stec_sat_list[1].stec_coeff[1] = 64;
  msg->ssr_stec_correction.stec_sat_list[1].stec_coeff[2] = 65;
  msg->ssr_stec_correction.stec_sat_list[1].stec_coeff[3] = 66;
  msg->ssr_stec_correction.stec_sat_list[1].stec_quality_indicator = 5;
  msg->ssr_stec_correction.stec_sat_list[1].sv_id.constellation = 15;
  msg->ssr_stec_correction.stec_sat_list[1].sv_id.satId = 31;
  msg->ssr_stec_correction.tile_id = 10;
  msg->ssr_stec_correction.tile_set_id = 1;
}

void get_SbpMsgSsrTileDefinition(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgSsrTileDefinition;

  msg->ssr_tile_definition.bitmask = 1234567890;
  msg->ssr_tile_definition.cols = 32768;
  msg->ssr_tile_definition.corner_nw_lat = 1024;
  msg->ssr_tile_definition.corner_nw_lon = 2048;
  msg->ssr_tile_definition.rows = 16384;
  msg->ssr_tile_definition.spacing_lat = 4096;
  msg->ssr_tile_definition.spacing_lon = 8192;
  msg->ssr_tile_definition.ssr_sol_id = 31;
  msg->ssr_tile_definition.tile_id = 512;
  msg->ssr_tile_definition.tile_set_id = 256;
}

void get_SbpMsgDgnssStatus(sbp_msg_t *msg,
                           sbp_msg_type_t *msg_type,
                           uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgDgnssStatus;

  msg->dgnss_status.flags = 0;
  msg->dgnss_status.latency = 50;
  msg->dgnss_status.num_signals = 12;
  sbp_msg_dgnss_status_source_set(&msg->dgnss_status, "Skylark", false, NULL);
}

void get_SbpMsgGroupMeta(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 61166;
  *msg_type = SbpMsgGroupMeta;

  msg->group_meta.flags = 2;
  msg->group_meta.group_id = 1;
  msg->group_meta.group_msgs[0] = 65290;
  msg->group_meta.group_msgs[1] = 522;
  msg->group_meta.group_msgs[2] = 65282;
  msg->group_meta.n_group_msgs = 3;
}

void get_SbpMsgHeartbeat(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgHeartbeat;

  msg->heartbeat.flags = 12800;
}

void get_SbpMsgInsStatus(sbp_msg_t *msg,
                         sbp_msg_type_t *msg_type,
                         uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 789;
  *msg_type = SbpMsgInsStatus;

  msg->ins_status.flags = 536870921;
}

void get_SbpMsgInsUpdates(sbp_msg_t *msg,
                          sbp_msg_type_t *msg_type,
                          uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 789;
  *msg_type = SbpMsgInsUpdates;

  msg->ins_updates.gnsspos = 0;
  msg->ins_updates.gnssvel = 0;
  msg->ins_updates.nhc = 0;
  msg->ins_updates.speed = 0;
  msg->ins_updates.tow = 504489300;
  msg->ins_updates.wheelticks = 0;
  msg->ins_updates.zerovel = 0;
}

void get_SbpMsgSensorAidEvent(sbp_msg_t *msg,
                              sbp_msg_type_t *msg_type,
                              uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 35027;
  *msg_type = SbpMsgSensorAidEvent;

  msg->sensor_aid_event.flags = 0;
  msg->sensor_aid_event.n_accepted_meas = 0;
  msg->sensor_aid_event.n_attempted_meas = 0;
  msg->sensor_aid_event.n_available_meas = 0;
  msg->sensor_aid_event.sensor_id = 0;
  msg->sensor_aid_event.sensor_state = 0;
  msg->sensor_aid_event.sensor_type = 0;
  msg->sensor_aid_event.time = 326825520;
}

void get_SbpMsgStartup(sbp_msg_t *msg,
                       sbp_msg_type_t *msg_type,
                       uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgStartup;

  msg->startup.cause = 0;
  msg->startup.reserved = 0;
  msg->startup.startup_type = 0;
}

void get_SbpMsgStatusJournal(sbp_msg_t *msg,
                             sbp_msg_type_t *msg_type,
                             uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 35027;
  *msg_type = SbpMsgStatusJournal;

  msg->status_journal.journal[0].report.component = 6;
  msg->status_journal.journal[0].report.generic = 1;
  msg->status_journal.journal[0].report.specific = 13;
  msg->status_journal.journal[0].uptime = 4242;
  msg->status_journal.journal[1].report.component = 6;
  msg->status_journal.journal[1].report.generic = 1;
  msg->status_journal.journal[1].report.specific = 14;
  msg->status_journal.journal[1].uptime = 5050;
  msg->status_journal.journal[2].report.component = 6;
  msg->status_journal.journal[2].report.generic = 1;
  msg->status_journal.journal[2].report.specific = 15;
  msg->status_journal.journal[2].uptime = 8888;
  msg->status_journal.n_journal = 3;
  msg->status_journal.reporting_system = 1;
  msg->status_journal.sbp_version = 1025;
  msg->status_journal.sequence_descriptor = 16;
  msg->status_journal.total_status_reports = 100;
}

void get_SbpMsgMeasurementState(sbp_msg_t *msg,
                                sbp_msg_type_t *msg_type,
                                uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 31183;
  *msg_type = SbpMsgMeasurementState;

  msg->measurement_state.n_states = 79;
  msg->measurement_state.states[0].cn0 = 162;
  msg->measurement_state.states[0].mesid.code = 0;
  msg->measurement_state.states[0].mesid.sat = 29;
  msg->measurement_state.states[1].cn0 = 0;
  msg->measurement_state.states[1].mesid.code = 0;
  msg->measurement_state.states[1].mesid.sat = 0;
  msg->measurement_state.states[2].cn0 = 0;
  msg->measurement_state.states[2].mesid.code = 0;
  msg->measurement_state.states[2].mesid.sat = 0;
  msg->measurement_state.states[3].cn0 = 201;
  msg->measurement_state.states[3].mesid.code = 0;
  msg->measurement_state.states[3].mesid.sat = 27;
  msg->measurement_state.states[4].cn0 = 168;
  msg->measurement_state.states[4].mesid.code = 0;
  msg->measurement_state.states[4].mesid.sat = 20;
  msg->measurement_state.states[5].cn0 = 184;
  msg->measurement_state.states[5].mesid.code = 0;
  msg->measurement_state.states[5].mesid.sat = 32;
  msg->measurement_state.states[6].cn0 = 187;
  msg->measurement_state.states[6].mesid.code = 0;
  msg->measurement_state.states[6].mesid.sat = 15;
  msg->measurement_state.states[7].cn0 = 0;
  msg->measurement_state.states[7].mesid.code = 0;
  msg->measurement_state.states[7].mesid.sat = 0;
  msg->measurement_state.states[8].cn0 = 210;
  msg->measurement_state.states[8].mesid.code = 0;
  msg->measurement_state.states[8].mesid.sat = 18;
  msg->measurement_state.states[9].cn0 = 167;
  msg->measurement_state.states[9].mesid.code = 0;
  msg->measurement_state.states[9].mesid.sat = 16;
  msg->measurement_state.states[10].cn0 = 0;
  msg->measurement_state.states[10].mesid.code = 0;
  msg->measurement_state.states[10].mesid.sat = 0;
  msg->measurement_state.states[11].cn0 = 213;
  msg->measurement_state.states[11].mesid.code = 0;
  msg->measurement_state.states[11].mesid.sat = 23;
  msg->measurement_state.states[12].cn0 = 223;
  msg->measurement_state.states[12].mesid.code = 0;
  msg->measurement_state.states[12].mesid.sat = 10;
  msg->measurement_state.states[13].cn0 = 0;
  msg->measurement_state.states[13].mesid.code = 0;
  msg->measurement_state.states[13].mesid.sat = 0;
  msg->measurement_state.states[14].cn0 = 0;
  msg->measurement_state.states[14].mesid.code = 0;
  msg->measurement_state.states[14].mesid.sat = 0;
  msg->measurement_state.states[15].cn0 = 0;
  msg->measurement_state.states[15].mesid.code = 0;
  msg->measurement_state.states[15].mesid.sat = 0;
  msg->measurement_state.states[16].cn0 = 0;
  msg->measurement_state.states[16].mesid.code = 0;
  msg->measurement_state.states[16].mesid.sat = 0;
  msg->measurement_state.states[17].cn0 = 202;
  msg->measurement_state.states[17].mesid.code = 2;
  msg->measurement_state.states[17].mesid.sat = 131;
  msg->measurement_state.states[18].cn0 = 192;
  msg->measurement_state.states[18].mesid.code = 1;
  msg->measurement_state.states[18].mesid.sat = 27;
  msg->measurement_state.states[19].cn0 = 165;
  msg->measurement_state.states[19].mesid.code = 1;
  msg->measurement_state.states[19].mesid.sat = 15;
  msg->measurement_state.states[20].cn0 = 146;
  msg->measurement_state.states[20].mesid.code = 1;
  msg->measurement_state.states[20].mesid.sat = 29;
  msg->measurement_state.states[21].cn0 = 170;
  msg->measurement_state.states[21].mesid.code = 1;
  msg->measurement_state.states[21].mesid.sat = 32;
  msg->measurement_state.states[22].cn0 = 201;
  msg->measurement_state.states[22].mesid.code = 1;
  msg->measurement_state.states[22].mesid.sat = 18;
  msg->measurement_state.states[23].cn0 = 0;
  msg->measurement_state.states[23].mesid.code = 0;
  msg->measurement_state.states[23].mesid.sat = 0;
  msg->measurement_state.states[24].cn0 = 0;
  msg->measurement_state.states[24].mesid.code = 0;
  msg->measurement_state.states[24].mesid.sat = 0;
  msg->measurement_state.states[25].cn0 = 0;
  msg->measurement_state.states[25].mesid.code = 0;
  msg->measurement_state.states[25].mesid.sat = 0;
  msg->measurement_state.states[26].cn0 = 212;
  msg->measurement_state.states[26].mesid.code = 1;
  msg->measurement_state.states[26].mesid.sat = 23;
  msg->measurement_state.states[27].cn0 = 205;
  msg->measurement_state.states[27].mesid.code = 1;
  msg->measurement_state.states[27].mesid.sat = 10;
  msg->measurement_state.states[28].cn0 = 0;
  msg->measurement_state.states[28].mesid.code = 0;
  msg->measurement_state.states[28].mesid.sat = 0;
  msg->measurement_state.states[29].cn0 = 230;
  msg->measurement_state.states[29].mesid.code = 3;
  msg->measurement_state.states[29].mesid.sat = 96;
  msg->measurement_state.states[30].cn0 = 0;
  msg->measurement_state.states[30].mesid.code = 0;
  msg->measurement_state.states[30].mesid.sat = 0;
  msg->measurement_state.states[31].cn0 = 214;
  msg->measurement_state.states[31].mesid.code = 3;
  msg->measurement_state.states[31].mesid.sat = 101;
  msg->measurement_state.states[32].cn0 = 212;
  msg->measurement_state.states[32].mesid.code = 3;
  msg->measurement_state.states[32].mesid.sat = 103;
  msg->measurement_state.states[33].cn0 = 209;
  msg->measurement_state.states[33].mesid.code = 3;
  msg->measurement_state.states[33].mesid.sat = 104;
  msg->measurement_state.states[34].cn0 = 157;
  msg->measurement_state.states[34].mesid.code = 3;
  msg->measurement_state.states[34].mesid.sat = 106;
  msg->measurement_state.states[35].cn0 = 230;
  msg->measurement_state.states[35].mesid.code = 3;
  msg->measurement_state.states[35].mesid.sat = 102;
  msg->measurement_state.states[36].cn0 = 0;
  msg->measurement_state.states[36].mesid.code = 0;
  msg->measurement_state.states[36].mesid.sat = 0;
  msg->measurement_state.states[37].cn0 = 0;
  msg->measurement_state.states[37].mesid.code = 0;
  msg->measurement_state.states[37].mesid.sat = 0;
  msg->measurement_state.states[38].cn0 = 189;
  msg->measurement_state.states[38].mesid.code = 4;
  msg->measurement_state.states[38].mesid.sat = 101;
  msg->measurement_state.states[39].cn0 = 207;
  msg->measurement_state.states[39].mesid.code = 4;
  msg->measurement_state.states[39].mesid.sat = 96;
  msg->measurement_state.states[40].cn0 = 164;
  msg->measurement_state.states[40].mesid.code = 4;
  msg->measurement_state.states[40].mesid.sat = 106;
  msg->measurement_state.states[41].cn0 = 193;
  msg->measurement_state.states[41].mesid.code = 4;
  msg->measurement_state.states[41].mesid.sat = 104;
  msg->measurement_state.states[42].cn0 = 0;
  msg->measurement_state.states[42].mesid.code = 0;
  msg->measurement_state.states[42].mesid.sat = 0;
  msg->measurement_state.states[43].cn0 = 208;
  msg->measurement_state.states[43].mesid.code = 4;
  msg->measurement_state.states[43].mesid.sat = 102;
  msg->measurement_state.states[44].cn0 = 0;
  msg->measurement_state.states[44].mesid.code = 0;
  msg->measurement_state.states[44].mesid.sat = 0;
  msg->measurement_state.states[45].cn0 = 212;
  msg->measurement_state.states[45].mesid.code = 12;
  msg->measurement_state.states[45].mesid.sat = 27;
  msg->measurement_state.states[46].cn0 = 161;
  msg->measurement_state.states[46].mesid.code = 12;
  msg->measurement_state.states[46].mesid.sat = 29;
  msg->measurement_state.states[47].cn0 = 216;
  msg->measurement_state.states[47].mesid.code = 12;
  msg->measurement_state.states[47].mesid.sat = 32;
  msg->measurement_state.states[48].cn0 = 216;
  msg->measurement_state.states[48].mesid.code = 12;
  msg->measurement_state.states[48].mesid.sat = 30;
  msg->measurement_state.states[49].cn0 = 178;
  msg->measurement_state.states[49].mesid.code = 12;
  msg->measurement_state.states[49].mesid.sat = 20;
  msg->measurement_state.states[50].cn0 = 0;
  msg->measurement_state.states[50].mesid.code = 0;
  msg->measurement_state.states[50].mesid.sat = 0;
  msg->measurement_state.states[51].cn0 = 0;
  msg->measurement_state.states[51].mesid.code = 0;
  msg->measurement_state.states[51].mesid.sat = 0;
  msg->measurement_state.states[52].cn0 = 0;
  msg->measurement_state.states[52].mesid.code = 0;
  msg->measurement_state.states[52].mesid.sat = 0;
  msg->measurement_state.states[53].cn0 = 0;
  msg->measurement_state.states[53].mesid.code = 0;
  msg->measurement_state.states[53].mesid.sat = 0;
  msg->measurement_state.states[54].cn0 = 0;
  msg->measurement_state.states[54].mesid.code = 0;
  msg->measurement_state.states[54].mesid.sat = 0;
  msg->measurement_state.states[55].cn0 = 0;
  msg->measurement_state.states[55].mesid.code = 0;
  msg->measurement_state.states[55].mesid.sat = 0;
  msg->measurement_state.states[56].cn0 = 0;
  msg->measurement_state.states[56].mesid.code = 0;
  msg->measurement_state.states[56].mesid.sat = 0;
  msg->measurement_state.states[57].cn0 = 0;
  msg->measurement_state.states[57].mesid.code = 0;
  msg->measurement_state.states[57].mesid.sat = 0;
  msg->measurement_state.states[58].cn0 = 0;
  msg->measurement_state.states[58].mesid.code = 0;
  msg->measurement_state.states[58].mesid.sat = 0;
  msg->measurement_state.states[59].cn0 = 0;
  msg->measurement_state.states[59].mesid.code = 0;
  msg->measurement_state.states[59].mesid.sat = 0;
  msg->measurement_state.states[60].cn0 = 0;
  msg->measurement_state.states[60].mesid.code = 0;
  msg->measurement_state.states[60].mesid.sat = 0;
  msg->measurement_state.states[61].cn0 = 0;
  msg->measurement_state.states[61].mesid.code = 0;
  msg->measurement_state.states[61].mesid.sat = 0;
  msg->measurement_state.states[62].cn0 = 0;
  msg->measurement_state.states[62].mesid.code = 0;
  msg->measurement_state.states[62].mesid.sat = 0;
  msg->measurement_state.states[63].cn0 = 203;
  msg->measurement_state.states[63].mesid.code = 14;
  msg->measurement_state.states[63].mesid.sat = 36;
  msg->measurement_state.states[64].cn0 = 0;
  msg->measurement_state.states[64].mesid.code = 0;
  msg->measurement_state.states[64].mesid.sat = 0;
  msg->measurement_state.states[65].cn0 = 158;
  msg->measurement_state.states[65].mesid.code = 14;
  msg->measurement_state.states[65].mesid.sat = 5;
  msg->measurement_state.states[66].cn0 = 194;
  msg->measurement_state.states[66].mesid.code = 14;
  msg->measurement_state.states[66].mesid.sat = 4;
  msg->measurement_state.states[67].cn0 = 192;
  msg->measurement_state.states[67].mesid.code = 14;
  msg->measurement_state.states[67].mesid.sat = 11;
  msg->measurement_state.states[68].cn0 = 207;
  msg->measurement_state.states[68].mesid.code = 14;
  msg->measurement_state.states[68].mesid.sat = 9;
  msg->measurement_state.states[69].cn0 = 0;
  msg->measurement_state.states[69].mesid.code = 0;
  msg->measurement_state.states[69].mesid.sat = 0;
  msg->measurement_state.states[70].cn0 = 0;
  msg->measurement_state.states[70].mesid.code = 0;
  msg->measurement_state.states[70].mesid.sat = 0;
  msg->measurement_state.states[71].cn0 = 0;
  msg->measurement_state.states[71].mesid.code = 0;
  msg->measurement_state.states[71].mesid.sat = 0;
  msg->measurement_state.states[72].cn0 = 218;
  msg->measurement_state.states[72].mesid.code = 20;
  msg->measurement_state.states[72].mesid.sat = 9;
  msg->measurement_state.states[73].cn0 = 176;
  msg->measurement_state.states[73].mesid.code = 20;
  msg->measurement_state.states[73].mesid.sat = 5;
  msg->measurement_state.states[74].cn0 = 217;
  msg->measurement_state.states[74].mesid.code = 20;
  msg->measurement_state.states[74].mesid.sat = 36;
  msg->measurement_state.states[75].cn0 = 200;
  msg->measurement_state.states[75].mesid.code = 20;
  msg->measurement_state.states[75].mesid.sat = 11;
  msg->measurement_state.states[76].cn0 = 205;
  msg->measurement_state.states[76].mesid.code = 20;
  msg->measurement_state.states[76].mesid.sat = 4;
  msg->measurement_state.states[77].cn0 = 0;
  msg->measurement_state.states[77].mesid.code = 0;
  msg->measurement_state.states[77].mesid.sat = 0;
  msg->measurement_state.states[78].cn0 = 0;
  msg->measurement_state.states[78].mesid.code = 0;
  msg->measurement_state.states[78].mesid.sat = 0;
}

void get_SbpMsgTrackingStateDepB(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 55286;
  *msg_type = SbpMsgTrackingStateDepB;

  msg->tracking_state_dep_b.n_states = 11;
  msg->tracking_state_dep_b.states[0].cn0 = 39.24782180786133;
  msg->tracking_state_dep_b.states[0].sid.code = 0;
  msg->tracking_state_dep_b.states[0].sid.reserved = 0;
  msg->tracking_state_dep_b.states[0].sid.sat = 202;
  msg->tracking_state_dep_b.states[0].state = 1;
  msg->tracking_state_dep_b.states[1].cn0 = 36.09756088256836;
  msg->tracking_state_dep_b.states[1].sid.code = 0;
  msg->tracking_state_dep_b.states[1].sid.reserved = 0;
  msg->tracking_state_dep_b.states[1].sid.sat = 203;
  msg->tracking_state_dep_b.states[1].state = 1;
  msg->tracking_state_dep_b.states[2].cn0 = 37.62678527832031;
  msg->tracking_state_dep_b.states[2].sid.code = 0;
  msg->tracking_state_dep_b.states[2].sid.reserved = 0;
  msg->tracking_state_dep_b.states[2].sid.sat = 208;
  msg->tracking_state_dep_b.states[2].state = 1;
  msg->tracking_state_dep_b.states[3].cn0 = 39.020729064941406;
  msg->tracking_state_dep_b.states[3].sid.code = 0;
  msg->tracking_state_dep_b.states[3].sid.reserved = 0;
  msg->tracking_state_dep_b.states[3].sid.sat = 212;
  msg->tracking_state_dep_b.states[3].state = 1;
  msg->tracking_state_dep_b.states[4].cn0 = 42.03290557861328;
  msg->tracking_state_dep_b.states[4].sid.code = 0;
  msg->tracking_state_dep_b.states[4].sid.reserved = 0;
  msg->tracking_state_dep_b.states[4].sid.sat = 217;
  msg->tracking_state_dep_b.states[4].state = 1;
  msg->tracking_state_dep_b.states[5].cn0 = 37.43546676635742;
  msg->tracking_state_dep_b.states[5].sid.code = 0;
  msg->tracking_state_dep_b.states[5].sid.reserved = 0;
  msg->tracking_state_dep_b.states[5].sid.sat = 218;
  msg->tracking_state_dep_b.states[5].state = 1;
  msg->tracking_state_dep_b.states[6].cn0 = 38.4229621887207;
  msg->tracking_state_dep_b.states[6].sid.code = 0;
  msg->tracking_state_dep_b.states[6].sid.reserved = 0;
  msg->tracking_state_dep_b.states[6].sid.sat = 220;
  msg->tracking_state_dep_b.states[6].state = 1;
  msg->tracking_state_dep_b.states[7].cn0 = 38.91520309448242;
  msg->tracking_state_dep_b.states[7].sid.code = 0;
  msg->tracking_state_dep_b.states[7].sid.reserved = 0;
  msg->tracking_state_dep_b.states[7].sid.sat = 222;
  msg->tracking_state_dep_b.states[7].state = 1;
  msg->tracking_state_dep_b.states[8].cn0 = 42.62259292602539;
  msg->tracking_state_dep_b.states[8].sid.code = 0;
  msg->tracking_state_dep_b.states[8].sid.reserved = 0;
  msg->tracking_state_dep_b.states[8].sid.sat = 225;
  msg->tracking_state_dep_b.states[8].state = 1;
  msg->tracking_state_dep_b.states[9].cn0 = -1.0;
  msg->tracking_state_dep_b.states[9].sid.code = 0;
  msg->tracking_state_dep_b.states[9].sid.reserved = 0;
  msg->tracking_state_dep_b.states[9].sid.sat = 0;
  msg->tracking_state_dep_b.states[9].state = 0;
  msg->tracking_state_dep_b.states[10].cn0 = -1.0;
  msg->tracking_state_dep_b.states[10].sid.code = 0;
  msg->tracking_state_dep_b.states[10].sid.reserved = 0;
  msg->tracking_state_dep_b.states[10].sid.sat = 0;
  msg->tracking_state_dep_b.states[10].state = 0;
}

void get_SbpMsgTrackingStateDepA(sbp_msg_t *msg,
                                 sbp_msg_type_t *msg_type,
                                 uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 1219;
  *msg_type = SbpMsgTrackingStateDepA;

  msg->tracking_state_dep_a.n_states = 11;
  msg->tracking_state_dep_a.states[0].cn0 = 11.230907440185547;
  msg->tracking_state_dep_a.states[0].prn = 0;
  msg->tracking_state_dep_a.states[0].state = 1;
  msg->tracking_state_dep_a.states[1].cn0 = 10.438665390014648;
  msg->tracking_state_dep_a.states[1].prn = 2;
  msg->tracking_state_dep_a.states[1].state = 1;
  msg->tracking_state_dep_a.states[2].cn0 = 9.732142448425293;
  msg->tracking_state_dep_a.states[2].prn = 3;
  msg->tracking_state_dep_a.states[2].state = 1;
  msg->tracking_state_dep_a.states[3].cn0 = 14.341922760009766;
  msg->tracking_state_dep_a.states[3].prn = 7;
  msg->tracking_state_dep_a.states[3].state = 1;
  msg->tracking_state_dep_a.states[4].cn0 = 7.8549017906188965;
  msg->tracking_state_dep_a.states[4].prn = 10;
  msg->tracking_state_dep_a.states[4].state = 1;
  msg->tracking_state_dep_a.states[5].cn0 = 5.0982866287231445;
  msg->tracking_state_dep_a.states[5].prn = 13;
  msg->tracking_state_dep_a.states[5].state = 1;
  msg->tracking_state_dep_a.states[6].cn0 = 6.741272926330566;
  msg->tracking_state_dep_a.states[6].prn = 22;
  msg->tracking_state_dep_a.states[6].state = 1;
  msg->tracking_state_dep_a.states[7].cn0 = 12.700549125671387;
  msg->tracking_state_dep_a.states[7].prn = 30;
  msg->tracking_state_dep_a.states[7].state = 1;
  msg->tracking_state_dep_a.states[8].cn0 = 15.893081665039062;
  msg->tracking_state_dep_a.states[8].prn = 31;
  msg->tracking_state_dep_a.states[8].state = 1;
  msg->tracking_state_dep_a.states[9].cn0 = 4.242738723754883;
  msg->tracking_state_dep_a.states[9].prn = 25;
  msg->tracking_state_dep_a.states[9].state = 1;
  msg->tracking_state_dep_a.states[10].cn0 = 6.97599983215332;
  msg->tracking_state_dep_a.states[10].prn = 6;
  msg->tracking_state_dep_a.states[10].state = 1;
}

void get_SbpMsgTrackingStateDetailedDep(sbp_msg_t *msg,
                                        sbp_msg_type_t *msg_type,
                                        uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 26427;
  *msg_type = SbpMsgTrackingStateDetailedDep;

  msg->tracking_state_detailed_dep.L.f = 169;
  msg->tracking_state_detailed_dep.L.i = 1319;
  msg->tracking_state_detailed_dep.P = 0;
  msg->tracking_state_detailed_dep.P_std = 0;
  msg->tracking_state_detailed_dep.acceleration = 108;
  msg->tracking_state_detailed_dep.clock_drift = 0;
  msg->tracking_state_detailed_dep.clock_offset = 0;
  msg->tracking_state_detailed_dep.cn0 = 177;
  msg->tracking_state_detailed_dep.corr_spacing = 40;
  msg->tracking_state_detailed_dep.doppler = 15701;
  msg->tracking_state_detailed_dep.doppler_std = 39;
  msg->tracking_state_detailed_dep.lock = 14032;
  msg->tracking_state_detailed_dep.misc_flags = 9;
  msg->tracking_state_detailed_dep.nav_flags = 0;
  msg->tracking_state_detailed_dep.pset_flags = 0;
  msg->tracking_state_detailed_dep.recv_time = 7909447587;
  msg->tracking_state_detailed_dep.sid.code = 0;
  msg->tracking_state_detailed_dep.sid.reserved = 0;
  msg->tracking_state_detailed_dep.sid.sat = 15;
  msg->tracking_state_detailed_dep.sync_flags = 1;
  msg->tracking_state_detailed_dep.tot.tow = 0;
  msg->tracking_state_detailed_dep.tot.wn = 0;
  msg->tracking_state_detailed_dep.tow_flags = 0;
  msg->tracking_state_detailed_dep.track_flags = 11;
  msg->tracking_state_detailed_dep.uptime = 1;
}

void get_SbpMsgOdometry(sbp_msg_t *msg,
                        sbp_msg_type_t *msg_type,
                        uint16_t *sender_id) {
  assert(msg);
  assert(msg_type);
  assert(sender_id);

  *sender_id = 66;
  *msg_type = SbpMsgOdometry;

  msg->odometry.flags = 1;
  msg->odometry.tow = 8;
  msg->odometry.velocity = 7;
}
