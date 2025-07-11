/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include "check_rtcm3.h"

#include <check.h>
#include <libsbp/sbp.h>
#include <libsbp/v4/observation.h>
#include <math.h>
#include <rtcm3/decode.h>
#include <rtcm3/encode.h>
#include <rtcm3/eph_decode.h>
#include <rtcm3/eph_encode.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <swiftnav/ephemeris.h>
#include <swiftnav/gnss_time.h>
#include <swiftnav/sid_set.h>

#include "config.h"

#define GPS_TOW_TOLERANCE 1e-4

gps_time_t current_time;

static double expected_L1CA_bias = 0.0;
static double expected_L1P_bias = 0.0;
static double expected_L2CA_bias = 0.0;
static double expected_L2P_bias = 0.0;

static sbp_v4_gps_time_t previous_obs_time = {.tow = 0, .wn = INVALID_TIME};
static u8 previous_n_meas = 0;
static u8 previous_num_obs = 0;

static struct rtcm3_sbp_state state;
static struct rtcm3_sbp_state state_gsv;
static struct rtcm3_out_state out_state;

static const uint32_t crc24qtab[256] = {
    0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC,
    0x9F7F17, 0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23,
    0xB8B2D5, 0x3EFE2E, 0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868,
    0xD0E493, 0xDC7D65, 0x5A319E, 0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646,
    0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7, 0x0CD1E9, 0x8A9D12, 0x8604E4,
    0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE, 0xAD50D0, 0x2B1C2B,
    0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7, 0xC99F60,
    0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
    0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5,
    0xF7614E, 0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8,
    0x00903E, 0x86DCC5, 0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A,
    0xAD88F1, 0xA11107, 0x275DFC, 0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD,
    0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C, 0x7D6C62, 0xFB2099, 0xF7B96F,
    0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375, 0x15723B, 0x933EC0,
    0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C, 0xB4F302,
    0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
    0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E,
    0x4F43A5, 0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791,
    0x688E67, 0xEEC29C, 0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145,
    0x26EDBE, 0x2A7448, 0xAC38B3, 0x92C69D, 0x148A66, 0x181390, 0x9E5F6B,
    0x01207C, 0x876C87, 0x8BF571, 0x0DB98A, 0xF6092D, 0x7045D6, 0x7CDC20,
    0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A, 0x578814, 0xD1C4EF,
    0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703, 0x3F964D,
    0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
    0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498,
    0x016863, 0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE,
    0xE3EB28, 0x65A7D3, 0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C,
    0x4EF3E7, 0x426A11, 0xC426EA, 0x2AE476, 0xACA88D, 0xA0317B, 0x267D80,
    0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61, 0x8B654F, 0x0D29B4, 0x01B042,
    0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58, 0xEFAAFF, 0x69E604,
    0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8, 0x4E2BC6,
    0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
    0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673,
    0xB94A88, 0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC,
    0x9E874A, 0x18CBB1, 0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7,
    0xF6D10C, 0xFA48FA, 0x7C0401, 0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9,
    0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538};

static sbp_packed_obs_content_t sbp_test_data[] = {
    {1076594107, {113150797, 178}, {2025, 90}, 200, 14, 15, {8, 0}},
    {1052792371, {110649219, 83}, {1784, 3}, 204, 14, 15, {10, 0}},
    {1211420676, {127321212, 230}, {-2004, 131}, 148, 14, 15, {13, 0}},
    {1145898161, {120434726, 100}, {-691, 105}, 184, 14, 15, {15, 0}},
    {1193292504, {125415891, 224}, {-3715, 140}, 176, 14, 15, {16, 0}},
    {1212730365, {127458823, 44}, {3918, 243}, 164, 14, 15, {18, 0}},
    {1028016946, {108045309, 73}, {-373, 29}, 196, 14, 15, {20, 0}},
    {1146192135, {120465590, 214}, {-2489, 240}, 192, 14, 15, {21, 0}},
    {1022557522, {107471517, 97}, {-276, 33}, 212, 14, 15, {27, 0}},
    {1222990444, {128537159, 17}, {-902, 76}, 148, 14, 15, {30, 0}},
    {1076594282, {88169456, 168}, {1578, 51}, 176, 14, 15, {8, 1}},
    {1052792374, {86220160, 1}, {1390, 100}, 180, 14, 15, {10, 1}},
    {1145898151, {93845224, 62}, {-539, 207}, 148, 14, 15, {15, 1}},
    {1022557545, {83744032, 221}, {-215, 37}, 188, 14, 15, {27, 1}},
    {1222990548, {100158785, 33}, {-704, 56}, 132, 13, 15, {30, 1}},
    {1065177381, {113679866, 211}, {-1361, 186}, 184, 14, 15, {2, 3}},
    {996013864, {106634862, 13}, {1314, 57}, 200, 14, 15, {3, 3}},
    {1128091581, {120817628, 56}, {4471, 1}, 176, 14, 15, {4, 3}},
    {1141411804, {121901553, 33}, {-3635, 8}, 184, 14, 15, {9, 3}},
    {1079759346, {115114554, 22}, {-540, 62}, 188, 14, 15, {10, 3}},
    {1170011510, {125043767, 130}, {3080, 218}, 160, 14, 15, {11, 3}},
    {1011426617, {107981325, 222}, {-2569, 35}, 200, 14, 15, {18, 3}},
    {970789084, {103861396, 130}, {1345, 241}, 180, 14, 15, {19, 3}},
    {1065177465, {88417685, 1}, {-1059, 235}, 160, 14, 15, {2, 4}},
    {996014042, {82938235, 192}, {1022, 59}, 180, 14, 15, {3, 4}},
    {1128091758, {93969291, 225}, {3477, 209}, 160, 11, 15, {4, 4}},
    {1141411880, {94812306, 17}, {-2827, 66}, 168, 14, 15, {9, 4}},
    {1079759506, {89533572, 17}, {-420, 93}, 136, 14, 15, {10, 4}},
    {1170011343, {97256230, 93}, {2396, 185}, 128, 8, 15, {11, 4}},
    {1011426783, {83985473, 117}, {-1998, 17}, 180, 14, 15, {18, 4}},
    {970789145, {80781068, 14}, {1046, 255}, 164, 14, 15, {19, 4}},
    {1905584379, {198457625, 111}, {310, 79}, 164, 12, 15, {6, 12}},
    {1904198770, {198313311, 251}, {1273, 149}, 168, 13, 15, {9, 12}},
    {1124221717, {117082227, 36}, {-611, 155}, 180, 13, 15, {14, 12}},
    {1905584415, {153459993, 183}, {239, 150}, 168, 12, 15, {6, 13}},
    {1904198738, {153348404, 139}, {985, 53}, 176, 13, 15, {9, 13}},
    {1124219524, {90535361, 31}, {-472, 18}, 180, 13, 15, {14, 13}},
    {1308619286, {137536820, 202}, {166, 211}, 176, 13, 15, {2, 14}},
    {1367072989, {143680335, 86}, {3236, 169}, 160, 13, 15, {4, 14}},
    {1111185587, {116786402, 20}, {1435, 244}, 180, 13, 15, {11, 14}},
    {1116077611, {117300562, 142}, {-1454, 238}, 180, 13, 15, {12, 14}},
    {1243548316, {130697813, 141}, {1221, 137}, 172, 13, 15, {19, 14}},
    {1297377968, {136355345, 162}, {-2489, 229}, 168, 13, 15, {25, 14}},
    {1308619388, {105385329, 253}, {127, 202}, 160, 13, 15, {2, 20}},
    {1367073089, {110092710, 5}, {2479, 154}, 168, 13, 15, {4, 20}},
    {1111185468, {89485640, 18}, {1100, 108}, 172, 13, 15, {11, 20}},
    {1116077555, {89879619, 94}, {-1114, 214}, 172, 13, 15, {12, 20}},
    {1243548284, {100145050, 137}, {935, 195}, 164, 13, 15, {19, 20}},
    {1297378054, {104480052, 213}, {-1907, 185}, 168, 13, 15, {25, 20}}};

static sbp_packed_obs_content_t sbp_test_data_old[] = {
    {1076594107, {113150797, 178}, {2025, 90}, 200, 14, 15, {8, 0}},
    {1052792371, {110649219, 83}, {1784, 3}, 204, 14, 15, {10, 0}},
    {1211420676, {127321212, 230}, {-2004, 131}, 148, 14, 15, {13, 0}},
    {1145898161, {120434726, 100}, {-691, 105}, 184, 14, 15, {15, 0}},
    {1193292504, {125415891, 224}, {-3715, 140}, 176, 14, 15, {16, 0}},
    {1212730365, {127458823, 44}, {3918, 243}, 164, 14, 15, {18, 0}},
    {1028016946, {108045309, 73}, {-373, 29}, 196, 14, 15, {20, 0}},
    {1146192135, {120465590, 214}, {-2489, 240}, 192, 14, 15, {21, 0}},
    {1022557522, {107471517, 97}, {-276, 33}, 212, 14, 15, {27, 0}},
    {1222990444, {128537159, 17}, {-902, 76}, 148, 14, 15, {30, 0}},
    {1076594282, {88169456, 168}, {1578, 51}, 176, 14, 15, {8, 1}},
    {1052792374, {86220160, 1}, {1390, 100}, 180, 14, 15, {10, 1}},
    {1145898151, {93845224, 62}, {-539, 207}, 148, 14, 15, {15, 1}},
    {1022557545, {83744032, 221}, {-215, 37}, 188, 14, 15, {27, 1}},
    {1222990548, {100158785, 33}, {-704, 56}, 132, 13, 15, {30, 1}},
    {1065177381, {113679866, 211}, {-1361, 186}, 184, 14, 15, {2, 3}},
    {996013864, {106634862, 13}, {1314, 57}, 200, 14, 15, {3, 3}},
    {1128091581, {120817628, 56}, {4471, 1}, 176, 14, 15, {4, 3}},
    {1141411804, {121901553, 33}, {-3635, 8}, 184, 14, 15, {9, 3}},
    {1079759346, {115114554, 22}, {-540, 62}, 188, 14, 15, {10, 3}},
    {1170011510, {125043767, 130}, {3080, 218}, 160, 14, 15, {11, 3}},
    {1011426617, {107981325, 222}, {-2569, 35}, 200, 14, 15, {18, 3}},
    {970789084, {103861396, 130}, {1345, 241}, 180, 14, 15, {19, 3}},
    {1065177465, {88417685, 1}, {-1059, 235}, 160, 14, 15, {2, 4}},
    {996014042, {82938235, 192}, {1022, 59}, 180, 14, 15, {3, 4}},
    {1128091758, {93969291, 225}, {3477, 209}, 160, 11, 15, {4, 4}},
    {1141411880, {94812306, 17}, {-2827, 66}, 168, 14, 15, {9, 4}},
    {1079759506, {89533572, 17}, {-420, 93}, 136, 14, 15, {10, 4}},
    {1170011343, {97256230, 93}, {2396, 185}, 128, 8, 15, {11, 4}},
    {1011426783, {83985473, 117}, {-1998, 17}, 180, 14, 15, {18, 4}},
    {970789145, {80781068, 14}, {1046, 255}, 164, 14, 15, {19, 4}},
    {1905584379, {198457625, 111}, {310, 79}, 164, 12, 15, {6, 12}},
    {1904198770, {198313311, 251}, {1273, 149}, 168, 13, 15, {9, 12}},
    {1124221717, {117082227, 36}, {-611, 155}, 180, 13, 15, {14, 12}},
    {1905584415, {153459993, 183}, {239, 150}, 168, 12, 15, {6, 13}},
    {1904198738, {153348404, 139}, {985, 53}, 176, 13, 15, {9, 13}},
    {1124219524, {90535361, 31}, {-472, 18}, 180, 13, 15, {14, 13}},
    {1308619286, {137536820, 202}, {166, 211}, 176, 13, 15, {2, 14}},
    {1367072989, {143680335, 86}, {3236, 169}, 160, 13, 15, {4, 14}},
    {1111185587, {116786402, 20}, {1435, 244}, 180, 13, 15, {11, 14}},
    {1116077611, {117300562, 142}, {-1454, 238}, 180, 13, 15, {12, 14}},
    {1243548316, {130697813, 141}, {1221, 137}, 172, 13, 15, {19, 14}},
    {1297377968, {136355345, 162}, {-2489, 229}, 168, 13, 15, {25, 14}},
    {1308619388, {105385329, 253}, {127, 202}, 160, 13, 15, {2, 20}},
    {1367073089, {110092710, 5}, {2479, 154}, 168, 13, 15, {4, 20}},
    {1111185468, {89485640, 18}, {1100, 108}, 172, 13, 15, {11, 20}},
    {1116077555, {89879619, 94}, {-1114, 214}, 172, 13, 15, {12, 20}},
    {1243548284, {100145050, 137}, {935, 195}, 164, 13, 15, {19, 20}},
    {1297378054, {104480052, 213}, {-1907, 185}, 168, 13, 15, {25, 20}}};

static uint32_t crc24q(const uint8_t *buf, uint32_t len, uint32_t crc) {
  for (uint32_t i = 0; i < len; i++) {
    crc = ((crc << 8) & 0xFFFFFF) ^ crc24qtab[((crc >> 16) ^ buf[i]) & 0xff];
  }
  return crc;
}

/* Difference between two sbp time stamps in seconds */
static double sbp_time_diff(const sbp_v4_gps_time_t *end,
                            const sbp_v4_gps_time_t *beginning) {
  s32 week_diff = end->wn - beginning->wn;
  double dt = (double)end->tow / SECS_MS - (double)beginning->tow / SECS_MS;
  dt += week_diff * WEEK_SECS;
  return dt;
}

void update_obs_time(const sbp_msg_obs_t *msg) {
  gps_time_t obs_time = {.tow = (double)msg[0].header.t.tow / SECS_MS,
                         .wn = msg[0].header.t.wn};
  rtcm2sbp_set_time(&obs_time, NULL, &state);
}

void sbp_callback_gps(uint16_t sender_id,
                      sbp_msg_type_t msg_type,
                      const sbp_msg_t *msg,
                      void *context) {
  (void)sender_id;
  (void)context;
  static uint32_t msg_count = 0;
  /* ignore log messages */
  if (msg_type == SbpMsgLog) {
    return;
  }
  if (msg_count == 3 || msg_count == 20 || msg_count == 42) {
    ck_assert_uint_eq(msg_type, SbpMsgBasePosEcef);
  } else if (msg_count == 4) {
    ck_assert_uint_eq(msg_type, SbpMsgGloBiases);
  } else {
    ck_assert_uint_eq(msg_type, SbpMsgObs);
    update_obs_time((const sbp_msg_obs_t *)msg);
  }
  msg_count++;
}

void sbp_callback_gps_eph(uint16_t sender_id,
                          sbp_msg_type_t msg_type,
                          const sbp_msg_t *sbp_msg,
                          void *context) {
  (void)sender_id;
  (void)context;
  static bool checked_eph = false;
  /* ignore log messages */
  if (msg_type == SbpMsgEphemerisGps && !checked_eph) {
    checked_eph = true;
    const sbp_msg_ephemeris_gps_t *msg =
        (const sbp_msg_ephemeris_gps_t *)sbp_msg;
    ck_assert(msg->common.sid.sat == 1);
    ck_assert(msg->common.sid.code == CODE_GPS_L1CA);
    ck_assert(msg->common.toe.wn == 2012);
    ck_assert(msg->common.toe.tow == 489600);
    ck_assert(fabs(msg->common.ura - 2.8) < FLOAT_EPS);
    ck_assert(msg->common.fit_interval == 14400);
    ck_assert(msg->common.valid == 1);
    ck_assert(msg->common.health_bits == 0);

    ck_assert(fabs(msg->tgd - 5.587935447692871e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->c_rs - 10.34375) < FLOAT_EPS);
    ck_assert(fabs(msg->c_rc - 269.15625) < FLOAT_EPS);
    ck_assert(fabs(msg->c_uc - 7.599592208862305e-7) < FLOAT_EPS);
    ck_assert(fabs(msg->c_us - 6.021931767463684e-6) < FLOAT_EPS);
    ck_assert(fabs(msg->c_ic + 1.1175870895385742e-8) < FLOAT_EPS);
    ck_assert(fabs(msg->c_is - 1.4901161193847656e-7) < FLOAT_EPS);

    ck_assert(fabs(msg->dn - 4.626621288776081e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->m0 - 1.0410100899287642) < FLOAT_EPS);
    ck_assert(fabs(msg->ecc - 8.054995676502585e-3) < FLOAT_EPS);
    ck_assert(fabs(msg->sqrta - 5153.665510177612) < FLOAT_EPS);
    ck_assert(fabs(msg->omega0 - 2.4056622999366137) < FLOAT_EPS);
    ck_assert(fabs(msg->omegadot + 8.301774373653793e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->w - 0.673068866191342) < FLOAT_EPS);
    ck_assert(fabs(msg->inc - 0.9720926277884092) < FLOAT_EPS);
    ck_assert(fabs(msg->inc_dot - 6.643133856047175e-11) < FLOAT_EPS);

    ck_assert(fabs(msg->af0 + 7.250206544995308e-5) < FLOAT_EPS);
    ck_assert(fabs(msg->af1 + 3.979039320256561e-12) < FLOAT_EPS);
    ck_assert(fabs(msg->af2 - 0.0) < FLOAT_EPS);

    ck_assert(msg->toc.wn == 2012);
    ck_assert(msg->toc.tow == 489600);
    ck_assert(msg->iode == 104);
    ck_assert(msg->iodc == 104);
  }
}

void sbp_callback_glo_eph(uint16_t sender_id,
                          sbp_msg_type_t msg_type,
                          const sbp_msg_t *sbp_msg,
                          void *context) {
  (void)sender_id;
  (void)context;
  static bool checked_eph = false;
  /* ignore log messages */
  if (msg_type == SbpMsgEphemerisGlo && !checked_eph) {
    // clang-format off
    /* Truth data from related RINEX file
     R03 2018 08 20 22 45 00  1.410758122802e-04  0.000000000000e+00  8.100000000000e+04
     7.333627929688e+03      -1.816708564758e+00  0.000000000000e+00  0.000000000000e+00
     1.684325878906e+04      -1.609944343567e+00 -9.313225746155e-10  5.000000000000e+00
    -1.763590478516e+04      -2.291357994080e+00  1.862645149231e-09  0.000000000000e+00
     */
    /* Truth from Haskell converter
    {
      "gamma": 0,
      "vel": [
        -1816.7085647583008,
        -1609.9443435668945,
        -2291.35799407959
      ],
      "iod": 28,
      "d_tau": -2.7939677238464355e-09,
      "pos": [
        7333627.9296875,
        16843258.7890625,
        -17635904.78515625
      ],
      "crc": 39648,
      "common": {
        "health_bits": 0,
        "fit_interval": 2400,
        "valid": 1,
        "sid": {
          "sat": 3,
          "code": 3
        },
        "toe": {
          "wn": 2015,
          "tow": 254718
        },
        "ura": 2.5
      },
      "tau": -0.00014107581228017807,
      "fcn": 13,
      "acc": [
        0,
        -9.313225746154785e-07,
        1.862645149230957e-06
      ]
    }*/
    // clang-format on
    checked_eph = true;
    const sbp_msg_ephemeris_glo_t *msg =
        (const sbp_msg_ephemeris_glo_t *)sbp_msg;
    ck_assert(msg->common.sid.sat == 3);
    ck_assert(msg->common.sid.code == CODE_GLO_L1OF);
    ck_assert(msg->common.toe.wn == 2015);
    ck_assert(msg->common.toe.tow == 168318);
    ck_assert(fabs(msg->common.ura - 2.5) < FLOAT_EPS);
    ck_assert(msg->common.fit_interval == 2400);
    ck_assert(msg->common.valid == 1);
    ck_assert(msg->common.health_bits == 0);
    ck_assert(msg->iod == 28);

    ck_assert(fabs(msg->pos[0] - 7.333627929688e6) <
              GLO_SATELLITE_POSITION_EPS_METERS);
    ck_assert(fabs(msg->pos[1] - 1.684325878906e7) <
              GLO_SATELLITE_POSITION_EPS_METERS);
    ck_assert(fabs(msg->pos[2] - -1.763590478516e7) <
              GLO_SATELLITE_POSITION_EPS_METERS);

    ck_assert(fabs(msg->vel[0] - -1.816708564758e3) < FLOAT_EPS);
    ck_assert(fabs(msg->vel[1] - -1.609944343567e3) < FLOAT_EPS);
    ck_assert(fabs(msg->vel[2] - -2.291357994080e3) < FLOAT_EPS);

    ck_assert(fabsf(msg->acc[0] - 0) < FLOAT_EPS);
    ck_assert(fabsf(msg->acc[1] - -9.313225746155e-7f) < FLOAT_EPS);
    ck_assert(fabsf(msg->acc[2] - 1.862645149231e-6f) < FLOAT_EPS);

    ck_assert(fabs(msg->gamma - 0) < FLOAT_EPS);
    ck_assert(fabs(msg->d_tau - -2.7939677238464355e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->tau - -0.00014107581228017807) < FLOAT_EPS);
  }
}

void sbp_callback_gal_eph(uint16_t sender_id,
                          sbp_msg_type_t msg_type,
                          const sbp_msg_t *sbp_msg,
                          void *context) {
  (void)sender_id;
  (void)context;
  static bool checked_eph = false;
  /* ignore log messages */
  if (msg_type == SbpMsgEphemerisGal && !checked_eph) {
    // clang-format off
/*
 E01 2018 08 14 04 00 00  -4.123143153265e-04 -8.284928298963e-12 0.000000000000e+00
     5.600000000000e+01   -7.200000000000e+01 3.055484416047e-09 4.735169303722e-01
    -3.268942236900e-06    2.873298944905e-04 7.478520274162e-06 5.440604309082e+03
     1.872000000000e+05    7.264316082001e-08-3.479377544441e-01-2.421438694000e-08
     9.920462291303e-01    1.913437500000e+02-2.811246434921e+00-5.637734834285e-09
    -7.757465986739e-10    5.170000000000e+02 2.014000000000e+03
     3.120000000000e+00    0.000000000000e+00-4.889443516731e-09-5.587935447693e-09
     1.878640000000e+05
     */
    // clang-format on
    checked_eph = true;
    const sbp_msg_ephemeris_gal_t *msg =
        (const sbp_msg_ephemeris_gal_t *)sbp_msg;
    ck_assert(msg->common.sid.sat == 1);
    ck_assert(msg->common.sid.code == CODE_GAL_E1B);
    ck_assert(msg->common.toe.wn == 2014);
    ck_assert(msg->common.toe.tow == 187200);
    ck_assert(fabs(msg->common.ura - 3.12) < FLOAT_EPS);
    ck_assert(msg->common.fit_interval == 14400);
    ck_assert(msg->common.valid == 1);
    ck_assert(msg->common.health_bits == 0);

    ck_assert(fabs(msg->bgd_e1e5a - -4.889443516731e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->bgd_e1e5b - 0) < FLOAT_EPS);
    ck_assert(fabs(msg->c_rs - -7.200000000000e1) < FLOAT_EPS);
    ck_assert(fabs(msg->c_rc - 1.913437500000e2) < FLOAT_EPS);
    ck_assert(fabs(msg->c_uc - -3.268942236900e-6) < FLOAT_EPS);
    ck_assert(fabs(msg->c_us - 7.478520274162e-6) < FLOAT_EPS);
    ck_assert(fabs(msg->c_ic - 7.264316082001e-8) < FLOAT_EPS);
    ck_assert(fabs(msg->c_is - -2.421438694000e-8) < FLOAT_EPS);

    ck_assert(fabs(msg->dn - 3.055484416047e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->m0 - 4.735169303722e-1) < FLOAT_EPS);
    ck_assert(fabs(msg->ecc - 2.873298944905e-4) < FLOAT_EPS);
    ck_assert(fabs(msg->sqrta - 5.440604309082e3) < FLOAT_EPS);
    ck_assert(fabs(msg->omega0 - -3.479377544441e-1) < FLOAT_EPS);
    ck_assert(fabs(msg->omegadot - -5.637734834285e-9) < FLOAT_EPS);
    ck_assert(fabs(msg->w - -2.811246434921e0) < FLOAT_EPS);
    ck_assert(fabs(msg->inc - 9.920462291303e-1) < FLOAT_EPS);
    ck_assert(fabs(msg->inc_dot - -7.757465986739e-10) < FLOAT_EPS);

    ck_assert(fabs(msg->af0 - -4.123143153265e-4) < FLOAT_EPS);
    ck_assert(fabs(msg->af1 - -8.284928298963e-12) < FLOAT_EPS);
    ck_assert(fabs(msg->af2 - 0.0) < FLOAT_EPS);

    ck_assert(msg->toc.wn == 2014);
    ck_assert(msg->toc.tow == 187200);
    ck_assert(msg->iode == 56);
    ck_assert(msg->iodc == 56);
  }
}

void sbp_callback_bds_eph(uint16_t sender_id,
                          sbp_msg_type_t msg_type,
                          const sbp_msg_t *sbp_msg,
                          void *context) {
  (void)sender_id;
  (void)context;
  static bool checked_eph = false;
  /* ignore log messages */
  if (msg_type == SbpMsgEphemerisBds && !checked_eph) {
    // clang-format off
/*
C06 2018 08 14 04 00 00 2.242858754471e-04 2.508659946443e-11 1.924458856162e-18
     6.000000000000e+00-4.404687500000e+01 6.643133856047e-10 1.319269256677e+00
    -5.727633833885e-07 7.140220026486e-03 3.428151831031e-05 6.492804334641e+03
     1.872000000000e+05 1.392327249050e-07-2.286649500564e+00 2.207234501839e-07
     9.452314900799e-01-8.311562500000e+02-2.220683918913e+00-1.816147078387e-09
     7.307447241652e-10 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00
     2.000000000000e+00 0.000000000000e+00 8.100000000000e-09-1.800000000000e-09
     0.000000000000e+00 5.000000000000e+00
     */
    // clang-format on
    checked_eph = true;
    const sbp_msg_ephemeris_bds_t *msg =
        (const sbp_msg_ephemeris_bds_t *)sbp_msg;
    ck_assert(msg->common.sid.sat == 6);
    ck_assert(msg->common.sid.code == CODE_BDS2_B1);

    ck_assert(msg->common.toe.wn == 2014);
    ck_assert(msg->common.toe.tow == 187214);
    ck_assert(fabs(msg->common.ura - 2.0) < FLOAT_EPS);
    ck_assert(msg->common.fit_interval == 10800);
    ck_assert(msg->common.valid == 1);
    ck_assert(msg->common.health_bits == 0);

    ck_assert(fabs(msg->tgd1 - 8.100000000000e-9) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->tgd2 - -1.800000000000e-9) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->c_rs - -4.40468750000e1) < FLOAT_EPS);
    ck_assert(fabs(msg->c_rc - -8.31156250000000e2) < FLOAT_EPS);
    ck_assert(fabs(msg->c_uc - -5.7276338338850e-7) < FLOAT_EPS);
    ck_assert(fabs(msg->c_us - 3.428151831031e-5) < FLOAT_EPS);
    ck_assert(fabs(msg->c_ic - 1.392327249050e-7) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->c_is - 2.207234501839e-7) * 1e9 < FLOAT_EPS);

    ck_assert(fabs(msg->dn - 6.643133856047e-10) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->m0 - 1.319269256677) < FLOAT_EPS);
    ck_assert(fabs(msg->ecc - 7.140220026486e-3) < FLOAT_EPS);
    ck_assert(fabs(msg->sqrta - 6.492804334641e3) < FLOAT_EPS);
    ck_assert(fabs(msg->omega0 - -2.286649500564) < FLOAT_EPS);
    ck_assert(fabs(msg->omegadot - -1.816147078387e-9) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->w - -2.220683918913) < FLOAT_EPS);
    ck_assert(fabs(msg->inc - 9.452314900799e-1) < FLOAT_EPS);
    ck_assert(fabs(msg->inc_dot - 7.307447241652e-10) * 1e9 < FLOAT_EPS);

    ck_assert(fabs(msg->af0 - 2.242858754471e-4) < FLOAT_EPS);
    ck_assert(fabs(msg->af1 - 2.508659946443e-11) * 1e9 < FLOAT_EPS);
    ck_assert(fabs(msg->af2 - 1.924458856162e-18) * 1e9 < FLOAT_EPS);

    ck_assert(msg->toc.wn == 2014);
    ck_assert(msg->toc.tow == 187214);
    ck_assert(msg->iode == 6);
    ck_assert(msg->iodc == 5);
  }
}

void sbp_callback_eph_wn_rollover(uint16_t sender_id,
                                  sbp_msg_type_t msg_type,
                                  const sbp_msg_t *sbp_msg,
                                  void *context) {
  (void)sender_id;
  (void)context;
  /* ignore log messages */
  if (msg_type == SbpMsgEphemerisGps) {
    const sbp_msg_ephemeris_gps_t *msg =
        (const sbp_msg_ephemeris_gps_t *)sbp_msg;
    ck_assert(msg->common.toe.wn == 2026);
    ck_assert(msg->common.toe.tow == 0);
  }
}

void sbp_callback_eph_wn_rollover2(uint16_t sender_id,
                                   sbp_msg_type_t msg_type,
                                   const sbp_msg_t *sbp_msg,
                                   void *context) {
  (void)sender_id;
  (void)context;
  gps_time_t toe = GPS_TIME_UNKNOWN;
  gps_time_t toc = GPS_TIME_UNKNOWN;

  switch (msg_type) {
    case SbpMsgEphemerisGps: {
      const sbp_msg_ephemeris_gps_t *msg =
          (const sbp_msg_ephemeris_gps_t *)sbp_msg;
      toe.wn = msg->common.toe.wn;
      toe.tow = msg->common.toe.tow;
      toc.wn = msg->toc.wn;
      toc.tow = msg->toc.tow;
      break;
    }
    case SbpMsgEphemerisGlo: {
      const sbp_msg_ephemeris_glo_t *msg =
          (const sbp_msg_ephemeris_glo_t *)sbp_msg;
      toe.wn = msg->common.toe.wn;
      toe.tow = msg->common.toe.tow;
      toc = toe;
      break;
    }
    case SbpMsgEphemerisGal: {
      const sbp_msg_ephemeris_gal_t *msg =
          (const sbp_msg_ephemeris_gal_t *)sbp_msg;
      toe.wn = msg->common.toe.wn;
      toe.tow = msg->common.toe.tow;
      toc.wn = msg->toc.wn;
      toc.tow = msg->toc.tow;
      break;
    }
    case SbpMsgEphemerisBds: {
      const sbp_msg_ephemeris_bds_t *msg =
          (const sbp_msg_ephemeris_bds_t *)sbp_msg;
      toe.wn = msg->common.toe.wn;
      toe.tow = msg->common.toe.tow;
      toc.wn = msg->toc.wn;
      toc.tow = msg->toc.tow;
      break;
    }
    default:
      return;
  }
  ck_assert(gps_time_valid(&toe));
  ck_assert(gps_time_valid(&toc));
  ck_assert(gpsdifftime(&toe, &toc) == 0);

  /* time of ephemeris is reasonably close to current time */
  ck_assert(fabs(gpsdifftime(&toe, &current_time)) < 4 * 3600);
}

void sbp_callback_1012_first(uint16_t sender_id,
                             sbp_msg_type_t msg_type,
                             const sbp_msg_t *sbp_msg,
                             void *context) {
  (void)sender_id;
  (void)context;
  if (msg_type == SbpMsgObs) {
    const sbp_msg_obs_t *msg = (const sbp_msg_obs_t *)sbp_msg;
    u8 num_sbp_msgs = msg->header.n_obs >> 4;
    ck_assert_uint_gt(num_sbp_msgs, 2);
    update_obs_time(msg);
  }
}

void sbp_callback_glo_5hz(uint16_t sender_id,
                          sbp_msg_type_t msg_type,
                          const sbp_msg_t *sbp_msg,
                          void *context) {
  (void)sender_id;
  (void)context;
  if (msg_type == SbpMsgObs) {
    const sbp_msg_obs_t *msg = (const sbp_msg_obs_t *)sbp_msg;
    u8 num_sbp_msgs = msg->header.n_obs >> 4;
    /* every epoch should have 4 observation messages */
    ck_assert_uint_ge(num_sbp_msgs, 4);
    update_obs_time(msg);
  }
}

void sbp_callback_glo_day_rollover(uint16_t sender_id,
                                   sbp_msg_type_t msg_type,
                                   const sbp_msg_t *sbp_msg,
                                   void *context) {
  (void)sender_id;
  (void)context;
  if (msg_type == SbpMsgObs) {
    const sbp_msg_obs_t *msg = (const sbp_msg_obs_t *)sbp_msg;
    u8 num_sbp_msgs = msg->header.n_obs >> 4;
    ck_assert_uint_gt(num_sbp_msgs, 2);
    update_obs_time(msg);
  }
}

void check_biases(const sbp_msg_glo_biases_t *sbp_glo_msg) {
  if (sbp_glo_msg->mask & 0x01) {
    ck_assert(fabs(sbp_glo_msg->l1ca_bias / GLO_BIAS_RESOLUTION -
                   expected_L1CA_bias) < 1e-8);
  }
  if (sbp_glo_msg->mask & 0x02) {
    ck_assert(fabs(sbp_glo_msg->l1p_bias / GLO_BIAS_RESOLUTION -
                   expected_L1P_bias) < 1e-8);
  }
  if (sbp_glo_msg->mask & 0x04) {
    ck_assert(fabs(sbp_glo_msg->l2ca_bias / GLO_BIAS_RESOLUTION -
                   expected_L2CA_bias) < 1e-8);
  }
  if (sbp_glo_msg->mask & 0x08) {
    ck_assert(fabs(sbp_glo_msg->l2p_bias / GLO_BIAS_RESOLUTION -
                   expected_L2P_bias) < 1e-8);
  }
}

void sbp_callback_bias(uint16_t sender_id,
                       sbp_msg_type_t msg_type,
                       const sbp_msg_t *sbp_msg,
                       void *context) {
  (void)sender_id;
  (void)context;
  if (msg_type == SbpMsgGloBiases) {
    const sbp_msg_glo_biases_t *sbp_glo_msg =
        (const sbp_msg_glo_biases_t *)sbp_msg;
    check_biases(sbp_glo_msg);
  } else if (msg_type == SbpMsgObs) {
    const sbp_msg_obs_t *sbp_glo_msg = (const sbp_msg_obs_t *)sbp_msg;
    update_obs_time(sbp_glo_msg);
  }
}

static bool no_duplicate_observations(const sbp_msg_obs_t *sbp_obs, u8 length) {
  static sbp_v4_gps_time_t epoch_time;
  static gnss_sid_set_t sid_set;

  if (epoch_time.wn != sbp_obs->header.t.wn ||
      epoch_time.tow != sbp_obs->header.t.tow) {
    epoch_time = sbp_obs->header.t;
    sid_set_init(&sid_set);
  }

  u8 num_obs = (length - 11) / 17;
  for (u8 i = 0; i < num_obs; i++) {
    gnss_signal_t sid = {.code = sbp_obs->obs[i].sid.code,
                         .sat = sbp_obs->obs[i].sid.sat};
    if (sid_set_contains(&sid_set, sid)) {
      return false;
    }
    sid_set_add(&sid_set, sid);
  }
  return true;
}

void sbp_callback_msm_switching(uint16_t sender_id,
                                sbp_msg_type_t msg_type,
                                const sbp_msg_t *sbp_msg,
                                void *context) {
  (void)sender_id;
  (void)context;
  const u32 MAX_OBS_GAP_S = MSM_TIMEOUT_SEC + 10;

  if (msg_type == SbpMsgObs) {
    const sbp_msg_obs_t *sbp_obs = &sbp_msg->obs;
    if (previous_obs_time.wn != INVALID_TIME) {
      double dt = sbp_time_diff(&sbp_obs->header.t, &previous_obs_time);
      /* make sure time does not run backwards */
      ck_assert(dt >= 0);
      /* check there aren't too long gaps between observations */
      ck_assert(dt < MAX_OBS_GAP_S);
      /* check there are no duplicate sids */
      ck_assert(no_duplicate_observations(sbp_obs, sbp_obs->n_obs));
    }
    previous_obs_time = sbp_obs->header.t;
    update_obs_time(sbp_obs);
  }
}

void sbp_callback_msm_no_gaps(uint16_t sender_id,
                              sbp_msg_type_t msg_type,
                              const sbp_msg_t *sbp_msg,
                              void *context) {
  (void)sender_id;
  (void)context;
  const u32 MAX_OBS_GAP_S = 1;

  if (msg_type == SbpMsgObs) {
    const sbp_msg_obs_t *sbp_obs = (const sbp_msg_obs_t *)sbp_msg;

    if (previous_obs_time.wn != INVALID_TIME) {
      double dt = sbp_time_diff(&sbp_obs->header.t, &previous_obs_time);
      /* make sure time does not run backwards */
      ck_assert(dt >= 0);
      /* check there's an observation for every second */
      ck_assert(dt <= MAX_OBS_GAP_S);
      /* check there are no duplicate sids */
      ck_assert(no_duplicate_observations(sbp_obs, sbp_obs->n_obs));

      u8 n_meas = sbp_obs->header.n_obs;
      u8 seq_counter = n_meas & 0x0F;
      u8 seq_size = n_meas >> 4;
      u8 prev_seq_counter = previous_n_meas & 0x0F;
      u8 prev_seq_size = previous_n_meas >> 4;

      ck_assert_uint_gt(seq_size, 0);

      if (seq_counter == 0) {
        /* new observation sequence, verify that the last one did complete */
        ck_assert_uint_eq(prev_seq_counter, prev_seq_size - 1);
      } else {
        /* verify that the previous sequence continues */
        ck_assert_uint_eq(sbp_obs->header.t.wn, previous_obs_time.wn);
        ck_assert_uint_eq(sbp_obs->header.t.tow, previous_obs_time.tow);
        ck_assert_uint_eq(seq_size, prev_seq_size);
        ck_assert_uint_eq(seq_counter, prev_seq_counter + 1);
      }
      if (seq_counter < seq_size - 1) {
        /* verify that all but the last packet in the sequence are full */
        ck_assert_uint_eq(sbp_obs->n_obs, 14);
      } else {
        /* last message in sequence, check the total number of observations */
        u8 num_obs = (seq_size - 1) * 14 + sbp_obs->n_obs;
        if (previous_num_obs > 0) {
          /* must not lose more than 6 observations between epochs */
          ck_assert_uint_ge(num_obs, previous_num_obs - 6);
        }
        previous_num_obs = num_obs;
      }
    }
    previous_obs_time = sbp_obs->header.t;
    previous_n_meas = sbp_obs->header.n_obs;
    update_obs_time(sbp_obs);
  }
}

void sbp_callback_msm7_beidou_invalid_pr(uint16_t sender_id,
                                         sbp_msg_type_t msg_type,
                                         const sbp_msg_t *sbp_msg,
                                         void *context) {
  (void)sender_id;
  (void)context;
  if (msg_type == SbpMsgObs) {
    const sbp_msg_obs_t *sbp_obs = &sbp_msg->obs;
    u8 num_obs = sbp_obs->n_obs;
    for (u8 i = 0; i < num_obs; i++) {
      const sbp_packed_obs_content_t *converted_obs = &sbp_obs->obs[i];
      // In the RTCM file used for this test, Beidou satellite 32 code B2a has
      // the fine pseudorange valid flag set to false. The RTCM converter will
      // filter out that observation as faulty. Hence, the below boolean checks
      // that the satellite + signal is missing in the received buffer.
      bool faulty_beidou_sat_exists =
          (converted_obs->sid.sat == 32 && converted_obs->sid.code == 48);
      ck_assert_uint_eq(faulty_beidou_sat_exists, false);
    }
  }
}

/* sanity check the length and CRC of the message */
bool verify_crc(uint8_t *buffer, uint32_t buffer_length) {
  if (buffer_length < 6) {
    /* buffer too short to be a valid message */
    return false;
  }
  uint16_t message_size = ((buffer[1] & 0x3) << 8) | buffer[2];
  if (buffer_length < (uint32_t)message_size + 6) {
    /* buffer too short to contain the message */
    return false;
  }
  /* Verify CRC */
  uint32_t computed_crc = crc24q(buffer, 3 + message_size, 0);
  uint32_t frame_crc = (buffer[message_size + 3] << 16) |
                       (buffer[message_size + 4] << 8) |
                       (buffer[message_size + 5] << 0);

  return (frame_crc == computed_crc);
}

void test_RTCM3(const char *filename,
                void (*cb_rtcm_to_sbp)(uint16_t sender_id,
                                       sbp_msg_type_t msg_type,
                                       const sbp_msg_t *msg,
                                       void *context),
                gps_time_t current_time_) {
  const int8_t leap_seconds = 18;

  rtcm2sbp_init(&state, NULL, cb_rtcm_to_sbp, NULL, NULL);
  rtcm2sbp_set_time(&current_time_, &leap_seconds, &state);

  previous_obs_time.wn = INVALID_TIME;
  previous_n_meas = 0;
  previous_num_obs = 0;

  FILE *fp = fopen(filename, "rb");
  u8 buffer[MAX_FILE_SIZE] = {0};
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }

  uint32_t file_size = fread(buffer, 1, MAX_FILE_SIZE, fp);
  uint32_t buffer_index = 0;

  while (buffer_index < file_size) {
    if (buffer[buffer_index] != RTCM3_PREAMBLE) {
      /* not a start of RTCM frame, seeking forward */
      buffer_index++;
      continue;
    }

    uint16_t message_size =
        ((buffer[buffer_index + 1] & 0x3) << 8) | buffer[buffer_index + 2];

    if (message_size == 0) {
      buffer_index++;
      continue;
    }
    if (message_size > 1023) {
      /* too large message */
      buffer_index++;
      continue;
    }

    if (!verify_crc(&buffer[buffer_index], file_size - buffer_index)) {
      /* CRC failure */
      buffer_index++;
      continue;
    }

    rtcm2sbp_decode_frame(
        &buffer[buffer_index], message_size + RTCM3_MSG_OVERHEAD, &state);
    /* skip pointer to the end of this message */
    buffer_index += message_size + 6;
  }
  fclose(fp);
}

static s32 sbp_read_file(u8 *buff, u32 n, void *context) {
  FILE *f = (FILE *)context;
  return fread(buff, 1, n, f);
}

static void ephemeris_glo_callback(u16 sender_id,
                                   sbp_msg_type_t msg_type,
                                   const sbp_msg_t *msg,
                                   void *context) {
  assert(msg_type == SbpMsgEphemerisGlo);

  (void)context;
  (void)sender_id;
  (void)msg_type;
  const sbp_msg_ephemeris_glo_t *e = &msg->ephemeris_glo;

  /* extract just the FCN field */
  sbp2rtcm_set_glo_fcn(e->common.sid, e->fcn, &out_state);
}

static void sbp2rtcm_sbp_void_cb(uint16_t sender_id,
                                 sbp_msg_type_t msg_type,
                                 const sbp_msg_t *msg,
                                 void *context) {
  sbp2rtcm_sbp_cb(sender_id, msg_type, msg, (struct rtcm3_out_state *)context);
}

static void test_SBP(const char *filename,
                     s32 (*cb_sbp_to_rtcm)(u8 *buffer,
                                           u16 length,
                                           void *context),
                     gps_time_t current_time_,
                     msm_enum msm_type) {
  (void)current_time_;
  const int8_t leap_seconds = 18;
  sbp2rtcm_init(&out_state, cb_sbp_to_rtcm, NULL);
  sbp2rtcm_set_leap_second(&leap_seconds, &out_state);

  sbp2rtcm_set_rtcm_out_mode(msm_type, &out_state);

  previous_obs_time.wn = INVALID_TIME;
  previous_n_meas = 0;
  previous_num_obs = 0;

  FILE *fp = fopen(filename, "rb");
  if (fp == NULL) {
    fprintf(stderr, "Can't open input file! %s\n", filename);
    exit(1);
  }

  sbp_state_t s;
  sbp_state_init(&s);

  struct {
    sbp_msg_type_t msg_type;
    sbp_msg_callbacks_node_t node;
  } cb[] = {{SbpMsgBasePosEcef, {}},
            {SbpMsgGloBiases, {}},
            {SbpMsgObs, {}},
            {SbpMsgOsr, {}},
            {SbpMsgEphemerisGps, {}},
            {SbpMsgEphemerisBds, {}},
            {SbpMsgEphemerisGal, {}}};

  for (size_t i = 0; i < ARRAY_SIZE(cb); i++) {
    sbp_callback_register(
        &s, cb[i].msg_type, &sbp2rtcm_sbp_void_cb, &out_state, &cb[i].node);
  }
  /* msg_type == SbpMsgEphemerisGlo => call twice w. different cb functions */
  sbp_msg_callbacks_node_t sbp_ephemeris_glo_callback_node;
  sbp_callback_register(&s,
                        SbpMsgEphemerisGlo,
                        &sbp2rtcm_sbp_void_cb,
                        &out_state,
                        &sbp_ephemeris_glo_callback_node);
  sbp_callback_register(&s,
                        SbpMsgEphemerisGlo,
                        &ephemeris_glo_callback,
                        &out_state,
                        &sbp_ephemeris_glo_callback_node);
  sbp_state_set_io_context(&s, fp);

  while (!feof(fp)) {
    sbp_process(&s, &sbp_read_file);
  }
  fclose(fp);
}

static s32 rtcm_sanity_check_cb(u8 *buffer, u16 length, void *context) {
  (void)context;

  u16 byte = 0;
  ck_assert_uint_eq(buffer[byte], RTCM3_PREAMBLE);
  byte = 1;
  u16 message_size = ((buffer[byte] & 0x3) << 8) | buffer[byte + 1];
  ck_assert_uint_eq(message_size, length - RTCM3_MSG_OVERHEAD);
  return message_size;
}

static s32 rtcm_gps_eph_cb(u8 *buffer, u16 length, void *context) {
  (void)context;
  size_t buffer_index = 0;

  ck_assert(buffer[buffer_index] == RTCM3_PREAMBLE);

  uint16_t message_size =
      ((buffer[buffer_index + 1] & 0x3) << 8) | buffer[buffer_index + 2];
  ck_assert_uint_eq(message_size, length - RTCM3_MSG_OVERHEAD);
  ck_assert(message_size < 1023);
  ck_assert(verify_crc(&buffer[buffer_index], length - buffer_index));

  uint16_t byte = 3;
  uint16_t message_type = (buffer[byte] << 4) | ((buffer[byte + 1] >> 4) & 0xf);

  static bool checked_eph_gps = false;
  if (!checked_eph_gps && message_type == 1019) {
    checked_eph_gps = true;
    rtcm_msg_eph msg_eph;
    swiftnav_in_bitstream_t bitstream;
    swiftnav_in_bitstream_init(&bitstream, &buffer[byte], message_size * 8);
    rtcm3_decode_gps_eph_bitstream(&bitstream, &msg_eph);

    ck_assert(msg_eph.constellation == RTCM_CONSTELLATION_GPS);
    ck_assert_uint_eq(msg_eph.sat_id, 1);
    ck_assert_uint_eq(msg_eph.wn, 16);
    ck_assert_uint_eq(msg_eph.ura, 0);
    ck_assert_uint_eq(msg_eph.data.kepler.codeL2, 1);
    ck_assert_int_eq(msg_eph.data.kepler.inc_dot, -1278);
    ck_assert_uint_eq(msg_eph.data.kepler.iode, 82);
    ck_assert_uint_eq(msg_eph.toe, 19800);
    ck_assert_int_eq(msg_eph.data.kepler.af2, 0);
    ck_assert_int_eq(msg_eph.data.kepler.af1, -97);
    ck_assert_int_eq(msg_eph.data.kepler.af0, -178235);
    ck_assert_uint_eq(msg_eph.data.kepler.iodc, 82);
    ck_assert_int_eq(msg_eph.data.kepler.crs, -4333);
    ck_assert_int_eq(msg_eph.data.kepler.dn, 12543);
    ck_assert_int_eq(msg_eph.data.kepler.m0, 951542738);
    ck_assert_int_eq(msg_eph.data.kepler.cuc, -3726);
    ck_assert_uint_eq(msg_eph.data.kepler.ecc, 78044687);
    ck_assert_int_eq(msg_eph.data.kepler.cus, 2002);
    ck_assert_uint_eq(msg_eph.data.kepler.sqrta, 2701998303);
    ck_assert_uint_eq(msg_eph.data.kepler.toc, 19800);
    ck_assert_int_eq(msg_eph.data.kepler.cic, -82);
    ck_assert_int_eq(msg_eph.data.kepler.omega0, 1487724627);
    ck_assert_int_eq(msg_eph.data.kepler.cis, 110);
    ck_assert_int_eq(msg_eph.data.kepler.inc, 667415864);
    ck_assert_int_eq(msg_eph.data.kepler.crc, 10189);
    ck_assert_int_eq(msg_eph.data.kepler.w, 508965259);
    ck_assert_int_eq(msg_eph.data.kepler.omegadot, -23477);
    ck_assert_int_eq(msg_eph.data.kepler.tgd.gps_s, 12);
    ck_assert_uint_eq(msg_eph.health_bits, 0);
    ck_assert_uint_eq(msg_eph.data.kepler.L2_data_bit, 0);
    ck_assert_uint_eq(msg_eph.fit_interval, 0);
  }
  return message_size;
}

static s32 rtcm_gal_fnav_eph_cb(u8 *buffer, u16 length, void *context) {
  (void)context;
  size_t buffer_index = 0;

  ck_assert(buffer[buffer_index] == RTCM3_PREAMBLE);

  uint16_t message_size =
      ((buffer[buffer_index + 1] & 0x3) << 8) | buffer[buffer_index + 2];
  ck_assert_uint_eq(message_size, length - RTCM3_MSG_OVERHEAD);
  ck_assert(message_size < 1023);
  ck_assert(verify_crc(&buffer[buffer_index], length - buffer_index));

  uint16_t byte = 3;
  uint16_t message_type = (buffer[byte] << 4) | ((buffer[byte + 1] >> 4) & 0xf);

  static bool checked_eph_gal_fnav = false;
  if (!checked_eph_gal_fnav && message_type == 1045) {
    printf("GAL FNAV Eph checked\n");
    checked_eph_gal_fnav = true;
    rtcm_msg_eph msg_eph;
    swiftnav_in_bitstream_t bitstream;
    swiftnav_in_bitstream_init(&bitstream, &buffer[byte], message_size * 8);
    rtcm3_decode_gal_eph_fnav_bitstream(&bitstream, &msg_eph);

    ck_assert(msg_eph.constellation == RTCM_CONSTELLATION_GAL);
    ck_assert_uint_eq(msg_eph.sat_id, 1);
    ck_assert_uint_eq(msg_eph.wn, 1040);
    ck_assert_uint_eq(msg_eph.ura, 122);
    ck_assert_int_eq(msg_eph.data.kepler.inc_dot, -1353);
    ck_assert_uint_eq(msg_eph.data.kepler.iode, 14);
    ck_assert_uint_eq(msg_eph.toe, 5260);
    ck_assert_int_eq(msg_eph.data.kepler.af2, 0);
    ck_assert_int_eq(msg_eph.data.kepler.af1, -569);
    ck_assert_int_eq(msg_eph.data.kepler.af0, -11317458);
    ck_assert_int_eq(msg_eph.data.kepler.crs, 2084);
    ck_assert_int_eq(msg_eph.data.kepler.dn, 6367);
    ck_assert_int_eq(msg_eph.data.kepler.m0, 1164441757);
    ck_assert_int_eq(msg_eph.data.kepler.cuc, 1690);
    ck_assert_uint_eq(msg_eph.data.kepler.ecc, 1899057);
    ck_assert_int_eq(msg_eph.data.kepler.cus, 7169);
    ck_assert_uint_eq(msg_eph.data.kepler.sqrta, 2852451186);
    ck_assert_uint_eq(msg_eph.data.kepler.toc, 5260);
    ck_assert_int_eq(msg_eph.data.kepler.cic, 26);
    ck_assert_int_eq(msg_eph.data.kepler.omega0, -173178173);
    ck_assert_int_eq(msg_eph.data.kepler.cis, 4);
    ck_assert_int_eq(msg_eph.data.kepler.inc, 674889763);
    ck_assert_int_eq(msg_eph.data.kepler.crc, 2011);
    ck_assert_int_eq(msg_eph.data.kepler.w, -1872118354);
    ck_assert_int_eq(msg_eph.data.kepler.omegadot, -14100);
    ck_assert_int_eq(msg_eph.data.kepler.tgd.gal_s[0], -8);
    ck_assert_int_eq(msg_eph.data.kepler.tgd.gal_s[1], 0);
    ck_assert_uint_eq(msg_eph.health_bits, 0);
    ck_assert_uint_eq(msg_eph.fit_interval, 0);
  }
  return message_size;
}

static s32 rtcm_gal_inav_eph_cb(u8 *buffer, u16 length, void *context) {
  (void)context;
  size_t buffer_index = 0;

  ck_assert(buffer[buffer_index] == RTCM3_PREAMBLE);

  uint16_t message_size =
      ((buffer[buffer_index + 1] & 0x3) << 8) | buffer[buffer_index + 2];
  ck_assert_uint_eq(message_size, length - RTCM3_MSG_OVERHEAD);
  ck_assert(message_size < 1023);
  ck_assert(verify_crc(&buffer[buffer_index], length - buffer_index));

  uint16_t byte = 3;
  uint16_t message_type = (buffer[byte] << 4) | ((buffer[byte + 1] >> 4) & 0xf);

  static bool checked_eph_gal_inav = false;
  if (!checked_eph_gal_inav && message_type == 1046) {
    printf("GAL INAV Eph checked\n");
    checked_eph_gal_inav = true;
    rtcm_msg_eph msg_eph;
    swiftnav_in_bitstream_t bitstream;
    swiftnav_in_bitstream_init(&bitstream, &buffer[byte], message_size * 8);
    rtcm3_decode_gal_eph_inav_bitstream(&bitstream, &msg_eph);

    ck_assert(msg_eph.constellation == RTCM_CONSTELLATION_GAL);
    ck_assert_uint_eq(msg_eph.sat_id, 1);
    ck_assert_uint_eq(msg_eph.wn, 1040);
    ck_assert_uint_eq(msg_eph.ura, 122);
    ck_assert_int_eq(msg_eph.data.kepler.inc_dot, -1353);
    ck_assert_uint_eq(msg_eph.data.kepler.iode, 14);
    ck_assert_uint_eq(msg_eph.toe, 5260);
    ck_assert_int_eq(msg_eph.data.kepler.af2, 0);
    ck_assert_int_eq(msg_eph.data.kepler.af1, -568);
    ck_assert_int_eq(msg_eph.data.kepler.af0, -11317428);
    ck_assert_int_eq(msg_eph.data.kepler.crs, 2084);
    ck_assert_int_eq(msg_eph.data.kepler.dn, 6367);
    ck_assert_int_eq(msg_eph.data.kepler.m0, 1164441757);
    ck_assert_int_eq(msg_eph.data.kepler.cuc, 1690);
    ck_assert_uint_eq(msg_eph.data.kepler.ecc, 1899057);
    ck_assert_int_eq(msg_eph.data.kepler.cus, 7169);
    ck_assert_uint_eq(msg_eph.data.kepler.sqrta, 2852451186);
    ck_assert_uint_eq(msg_eph.data.kepler.toc, 5260);
    ck_assert_int_eq(msg_eph.data.kepler.cic, 26);
    ck_assert_int_eq(msg_eph.data.kepler.omega0, -173178173);
    ck_assert_int_eq(msg_eph.data.kepler.cis, 4);
    ck_assert_int_eq(msg_eph.data.kepler.inc, 674889763);
    ck_assert_int_eq(msg_eph.data.kepler.crc, 2011);
    ck_assert_int_eq(msg_eph.data.kepler.w, -1872118354);
    ck_assert_int_eq(msg_eph.data.kepler.omegadot, -14100);
    ck_assert_int_eq(msg_eph.data.kepler.tgd.gal_s[0], -8);
    ck_assert_int_eq(msg_eph.data.kepler.tgd.gal_s[1], -9);
    ck_assert_uint_eq(msg_eph.health_bits, 0);
    ck_assert_uint_eq(msg_eph.fit_interval, 0);
  }
  return message_size;
}

void set_expected_bias(double L1CA_bias,
                       double L1P_bias,
                       double L2CA_bias,
                       double L2P_bias) {
  expected_L1CA_bias = L1CA_bias;
  expected_L1P_bias = L1P_bias;
  expected_L2CA_bias = L2CA_bias;
  expected_L2P_bias = L2P_bias;
}

/* end rtcm helpers */

/* fixture globals and functions */

void rtcm3_setup_basic(void) {
  current_time.wn = 1945;
  current_time.tow = 211190;
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&current_time, NULL, &state);
}

/* end fixtures */

START_TEST(test_gps_time) {
  current_time.wn = 1945;
  current_time.tow = 277500;
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/RTCM3.bin", sbp_callback_gps, current_time);
}
END_TEST

START_TEST(test_glo_day_rollover) {
  current_time.wn = 1959;
  current_time.tow = 510191;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/glo_day_rollover.rtcm",
             sbp_callback_glo_day_rollover,
             current_time);
}
END_TEST

START_TEST(test_1012_first) {
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/1012_first.rtcm",
             sbp_callback_1012_first,
             current_time);
}
END_TEST

START_TEST(test_glo_5hz) {
  current_time.wn = 2036;
  current_time.tow = 204236;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/piksi-5Hz.rtcm3",
             sbp_callback_glo_5hz,
             current_time);
}
END_TEST

START_TEST(test_glonass_computer_time_invalid_input) {
  const gps_time_t gps_time = (gps_time_t){.wn = 2165, .tow = 237600.0};
  const int8_t leap_seconds = 18;

  gps_time_t obs_time;

  /**
   * Valid test case setup.
   */
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

  compute_glo_time(69300000, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, 2165);
  ck_assert_double_eq_tol(obs_time.tow, 231318.0, GPS_TOW_TOLERANCE);

  /**
   * Valid test case now has an invalid leap second, regardless, it should now
   * return an valid time since the GPS time specified is less than the expiry
   * date.
   */
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, NULL, &state);

  compute_glo_time(69300000, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, 2165);
  ck_assert_double_eq_tol(obs_time.tow, 231318.0, GPS_TOW_TOLERANCE);

  /**
   * Valid test case now pushes a GLONASS TOD value that is too large, therefore
   * it should now return an invalid time.
   */
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

  compute_glo_time(86401000, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, WN_UNKNOWN);
  ck_assert(!gps_time_valid(&obs_time));
}

START_TEST(test_glonass_tod_rollover) {
  /**
   * Time: UTC(SU): Jun 23, 2021 @ 23:59:59.999
   *       UTC: Jun 23, 2021 @ 20:59:59.999
   *       GPS: Jun 23, 2021 @ 21:00:17.999
   *
   * GLONNAS time: TOD: 86399999 ms
   * GPS time: WN 2163 TOW: 334817.999 s
   *
   * Inaccuracy of 10 hours is used. This level of inaccuracy is far higher than
   * the worst ephemeride timing inaccuracy and 2 hours below the maximum
   * threshold of half a day when dealing with GLONASS time of day rollover.
   */

  const int8_t leap_seconds = 18;
  gps_time_t obs_time;

  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);

  const double rover_inaccuracy_seconds = 36000.0;
  const size_t increments = 100;
  const double increment_seconds = rover_inaccuracy_seconds / increments;

  for (size_t i = 0; i <= increments; ++i) {
    double offset_seconds = (double)i * increment_seconds;
    gps_time_t gps_time;

    /**
     * Observation is still at 1ms before midnight, rover is some time before
     * midnight
     */
    gps_time = (gps_time_t){.wn = 2163, .tow = 334818.0};
    add_secs(&gps_time, -offset_seconds);
    rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

    compute_glo_time(86399999, &obs_time, &gps_time, &state);
    ck_assert_int_eq(obs_time.wn, 2163);
    ck_assert_double_eq_tol(obs_time.tow, 334817.999, GPS_TOW_TOLERANCE);

    /**
     * Observation has rolled over to the next day, rover is some time before
     * midnight
     */
    gps_time = (gps_time_t){.wn = 2163, .tow = 334817.999};
    add_secs(&gps_time, -offset_seconds);
    rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

    compute_glo_time(0, &obs_time, &gps_time, &state);
    ck_assert_int_eq(obs_time.wn, 2163);
    ck_assert_double_eq_tol(obs_time.tow, 334818.0, GPS_TOW_TOLERANCE);

    /**
     * Observation is still at 1ms before midnight, rover is some time after
     * midnight
     */
    gps_time = (gps_time_t){.wn = 2163, .tow = 334818.0};
    add_secs(&gps_time, offset_seconds);
    rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

    compute_glo_time(86399999, &obs_time, &gps_time, &state);
    ck_assert_int_eq(obs_time.wn, 2163);
    ck_assert_double_eq_tol(obs_time.tow, 334817.999, GPS_TOW_TOLERANCE);

    /**
     * Observation has rolled over to the next day, rover is some time after
     * midnight
     */
    gps_time = (gps_time_t){.wn = 2163, .tow = 334817.999};
    add_secs(&gps_time, offset_seconds);
    rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

    compute_glo_time(0, &obs_time, &gps_time, &state);
    ck_assert_int_eq(obs_time.wn, 2163);
    ck_assert_double_eq_tol(obs_time.tow, 334818.0, GPS_TOW_TOLERANCE);
  }
}

START_TEST(test_glonass_wn_rollover) {
  /**
   * Time: UTC(SU): Jun 20, 2021 @ 2:59:41.999
   *       UTC: Jun 19, 2021 @ 23:59:41.999
   *       GPS: Jun 19, 2021 @ 23:59:59.999
   *
   * GLONNAS time: TOD: 10781999 ms
   * GPS time: WN 2162 TOW: 604799.999 s
   *
   * Inaccuracy of 10 hours is used. This level of inaccuracy is far higher than
   * the worst ephemeride timing inaccuracy and 2 hours below the maximum
   * threshold of half a day when dealing with GLONASS time of day rollover.
   */

  const int8_t leap_seconds = 18;
  gps_time_t obs_time;

  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);

  const double rover_inaccuracy_seconds = 36000.0;
  const size_t increments = 100;
  const double increment_seconds = rover_inaccuracy_seconds / increments;

  for (size_t i = 0; i <= increments; ++i) {
    gps_time_t gps_time;
    double offset_seconds = (double)i * increment_seconds;

    /**
     * Observation is still at 1ms before GPS WN roll over, rover is some time
     * before WN roll over
     */
    gps_time = (gps_time_t){.wn = 2162, .tow = 604799.999};
    add_secs(&gps_time, -offset_seconds);
    rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

    compute_glo_time(10781999, &obs_time, &gps_time, &state);
    ck_assert_int_eq(obs_time.wn, 2162);
    ck_assert_double_eq_tol(obs_time.tow, 604799.999, GPS_TOW_TOLERANCE);

    /**
     * Observation has rolled over to the next GPS WN, rover is some time
     * before WN roll over
     */
    gps_time = (gps_time_t){.wn = 2162, .tow = 604799.999};
    add_secs(&gps_time, -offset_seconds);
    rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

    compute_glo_time(10782000, &obs_time, &gps_time, &state);
    ck_assert_int_eq(obs_time.wn, 2163);
    ck_assert_double_eq_tol(obs_time.tow, 0, GPS_TOW_TOLERANCE);

    /**
     * Observation is still at 1ms before GPS WN roll over, rover is some time
     * after WN roll over
     */
    gps_time = (gps_time_t){.wn = 2163, .tow = 0};
    add_secs(&gps_time, offset_seconds);
    rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

    compute_glo_time(10781999, &obs_time, &gps_time, &state);
    ck_assert_int_eq(obs_time.wn, 2162);
    ck_assert_double_eq_tol(obs_time.tow, 604799.999, GPS_TOW_TOLERANCE);

    /**
     * Observation has rolled over to the next GPS WN, rover is some time
     * after WN roll over
     */
    gps_time = (gps_time_t){.wn = 2163, .tow = 0};
    add_secs(&gps_time, offset_seconds);
    rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

    compute_glo_time(10782000, &obs_time, &gps_time, &state);
    ck_assert_int_eq(obs_time.wn, 2163);
    ck_assert_double_eq_tol(obs_time.tow, 0, GPS_TOW_TOLERANCE);
  }
}

START_TEST(test_glonass_leap_second_event_ideal) {
  /**
   * Time: UTC(SU): Jan 1, 2017 @ 02:59:59.999
   *       UTC: Dec 31, 2016 @ 23:59:59.999
   *       GPS: Jan 1, 2017 @ 00:00:16.999
   *
   * GLONNAS time: TOD: 10799999 ms
   * GPS time: WN 1930 TOW: 16.999 s
   *
   * NOTE: because of the way that RTCM to SBP converter is written, leap
   * seconds is expressed as the offset from UTC->GPS rather than UTC(SU)->GPS
   * (for historical reasons), therefore this test case assumes this. In
   * reality, this means that the GLONASS conversion will be off 1 second for 3
   * hours until the UTC->GPS leap second matches up with the UTC(SU)->GPS leap
   * second. To give you context, the UTC->GPS leap second takes place after
   * Dec 31, 2016 @ 23:59:59 UTC where as the leap second for the GLONASS
   * constellation take place after place at Dec 31, 2016 @ 20:59:59 UTC.
   */

  gps_time_t gps_time;
  gps_time_t obs_time;

  /**
   * Both observation and rover are set to 1ms before leap second event
   */
  gps_time = (gps_time_t){.wn = 1930, .tow = 16.999};
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, NULL, &state);

  compute_glo_time(10799999, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, 1930);
  ck_assert_double_eq_tol(obs_time.tow, 16.999, GPS_TOW_TOLERANCE);

  /**
   * Observation has moved forward by 1s and is at the end of the leap second
   * event, rover is still at 1ms before leap second event
   */
  gps_time = (gps_time_t){.wn = 1930, .tow = 16.999};
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, NULL, &state);

  compute_glo_time(10800999, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, 1930);
  ck_assert_double_eq_tol(obs_time.tow, 17.999, GPS_TOW_TOLERANCE);

  /**
   * Observation has moved forward by 1s + 1ms and has past the leap second
   * event, rover is still at 1ms before leap second event.
   *
   * NOTE: Because the leap seconds hasn't been updated, we might have a 1
   * second error between the observation's real time of 18.0 and the one
   * generated, this is the reasoning behind the assertion of tow =
   * [17.0, 18.0].
   */
  gps_time = (gps_time_t){.wn = 1930, .tow = 16.999};
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, NULL, &state);

  compute_glo_time(10801000, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, 1930);
  ck_assert_double_ge_tol(obs_time.tow, 17.0, GPS_TOW_TOLERANCE);
  ck_assert_double_le_tol(obs_time.tow, 18.0, GPS_TOW_TOLERANCE);

  /**
   * Observation is set to 1ms before leap second event and rover is at the end
   * of the leap second event
   */
  gps_time = (gps_time_t){.wn = 1930, .tow = 17.999};
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, NULL, &state);

  compute_glo_time(10799999, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, 1930);
  ck_assert_double_eq_tol(obs_time.tow, 16.999, GPS_TOW_TOLERANCE);

  /**
   * Observation has moved forward by 1s and is at the end of the leap second
   * event and rover has moved forward by 1s
   */
  gps_time = (gps_time_t){.wn = 1930, .tow = 17.999};
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, NULL, &state);

  compute_glo_time(10800999, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, 1930);
  ck_assert_double_eq_tol(obs_time.tow, 17.999, GPS_TOW_TOLERANCE);

  /**
   * Observation has moved forward by 1s + 1ms and has past the leap second
   * event and rover has moved forward by 1s
   *
   * NOTE: Because the leap seconds hasn't been updated, we might have a 1
   * second error between the observation's real time of 18.0 and the one
   * generated, this is the reasoning behind the assertion of tow =
   * [17.0, 18.0].
   */
  gps_time = (gps_time_t){.wn = 1930, .tow = 17.999};
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, NULL, &state);

  compute_glo_time(10801000, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, 1930);
  ck_assert_double_ge_tol(obs_time.tow, 17.0, GPS_TOW_TOLERANCE);
  ck_assert_double_le_tol(obs_time.tow, 18.0, GPS_TOW_TOLERANCE);

  /**
   * Observation is set to 1ms before leap second event and rover has moved
   * forward by 1s + 1ms (leap second has occurred).
   *
   * NOTE: Because the leap seconds has taken place while the observation is
   * operating in a different leap second value, we might have a 1 second error
   * between the observation's real time of 594016.999 and the one generated,
   * this is the reasoning behind the assertion of tow = [594016.999,
   * 594017.999].
   */
  gps_time = (gps_time_t){.wn = 1930, .tow = 18.0};
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, NULL, &state);

  compute_glo_time(10799999, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, 1930);
  ck_assert_double_ge_tol(obs_time.tow, 16.999, GPS_TOW_TOLERANCE);
  ck_assert_double_le_tol(obs_time.tow, 17.999, GPS_TOW_TOLERANCE);

  /**
   * Observation has moved forward by 1s and is at the end of the leap second
   * event and rover has moved forward by 1s + 1ms (leap second has occurred)
   *
   * NOTE: Because the leap seconds has taken place while the observation is
   * operating in a different leap second value, we might have a 1 second error
   * between the observation's real time of 594017.999 and the one generated,
   * this is the reasoning behind the assertion of tow = [594017.999,
   * 594018.999].
   */
  gps_time = (gps_time_t){.wn = 1930, .tow = 18.0};
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, NULL, &state);

  compute_glo_time(10800999, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, 1930);
  ck_assert_double_ge_tol(obs_time.tow, 17.999, GPS_TOW_TOLERANCE);
  ck_assert_double_le_tol(obs_time.tow, 18.999, GPS_TOW_TOLERANCE);

  /**
   * Observation has moved forward by 1s + 1ms and has past the leap second
   * event and rover has moved forward by 1s + 1ms (leap second has occurred)
   */
  gps_time = (gps_time_t){.wn = 1930, .tow = 18.0};
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&gps_time, NULL, &state);

  compute_glo_time(10801000, &obs_time, &gps_time, &state);
  ck_assert_int_eq(obs_time.wn, 1930);
  ck_assert_double_eq_tol(obs_time.tow, 19.0, GPS_TOW_TOLERANCE);
}

/* Test parsing of raw file with MSM5 obs */
START_TEST(test_msm5_parse) {
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/jenoba-jrr32m.rtcm3",
             sbp_callback_msm_no_gaps,
             current_time);
}
END_TEST

/* Test parsing of raw file with MSM7 obs */
START_TEST(test_msm7_parse) {
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/msm7.rtcm",
             sbp_callback_msm_no_gaps,
             current_time);

  current_time.wn = 2051;
  current_time.tow = 412185;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/beidou_invalid_fine_pseudorange.rtcm",
             sbp_callback_msm7_beidou_invalid_pr,
             current_time);
}
END_TEST

START_TEST(test_msm_missing_obs) {
  current_time.wn = 2007;
  current_time.tow = 289790;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/missing-gps.rtcm",
             sbp_callback_msm_no_gaps,
             current_time);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/dropped-packets-STR24.rtcm3",
             sbp_callback_msm_no_gaps,
             current_time);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/missing-beidou-STR17.rtcm3",
             sbp_callback_msm_no_gaps,
             current_time);
}
END_TEST

START_TEST(test_msm_week_rollover) {
  current_time.wn = 2009;
  current_time.tow = 604200;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/week-rollover-STR17.rtcm3",
             sbp_callback_msm_no_gaps,
             current_time);
  current_time.wn = 2009;
  current_time.tow = 604200;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/week-rollover-STR24.rtcm3",
             sbp_callback_msm_no_gaps,
             current_time);
}
END_TEST

/* Test switching between RTCM streams: legacy->MSM->legacy->MSM. The file
 * contains 2 minutes raw data per feed (with about 10s pause when switching)
 * from these EUREF stations:
 * OSLS00NOR0 TRIMBLE NETR9         RTCM 3.1
 *          1004(1),1005(5),1007(5),1012(1),1033(5),4094(5)
 * KUNZ00CZE0 TRIMBLE SPS855        RTCM 3.2
 *          1006(10),1008(10),1013(10),1033(10),1074(1),1084(1),1094(1),1124(1),1230(10)
 * SUR400EST0 LEICA GR25            RTCM 3.2
 *          1004(1),1006(10),1008(10),1012(1),1013(10),1033(10),1230(10)
 * ORIV00FIN0 JAVAD TRE_G3TH DELTA  RTCM 3.3
 *          1006(30),1008(30),1019,1020,1077(1),1087(1),1097(1),1107(1),1117(1),1127(1),1230(60)
 *
 * Check that there are no more than 60s gaps in observations when switching
 * from MSM feed to legacy feed.
 *
 */
START_TEST(test_msm_switching) {
  current_time.wn = 2002;
  current_time.tow = 308700;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/switch-legacy-msm-legacy-msm.rtcm",
             sbp_callback_msm_switching,
             current_time);
}
END_TEST

/* parse an artificially generated stream with both legacy and MSM observations,
 * verify there's no crash or duplicate observations
 */
START_TEST(test_msm_mixed) {
  current_time.wn = 2002;
  current_time.tow = 375900;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/mixed-msm-legacy.rtcm",
             sbp_callback_msm_no_gaps,
             current_time);
}
END_TEST

/* Missing GAL observations */
START_TEST(test_msm_gal_gaps) {
  current_time.wn = 2036;
  current_time.tow = 75680.000;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/20190120_1956_rover.023",
             sbp_callback_msm_no_gaps,
             current_time);
}
END_TEST

/* Test 1033 message sources */
START_TEST(test_bias_trm) {
  set_expected_bias(
      TRIMBLE_BIAS_M, TRIMBLE_BIAS_M, TRIMBLE_BIAS_M, TRIMBLE_BIAS_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/trimble.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_jav) {
  set_expected_bias(JAVAD_BIAS_L1CA_M, 0.0, 0.0, JAVAD_BIAS_L2P_M);
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/javad.rtcm", sbp_callback_bias, current_time);
}
END_TEST

START_TEST(test_bias_nov) {
  set_expected_bias(
      NOVATEL_BIAS_M, NOVATEL_BIAS_M, NOVATEL_BIAS_M, NOVATEL_BIAS_M);
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/leica.rtcm", sbp_callback_bias, current_time);
}
END_TEST

START_TEST(test_bias_sep) {
  set_expected_bias(SEPTENTRIO_BIAS_M,
                    SEPTENTRIO_BIAS_M,
                    SEPTENTRIO_BIAS_M,
                    SEPTENTRIO_BIAS_M);
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/sept.rtcm", sbp_callback_bias, current_time);
}
END_TEST

START_TEST(test_bias_nav) {
  set_expected_bias(NAVCOM_BIAS_L1CA_M, 0.0, 0.0, NAVCOM_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/navcom.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_top) {
  set_expected_bias(TOPCON_BIAS_M, TOPCON_BIAS_M, TOPCON_BIAS_M, TOPCON_BIAS_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/topcon.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_hem) {
  set_expected_bias(HEMISPHERE_BIAS_L1CA_M, 0.0, 0.0, HEMISPHERE_BIAS_L2P_M);
  test_RTCM3(
      RELATIVE_PATH_PREFIX "/data/hemi.rtcm", sbp_callback_bias, current_time);
}
END_TEST

/* Test 1033 messages from GEO++ */
START_TEST(test_bias_gpp_ash1) {
  set_expected_bias(GPP_ASH1_BIAS_L1CA_M, 0.0, 0.0, GPP_ASH1_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_ASH1.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_hem) {
  set_expected_bias(GPP_HEM_BIAS_L1CA_M, 0.0, 0.0, GPP_HEM_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_HEM.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_jav) {
  set_expected_bias(GPP_JAV_BIAS_L1CA_M, 0.0, 0.0, GPP_JAV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_JAV.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_jps) {
  set_expected_bias(GPP_JPS_BIAS_L1CA_M, 0.0, 0.0, GPP_JPS_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_JPS.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_lei) {
  set_expected_bias(GPP_NOV_BIAS_L1CA_M, 0.0, 0.0, GPP_NOV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_LEI.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_nav) {
  set_expected_bias(GPP_NAV_BIAS_L1CA_M, 0.0, 0.0, GPP_NAV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_NAV.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_nov) {
  set_expected_bias(GPP_NOV_BIAS_L1CA_M, 0.0, 0.0, GPP_NOV_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_NOV.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_nvr) {
  set_expected_bias(GPP_NVR_BIAS_L1CA_M, 0.0, 0.0, GPP_NVR_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_NVR.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_sep1) {
  set_expected_bias(GPP_SEP_BIAS_L1CA_M, 0.0, 0.0, GPP_SEP_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_SEP1.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_sok) {
  set_expected_bias(GPP_SOK_BIAS_L1CA_M, 0.0, 0.0, GPP_SOK_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_SOK.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_tps1) {
  set_expected_bias(GPP_TPS_BIAS_L1CA_M, 0.0, 0.0, GPP_TPS_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_TPS1.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(test_bias_gpp_trm) {
  set_expected_bias(GPP_TRM_BIAS_L1CA_M, 0.0, 0.0, GPP_TRM_BIAS_L2P_M);
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/geo++_TRM.rtcm",
             sbp_callback_bias,
             current_time);
}
END_TEST

START_TEST(tc_rtcm_eph_gps) {
  current_time.wn = 2012;
  current_time.tow = 489000;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/eph.rtcm",
             sbp_callback_gps_eph,
             current_time);
}
END_TEST

START_TEST(tc_rtcm_eph_glo) {
  current_time.wn = 2015;
  current_time.tow = 168318;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/test_glo_eph.rtcm",
             sbp_callback_glo_eph,
             current_time);
}
END_TEST

START_TEST(tc_rtcm_eph_gal) {
  current_time.wn = 2014;
  current_time.tow = 187816;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/test_gal_eph.rtcm",
             sbp_callback_gal_eph,
             current_time);
}
END_TEST

START_TEST(test_sbp_to_rtcm_legacy) {
  current_time.wn = 2020;
  current_time.tow = 211000;

  test_SBP(RELATIVE_PATH_PREFIX "/data/piksi-gps-glo.sbp",
           rtcm_sanity_check_cb,
           current_time,
           MSM_UNKNOWN);
}
END_TEST

START_TEST(test_sbp_to_rtcm_gps_eph) {
  current_time.wn = 2020;
  current_time.tow = 211000;

  test_SBP(RELATIVE_PATH_PREFIX "/data/igseph.sbp",
           rtcm_gps_eph_cb,
           current_time,
           MSM_UNKNOWN);
}
END_TEST

START_TEST(test_sbp_to_rtcm_gal_inav_eph) {
  current_time.wn = 2020;
  current_time.tow = 211000;

  test_SBP(RELATIVE_PATH_PREFIX "/data/igseph.sbp",
           rtcm_gal_inav_eph_cb,
           current_time,
           MSM_UNKNOWN);
}
END_TEST

START_TEST(test_sbp_to_rtcm_gal_fnav_eph) {
  current_time.wn = 2020;
  current_time.tow = 211000;

  test_SBP(RELATIVE_PATH_PREFIX "/data/igseph.sbp",
           rtcm_gal_fnav_eph_cb,
           current_time,
           MSM_UNKNOWN);
}
END_TEST

START_TEST(test_sbp_to_rtcm_msm) {
  current_time.wn = 2020;
  current_time.tow = 211000;

  test_SBP(RELATIVE_PATH_PREFIX "/data/piksi-gps-glo.sbp",
           rtcm_sanity_check_cb,
           current_time,
           MSM5);
}
END_TEST

START_TEST(tc_rtcm_eph_bds) {
  current_time.wn = 2014;
  current_time.tow = 187816;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/test_bds_eph.rtcm",
             sbp_callback_bds_eph,
             current_time);
}
END_TEST

START_TEST(tc_rtcm_eph_wn_rollover) {
  current_time.wn = 2025;
  current_time.tow = 487800;
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/eph_wn.rtcm",
             sbp_callback_eph_wn_rollover,
             current_time);
}
END_TEST

START_TEST(tc_rtcm_eph_wn_rollover2) {
  current_time.wn = 2050;
  current_time.tow = 208800;
  /* short RTCM3 capture from a RTCM3EPH stream */
  test_RTCM3(RELATIVE_PATH_PREFIX "/data/wn-rollover2.rtcm",
             sbp_callback_eph_wn_rollover2,
             current_time);
}
END_TEST

START_TEST(tc_rtcm_eph_invalid_gal_time) {
  current_time = (gps_time_t){.wn = 3198, .tow = 733440};

  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_time(&current_time, NULL, &state);

  rtcm_msg_eph rtcm_ephemeris;
  const rtcm_msg_eph template_rtcm_ephemeris = {
      .sat_id = 31,
      .constellation = RTCM_CONSTELLATION_GAL,
      .wn = 2174,
      .toe = 8992,
      .ura = 107,
      .fit_interval = 0,
      .health_bits = 0,
      .data = {.kepler = {.tgd =
                              {
                                  .gal_s[0] = 20,
                                  .gal_s[1] = 0,
                              },
                          .crc = 9765,
                          .crs = 2135,
                          .cuc = 1742,
                          .cus = 1109,
                          .cic = -15,
                          .cis = 5,
                          .dn = 9134,
                          .m0 = 1844301003,
                          .ecc = 3887045,
                          .sqrta = 2852448227,
                          .omega0 = -892216638,
                          .omegadot = -16740,
                          .w = -1071265542,
                          .inc = 666967268,
                          .inc_dot = 436,
                          .af0 = -8339072,
                          .af1 = -33,
                          .af2 = 0,
                          .toc = 8992,
                          .iodc = 0,
                          .iode = 37,
                          .codeL2 = 0,
                          .L2_data_bit = false}}};

  sbp_msg_ephemeris_gal_t sbp_ephemeris;

  memcpy(&rtcm_ephemeris, &template_rtcm_ephemeris, sizeof(rtcm_ephemeris));
  ck_assert(rtcm3_gal_eph_to_sbp(
      &rtcm_ephemeris, EPH_SOURCE_GAL_FNAV, &sbp_ephemeris, &state));

  memcpy(&rtcm_ephemeris, &template_rtcm_ephemeris, sizeof(rtcm_ephemeris));
  rtcm_ephemeris.toe = 12224;
  ck_assert(!rtcm3_gal_eph_to_sbp(
      &rtcm_ephemeris, EPH_SOURCE_GAL_FNAV, &sbp_ephemeris, &state));

  memcpy(&rtcm_ephemeris, &template_rtcm_ephemeris, sizeof(rtcm_ephemeris));
  rtcm_ephemeris.data.kepler.toc = 12224;
  ck_assert(!rtcm3_gal_eph_to_sbp(
      &rtcm_ephemeris, EPH_SOURCE_GAL_FNAV, &sbp_ephemeris, &state));
}
END_TEST

static s32 rtcm_roundtrip_cb(u8 *buffer, u16 length, void *context) {
  (void)context;

  u16 byte = 0;
  ck_assert_uint_eq(buffer[byte], RTCM3_PREAMBLE);
  byte = 1;
  u16 message_size = ((buffer[byte] & 0x3) << 8) | buffer[byte + 1];
  ck_assert_uint_eq(message_size, length - RTCM3_MSG_OVERHEAD);

  /* feed the received RTCM frame back to decoder */
  rtcm2sbp_decode_frame(buffer, length, &state);
  return message_size;
}

/* structure to hold info on which GLO sats we have provided
 * channel number information to the decoder. If we have not
 * provided the channel number information, we can not
 * expect the decoder to provide valid carrier and doppler.
 */
struct glo_known_fcn_info {
  size_t num_known_fcn;
  u8 known_fcn_sat_id[NUM_SATS_GLO];
};

static void sbp_roundtrip_cb(uint16_t sender_id,
                             sbp_msg_type_t msg_type,
                             const sbp_msg_t *sbp_msg,
                             void *context) {
  (void)sender_id;
  struct glo_known_fcn_info *known_fcn = context;

  ck_assert_uint_eq(msg_type, SbpMsgObs);
  const sbp_msg_obs_t *sbp_obs = &sbp_msg->obs;
  u8 num_obs = sbp_obs->n_obs;

  for (u8 i = 0; i < num_obs; i++) {
    const sbp_packed_obs_content_t *converted_obs = &sbp_obs->obs[i];
    /* find the original SBP observation */
    sbp_packed_obs_content_t *orig_obs = NULL;
    for (u8 j = 0; j < ARRAY_SIZE(sbp_test_data); j++) {
      if (sbp_test_data[j].sid.code == converted_obs->sid.code &&
          sbp_test_data[j].sid.sat == converted_obs->sid.sat) {
        orig_obs = &sbp_test_data[j];
        break;
      }
    }
    ck_assert_ptr_ne(orig_obs, NULL);

    /* If this is a GLO satellite, check if we have provided channel
     * number information to the decoder. Then we know if to expect
     * valid carrier and doppler for this satellite or not */
    bool is_glonass =
        (code_to_constellation(converted_obs->sid.code) == CONSTELLATION_GLO);
    bool glo_fcn_is_known = false;
    if (is_glonass) {
      for (size_t k = 0; k < known_fcn->num_known_fcn; k++) {
        if (known_fcn->known_fcn_sat_id[k] == converted_obs->sid.sat) {
          glo_fcn_is_known = true;
          break;
        }
      }
    }
    bool act_on_unknown_glo_channel_number = is_glonass && !glo_fcn_is_known;

    /* check that the sbp->msm->sbp converted observation is identical to the
     * original */
    ck_assert_uint_eq(orig_obs->P, converted_obs->P);
    if (!act_on_unknown_glo_channel_number) {
      ck_assert_uint_eq(orig_obs->L.i, converted_obs->L.i);
      ck_assert_uint_eq(orig_obs->L.f, converted_obs->L.f);
      ck_assert_uint_eq(orig_obs->D.i, converted_obs->D.i);
      ck_assert_uint_eq(orig_obs->D.f, converted_obs->D.f);
    }
    ck_assert_uint_eq(orig_obs->cn0, converted_obs->cn0);
    ck_assert_uint_eq(orig_obs->lock, converted_obs->lock);

    u8 expected_flags = orig_obs->flags;
    if (act_on_unknown_glo_channel_number) {
      /* unset carrier and doppler valid flags, as channel number was unknown
       * to the decoder */
      expected_flags &= ~MSG_OBS_FLAGS_PHASE_VALID;
      expected_flags &= ~MSG_OBS_FLAGS_HALF_CYCLE_KNOWN;
      expected_flags &= ~MSG_OBS_FLAGS_DOPPLER_VALID;
    }

    ck_assert_uint_eq(expected_flags, converted_obs->flags);
  }
}

START_TEST(test_sbp_to_msm_roundtrip) {
  current_time.wn = 2022;
  current_time.tow = 210853;

  struct glo_known_fcn_info known_fcn = {0};
  const int8_t leap_seconds = (int8_t)get_gps_utc_offset(&current_time, NULL);

  sbp2rtcm_init(&out_state, rtcm_roundtrip_cb, NULL);
  sbp2rtcm_set_leap_second(&leap_seconds, &out_state);

  rtcm2sbp_init(&state, NULL, sbp_roundtrip_cb, NULL, &known_fcn);
  rtcm2sbp_set_time(&current_time, &leap_seconds, &state);

  /* set the FCNs for couple of GLO satellites */
  sbp_v4_gnss_signal_t sid = {2, CODE_GLO_L1OF};
  sbp_v4_gnss_signal_t sid2 = {2, CODE_GLO_L1OF};
  sbp2rtcm_set_glo_fcn(sid, 4, &out_state);
  rtcm2sbp_set_glo_fcn(sid2, 4, &state);
  known_fcn.known_fcn_sat_id[known_fcn.num_known_fcn] = sid.sat;
  known_fcn.num_known_fcn++;
  sid.sat = 3;
  sbp2rtcm_set_glo_fcn(sid, 13, &out_state);
  rtcm2sbp_set_glo_fcn(sid2, 13, &state);
  known_fcn.known_fcn_sat_id[known_fcn.num_known_fcn] = sid.sat;
  known_fcn.num_known_fcn++;
  sid.sat = 11;
  sbp2rtcm_set_glo_fcn(sid, 8, &out_state);
  rtcm2sbp_set_glo_fcn(sid2, 8, &state);
  known_fcn.known_fcn_sat_id[known_fcn.num_known_fcn] = sid.sat;
  known_fcn.num_known_fcn++;

  rtcm2sbp_set_time(&current_time, NULL, &state);

  memcpy(
      out_state.sbp_obs_buffer, sbp_test_data_old, sizeof(sbp_test_data_old));
  out_state.n_sbp_obs = ARRAY_SIZE(sbp_test_data_old);

  sbp_buffer_to_msm(&out_state);
}
END_TEST

// Get an example GPS orbit for testing
static sbp_msg_ephemeris_gps_t get_example_gps_eph() {
  sbp_msg_ephemeris_gps_t ret = {};
  ret.common.sid.sat = 25;
  ret.common.sid.code = CODE_GPS_L1CA;
  ret.common.toe.wn = 2022;
  ret.common.toe.tow = 460800;
  ret.common.ura = (float)2.8;
  ret.common.fit_interval = 14400;
  ret.common.valid = 1;
  ret.common.health_bits = 0;
  ret.tgd = (float)(-3 * 1e-10);
  ret.c_rc = 167.140625;
  ret.c_rs = -18.828125;
  ret.c_uc = -9.0105459094047546e-07;
  ret.c_us = 9.4850547611713409e-06;
  ret.c_ic = -4.0978193283081055e-08;
  ret.c_is = 1.0104849934577942e-07;
  ret.dn = 3.9023054038264214e-09;
  ret.m0 = 0.39869951815527438;
  ret.ecc = 0.00043709692545235157;
  ret.sqrta = 5282.6194686889648;
  ret.omega0 = 2.2431156200949509;
  ret.omegadot = -6.6892072037584707e-09;
  ret.w = 0.39590413040186828;
  ret.inc = 0.95448398903792575;
  ret.inc_dot = -6.2716898124832475e-10;
  ret.af0 = -0.00050763087347149849;
  ret.af1 = -1.3019807454384136e-11;
  ret.af2 = 0.000000;
  ret.toc.wn = 2022;
  ret.toc.tow = 460800;
  ret.iodc = 250;
  ret.iode = 250;
  return ret;
}

// Compare two GPS orbits to make sure they are the same
static void compare_gps_ephs(const sbp_msg_ephemeris_gps_t *first,
                             const sbp_msg_ephemeris_gps_t *second) {
  (void)first;
  (void)second;
  assert(first->common.sid.sat == second->common.sid.sat);
  assert(first->common.sid.code == second->common.sid.code);
  assert(first->common.toe.wn == second->common.toe.wn);
  assert(first->common.toe.tow == second->common.toe.tow);
  assert(fabs(first->common.ura - second->common.ura) < 1e-12);
  assert(first->common.fit_interval == second->common.fit_interval);
  assert(first->common.health_bits == second->common.health_bits);
  assert(first->common.valid == second->common.valid);

  assert(fabs(first->tgd - second->tgd) <= C_1_2P31 / 2);
  assert(fabs(first->c_rs - second->c_rs) <= C_1_2P5 / 2);
  assert(fabs(first->c_rc - second->c_rc) <= C_1_2P5 / 2);
  assert(fabs(first->c_uc - second->c_uc) <= C_1_2P29 / 2);
  assert(fabs(first->c_us - second->c_us) <= C_1_2P29 / 2);
  assert(fabs(first->c_ic - second->c_ic) <= C_1_2P29 / 2);
  assert(fabs(first->c_is - second->c_is) <= C_1_2P29 / 2);
  assert(fabs(first->dn - second->dn) <= C_1_2P43 * M_PI / 2);
  assert(fabs(first->m0 - second->m0) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->ecc - second->ecc) <= C_1_2P33 / 2);
  assert(fabs(first->sqrta - second->sqrta) <= C_1_2P19 / 2);
  assert(fabs(first->omega0 - second->omega0) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->omegadot - second->omegadot) <= C_1_2P43 * M_PI / 2);
  assert(fabs(first->w - second->w) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->inc - second->inc) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->inc_dot - second->inc_dot) <= C_1_2P43 * M_PI / 2);
  assert(fabs(first->af0 - second->af0) <= C_1_2P31 / 2);
  assert(fabs(first->af1 - second->af1) <= C_1_2P43 / 2);
  assert(fabs(first->af2 - second->af2) <= C_1_2P55 / 2);
  assert(first->toc.wn == second->toc.wn);
  assert(first->toc.tow == second->toc.tow);
  assert(first->iode == second->iode);
  assert(first->iodc == second->iodc);
}

// Get an example GLO orbit for testing
static sbp_msg_ephemeris_glo_t get_example_glo_eph() {
  sbp_msg_ephemeris_glo_t ret = {};
  ret.common.sid.sat = 25;
  ret.common.sid.code = CODE_GLO_L1OF;
  ret.common.toe.wn = 2022;
  ret.common.toe.tow = 220518;
  ret.common.fit_interval = 2400;
  ret.common.valid = 1;
  ret.common.health_bits = 1;
  ret.common.ura = (float)10.0;
  ret.gamma = (float)(2.4 * 1e-31);
  ret.tau = (float)(1.38 * 1e-10);
  ret.d_tau = (float)(2.56 * 1e-10);
  ret.pos[0] = 647838.345;
  ret.pos[1] = 875308.747;
  ret.pos[2] = 234597.325;
  ret.vel[0] = 24.435;
  ret.vel[1] = 72.7643;
  ret.vel[2] = 55.876;
  ret.acc[0] = (float)(1.3425 * 1e-6);
  ret.acc[1] = (float)(1.765 * 1e-6);
  ret.acc[2] = (float)(2.23 * 1e-6);
  ret.fcn = 15;
  ret.iod = 4;
  return ret;
}

// Compare two GLO orbits to make sure they are the same
static void compare_glo_ephs(const sbp_msg_ephemeris_glo_t *first,
                             const sbp_msg_ephemeris_glo_t *second) {
  (void)first;
  (void)second;
  assert(first->common.sid.sat == second->common.sid.sat);
  assert(first->common.sid.code == second->common.sid.code);
  assert(first->common.toe.wn == second->common.toe.wn);
  assert(first->common.toe.tow == second->common.toe.tow);
  assert((first->common.ura / second->common.ura < 2 &&
          second->common.ura / first->common.ura < 2) ||
         (first->common.ura > 512.0 && second->common.ura == 6144.0) ||
         (second->common.ura > 512.0 && first->common.ura == 6144.0));
  assert(first->common.fit_interval == second->common.fit_interval);
  assert(first->common.health_bits == second->common.health_bits);
  assert(first->common.valid == second->common.valid);

  assert(fabs(first->gamma - second->gamma) <= C_1_2P40 / 2);
  assert(fabs(first->tau - second->tau) <= C_1_2P30 / 2);
  assert(fabs(first->d_tau - second->d_tau) <= C_1_2P30 / 2);
  assert(fabs(first->pos[0] - second->pos[0]) <= C_1_2P11 * 1000 / 2);
  assert(fabs(first->pos[1] - second->pos[1]) <= C_1_2P11 * 1000 / 2);
  assert(fabs(first->pos[2] - second->pos[2]) <= C_1_2P11 * 1000 / 2);
  assert(fabs(first->vel[0] - second->vel[0]) <= C_1_2P20 * 1000 / 2);
  assert(fabs(first->vel[1] - second->vel[1]) <= C_1_2P20 * 1000 / 2);
  assert(fabs(first->vel[2] - second->vel[2]) <= C_1_2P20 * 1000 / 2);
  assert(fabs(first->acc[0] - second->acc[0]) <= C_1_2P30 * 1000 / 2);
  assert(fabs(first->acc[1] - second->acc[1]) <= C_1_2P30 * 1000 / 2);
  assert(fabs(first->acc[2] - second->acc[2]) <= C_1_2P30 * 1000 / 2);
  assert(first->fcn == second->fcn);
  assert(first->iod == second->iod);
}

// Get an example BDS orbit for testing
static sbp_msg_ephemeris_bds_t get_example_bds_eph() {
  sbp_msg_ephemeris_bds_t ret = {};
  ret.common.sid.sat = 25;
  ret.common.sid.code = CODE_BDS2_B1;
  ret.common.toe.wn = 2022;
  ret.common.toe.tow = 460814;
  ret.common.ura = (float)2.8;
  ret.common.fit_interval = 3 * HOUR_SECS;
  ret.common.valid = 1;
  ret.common.health_bits = 0;
  ret.tgd1 = (float)(-3 * 1e-10);
  ret.tgd2 = (float)(-4 * 1e-10);
  ret.c_rc = 167.140625;
  ret.c_rs = -18.828125;
  ret.c_uc = -9.0105459094047546e-07;
  ret.c_us = 9.4850547611713409e-06;
  ret.c_ic = -4.0978193283081055e-08;
  ret.c_is = 1.0104849934577942e-07;
  ret.dn = 3.9023054038264214e-09;
  ret.m0 = 0.39869951815527438;
  ret.ecc = 0.00043709692545235157;
  ret.sqrta = 5282.6194686889648;
  ret.omega0 = 2.2431156200949509;
  ret.omegadot = -6.6892072037584707e-09;
  ret.w = 0.39590413040186828;
  ret.inc = 0.95448398903792575;
  ret.inc_dot = -6.2716898124832475e-10;
  ret.af0 = -0.00050763087347149849;
  ret.af1 = -1.3019807454384136e-11;
  ret.af2 = 0.000000;
  ret.toc.wn = 2022;
  ret.toc.tow = 460814;
  ret.iodc = 954;
  ret.iode = 250;
  return ret;
}

// Compare two BDS orbits to make sure they are the same
static void compare_bds_ephs(const sbp_msg_ephemeris_bds_t *first,
                             const sbp_msg_ephemeris_bds_t *second) {
  (void)first;
  (void)second;
  assert(first->common.sid.sat == second->common.sid.sat);
  assert(first->common.sid.code == second->common.sid.code);
  assert(first->common.toe.wn == second->common.toe.wn);
  assert(first->common.toe.tow == second->common.toe.tow);
  assert(fabs(first->common.ura - second->common.ura) < 1e-12);
  assert(first->common.fit_interval == second->common.fit_interval);
  assert(first->common.health_bits == second->common.health_bits);
  assert(first->common.valid == second->common.valid);

  assert(fabs(first->tgd1 - second->tgd1) <= 1e-10 / 2);
  assert(fabs(first->tgd2 - second->tgd2) <= 1e-10 / 2);
  assert(fabs(first->c_rs - second->c_rs) <= C_1_2P6 / 2);
  assert(fabs(first->c_rc - second->c_rc) <= C_1_2P6 / 2);
  assert(fabs(first->c_uc - second->c_uc) <= C_1_2P31 / 2);
  assert(fabs(first->c_us - second->c_us) <= C_1_2P31 / 2);
  assert(fabs(first->c_ic - second->c_ic) <= C_1_2P31 / 2);
  assert(fabs(first->c_is - second->c_is) <= C_1_2P31 / 2);
  assert(fabs(first->dn - second->dn) <= C_1_2P43 * M_PI / 2);
  assert(fabs(first->m0 - second->m0) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->ecc - second->ecc) <= C_1_2P33 / 2);
  assert(fabs(first->sqrta - second->sqrta) <= C_1_2P19 / 2);
  assert(fabs(first->omega0 - second->omega0) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->omegadot - second->omegadot) <= C_1_2P43 * M_PI / 2);
  assert(fabs(first->w - second->w) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->inc - second->inc) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->inc_dot - second->inc_dot) < C_1_2P43 * M_PI / 2);
  assert(fabs(first->af0 - second->af0) <= C_1_2P33 / 2);
  assert(fabs(first->af1 - second->af1) <= C_1_2P50 / 2);
  assert(fabs(first->af2 - second->af2) <= C_1_2P66 / 2);
  assert(first->toc.wn == second->toc.wn);
  assert(first->toc.tow == second->toc.tow);
  assert(first->iode == second->iode);
  assert(first->iodc == second->iodc);
}

// Get an example GAL orbit for testing
static sbp_msg_ephemeris_gal_t get_example_gal_eph() {
  sbp_msg_ephemeris_gal_t ret = {};
  ret.common.sid.sat = 25;
  ret.common.sid.code = CODE_GAL_E1B;
  ret.common.toe.wn = 2022;
  ret.common.toe.tow = 460800;
  ret.common.ura = (float)2.8;
  ret.common.fit_interval = 4 * HOUR_SECS;
  ret.common.valid = 1;
  ret.common.health_bits = 0;
  ret.bgd_e1e5a = (float)(-3 * 1e-10);
  ret.bgd_e1e5b = (float)(-2 * 1e-10);
  ret.c_rc = 167.140625;
  ret.c_rs = -18.828125;
  ret.c_uc = -9.0105459094047546e-07;
  ret.c_us = 9.4850547611713409e-06;
  ret.c_ic = -4.0978193283081055e-08;
  ret.c_is = 1.0104849934577942e-07;
  ret.dn = 3.9023054038264214e-09;
  ret.m0 = 0.39869951815527438;
  ret.ecc = 0.00043709692545235157;
  ret.sqrta = 5282.6194686889648;
  ret.omega0 = 2.2431156200949509;
  ret.omegadot = -6.6892072037584707e-09;
  ret.w = 0.39590413040186828;
  ret.inc = 0.95448398903792575;
  ret.inc_dot = -6.2716898124832475e-10;
  ret.af0 = -0.00050763087347149849;
  ret.af1 = -1.3019807454384136e-11;
  ret.af2 = 0.000000;
  ret.toc.wn = 2022;
  ret.toc.tow = 460800;
  ret.iodc = 954;
  ret.iode = 954;
  return ret;
}

// Compare two GAL orbits to make sure they are the same
static void compare_gal_ephs(const sbp_msg_ephemeris_gal_t *first,
                             const sbp_msg_ephemeris_gal_t *second) {
  (void)first;
  (void)second;
  assert(first->common.sid.sat == second->common.sid.sat);
  assert(first->common.sid.code == second->common.sid.code);
  ck_assert_uint_eq(first->common.toe.wn, second->common.toe.wn);
  assert(first->common.toe.tow == second->common.toe.tow);
  assert(fabs(first->common.ura - second->common.ura) < 1e-12);
  assert(first->common.fit_interval == second->common.fit_interval);
  assert(first->common.health_bits == second->common.health_bits);
  assert(first->common.valid == second->common.valid);

  assert(fabs(first->bgd_e1e5a - second->bgd_e1e5a) <= C_1_2P32 / 2);
  assert(fabs(first->bgd_e1e5b - second->bgd_e1e5b) <= C_1_2P32 / 2);
  assert(fabs(first->c_rs - second->c_rs) <= C_1_2P5 / 2);
  assert(fabs(first->c_rc - second->c_rc) <= C_1_2P5 / 2);
  assert(fabs(first->c_uc - second->c_uc) <= C_1_2P29 / 2);
  assert(fabs(first->c_us - second->c_us) <= C_1_2P29 / 2);
  assert(fabs(first->c_ic - second->c_ic) <= C_1_2P29 / 2);
  assert(fabs(first->c_is - second->c_is) <= C_1_2P29 / 2);
  assert(fabs(first->dn - second->dn) <= C_1_2P43 * M_PI / 2);
  assert(fabs(first->m0 - second->m0) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->ecc - second->ecc) <= C_1_2P33 / 2);
  assert(fabs(first->sqrta - second->sqrta) <= C_1_2P19 / 2);
  assert(fabs(first->omega0 - second->omega0) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->omegadot - second->omegadot) <= C_1_2P43 * M_PI / 2);
  assert(fabs(first->w - second->w) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->inc - second->inc) <= C_1_2P31 * M_PI / 2);
  assert(fabs(first->inc_dot - second->inc_dot) < C_1_2P43 * M_PI / 2);
  assert(fabs(first->af0 - second->af0) <= C_1_2P34 / 2);
  assert(fabs(first->af1 - second->af1) <= C_1_2P46 / 2);
  assert(fabs(first->af2 - second->af2) <= C_1_2P59 / 2);
  assert(first->toc.wn == second->toc.wn);
  assert(first->toc.tow == second->toc.tow);
  assert(first->iode == second->iode);
  assert(first->iodc == second->iodc);
}

START_TEST(test_rtcm_1013) {
  static const uint8_t rtcm_1013_payload[] = {
      0x3f, 0x50, 0x01, 0xe8, 0xba, 0x25, 0xbf, 0x80, 0x48};

  gps_time_t gps_time;
  int8_t leap_seconds;
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_decode_payload(rtcm_1013_payload, sizeof(rtcm_1013_payload), &state);
  ck_assert(rtcm_get_gps_time(&gps_time, &state));
  ck_assert(rtcm_get_leap_seconds(&leap_seconds, &state));
  ck_assert_int_eq(gps_time.wn, 2190);
  ck_assert_double_eq_tol(gps_time.tow, 364945, GPS_TOW_TOLERANCE);
  ck_assert_int_eq(leap_seconds, 18);
}
END_TEST

START_TEST(test_rtcm_1013_handle) {
  gps_time_t gps_time;
  int8_t leap_seconds;
  rtcm_msg_1013 msg;

  // VALIDATE WORKS JUST BEFORE MJD ROLLOVER AT 23/04/2038
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_gps_week_reference(GPS_WEEK_REFERENCE, &state);

  msg = (rtcm_msg_1013){0};
  msg.mjd = UINT16_MAX;
  msg.leap_second = 0;
  msg.utc = DAY_SECS - 1;

  rtcm3_1013_handle(&msg, &state);
  ck_assert(rtcm_get_gps_time(&gps_time, &state));
  ck_assert(rtcm_get_leap_seconds(&leap_seconds, &state));
  ck_assert_int_eq(gps_time.wn, 3041);
  ck_assert_double_eq_tol(gps_time.tow, 431999, GPS_TOW_TOLERANCE);
  ck_assert_int_eq(leap_seconds, 0);

  // VALIDATE WORKS JUST BEFORE MJD ROLLOVER AT 23/04/2038
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_gps_week_reference(GPS_WEEK_REFERENCE, &state);

  msg = (rtcm_msg_1013){0};
  msg.mjd = 0;
  msg.leap_second = 0;
  msg.utc = 0;

  rtcm3_1013_handle(&msg, &state);
  ck_assert(rtcm_get_gps_time(&gps_time, &state));
  ck_assert(rtcm_get_leap_seconds(&leap_seconds, &state));
  ck_assert_int_eq(gps_time.wn, 3041);
  ck_assert_double_eq_tol(gps_time.tow, 432000, GPS_TOW_TOLERANCE);
  ck_assert_int_eq(leap_seconds, 0);

  // VALIDATE WORKS JUST BEFORE 2^16 DAYS FROM GPS_WEEK_REFERENCE WHICH IS
  // 31/10/2374
  rtcm2sbp_init(&state, NULL, NULL, NULL, NULL);
  rtcm2sbp_set_gps_week_reference(GPS_WEEK_REFERENCE, &state);

  msg = (rtcm_msg_1013){0};
  msg.mjd = 57375;
  msg.leap_second = 0;
  msg.utc = DAY_SECS - 1;

  rtcm3_1013_handle(&msg, &state);
  ck_assert(rtcm_get_gps_time(&gps_time, &state));
  ck_assert(rtcm_get_leap_seconds(&leap_seconds, &state));
  ck_assert_int_eq(gps_time.wn, 11238);
  ck_assert_double_eq_tol(gps_time.tow, 172799, GPS_TOW_TOLERANCE);
  ck_assert_int_eq(leap_seconds, 0);
}
END_TEST

START_TEST(test_sbp_to_rtcm_1019_validity_decoding) {
  // Test out the four cases which should return a valid ephemeris - when the
  // 9th and 10th bits of the iodc are 00, 01, 10 and 11 but the other 8 bits
  // are the same as the iode.
  sbp_msg_ephemeris_gps_t msg_1019_in = get_example_gps_eph();
  rtcm_msg_eph rtcm_msg_1019;
  sbp_to_rtcm3_gps_eph(&msg_1019_in, &rtcm_msg_1019, &out_state);
  sbp_msg_ephemeris_gps_t msg_1019_out;
  rtcm_msg_1019.data.kepler.iodc = 250;
  rtcm_msg_1019.data.kepler.iode = 250;
  rtcm3_gps_eph_to_sbp(&rtcm_msg_1019, &msg_1019_out, &state);
  assert(msg_1019_out.common.valid == 1);
  rtcm_msg_1019.data.kepler.iodc = 506;
  rtcm_msg_1019.data.kepler.iode = 250;
  rtcm3_gps_eph_to_sbp(&rtcm_msg_1019, &msg_1019_out, &state);
  assert(msg_1019_out.common.valid == 1);
  rtcm_msg_1019.data.kepler.iodc = 1018;
  rtcm_msg_1019.data.kepler.iode = 250;
  rtcm3_gps_eph_to_sbp(&rtcm_msg_1019, &msg_1019_out, &state);
  assert(msg_1019_out.common.valid == 1);
  rtcm_msg_1019.data.kepler.iodc = 762;
  rtcm_msg_1019.data.kepler.iode = 250;
  rtcm3_gps_eph_to_sbp(&rtcm_msg_1019, &msg_1019_out, &state);
  assert(msg_1019_out.common.valid == 1);

  // Test out the four cases which should return an invalid ephemeris - when the
  // 9th and 10th bits of the iodc are 00, 01, 10 and 11 but the other 8 bits
  // are different to the iode.
  rtcm_msg_1019.data.kepler.iodc = 250;
  rtcm_msg_1019.data.kepler.iode = 251;
  rtcm3_gps_eph_to_sbp(&rtcm_msg_1019, &msg_1019_out, &state);
  assert(msg_1019_out.common.valid == 0);
  rtcm_msg_1019.data.kepler.iodc = 506;
  rtcm_msg_1019.data.kepler.iode = 251;
  rtcm3_gps_eph_to_sbp(&rtcm_msg_1019, &msg_1019_out, &state);
  assert(msg_1019_out.common.valid == 0);
  rtcm_msg_1019.data.kepler.iodc = 1018;
  rtcm_msg_1019.data.kepler.iode = 251;
  rtcm3_gps_eph_to_sbp(&rtcm_msg_1019, &msg_1019_out, &state);
  assert(msg_1019_out.common.valid == 0);
  rtcm_msg_1019.data.kepler.iodc = 762;
  rtcm_msg_1019.data.kepler.iode = 251;
  rtcm3_gps_eph_to_sbp(&rtcm_msg_1019, &msg_1019_out, &state);
  assert(msg_1019_out.common.valid == 0);
}
END_TEST

START_TEST(test_sbp_to_rtcm_1019_roundtrip) {
  gps_time_t gps_time = (gps_time_t){.wn = 2022, .tow = 210853};
  rtcm2sbp_set_time(&gps_time, NULL, &state);

  sbp_msg_ephemeris_gps_t msg_1019_in = get_example_gps_eph();
  rtcm_msg_eph rtcm_msg_1019;
  sbp_to_rtcm3_gps_eph(&msg_1019_in, &rtcm_msg_1019, &out_state);
  sbp_msg_ephemeris_gps_t msg_1019_out;
  rtcm3_gps_eph_to_sbp(&rtcm_msg_1019, &msg_1019_out, &state);
  compare_gps_ephs(&msg_1019_in, &msg_1019_out);
}
END_TEST

START_TEST(test_sbp_to_rtcm_1020_roundtrip) {
  gps_time_t gps_time = (gps_time_t){.wn = 2022, .tow = 210853};
  int8_t leap_seconds = 18;
  rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

  sbp_msg_ephemeris_glo_t msg_1020_in = get_example_glo_eph();
  rtcm_msg_eph rtcm_msg_1020;
  sbp_to_rtcm3_glo_eph(&msg_1020_in, &rtcm_msg_1020, &out_state);
  sbp_msg_ephemeris_glo_t msg_1020_out;
  rtcm3_glo_eph_to_sbp(&rtcm_msg_1020, &msg_1020_out, &state);
  compare_glo_ephs(&msg_1020_in, &msg_1020_out);
}
END_TEST

START_TEST(test_sbp_to_rtcm_1042_roundtrip) {
  gps_time_t gps_time = (gps_time_t){.wn = 2022, .tow = 210853};
  int8_t leap_seconds = 18;
  rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

  sbp_msg_ephemeris_bds_t msg_1042_in = get_example_bds_eph();

  rtcm_msg_eph rtcm_msg_1042;
  sbp_to_rtcm3_bds_eph(&msg_1042_in, &rtcm_msg_1042, &out_state);
  sbp_msg_ephemeris_bds_t msg_1042_out;
  rtcm3_bds_eph_to_sbp(&rtcm_msg_1042, &msg_1042_out, &state);
  compare_bds_ephs(&msg_1042_in, &msg_1042_out);
}
END_TEST

START_TEST(test_sbp_to_rtcm_1045_roundtrip) {
  gps_time_t gps_time = (gps_time_t){.wn = 2022, .tow = 210853};
  int8_t leap_seconds = 18;
  rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

  sbp_msg_ephemeris_gal_t msg_1045_in = get_example_gal_eph();

  rtcm_msg_eph rtcm_msg_1045;
  sbp_to_rtcm3_gal_eph(&msg_1045_in, &rtcm_msg_1045, &out_state);
  sbp_msg_ephemeris_gal_t msg_1045_out;
  rtcm3_gal_eph_to_sbp(
      &rtcm_msg_1045, EPH_SOURCE_GAL_FNAV, &msg_1045_out, &state);
  compare_gal_ephs(&msg_1045_in, &msg_1045_out);
}
END_TEST

START_TEST(test_sbp_to_rtcm_1046_roundtrip) {
  gps_time_t gps_time = (gps_time_t){.wn = 2022, .tow = 210853};
  int8_t leap_seconds = 18;
  rtcm2sbp_set_time(&gps_time, &leap_seconds, &state);

  sbp_msg_ephemeris_gal_t msg_1046_in = get_example_gal_eph();

  rtcm_msg_eph rtcm_msg_1046;
  sbp_to_rtcm3_gal_eph(&msg_1046_in, &rtcm_msg_1046, &out_state);
  sbp_msg_ephemeris_gal_t msg_1046_out;
  rtcm3_gal_eph_to_sbp(
      &rtcm_msg_1046, EPH_SOURCE_GAL_INAV, &msg_1046_out, &state);
  compare_gal_ephs(&msg_1046_in, &msg_1046_out);
}
END_TEST

/* ===== Test rtcm 999 STGSV message conversion =====
 * Sample payload for single & multiple msg for rtcm 999 stgsv msg conversion:
 *  - rtcm_999_stgsv_payload_gps: multi_msg_indicator == true.
 *  - rtcm_999_stgsv_payload_bds7: multi_msg_indicator == true.
 *  - rtcm_999_stgsv_payload_bds13: multi_msg_indicator == false.
 * To test STGSV decoding, a map "cons_meas_map" generated by
 * observation msg is essential.
 * */

static const uint8_t rtcm_999_stgsv_payload_gps[50] = {
    0x3e, 0x71, 0xc5, 0xd8, 0x06, 0x6c, 0x01, 0x10, 0x7a, 0x84,
    0x00, 0x00, 0x3e, 0x2b, 0x65, 0xff, 0xff, 0x31, 0x18, 0x0d,
    0x8f, 0x0e, 0x3d, 0x42, 0xff, 0xc3, 0xaa, 0x03, 0xbf, 0xe2,
    0xa5, 0x92, 0x81, 0xf0, 0xe9, 0x60, 0x87, 0xf9, 0x62, 0x20,
    0x87, 0xfd, 0x05, 0x1c, 0x35, 0xfe, 0x62, 0xf0, 0xff, 0xff};

static const uint8_t rtcm_999_stgsv_payload_bds7[92] = {
    0x3e, 0x71, 0xc5, 0xd8, 0x06, 0x6c, 0x1f, 0xdb, 0xc4, 0x0e, 0x49, 0x0c,
    0x3e, 0x5c, 0x00, 0xff, 0xff, 0x0d, 0x91, 0xff, 0xff, 0x90, 0x4e, 0xbf,
    0xff, 0xca, 0xc2, 0xe4, 0x3f, 0xe7, 0x12, 0xbf, 0xff, 0xf2, 0xd8, 0x1f,
    0xff, 0xf9, 0xbc, 0x37, 0xff, 0xfc, 0x76, 0x1d, 0xff, 0xfe, 0xa2, 0x4e,
    0xff, 0xff, 0x1f, 0x44, 0x7f, 0xff, 0x98, 0x4d, 0xc4, 0xbf, 0xcd, 0x1c,
    0x7f, 0xff, 0xe1, 0xe3, 0x32, 0x12, 0x54, 0x91, 0xcf, 0xff, 0xf8, 0x55,
    0x13, 0xff, 0xfc, 0x26, 0x59, 0xff, 0xfe, 0x8a, 0x6a, 0xff, 0xff, 0x30,
    0xa1, 0x7f, 0xff, 0x9a, 0x40, 0x7f, 0xff, 0xc0};

static const uint8_t rtcm_999_stgsv_payload_bds13[22] = {
    0x3e, 0x71, 0xc5, 0xd8, 0x06, 0x6c, 0x34, 0xc0, 0x00, 0x00, 0x00,
    0x00, 0x3c, 0x67, 0x55, 0xff, 0xff, 0x12, 0x44, 0xff, 0xff, 0x80};

static void set_sample_cons_meas_map(struct rtcm3_sbp_state *rtcm2sbp_state) {
  struct rtcm_gnss_signal *map = rtcm2sbp_state->cons_meas_map;
  struct cons_meas_map_shortform {
    constellation_t cons;
    struct {
      uint8_t sid;
      struct {
        uint8_t n_signal;
        struct {
          code_t code;
          uint8_t cn0;
        } signal[3];
      };
    };
  };

  // update special values of maps.
  struct cons_meas_map_shortform obs[] = {
      {CONSTELLATION_GPS,
       {5, {2, {{CODE_GPS_L1CA, 107}, {CODE_GPS_L5Q, 120}, {-1, 0}}}}},
      {CONSTELLATION_GPS, {11, {1, {{CODE_GPS_L1CA, 44}, {-1, 0}, {-1, 0}}}}},
      {CONSTELLATION_GPS, {12, {1, {{CODE_GPS_L1CA, 118}, {-1, 0}, {-1, 0}}}}},
      {CONSTELLATION_GPS,
       {13, {2, {{CODE_GPS_L1CA, 160}, {CODE_GPS_L5Q, 123}, {-1, 0}}}}},
      {CONSTELLATION_GPS, {14, {1, {{CODE_GPS_L1CA, 63}, {-1, 0}, {-1, 0}}}}},
      {CONSTELLATION_GPS, {16, {1, {{CODE_GPS_L1CA, 133}, {-1, 0}, {-1, 0}}}}},
      {CONSTELLATION_GPS, {18, {1, {{CODE_GPS_L1CA, 104}, {-1, 0}, {-1, 0}}}}},

      {CONSTELLATION_GAL, {0, {1, {{CODE_GAL_E1X, 115}, {-1, 0}, {-1, 0}}}}},
      {CONSTELLATION_GAL, {12, {1, {{CODE_GAL_E1X, 68}, {-1, 0}, {-1, 0}}}}},
      {CONSTELLATION_GAL,
       {20, {2, {{CODE_GAL_E1X, 120}, {CODE_GAL_E5Q, 125}, {-1, 0}}}}},
      {CONSTELLATION_GAL, {25, {1, {{CODE_GAL_E1X, 128}, {-1, 0}, {-1, 0}}}}},
      {CONSTELLATION_GAL,
       {32, {2, {{CODE_GAL_E1X, 140}, {CODE_GAL_E5Q, 114}, {-1, 0}}}}},

      {CONSTELLATION_BDS, {3, {1, {{CODE_BDS2_B1, 131}, {-1, 0}, {-1, 0}}}}},
      {CONSTELLATION_BDS, {15, {1, {{CODE_BDS2_B1, 72}, {-1, 0}, {-1, 0}}}}},
      {CONSTELLATION_BDS,
       {23, {2, {{CODE_BDS2_B1, 133}, {CODE_BDS3_B5Q, 149}, {-1, 0}}}}}};

  for (size_t i = 0; i < ARRAY_SIZE(obs); i++) {
    struct cons_meas_map_shortform sig = obs[i];
    (map + sig.cons)->sat_data[sig.sid].n_signal = sig.n_signal;
    for (size_t j = 0; j < sig.n_signal; j++) {
      (map + sig.cons)->sat_data[sig.sid].sig_data[j].code = sig.signal[j].code;
      (map + sig.cons)->sat_data[sig.sid].sig_data[j].cn0 = sig.signal[j].cn0;
    }
  }
}

/* Assign expected values of "azel" and "meas state".
 * The naming convention of these functions is
 *  <get_sbp_msg_sv_az_el_ev_xxx()>
 *  <get_sbp_msg_measurement_state_ev_xxx()>
 * which is equiv to rtcm_999_stgsv_payload_xxx. */
static bool get_sbp_msg_sv_az_el_ev_gps(sbp_msg_sv_az_el_t *sv_az_el_ev) {
  int16_t sv_az_el_ev_ar[9][4] = {{6, CODE_GPS_L1CA, 49, 24},
                                  {6, CODE_GPS_L5Q, 49, 24},
                                  {12, CODE_GPS_L1CA, 28, 122},
                                  {13, CODE_GPS_L1CA, 14, 168},
                                  {14, CODE_GPS_L1CA, 21, 44},
                                  {14, CODE_GPS_L5Q, 21, 44},
                                  {15, CODE_GPS_L1CA, 14, 150},
                                  {17, CODE_GPS_L1CA, 44, 68},
                                  {19, CODE_GPS_L1CA, 65, 71}};

  uint8_t first_index = sv_az_el_ev->n_azel;
  sv_az_el_ev->n_azel = ARRAY_SIZE(sv_az_el_ev_ar) + first_index;
  if (sv_az_el_ev->n_azel > SBP_MSG_SV_AZ_EL_AZEL_MAX) {
    return false;
  }

  for (size_t i = first_index; i < sv_az_el_ev->n_azel; i++) {
    sv_az_el_ev->azel[i].sid.sat = sv_az_el_ev_ar[i - first_index][0];
    sv_az_el_ev->azel[i].sid.code = sv_az_el_ev_ar[i - first_index][1];
    sv_az_el_ev->azel[i].el = (int8_t)sv_az_el_ev_ar[i - first_index][2];
    sv_az_el_ev->azel[i].az = (uint8_t)sv_az_el_ev_ar[i - first_index][3];
  }
  return true;
}

static bool get_sbp_msg_sv_az_el_ev_bds7(sbp_msg_sv_az_el_t *sv_az_el_ev) {
  int16_t sv_az_el_ev_ar[4][4] = {{4, CODE_BDS2_B1, 43, 11},
                                  {16, CODE_BDS2_B1, 48, 155},
                                  {24, CODE_BDS2_B1, 15, 25},
                                  {24, CODE_BDS3_B5Q, 15, 25}};

  uint8_t first_index = sv_az_el_ev->n_azel;
  sv_az_el_ev->n_azel = ARRAY_SIZE(sv_az_el_ev_ar) + first_index;
  if (sv_az_el_ev->n_azel > SBP_MSG_SV_AZ_EL_AZEL_MAX) {
    return false;
  }

  for (size_t i = first_index; i < sv_az_el_ev->n_azel; i++) {
    sv_az_el_ev->azel[i].sid.sat = sv_az_el_ev_ar[i - first_index][0];
    sv_az_el_ev->azel[i].sid.code = sv_az_el_ev_ar[i - first_index][1];
    sv_az_el_ev->azel[i].el = (int8_t)sv_az_el_ev_ar[i - first_index][2];
    sv_az_el_ev->azel[i].az = (uint8_t)sv_az_el_ev_ar[i - first_index][3];
  }
  return true;
}

static bool get_sbp_msg_sv_az_el_ev_bds13(sbp_msg_sv_az_el_t *sv_az_el_ev) {
  uint8_t first_index = sv_az_el_ev->n_azel;
  sv_az_el_ev->n_azel = 0 + first_index;

  if (sv_az_el_ev->n_azel > SBP_MSG_SV_AZ_EL_AZEL_MAX) {
    return false;
  }

  return true;
}

static bool get_sbp_msg_measurement_state_ev_gps(
    sbp_msg_measurement_state_t *measurement_state_ev) {
  uint8_t measurement_state_ev_ar[9][3] = {{6, CODE_GPS_L1CA, 107},
                                           {6, CODE_GPS_L5Q, 120},
                                           {12, CODE_GPS_L1CA, 44},
                                           {13, CODE_GPS_L1CA, 118},
                                           {14, CODE_GPS_L1CA, 160},
                                           {14, CODE_GPS_L5Q, 123},
                                           {15, CODE_GPS_L1CA, 63},
                                           {17, CODE_GPS_L1CA, 133},
                                           {19, CODE_GPS_L1CA, 104}};

  uint8_t first_index = measurement_state_ev->n_states;
  measurement_state_ev->n_states =
      ARRAY_SIZE(measurement_state_ev_ar) + first_index;
  if (measurement_state_ev->n_states > SBP_MSG_MEASUREMENT_STATE_STATES_MAX) {
    return false;
  }

  for (size_t i = first_index; i < measurement_state_ev->n_states; i++) {
    measurement_state_ev->states[i].mesid.code =
        measurement_state_ev_ar[i - first_index][1];
    measurement_state_ev->states[i].mesid.sat =
        measurement_state_ev_ar[i - first_index][0];
    measurement_state_ev->states[i].cn0 =
        measurement_state_ev_ar[i - first_index][2];
  }
  return true;
}

static bool get_sbp_msg_measurement_state_ev_bds7(
    sbp_msg_measurement_state_t *measurement_state_ev) {
  uint8_t measurement_state_ev_ar[4][3] = {{4, CODE_BDS2_B1, 131},
                                           {16, CODE_BDS2_B1, 72},
                                           {24, CODE_BDS2_B1, 133},
                                           {24, CODE_BDS3_B5Q, 149}};

  uint8_t first_index = measurement_state_ev->n_states;
  measurement_state_ev->n_states =
      ARRAY_SIZE(measurement_state_ev_ar) + first_index;
  if (measurement_state_ev->n_states > SBP_MSG_MEASUREMENT_STATE_STATES_MAX) {
    return false;
  }

  for (size_t i = first_index; i < measurement_state_ev->n_states; i++) {
    measurement_state_ev->states[i].mesid.code =
        measurement_state_ev_ar[i - first_index][1];
    measurement_state_ev->states[i].mesid.sat =
        measurement_state_ev_ar[i - first_index][0];
    measurement_state_ev->states[i].cn0 =
        measurement_state_ev_ar[i - first_index][2];
  }
  return true;
}

static bool get_sbp_msg_measurement_state_ev_bds13(
    sbp_msg_measurement_state_t *measurement_state_ev) {
  uint8_t first_index = measurement_state_ev->n_states;

  measurement_state_ev->n_states = 0 + first_index;
  if (measurement_state_ev->n_states > SBP_MSG_MEASUREMENT_STATE_STATES_MAX) {
    return false;
  }

  return true;
}

typedef struct {
  sbp_msg_sv_az_el_t sv_az_el_ev;
  sbp_msg_measurement_state_t meas_state_ev;
} rtcm_999_stgsv_test;  // Test msg content only

void cb_test_rtcm_999_stgsv_to_sbp(uint16_t sender_id,
                                   sbp_msg_type_t msg_type,
                                   const sbp_msg_t *msg,
                                   void *content) {
  (void)sender_id;

  rtcm_999_stgsv_test *ctx = (rtcm_999_stgsv_test *)content;
  if (msg_type == SbpMsgSvAzEl) {
    ck_assert_int_eq(msg->sv_az_el.n_azel, ctx->sv_az_el_ev.n_azel);
    for (uint8_t i = 0; i < msg->sv_az_el.n_azel; i++) {
      ck_assert_int_eq(msg->sv_az_el.azel[i].sid.sat,
                       ctx->sv_az_el_ev.azel[i].sid.sat);
      ck_assert_int_eq(msg->sv_az_el.azel[i].sid.code,
                       ctx->sv_az_el_ev.azel[i].sid.code);
      ck_assert_int_eq(msg->sv_az_el.azel[i].el, ctx->sv_az_el_ev.azel[i].el);
      ck_assert_int_eq(msg->sv_az_el.azel[i].az, ctx->sv_az_el_ev.azel[i].az);
    }
  } else if (msg_type == SbpMsgMeasurementState) {
    ck_assert_int_eq(msg->measurement_state.n_states,
                     ctx->meas_state_ev.n_states);
    for (uint8_t i = 0; i < msg->measurement_state.n_states; i++) {
      ck_assert_int_eq(msg->measurement_state.states[i].mesid.sat,
                       ctx->meas_state_ev.states[i].mesid.sat);
      ck_assert_int_eq(msg->measurement_state.states[i].mesid.code,
                       ctx->meas_state_ev.states[i].mesid.code);
      ck_assert_int_eq(msg->measurement_state.states[i].cn0,
                       ctx->meas_state_ev.states[i].cn0);
    }
  } else {
    ck_assert(false);
  }
}

// callback function for the test
// "test_rtcm_999_stgsv_to_sbp_multi_msg_missing_final_msg"
void cb_test_rtcm_999_stgsv_to_sbp_missing_msg(uint16_t sender_id,
                                               sbp_msg_type_t msg_type,
                                               const sbp_msg_t *msg,
                                               void *content) {
  (void)sender_id;

  static uint8_t count = 0;
  uint8_t index = (++count < 2 ? 0 : 1);
  rtcm_999_stgsv_test *ctx = (rtcm_999_stgsv_test *)content + index;
  if (msg_type == SbpMsgSvAzEl) {
    ck_assert_int_eq(msg->sv_az_el.n_azel, ctx->sv_az_el_ev.n_azel);
    for (uint8_t i = 0; i < msg->sv_az_el.n_azel; i++) {
      ck_assert_int_eq(msg->sv_az_el.azel[i].sid.sat,
                       ctx->sv_az_el_ev.azel[i].sid.sat);
      ck_assert_int_eq(msg->sv_az_el.azel[i].sid.code,
                       ctx->sv_az_el_ev.azel[i].sid.code);
      ck_assert_int_eq(msg->sv_az_el.azel[i].el, ctx->sv_az_el_ev.azel[i].el);
      ck_assert_int_eq(msg->sv_az_el.azel[i].az, ctx->sv_az_el_ev.azel[i].az);
    }
  } else if (msg_type == SbpMsgMeasurementState) {
    ck_assert_int_eq(msg->measurement_state.n_states,
                     ctx->meas_state_ev.n_states);
    for (uint8_t i = 0; i < msg->measurement_state.n_states; i++) {
      ck_assert_int_eq(msg->measurement_state.states[i].mesid.sat,
                       ctx->meas_state_ev.states[i].mesid.sat);
      ck_assert_int_eq(msg->measurement_state.states[i].mesid.code,
                       ctx->meas_state_ev.states[i].mesid.code);
      ck_assert_int_eq(msg->measurement_state.states[i].cn0,
                       ctx->meas_state_ev.states[i].cn0);
    }
  } else {
    ck_assert(false);
  }
}

START_TEST(test_rtcm_999_stgsv_to_sbp_single_msg) {
  // fake rtcm_999_stgsv_payload_gps to have multiple msg flag.
  uint8_t rtcm_999_stgsv_payload_gps_fake_finalmsg[ARRAY_SIZE(
      rtcm_999_stgsv_payload_gps)] = {};
  uint16_t payload_len = ARRAY_SIZE(rtcm_999_stgsv_payload_gps);
  memcpy(&rtcm_999_stgsv_payload_gps_fake_finalmsg,
         &rtcm_999_stgsv_payload_gps,
         payload_len);
  rtcm_999_stgsv_payload_gps_fake_finalmsg[13] &= 0xFD;

  rtcm_999_stgsv_test ev = {0};
  if (!get_sbp_msg_sv_az_el_ev_gps(&ev.sv_az_el_ev) ||
      !get_sbp_msg_measurement_state_ev_gps(&ev.meas_state_ev)) {
    assert(false);
  }

  rtcm2sbp_init(&state_gsv, NULL, cb_test_rtcm_999_stgsv_to_sbp, NULL, &ev);
  set_sample_cons_meas_map(&state_gsv);

  rtcm2sbp_decode_payload(
      rtcm_999_stgsv_payload_gps_fake_finalmsg, payload_len, &state_gsv);
}
END_TEST

/* FUNCTIONALITY: Test rtcm2sbp_decode_payload() when the multiple msg are
 * received still the last msg with "multiple msg flag == false".
 * EXPECTATION: All the processed sbp msg are stacked, and sent out at once. */
START_TEST(test_rtcm_999_stgsv_to_sbp_multi_msg) {
  uint16_t payload_len_gps = ARRAY_SIZE(rtcm_999_stgsv_payload_gps);
  uint16_t payload_len_bds7 = ARRAY_SIZE(rtcm_999_stgsv_payload_bds7);
  uint16_t payload_len_bds13 = ARRAY_SIZE(rtcm_999_stgsv_payload_bds13);

  rtcm_999_stgsv_test ev = {0};
  if (!get_sbp_msg_sv_az_el_ev_gps(&ev.sv_az_el_ev) ||
      !get_sbp_msg_measurement_state_ev_gps(&ev.meas_state_ev) ||
      !get_sbp_msg_sv_az_el_ev_bds7(&ev.sv_az_el_ev) ||
      !get_sbp_msg_measurement_state_ev_bds7(&ev.meas_state_ev) ||
      !get_sbp_msg_sv_az_el_ev_bds13(&ev.sv_az_el_ev) ||
      !get_sbp_msg_measurement_state_ev_bds13(&ev.meas_state_ev)) {
    assert(false);
  }

  rtcm2sbp_init(&state_gsv, NULL, cb_test_rtcm_999_stgsv_to_sbp, NULL, &ev);
  set_sample_cons_meas_map(&state_gsv);

  rtcm2sbp_decode_payload(
      rtcm_999_stgsv_payload_gps, payload_len_gps, &state_gsv);
  rtcm2sbp_decode_payload(
      rtcm_999_stgsv_payload_bds7, payload_len_bds7, &state_gsv);
  rtcm2sbp_decode_payload(
      rtcm_999_stgsv_payload_bds13, payload_len_bds13, &state_gsv);
}
END_TEST

/* FUNCTIONALITY: Test rtcm2sbp_decode_payload() when the final message
 * containing "multiple msg flag" is missing.
 * EXPECTATION: The msg missing final msg is sent out. Then the next full msg is
 * sent. */
START_TEST(test_rtcm_999_stgsv_to_sbp_multi_msg_missing_final_msg) {
  // fake rtcm_999_stgsv_payload_gps with tow of previous msg set. (tow =
  // 392173000)
  uint8_t rtcm_999_stgsv_payload_gps_fake_tow[ARRAY_SIZE(
      rtcm_999_stgsv_payload_gps)] = {};
  uint16_t payload_len_gps = ARRAY_SIZE(rtcm_999_stgsv_payload_gps);
  memcpy(&rtcm_999_stgsv_payload_gps_fake_tow,
         &rtcm_999_stgsv_payload_gps,
         payload_len_gps);
  rtcm_999_stgsv_payload_gps_fake_tow[4] = 0x05;
  rtcm_999_stgsv_payload_gps_fake_tow[5] = 0x72;

  uint16_t payload_len_bds13 = ARRAY_SIZE(rtcm_999_stgsv_payload_bds13);
  uint16_t payload_len_gps_fake_tow = payload_len_gps;

  rtcm_999_stgsv_test ev1 = {0};
  if (!get_sbp_msg_sv_az_el_ev_gps(&ev1.sv_az_el_ev) ||
      !get_sbp_msg_measurement_state_ev_gps(&ev1.meas_state_ev)) {
    assert(false);
  }  // msg 0 same as msg 1 except for tow (sample)

  rtcm_999_stgsv_test ev2 = {0};
  if (!get_sbp_msg_sv_az_el_ev_gps(&ev2.sv_az_el_ev) ||
      !get_sbp_msg_measurement_state_ev_gps(&ev2.meas_state_ev) ||
      !get_sbp_msg_sv_az_el_ev_bds13(&ev2.sv_az_el_ev) ||
      !get_sbp_msg_measurement_state_ev_bds13(&ev2.meas_state_ev)) {
    assert(false);
  }

  rtcm_999_stgsv_test ev[2] = {ev1, ev2};
  rtcm2sbp_init(
      &state_gsv, NULL, cb_test_rtcm_999_stgsv_to_sbp_missing_msg, NULL, &ev);
  set_sample_cons_meas_map(&state_gsv);

  rtcm2sbp_decode_payload(rtcm_999_stgsv_payload_gps_fake_tow,
                          payload_len_gps_fake_tow,
                          &state_gsv);
  rtcm2sbp_decode_payload(
      rtcm_999_stgsv_payload_gps, payload_len_gps, &state_gsv);
  rtcm2sbp_decode_payload(
      rtcm_999_stgsv_payload_bds13, payload_len_bds13, &state_gsv);
}
END_TEST

Suite *rtcm3_suite(void) {
  Suite *s = suite_create("RTCMv3");

  TCase *tc_core = tcase_create("Core");
  tcase_add_checked_fixture(tc_core, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_core, test_gps_time);
  tcase_add_test(tc_core, test_glo_day_rollover);
  tcase_add_test(tc_core, test_1012_first);
  tcase_add_test(tc_core, test_glo_5hz);
  tcase_add_test(tc_core, test_glonass_computer_time_invalid_input);
  tcase_add_test(tc_core, test_glonass_tod_rollover);
  tcase_add_test(tc_core, test_glonass_wn_rollover);
  tcase_add_test(tc_core, test_glonass_leap_second_event_ideal);
  suite_add_tcase(s, tc_core);

  TCase *tc_biases = tcase_create("Biases");
  tcase_add_checked_fixture(tc_biases, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_biases, test_bias_trm);
  tcase_add_test(tc_biases, test_bias_jav);
  tcase_add_test(tc_biases, test_bias_nov);
  tcase_add_test(tc_biases, test_bias_sep);
  tcase_add_test(tc_biases, test_bias_nav);
  tcase_add_test(tc_biases, test_bias_top);
  tcase_add_test(tc_biases, test_bias_hem);
  suite_add_tcase(s, tc_biases);

  TCase *tc_gpp_biases = tcase_create("Geo++ Biases");
  tcase_add_checked_fixture(tc_gpp_biases, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_ash1);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_hem);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_jav);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_jps);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_lei);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_nav);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_nov);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_nvr);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_sep1);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_sok);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_tps1);
  tcase_add_test(tc_gpp_biases, test_bias_gpp_trm);
  suite_add_tcase(s, tc_gpp_biases);

  TCase *tc_msm = tcase_create("MSM");
  tcase_add_checked_fixture(tc_msm, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_msm, test_msm5_parse);
  tcase_add_test(tc_msm, test_msm7_parse);
  tcase_add_test(tc_msm, test_msm_switching);
  tcase_add_test(tc_msm, test_msm_mixed);
  tcase_add_test(tc_msm, test_msm_missing_obs);
  tcase_add_test(tc_msm, test_msm_week_rollover);
  tcase_add_test(tc_msm, test_msm_gal_gaps);
  suite_add_tcase(s, tc_msm);

  TCase *tc_eph = tcase_create("ephemeris");
  tcase_add_checked_fixture(tc_eph, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_eph, tc_rtcm_eph_gps);
  tcase_add_test(tc_eph, tc_rtcm_eph_glo);
  tcase_add_test(tc_eph, tc_rtcm_eph_gal);
  tcase_add_test(tc_eph, tc_rtcm_eph_bds);
  tcase_add_test(tc_eph, tc_rtcm_eph_wn_rollover);
  tcase_add_test(tc_eph, tc_rtcm_eph_wn_rollover2);
  tcase_add_test(tc_eph, tc_rtcm_eph_invalid_gal_time);
  suite_add_tcase(s, tc_eph);

  TCase *tc_sbp_to_rtcm = tcase_create("sbp2rtcm");
  tcase_add_checked_fixture(tc_sbp_to_rtcm, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_legacy);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_msm);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_msm_roundtrip);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_gps_eph);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_gal_fnav_eph);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_gal_inav_eph);
  tcase_add_test(tc_sbp_to_rtcm, test_rtcm_1013);
  tcase_add_test(tc_sbp_to_rtcm, test_rtcm_1013_handle);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_1019_validity_decoding);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_1019_roundtrip);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_1020_roundtrip);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_1042_roundtrip);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_1045_roundtrip);
  tcase_add_test(tc_sbp_to_rtcm, test_sbp_to_rtcm_1046_roundtrip);
  suite_add_tcase(s, tc_sbp_to_rtcm);

  TCase *tc_rtcm_to_sbp = tcase_create("rtcm2sbp");
  tcase_add_checked_fixture(tc_rtcm_to_sbp, rtcm3_setup_basic, NULL);
  tcase_add_test(tc_rtcm_to_sbp, test_rtcm_999_stgsv_to_sbp_single_msg);
  tcase_add_test(tc_rtcm_to_sbp, test_rtcm_999_stgsv_to_sbp_multi_msg);
  tcase_add_test(tc_rtcm_to_sbp,
                 test_rtcm_999_stgsv_to_sbp_multi_msg_missing_final_msg);
  suite_add_tcase(s, tc_rtcm_to_sbp);

  return s;
}
