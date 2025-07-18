/*
 * Copyright (C) 2017 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef LIBRTCM_RTCM_DECODER_TESTS_H
#define LIBRTCM_RTCM_DECODER_TESTS_H

#include <rtcm3/messages.h>

static void test_rtcm_msg_num_to_msg_type(void);
static void test_rtcm_content_decode(void);
static void test_rtcm_999_stgsv_en_de(void);
static void test_rtcm_999_stgsv_de_en(void);
static void test_rtcm_999_restart_en_de(void);
static void test_rtcm_999_restart_de_en(void);
static void test_rtcm_999_aux_ttff_en_de(void);
static void test_rtcm_999_aux_ttff_de_en(void);
static void test_rtcm_1001(void);
static void test_rtcm_1002(void);
static void test_rtcm_1003(void);
static void test_rtcm_1004(void);
static void test_rtcm_1005(void);
static void test_rtcm_1006(void);
static void test_rtcm_1007(void);
static void test_rtcm_1008(void);
static void test_rtcm_1010(void);
static void test_rtcm_1012(void);
static void test_rtcm_1013_ok(void);
static void test_rtcm_1013_ok_zero_messages(void);
static void test_rtcm_1013_ok_max_messages(void);
static void test_rtcm_1013_message_type_mismatch(void);
static void test_rtcm_1013_invalid_message(void);
static void test_rtcm_1019(void);
static void test_rtcm_1020(void);
static void test_rtcm_1029(void);
static void test_rtcm_1033(void);
static void test_rtcm_1042(void);
static void test_rtcm_1045(void);
static void test_rtcm_1046(void);
static void test_rtcm_1230(void);
static void test_rtcm_msm4(void);
static void test_rtcm_msm5(void);
static void test_rtcm_msm5_glo(void);
static void test_rtcm_msm7(void);
static void test_rtcm_4062(void);
static void test_rtcm_random_bits(void);
static void test_msm_bit_utils(void);
static void test_lock_time_decoding(void);
static void test_logging(void);

bool payload_equals(const uint8_t lhs[], const uint8_t rhs[], uint8_t len);
bool msgobs_equals(const rtcm_obs_message *msg_in,
                   const rtcm_obs_message *msg_out);
bool msgobs_glo_equals(const rtcm_obs_message *msg_in,
                       const rtcm_obs_message *msg_out);
bool msg999stgsv_equals(const rtcm_msg_999_stgsv *lhs,
                        const rtcm_msg_999_stgsv *rhs);
bool msg999restart_equals(const rtcm_msg_999_restart *lhs,
                          const rtcm_msg_999_restart *rhs);
bool msg999aux_ttff_equals(const rtcm_msg_999_aux_ttff *lhs,
                           const rtcm_msg_999_aux_ttff *rhs);
bool msg1005_equals(const rtcm_msg_1005 *lhs, const rtcm_msg_1005 *rhs);
bool msg1006_equals(const rtcm_msg_1006 *lhs, const rtcm_msg_1006 *rhs);
bool msg1007_equals(const rtcm_msg_1007 *lhs, const rtcm_msg_1007 *rhs);
bool msg1008_equals(const rtcm_msg_1008 *lhs, const rtcm_msg_1008 *rhs);
bool msg1029_equals(const rtcm_msg_1029 *lhs, const rtcm_msg_1029 *rhs);
bool msg1033_equals(const rtcm_msg_1033 *lhs, const rtcm_msg_1033 *rhs);
bool msg1230_equals(const rtcm_msg_1230 *lhs, const rtcm_msg_1230 *rhs);

/* raw received 1077 message */
static const uint8_t msm7_raw[] = {
    0x43, 0x50, 0x00, 0x6F, 0x3B, 0x96, 0x02, 0x00, 0x00, 0x42, 0x2B, 0x52,
    0xC0, 0x80, 0x00, 0x00, 0x00, 0x28, 0x20, 0x40, 0x80, 0x7F, 0xF3, 0xF7,
    0x3D, 0xEE, 0x73, 0xFF, 0xFA, 0x92, 0x7A, 0xA2, 0x2A, 0x9A, 0x9A, 0x4A,
    0x2A, 0xA2, 0x22, 0x8A, 0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x8E,
    0xFD, 0xA4, 0xD5, 0xB6, 0xF7, 0xE0, 0xB9, 0x33, 0x79, 0x89, 0x09, 0x3A,
    0x0E, 0x18, 0x2F, 0x7E, 0x9F, 0x83, 0x75, 0xEF, 0x27, 0xA7, 0xC1, 0x30,
    0x01, 0xA7, 0xFD, 0xEF, 0xD2, 0x00, 0x22, 0x79, 0x93, 0xF1, 0x98, 0x58,
    0x2D, 0x85, 0x66, 0x50, 0x5D, 0xCA, 0x85, 0xE1, 0x48, 0x57, 0x83, 0x0D,
    0x59, 0xE8, 0xD7, 0xC0, 0x8D, 0xC0, 0x40, 0xDC, 0xBE, 0x0D, 0xD0, 0x2E,
    0xB5, 0xB9, 0x40, 0x00, 0x04, 0x00, 0x00, 0x0D, 0xF6, 0x48, 0xE0, 0x2D,
    0x0D, 0x54, 0xC8, 0xD5, 0x9F, 0x01, 0x4F, 0x98, 0x11, 0x1B, 0x00, 0xC4,
    0xD9, 0x24, 0x91, 0x12, 0x78, 0xF1, 0x22, 0x3B, 0x12, 0x32, 0x79, 0x2D,
    0x26, 0x12, 0xEA, 0xE9, 0x24, 0x4B, 0x12, 0x53, 0xE6, 0xA9, 0x8C, 0xEA,
    0x7C, 0xA6, 0x99, 0xD9, 0x79, 0x28, 0x47, 0x96, 0x4A, 0x78, 0x6D, 0xC9,
    0x4A, 0xFE, 0x14, 0xBC, 0xB9, 0x4B, 0x7D, 0x14, 0xAC, 0x29, 0x4A, 0xDA,
    0x9B, 0x61, 0xD9, 0xB4, 0x1F, 0x1B, 0xA0, 0x11, 0xBA, 0x4B, 0x1B, 0xA6,
    0xD8, 0xA2, 0xD3, 0x8A, 0x5A, 0x60, 0xA2, 0x53, 0x0A, 0x36, 0x18, 0x9B,
    0xD8, 0x01, 0x55, 0x15, 0x81, 0x53, 0x03, 0x81, 0x53, 0x14, 0x01, 0x53,
    0x15, 0x01, 0x5C, 0xD4, 0x03, 0x62, 0xD7, 0x03, 0x63, 0x7A, 0x03, 0x65,
    0xDC, 0x03, 0x65, 0xDB, 0x03, 0x6E, 0x7E, 0x7A, 0xCC, 0x20, 0xC0, 0x00,
    0x00, 0x40, 0x00, 0x00, 0x03, 0x80, 0xD9, 0x03, 0x81, 0x80, 0x83, 0x81,
    0x97, 0x03, 0x81, 0x93, 0x00, 0x33, 0xE2, 0x80, 0x4C, 0x86, 0x80, 0x4C,
    0x7D, 0x84, 0x88, 0xC9, 0x84, 0x89, 0x73, 0x04, 0x86, 0x86, 0x84, 0x86,
    0x90, 0x04, 0xB6, 0x99, 0x04, 0xB7, 0x41, 0x04, 0xB6, 0x6F, 0x84, 0xB6,
    0x6A, 0xFA, 0xA5, 0x5B, 0xFA, 0xA6, 0x02, 0xFA, 0xA6, 0xE6, 0x7E, 0x4B,
    0x12, 0xFE, 0x4B, 0xC0, 0x7E, 0x4C, 0xDD, 0x05, 0x2F, 0x6E, 0x85, 0x30,
    0x17, 0x85, 0x2F, 0xF3, 0x85, 0x2F, 0xF6, 0x85, 0x2E, 0x06, 0x06, 0xC7,
    0x73, 0x86, 0xC8, 0x12, 0x06, 0xCA, 0x7D, 0x86, 0xCA, 0x95, 0x86, 0xDA,
    0xD4, 0x02, 0x83, 0x71, 0x02, 0x84, 0x18, 0x82, 0x83, 0x7F, 0x82, 0x83,
    0x81, 0x02, 0x88, 0x9B, 0x3D, 0x2F, 0x4B, 0xD2, 0xF4, 0xBD, 0x2F, 0x4B,
    0xD2, 0xF4, 0xBD, 0x2F, 0x4A, 0xEE, 0x00, 0x00, 0x0F, 0x4B, 0xD2, 0xF4,
    0xBD, 0x2F, 0x43, 0x84, 0xE1, 0x3D, 0x2F, 0x4B, 0xD2, 0xF4, 0xBD, 0x2F,
    0x4B, 0xD2, 0xF4, 0xBD, 0x2F, 0x4B, 0xD2, 0xDE, 0x35, 0x2D, 0x4B, 0xD2,
    0xF4, 0xBD, 0x2F, 0x4B, 0xD2, 0xF4, 0xBD, 0x2F, 0x4B, 0xD2, 0xF4, 0xBD,
    0x2F, 0x4B, 0xD2, 0xF4, 0xBD, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09,
    0xC1, 0x7C, 0x5F, 0x28, 0x8C, 0x12, 0x98, 0x78, 0x1E, 0x0A, 0xD3, 0x1C,
    0x85, 0x00, 0x00, 0x03, 0x58, 0xB5, 0x2D, 0x4C, 0xB2, 0x38, 0x52, 0x14,
    0x89, 0xE1, 0x68, 0x5A, 0x26, 0x8D, 0x12, 0xB4, 0xAD, 0x30, 0xCD, 0x32,
    0xE0, 0xB8, 0x22, 0xC4, 0x41, 0x10, 0xE2, 0x32, 0x0C, 0x83, 0x7C, 0xF5,
    0x27, 0xC6, 0x41, 0x90, 0xA4, 0x2F, 0x8B, 0xD2, 0x40, 0x90, 0x2E, 0xCD,
    0x00, 0x69, 0xC0, 0xD3, 0x81, 0xA7, 0xE3, 0x5F, 0x86, 0xA8, 0xD8, 0x47,
    0xB0, 0x8F, 0x5F, 0xF2, 0xC0, 0x7D, 0x80, 0x62, 0x49, 0xB0, 0x00, 0x20,
    0x00, 0x6A, 0x63, 0xD4, 0xC7, 0xAA, 0x3B, 0x54, 0x0D, 0x41, 0xDA, 0x83,
    0xB5, 0x02, 0x09, 0x10, 0x92, 0x21, 0x22, 0xC4, 0x44, 0x50, 0x1B, 0x30,
    0x36, 0x60, 0x6B, 0xE0, 0xD4, 0x6E, 0x30, 0xDC, 0x61, 0xB8, 0xDF, 0x7D,
    0x36, 0xFA, 0x6E, 0x03, 0xF2, 0x29, 0xE4, 0x53, 0xC8, 0xA1, 0x91, 0x43,
    0x22, 0xA5, 0x57, 0x92, 0xAF, 0x25, 0x5C, 0x9A, 0xC4, 0x35, 0x8B, 0xED,
    0x5A, 0xDA, 0xB5, 0xB5, 0xF3, 0x6B, 0x86, 0xD7, 0x00};

static const rtcm_msm_header msm7_expected_header = {
    1077,
    0,
    466544000,
    1,
    0,
    0,
    0,
    0,
    0,
    0,
    {
        1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1,
        0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    },
    {
        0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
    },
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0,
     1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0,
     1, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

static const rtcm_msm_sat_data msm7_expected_sat_data[] = {
    {0, 24641242.0 / PRUNIT_GPS, 379.0},
    {0, 23832036.6 / PRUNIT_GPS, -705.0},
    {0, 25268639.7 / PRUNIT_GPS, 442.0},
    {0, 20888859.3 / PRUNIT_GPS, -540.0},
    {0, 25143628.6 / PRUNIT_GPS, -706.0},
    {0, 25164122.2 / PRUNIT_GPS, 608.0},
    {0, 22018643.5 / PRUNIT_GPS, 211.0},
    {0, 20868072.9 / PRUNIT_GPS, -67.0},
    {0, 25239948.6 / PRUNIT_GPS, -368.0},
    {0, 20541053.2 / PRUNIT_GPS, 68.0},
    {0, 24419032.6 / PRUNIT_GPS, -823.0},
    {0, 23216056.8 / PRUNIT_GPS, -461.0}};

static const rtcm_msm_signal_data msm7_expected_sig_data[] = {
    {82.19442001, 82.19441726, 671.744, 0, 39, {31}, 379.3314074},    /* 1C */
    {82.19441831, 82.19441677, 671.744, 0, 23.8, {31}, 379.3314074},  /* 1W */
    {82.19442538, 82.19441678, 671.744, 0, 23.8, {31}, 379.3317245},  /* 2W */
    {82.19442565, 82.19441678, 671.744, 0, 40.5, {31}, 379.3561455},  /* 2S */
    {82.19441938, 82.19441911, 671.744, 0, 48.2, {31}, 379.3370334},  /* 5X */
    {79.49532093, 79.49532386, 671.744, 0, 41.5, {31}, -706.0085554}, /* 1C */
    {79.49532296, 79.49532401, 671.744, 0, 30, {31}, -706.0085554},   /* etc */
    {79.495327, 79.49532458, 671.744, 0, 30, {31}, -706.0239375},
    {79.4953277, 79.49532458, 671.744, 0, 43.2, {31}, -706.011727},
    {79.49532796, 79.49532664, 671.744, 0, 49.8, {31}, -706.026592},
    {84.2867944, 84.28679183, 56.32, 0, 33.2, {31}, 442.470848},
    {84.28613281, 0, 0, 0, 0, {8}, 0},
    {84.28613281, 0, 0, 0, 0, {8}, 0},
    {69.67794744, 69.6779482, 671.744, 0, 53.5, {31}, -540.548207},
    {69.67794817, 69.67794836, 671.744, 0, 45.2, {31}, -540.548207},
    {69.6779378, 69.67794838, 671.744, 0, 45.2, {31}, -540.5593074},
    {69.6779381, 69.67794837, 671.744, 0, 50.8, {31}, -540.5593074},
    {83.87013719, 83.87012956, 655.36, 0, 35.5, {31}, -707.1312881},
    {83.87013348, 83.87013543, 278.528, 0, 20.5, {31}, -707.1312881},
    {83.87012891, 83.87013543, 278.528, 0, 20.5, {31}, -707.135094},
    {83.93875559, 83.93875332, 671.744, 0, 39.5, {31}, 608.4640188},
    {83.93875843, 83.93875348, 671.744, 0, 22.5, {31}, 608.4640188},
    {83.93875336, 83.93875278, 671.744, 0, 22.5, {31}, 608.4497467},
    {83.93875422, 83.93875279, 671.744, 0, 38.5, {31}, 608.4253257},
    {73.44657626, 73.44657674, 671.744, 0, 52.2, {31}, 211.0927712},
    {73.44657773, 73.44657689, 671.744, 0, 43.2, {31}, 211.0927712},
    {73.44656783, 73.4465767, 671.744, 0, 43.2, {31}, 211.0953085},
    {73.44656873, 73.44657669, 671.744, 0, 48.8, {31}, 211.0953085},
    {69.60807186, 69.60807165, 671.744, 0, 52.8, {31}, -67.91581182},
    {69.60807016, 69.60807181, 671.744, 0, 46, {31}, -67.91581182},
    {69.60805688, 69.60807202, 671.744, 0, 46, {31}, -67.91486035},
    {84.19130184, 84.19130208, 245.76, 0, 34.8, {31}, -368.846226},
    {84.19130544, 84.19130224, 167.936, 0, 17, {31}, -368.846226},
    {84.19129073, 84.19130251, 167.936, 0, 17, {31}, -368.8062643},
    {68.51789377, 68.51789461, 671.744, 0, 56.5, {31}, 68.44863411},
    {68.51789454, 68.51789476, 671.744, 0, 50, {31}, 68.44863411},
    {68.51789427, 68.51789473, 671.744, 0, 50, {31}, 68.45212282},
    {68.51789357, 68.51789474, 671.744, 0, 55.8, {31}, 68.45212282},
    {68.51789364, 68.51789428, 671.744, 0, 61.2, {31}, 68.44681391},
    {81.4535428, 81.45353876, 671.744, 0, 39.8, {31}, -824.0857794},
    {81.4535409, 81.45353891, 671.744, 0, 25, {31}, -824.0857794},
    {81.45354654, 81.45353949, 671.744, 0, 25, {31}, -824.0873652},
    {81.45354681, 81.45353951, 671.744, 0, 41, {31}, -824.0629442},
    {81.45354694, 81.45354338, 671.744, 0, 47.5, {31}, -824.0629442},
    {77.44058498, 77.4405831, 671.744, 0, 47.2, {31}, -461.9569201},
    {77.44058765, 77.44058325, 671.744, 0, 36, {31}, -461.9569201},
    {77.44058448, 77.44058311, 671.744, 0, 36, {31}, -461.9480397},
    {77.44058551, 77.44058311, 671.744, 0, 46.8, {31}, -461.9480397},
    {77.44057831, 77.44058433, 671.744, 0, 52, {31}, -461.9522868}};

#endif /* LIBRTCM_RTCM_DECODER_TESTS_H */
