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

#ifndef SWIFTNAV_RTCM3_CONSTANTS_H
#define SWIFTNAV_RTCM3_CONSTANTS_H

/** Fundamental RTCM3 constants */
#define RTCM3_PREAMBLE 0xD3

#define RTCM3_MSG_OVERHEAD 6U
#define RTCM3_MIN_MSG_LEN 2U  // At least 12 bit message type
#define RTCM3_MAX_MSG_LEN 1023
#define RTCM3_MAX_FRAME_LEN (RTCM3_MAX_MSG_LEN + RTCM3_MSG_OVERHEAD)
#define RTCM3_FIFO_SIZE 4096
#define RTCM3_BUFFER_SIZE \
  (RTCM3_FIFO_SIZE - RTCM3_MSG_OVERHEAD - RTCM3_MAX_MSG_LEN)

#define PRUNIT_GPS 299792.458 /**< RTCM v3 Unit of GPS Pseudorange (m) */
#define PRUNIT_GLO 599584.916 /**< RTCM v3 Unit of GLO Pseudorange (m) */
#define RTCM_MAX_SATS 32
#define CP_INVALID 0xFFF80000      /* Unsigned bit pattern 0x80000 */
#define PR_L1_INVALID 0xFFF80000   /* Unsigned bit pattern 0x80000 */
#define PR_L2_INVALID 0xFFFFE000   /* Unsigned bit pattern 0x20000 */
#define MSM_MAX_CELLS 64           /* Maximum number of cells in MSM message */
#define MSM_SATELLITE_MASK_SIZE 64 /* Maximum size of MSM satellite mask */
#define MSM_SIGNAL_MASK_SIZE 32    /* Maximum size of MSM signal mask */
#define MSM_ROUGH_RANGE_INVALID 0xFF   /* Unsigned bit pattern 0xFF */
#define MSM_ROUGH_RATE_INVALID (-8192) /* Signed bit pattern 0x2000 */
#define MSM_PR_INVALID (-16384)        /* Signed bit pattern 0x4000 */
#define MSM_PR_EXT_INVALID (-524288)   /* Signed bit pattern 0x80000 */
#define MSM_CP_INVALID (-2097152)      /* Signed bit pattern 0x200000 */
#define MSM_CP_EXT_INVALID (-8388608)  /* Signed bit pattern 0x800000 */
#define MSM_DOP_INVALID (-16384)       /* Signed bit pattern 0x4000 */

#define RTCM_MAX_STRING_LEN 32 /* Max length of strings in 1008, 1033, etc */

#define MSM_GLO_FCN_OFFSET 7 /* Offset for FCN coding in sat_info */
#define MSM_GLO_MAX_FCN 13
#define MSM_GLO_FCN_UNKNOWN 255

#define MT1012_GLO_FCN_OFFSET 7
#define MT1012_GLO_MAX_FCN 20

/* maximum value for time-of-week in integer milliseconds */
#define RTCM_MAX_TOW_MS (7 * 24 * 3600 * 1000 - 1)
/* maximum value for time-of-day in integer milliseconds */
#define RTCM_GLO_MAX_TOW_MS (24 * 3600 * 1000 - 1)

/* maximum value for antenna height DF028 */
#define RTCM_1006_MAX_ANTENNA_HEIGHT_M 6.5535

/* Fieldmask bit of RTCM 999 STGSV  */
typedef enum {
  RTCM_STGSV_FIELDMASK_BIT_EL = 0,
  RTCM_STGSV_FIELDMASK_BIT_AZ,
  RTCM_STGSV_FIELDMASK_BIT_CN0_B1,
  RTCM_STGSV_FIELDMASK_BIT_CN0_B2,
  RTCM_STGSV_FIELDMASK_BIT_CN0_B3,
  RTCM_STGSV_FIELDMASK_BIT_RES1,
  RTCM_STGSV_FIELDMASK_BIT_RES2,
  RTCM_STGSV_FIELDMASK_BIT_RES3,
  RTCM_STGSV_FIELDMASK_SIZE,
} rtcm_stgsv_field_mask;

/* Maximum size of STGSV satellite mask - Field DF07P */
#define RTCM_TESEOV_SATELLITE_MASK_SIZE 40
#define RTCM_TESEOV_SATELLITE_MASK_SIZE_GNSS13 24

#define RTCM_STGSV_CN0_NOT_VALID (uint8_t)0xFF
#define RTCM_STGSV_EL_NOT_VALID (int8_t)0x80
#define RTCM_STGSV_AZ_NOT_VALID (uint16_t)0x1FF

#define RTCM_STGSV_FIELDMASK_EL (uint8_t)(0x01 << RTCM_STGSV_FIELDMASK_BIT_EL)
#define RTCM_STGSV_FIELDMASK_AZ (uint16_t)(0x01 << RTCM_STGSV_FIELDMASK_BIT_AZ)
#define RTCM_STGSV_FIELDMASK_CN0_B1 \
  (uint8_t)(0x01 << RTCM_STGSV_FIELDMASK_BIT_CN0_B1)
#define RTCM_STGSV_FIELDMASK_CN0_B2 \
  (uint8_t)(0x01 << RTCM_STGSV_FIELDMASK_BIT_CN0_B2)
#define RTCM_STGSV_FIELDMASK_CN0_B3 \
  (uint8_t)(0x01 << RTCM_STGSV_FIELDMASK_BIT_CN0_B3)
#define RTCM_STGSV_FIELDMASK_AZEL \
  (RTCM_STGSV_FIELDMASK_EL | RTCM_STGSV_FIELDMASK_AZ)
#define RTCM_STGSV_FIELDMASK_CN0                               \
  (RTCM_STGSV_FIELDMASK_CN0_B1 | RTCM_STGSV_FIELDMASK_CN0_B2 | \
   RTCM_STGSV_FIELDMASK_CN0_B3)

#define MAX_NDF_FRAMES 63
#define MAX_NDF_FRAME_SIZE_BITS 4096 /* max uint12 */
#define MAX_NDF_FRAME_SIZE_WORDS (MAX_NDF_FRAME_SIZE_BITS / (8 * 4))

/** 2^-4 */
#define C_1_2P4 0.0625
/** 2^-8 */
#define C_1_2P8 0.00390625
/** 2^-10 */
#define C_1_2P10 0.0009765625
/** 2^-24 */
#define C_1_2P24 5.960464477539063e-08
/** 2^-29 */
#define C_1_2P29 1.862645149230957e-09
/** 2^-31 */
#define C_1_2P31 4.656612873077393e-10
/** 2^14 */
#define C_2P14 16384
/** 2^19 */
#define C_2P19 524288
/** 2^30 */
#define C_2P30 1073741824

/** Constant difference of Beidou time from GPS time */
#define BDS_SECOND_TO_GPS_SECOND 14

/** The official GPS value of the speed of light in m / s.
 * \note This is the exact value of the speed of light in vacuum (by the
 * definition of meters). */
#define GPS_C 299792458.0

/** The GPS L1 center frequency in Hz. */
#define GPS_L1_HZ 1.57542e9

/** The GPS L2 center frequency in Hz. */
#define GPS_L2_HZ 1.22760e9

/** The GLO L1 center frequency in Hz. */
#define GLO_L1_HZ 1.602e9

/** The GLO L2 center frequency in Hz. */
#define GLO_L2_HZ 1.246e9

/** Frequency range between two adjacent GLO channel in Hz for L1 band*/
#define GLO_L1_DELTA_HZ 5.625e5

/** Frequency range between two adjacent GLO channel in Hz for L2 band */
#define GLO_L2_DELTA_HZ 4.375e5

#define PRN_INVALID 0

#define SWIFT_PROPRIETARY_MSG 4062

#endif /* SWIFTNAV_RTCM3_CONSTANTS_H */
