/**
 * Copyright (C) 2022 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <gnss-converters/internal/time_truth_v2.h>
#include <gtest/gtest.h>

#include <condition_variable>
#include <cstdlib>
#include <mutex>
#include <thread>

namespace {
struct EphemerisTimeEstimatorInput {
  gnss_signal_t gnss_signal;
  gps_time_t gps_time;
};

constexpr double kDoubleTolerance = 1e-5;

constexpr int16_t kEphemerisTimeWn = 2192;
constexpr uint32_t kEphemerisTimeTowMs = 113279 * SECS_MS;
constexpr gps_time_t kEphemerisTimeGps{kEphemerisTimeTowMs / SECS_MS,
                                       kEphemerisTimeWn};

constexpr EphemerisTimeEstimatorInput kGpsEphemerisTimes[] = {
    {{1, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{2, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{3, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{4, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{5, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{6, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{7, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{8, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{9, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{10, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{12, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{13, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{14, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{15, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{16, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{17, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{18, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{19, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{20, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{21, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{22, static_cast<code_t>(0)}, {410400.000000, 2186}},
    {{23, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{24, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{25, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{26, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{27, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{29, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{30, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{31, static_cast<code_t>(0)}, {115200.000000, 2192}},
    {{32, static_cast<code_t>(0)}, {115200.000000, 2192}},
};

constexpr EphemerisTimeEstimatorInput kGalEphemerisTimes[] = {
    {{1, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{2, static_cast<code_t>(14)}, {108000.000000, 2192}},
    {{3, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{4, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{5, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{7, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{8, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{9, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{11, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{12, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{13, static_cast<code_t>(14)}, {108600.000000, 2192}},
    {{15, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{19, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{21, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{24, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{25, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{26, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{27, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{30, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{31, static_cast<code_t>(14)}, {109200.000000, 2192}},
    {{33, static_cast<code_t>(14)}, {111000.000000, 2192}},
    {{36, static_cast<code_t>(14)}, {108000.000000, 2192}},
};

constexpr EphemerisTimeEstimatorInput kBdsEphemerisTimes[] = {
    {{6, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{7, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{8, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{9, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{10, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{11, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{12, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{13, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{14, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{16, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{19, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{20, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{21, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{22, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{23, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{24, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{25, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{26, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{27, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{28, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{29, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{30, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{32, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{33, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{34, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{35, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{36, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{37, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{38, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{39, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{40, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{41, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{42, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{43, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{44, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{45, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{46, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{59, static_cast<code_t>(12)}, {111614.000000, 2192}},
    {{60, static_cast<code_t>(12)}, {111614.000000, 2192}},
};
}  // namespace

TEST(TimeTruth, GetQuartileIndices) {
  auto create_quartile_indices =
      [](size_t lower_quartile_first,
         size_t lower_quartile_second,
         size_t middle_quartile_first,
         size_t middle_quartile_second,
         size_t upper_quartile_first,
         size_t upper_quartile_second) -> gnss_converters::QuartileIndices {
    gnss_converters::QuartileIndices quartile_indices;
    quartile_indices.lower_quartile.first = lower_quartile_first;
    quartile_indices.lower_quartile.second = lower_quartile_second;
    quartile_indices.middle_quartile.first = middle_quartile_first;
    quartile_indices.middle_quartile.second = middle_quartile_second;
    quartile_indices.upper_quartile.first = upper_quartile_first;
    quartile_indices.upper_quartile.second = upper_quartile_second;
    return quartile_indices;
  };

  EXPECT_EQ(gnss_converters::get_quartile_indices(3),
            create_quartile_indices(0, 0, 1, 1, 2, 2));
  EXPECT_EQ(gnss_converters::get_quartile_indices(4),
            create_quartile_indices(0, 1, 1, 2, 2, 3));
  EXPECT_EQ(gnss_converters::get_quartile_indices(5),
            create_quartile_indices(0, 1, 2, 2, 3, 4));
  EXPECT_EQ(gnss_converters::get_quartile_indices(6),
            create_quartile_indices(1, 1, 2, 3, 4, 4));
  EXPECT_EQ(gnss_converters::get_quartile_indices(7),
            create_quartile_indices(1, 1, 3, 3, 5, 5));
  EXPECT_EQ(gnss_converters::get_quartile_indices(8),
            create_quartile_indices(1, 2, 3, 4, 5, 6));
  EXPECT_EQ(gnss_converters::get_quartile_indices(9),
            create_quartile_indices(1, 2, 4, 4, 6, 7));
  EXPECT_EQ(gnss_converters::get_quartile_indices(10),
            create_quartile_indices(2, 2, 4, 5, 7, 7));
  EXPECT_EQ(gnss_converters::get_quartile_indices(11),
            create_quartile_indices(2, 2, 5, 5, 8, 8));
  EXPECT_EQ(gnss_converters::get_quartile_indices(12),
            create_quartile_indices(2, 3, 5, 6, 8, 9));
  EXPECT_EQ(gnss_converters::get_quartile_indices(13),
            create_quartile_indices(2, 3, 6, 6, 9, 10));
  EXPECT_EQ(gnss_converters::get_quartile_indices(14),
            create_quartile_indices(3, 3, 6, 7, 10, 10));
  EXPECT_EQ(gnss_converters::get_quartile_indices(15),
            create_quartile_indices(3, 3, 7, 7, 11, 11));
}

TEST(TimeTruth, IsTowMsValid) {
  EXPECT_TRUE(gnss_converters::is_tow_ms_valid(0));
  EXPECT_TRUE(gnss_converters::is_tow_ms_valid(604'800'000 - 1));
  EXPECT_FALSE(gnss_converters::is_tow_ms_valid(604'800'000));
}

TEST(TimeTruth, TowMsWithinToleranceOperands) {
  // TEST VALIDITY BASE TIME 0s with TOLERANCE 5s -> [604795s, 5s]
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      0 * SECS_MS, 0 * SECS_MS, 5 * SECS_MS));
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      0 * SECS_MS, (604795) * SECS_MS, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      0 * SECS_MS, (604795) * SECS_MS - 1, 5 * SECS_MS));
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      0 * SECS_MS, 5 * SECS_MS, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      0 * SECS_MS, 5 * SECS_MS + 1, 5 * SECS_MS));

  // TEST VALIDITY BASE TIME 5s-1ms with TOLERANCE 5s -> [604800s-1ms, 10s-1ms]
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS - 1, WEEK_SECS * SECS_MS - 1, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS - 1, WEEK_SECS * SECS_MS - 2, 5 * SECS_MS));
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS - 1, 10 * SECS_MS - 1, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS - 1, 10 * SECS_MS, 5 * SECS_MS));

  // TEST VALIDITY BASE TIME 5s with TOLERANCE 5s -> [0ms, 10s]
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS, 0 * SECS_MS, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS, WEEK_SECS * SECS_MS - 1, 5 * SECS_MS));
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS, 10 * SECS_MS, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS, 10 * SECS_MS + 1, 5 * SECS_MS));

  // TEST VALIDITY BASE TIME 5s+1ms with TOLERANCE 5s -> [1ms, 10s + 1ms]
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS + 1, 0 * SECS_MS + 1, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS + 1, 0 * SECS_MS, 5 * SECS_MS));
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS + 1, 10 * SECS_MS + 1, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      5 * SECS_MS + 1, 10 * SECS_MS + 2, 5 * SECS_MS));

  // TEST VALIDITY BASE TIME 604800s-5s-1ms with TOLERANCE 5s ->
  // [604800s-10s-1ms, 604800s-1ms]
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS - 1,
      (WEEK_SECS - 10) * SECS_MS - 1,
      5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS - 1,
      (WEEK_SECS - 10) * SECS_MS - 2,
      5 * SECS_MS));
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS - 1, WEEK_SECS * SECS_MS - 1, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS - 1, 0 * SECS_MS, 5 * SECS_MS));

  // TEST VALIDITY BASE TIME 604800s-5s with TOLERANCE 5s -> [604800s-10s, 0s]
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS, (WEEK_SECS - 10) * SECS_MS, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS, (WEEK_SECS - 10) * SECS_MS - 1, 5 * SECS_MS));
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS, 0 * SECS_MS, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS, 0 * SECS_MS + 1, 5 * SECS_MS));

  // TEST VALIDITY BASE TIME 604800s-5s+1ms with TOLERANCE 5s ->
  // [604800s-10s+1ms, 1s]
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS + 1,
      (WEEK_SECS - 10) * SECS_MS + 1,
      5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS + 1, (WEEK_SECS - 10) * SECS_MS, 5 * SECS_MS));
  EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS + 1, 0 * SECS_MS + 1, 5 * SECS_MS));
  EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
      (WEEK_SECS - 5) * SECS_MS + 1, 0 * SECS_MS + 2, 5 * SECS_MS));
}

TEST(TimeTruth, TowMsWithinToleranceList) {
  static constexpr uint32_t kTolerance = 5 * SECS_MS;
  uint32_t latest_tow_ms;

  // TEST INVALID INPUT
  {
    static constexpr uint32_t tow_ms[] = {};

    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        nullptr, 10, kTolerance, &latest_tow_ms));
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        tow_ms, 0, kTolerance, &latest_tow_ms));
    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        tow_ms, 1, kTolerance, &latest_tow_ms));
  }

  // TEST IDENTICAL VALUES
  {
    static constexpr uint32_t kStart[] = {0, 0};
    static constexpr uint32_t kEnd[] = {WEEK_SECS * SECS_MS - 1,
                                        WEEK_SECS * SECS_MS - 1};
    static constexpr uint32_t kMiddle[] = {WEEK_SECS * SECS_MS / 2,
                                           WEEK_SECS * SECS_MS / 2};

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kStart, ARRAY_SIZE(kStart), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kStart[0]);

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kEnd, ARRAY_SIZE(kEnd), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kEnd[0]);

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kMiddle, ARRAY_SIZE(kMiddle), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kMiddle[0]);
  }

  // TEST VALIDITY BASE TIME 0s with TOLERANCE 5s -> [604795s, 5s]
  {
    static constexpr uint32_t kBase = 0 * SECS_MS;

    static constexpr uint32_t kLeftWithin[] = {kBase, (604795) * SECS_MS};
    static constexpr uint32_t kLeftOutside[] = {kBase, (604795) * SECS_MS - 1};
    static constexpr uint32_t kRightWithin[] = {kBase, 5 * SECS_MS};
    static constexpr uint32_t kRightOutside[] = {kBase, 5 * SECS_MS + 1};

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftWithin, ARRAY_SIZE(kLeftWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kLeftWithin[0]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftOutside, ARRAY_SIZE(kLeftOutside), kTolerance, &latest_tow_ms));

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kRightWithin, ARRAY_SIZE(kRightWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kRightWithin[1]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kRightOutside, ARRAY_SIZE(kRightOutside), kTolerance, &latest_tow_ms));
  }

  // TEST VALIDITY BASE TIME 5s-1ms with TOLERANCE 5s -> [604800s-1ms, 10s-1ms]
  {
    static constexpr uint32_t kBase = 5 * SECS_MS - 1;

    static constexpr uint32_t kLeftWithin[] = {kBase, WEEK_SECS * SECS_MS - 1};
    static constexpr uint32_t kLeftOutside[] = {kBase, WEEK_SECS * SECS_MS - 2};
    static constexpr uint32_t kRightWithin[] = {kBase, 10 * SECS_MS - 1};
    static constexpr uint32_t kRightOutside[] = {kBase, 10 * SECS_MS};

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftWithin, ARRAY_SIZE(kLeftWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kLeftWithin[0]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftOutside, ARRAY_SIZE(kLeftOutside), kTolerance, &latest_tow_ms));

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kRightWithin, ARRAY_SIZE(kRightWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kRightWithin[1]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kRightOutside, ARRAY_SIZE(kRightOutside), kTolerance, &latest_tow_ms));
  }

  // TEST VALIDITY BASE TIME 5s with TOLERANCE 5s -> [0ms, 10s]
  {
    static constexpr uint32_t kBase = 5 * SECS_MS;

    static constexpr uint32_t kLeftWithin[] = {kBase, 0 * SECS_MS};
    static constexpr uint32_t kLeftOutside[] = {kBase, WEEK_SECS * SECS_MS - 1};
    static constexpr uint32_t kRightWithin[] = {kBase, 10 * SECS_MS};
    static constexpr uint32_t kRightOutside[] = {kBase, 10 * SECS_MS + 1};

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftWithin, ARRAY_SIZE(kLeftWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kLeftWithin[0]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftOutside, ARRAY_SIZE(kLeftOutside), kTolerance, &latest_tow_ms));

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kRightWithin, ARRAY_SIZE(kRightWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kRightWithin[1]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kRightOutside, ARRAY_SIZE(kRightOutside), kTolerance, &latest_tow_ms));
  }

  // TEST VALIDITY BASE TIME 5s+1ms with TOLERANCE 5s -> [1ms, 10s + 1ms]
  {
    static constexpr uint32_t kBase = 5 * SECS_MS + 1;

    static constexpr uint32_t kLeftWithin[] = {kBase, 0 * SECS_MS + 1};
    static constexpr uint32_t kLeftOutside[] = {kBase, 0 * SECS_MS};
    static constexpr uint32_t kRightWithin[] = {kBase, 10 * SECS_MS + 1};
    static constexpr uint32_t kRightOutside[] = {kBase, 10 * SECS_MS + 2};

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftWithin, ARRAY_SIZE(kLeftWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kLeftWithin[0]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftOutside, ARRAY_SIZE(kLeftOutside), kTolerance, &latest_tow_ms));

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kRightWithin, ARRAY_SIZE(kRightWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kRightWithin[1]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kRightOutside, ARRAY_SIZE(kRightOutside), kTolerance, &latest_tow_ms));
  }

  // TEST VALIDITY BASE TIME 604800s-5s-1ms with TOLERANCE 5s ->
  // [604800s-10s-1ms, 604800s-1ms]
  {
    static constexpr uint32_t kBase = (WEEK_SECS - 5) * SECS_MS - 1;

    static constexpr uint32_t kLeftWithin[] = {kBase,
                                               (WEEK_SECS - 10) * SECS_MS - 1};
    static constexpr uint32_t kLeftOutside[] = {kBase,
                                                (WEEK_SECS - 10) * SECS_MS - 2};
    static constexpr uint32_t kRightWithin[] = {kBase, WEEK_SECS * SECS_MS - 1};
    static constexpr uint32_t kRightOutside[] = {kBase, 0 * SECS_MS};

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftWithin, ARRAY_SIZE(kLeftWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kLeftWithin[0]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftOutside, ARRAY_SIZE(kLeftOutside), kTolerance, &latest_tow_ms));

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kRightWithin, ARRAY_SIZE(kRightWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kRightWithin[1]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kRightOutside, ARRAY_SIZE(kRightOutside), kTolerance, &latest_tow_ms));
  }

  // TEST VALIDITY BASE TIME 604800s-5s with TOLERANCE 5s -> [604800s-10s, 0s]
  {
    static constexpr uint32_t kBase = (WEEK_SECS - 5) * SECS_MS;

    static constexpr uint32_t kLeftWithin[] = {kBase,
                                               (WEEK_SECS - 10) * SECS_MS};
    static constexpr uint32_t kLeftOutside[] = {kBase,
                                                (WEEK_SECS - 10) * SECS_MS - 1};
    static constexpr uint32_t kRightWithin[] = {kBase, 0 * SECS_MS};
    static constexpr uint32_t kRightOutside[] = {kBase, 0 * SECS_MS + 1};

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftWithin, ARRAY_SIZE(kLeftWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kLeftWithin[0]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftOutside, ARRAY_SIZE(kLeftOutside), kTolerance, &latest_tow_ms));

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kRightWithin, ARRAY_SIZE(kRightWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kRightWithin[1]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kRightOutside, ARRAY_SIZE(kRightOutside), kTolerance, &latest_tow_ms));
  }

  // TEST VALIDITY BASE TIME 604800s-5s+1ms with TOLERANCE 5s ->
  // [604800s-10s+1ms, 1s]
  {
    static constexpr uint32_t kBase = (WEEK_SECS - 5) * SECS_MS + 1;

    static constexpr uint32_t kLeftWithin[] = {kBase,
                                               (WEEK_SECS - 10) * SECS_MS + 1};
    static constexpr uint32_t kLeftOutside[] = {kBase,
                                                (WEEK_SECS - 10) * SECS_MS};
    static constexpr uint32_t kRightWithin[] = {kBase, 0 * SECS_MS + 1};
    static constexpr uint32_t kRightOutside[] = {kBase, 0 * SECS_MS + 2};

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftWithin, ARRAY_SIZE(kLeftWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kLeftWithin[0]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kLeftOutside, ARRAY_SIZE(kLeftOutside), kTolerance, &latest_tow_ms));

    EXPECT_TRUE(gnss_converters::is_tow_ms_within_tolerance(
        kRightWithin, ARRAY_SIZE(kRightWithin), kTolerance, &latest_tow_ms));
    EXPECT_EQ(latest_tow_ms, kRightWithin[1]);
    EXPECT_FALSE(gnss_converters::is_tow_ms_within_tolerance(
        kRightOutside, ARRAY_SIZE(kRightOutside), kTolerance, &latest_tow_ms));
  }
}

TEST(TimeTruth, MatchGpsTimeWithTowMs) {
  static constexpr uint32_t kTolerance = 5 * SECS_MS;
  static constexpr int16_t GPS_WN = 2192;

  // GPS + TOW BEFORE ROLLOVER WITHIN TOLERANCE
  {
    gps_time_t gps_time = {WEEK_SECS - 3, GPS_WN};
    uint32_t tow_ms = WEEK_SECS * SECS_MS - 1;

    EXPECT_TRUE(gnss_converters::match_gps_time_with_tow_ms(
        &gps_time, tow_ms, kTolerance));

    gps_time_t expected = {static_cast<double>(tow_ms) / SECS_MS, GPS_WN};
    EXPECT_LE(abs(gpsdifftime(&gps_time, &expected)), kDoubleTolerance);
  }

  // GPS BEFORE ROLLOVER, TOW AFTER ROLL OVER, WITHIN TOLERANCE
  {
    gps_time_t gps_time = {WEEK_SECS - 3, GPS_WN};
    uint32_t tow_ms = 0;

    EXPECT_TRUE(gnss_converters::match_gps_time_with_tow_ms(
        &gps_time, tow_ms, kTolerance));

    gps_time_t expected = {static_cast<double>(tow_ms) / SECS_MS, GPS_WN + 1};
    EXPECT_LE(abs(gpsdifftime(&gps_time, &expected)), kDoubleTolerance);
  }

  // GPS AFTER ROLLOVER, TOW AFTER ROLL OVER, WITHIN TOLERANCE
  {
    gps_time_t gps_time = {3, GPS_WN};
    uint32_t tow_ms = WEEK_SECS * SECS_MS - 1;

    EXPECT_TRUE(gnss_converters::match_gps_time_with_tow_ms(
        &gps_time, tow_ms, kTolerance));

    gps_time_t expected = {static_cast<double>(tow_ms) / SECS_MS, GPS_WN - 1};
    EXPECT_LE(abs(gpsdifftime(&gps_time, &expected)), kDoubleTolerance);
  }

  // GPS + TOW AFTER ROLLOVER WITHIN TOLERANCE
  {
    gps_time_t gps_time = {3, GPS_WN};
    uint32_t tow_ms = 1;

    EXPECT_TRUE(gnss_converters::match_gps_time_with_tow_ms(
        &gps_time, tow_ms, kTolerance));

    gps_time_t expected = {static_cast<double>(tow_ms) / SECS_MS, GPS_WN};
    EXPECT_LE(abs(gpsdifftime(&gps_time, &expected)), kDoubleTolerance);
  }

  // GPS + TOW BEFORE ROLLOVER WITHIN TOLERANCE
  {
    gps_time_t gps_time = {WEEK_SECS - kTolerance - 3, GPS_WN};
    uint32_t tow_ms = WEEK_SECS * SECS_MS - 1;

    EXPECT_FALSE(gnss_converters::match_gps_time_with_tow_ms(
        &gps_time, tow_ms, kTolerance));
  }

  // GPS BEFORE ROLLOVER, TOW AFTER ROLL OVER, WITHIN TOLERANCE
  {
    gps_time_t gps_time = {WEEK_SECS - kTolerance - 3, GPS_WN};
    uint32_t tow_ms = 0;

    EXPECT_FALSE(gnss_converters::match_gps_time_with_tow_ms(
        &gps_time, tow_ms, kTolerance));
  }

  // GPS AFTER ROLLOVER, TOW AFTER ROLL OVER, WITHIN TOLERANCE
  {
    gps_time_t gps_time = {3 + kTolerance, GPS_WN};
    uint32_t tow_ms = WEEK_SECS * SECS_MS - 1;

    EXPECT_FALSE(gnss_converters::match_gps_time_with_tow_ms(
        &gps_time, tow_ms, kTolerance));
  }

  // GPS + TOW AFTER ROLLOVER WITHIN TOLERANCE
  {
    gps_time_t gps_time = {3 + kTolerance, GPS_WN};
    uint32_t tow_ms = 1;

    EXPECT_FALSE(gnss_converters::match_gps_time_with_tow_ms(
        &gps_time, tow_ms, kTolerance));
  }

  // AVOID BUFFER OVERFLOW
  {
    gps_time_t gps_time = {WEEK_SECS - 3, INT16_MAX};
    uint32_t tow_ms = 1;

    EXPECT_TRUE(gnss_converters::match_gps_time_with_tow_ms(
        &gps_time, tow_ms, kTolerance));

    gps_time_t expected = {static_cast<double>(tow_ms) / SECS_MS, INT16_MAX};
    EXPECT_LE(abs(gpsdifftime(&gps_time, &expected)), kDoubleTolerance);
  }

  // AVOID BUFFER UNDERFLOW
  {
    gps_time_t gps_time = {3, 0};
    uint32_t tow_ms = WEEK_SECS * SECS_MS - 1;

    EXPECT_TRUE(gnss_converters::match_gps_time_with_tow_ms(
        &gps_time, tow_ms, kTolerance));

    gps_time_t expected = {static_cast<double>(tow_ms) / SECS_MS, 0};
    EXPECT_LE(abs(gpsdifftime(&gps_time, &expected)), kDoubleTolerance);
  }
}

TEST(TimeTruth, TimeEstimatorSingleThreadCheck) {
  struct State {
    int x;
    int y;
    int z;
  };

  struct TimeEstimator : public gnss_converters::TimeEstimator<State> {
   private:
    using Base = gnss_converters::TimeEstimator<State>;

   public:
    void reset() override {}

   public:
    bool save_state(const State &state) { return Base::save_state(state); }
  };

  TimeEstimator estimator;
  static_assert(std::is_same<decltype(estimator)::State, State>::value, "");

  State state;
  state.x = -1;
  state.y = -1;
  state.z = -1;

  EXPECT_TRUE(estimator.save_state(state));

  State state_copy;

  EXPECT_TRUE(estimator.load_state(&state_copy));
  EXPECT_EQ(state_copy.x, state.x);
  EXPECT_EQ(state_copy.y, state.y);
  EXPECT_EQ(state_copy.z, state.z);
}

TEST(TimeTruth, TimeEstimatorCheckLockFree) {
  static struct {
    bool is_in_operator;
    bool freeze_in_operator;
  } assignment_operator_data;

  static std::mutex assignment_operator_mutex;
  static std::condition_variable assignment_operator_condition_variable;

  struct State {
    int x;
    int y;
    int z;

    State &operator=(const State &other) {
      if (&other == this) {
        return *this;
      }

      std::unique_lock<std::mutex> lock(assignment_operator_mutex);
      assignment_operator_data.is_in_operator = true;
      assignment_operator_condition_variable.notify_one();

      assignment_operator_condition_variable.wait(
          lock, []() { return !assignment_operator_data.freeze_in_operator; });

      x = other.x;
      y = other.y;
      z = other.z;

      return *this;
    }
  };

  struct TimeEstimator : public gnss_converters::TimeEstimator<State> {
   private:
    using Base = gnss_converters::TimeEstimator<State>;

   public:
    void reset() override {}

   public:
    bool save_state(const State &state) { return Base::save_state(state); }
  };

  TimeEstimator estimator;
  static_assert(std::is_same<decltype(estimator)::State, State>::value, "");

  std::thread thread;

  /**
   * This portion of the logic is checking to make sure that if an external
   * thread is attempting to load state from the estimator while the current
   * thread is attempting to save the state, than the save state would return
   * false and abandon attempt at saving.
   *
   * The sequence of events are:
   *
   *  1. setup the current state with random values
   *  2. current thread locks the mutex and initializes the shared state
   *  3. a thread is spawned where it tries to immediately load the state of the
   *     estimator, it is blocked within State::operator= because the current
   *     thread has the lock
   *  4. the current thread releases the lock as it awaits for the condition
   *     that the other thread has invoked the load_state (can identify this
   *     through the call to State::operator=).
   *  5. the other thread unblocks itself as it has now the lock, it than
   *     proceeds to flagging that its within the State::operator= function
   *     before going back to sleep awaiting for the "freeze_in_operator" state.
   *  6. at this point the current thread wakes up and proceeds to try and save
   *     the state, only to realize that the other thread has the lock on the
   *     state object, so the save_state fails
   *  7. the current thread signals the other thread to unfreeze and finish
   *     loading the state
   */
  {
    State start_state;
    start_state.x = 1;
    start_state.y = 2;
    start_state.z = 3;
    estimator.save_state(start_state);

    std::unique_lock<std::mutex> lock(assignment_operator_mutex);
    assignment_operator_data.is_in_operator = false;
    assignment_operator_data.freeze_in_operator = true;

    thread = std::thread([&estimator]() {
      State thread_state{};

      EXPECT_TRUE(estimator.load_state(&thread_state));
      EXPECT_EQ(thread_state.x, 1);
      EXPECT_EQ(thread_state.y, 2);
      EXPECT_EQ(thread_state.z, 3);
    });

    assignment_operator_condition_variable.wait(
        lock, []() { return assignment_operator_data.is_in_operator; });

    State this_state;
    this_state.x = -1;
    this_state.y = -2;
    this_state.z = -3;
    EXPECT_FALSE(estimator.save_state(this_state));

    assignment_operator_data.freeze_in_operator = false;
    assignment_operator_condition_variable.notify_one();
  }
  thread.join();

  {
    State validate_state;
    EXPECT_TRUE(estimator.load_state(&validate_state));
    EXPECT_EQ(validate_state.x, 1);
    EXPECT_EQ(validate_state.y, 2);
    EXPECT_EQ(validate_state.z, 3);
  }

  {
    State start_state{};
    estimator.save_state(start_state);

    std::unique_lock<std::mutex> lock(assignment_operator_mutex);
    assignment_operator_data.is_in_operator = false;
    assignment_operator_data.freeze_in_operator = true;

    thread = std::thread([&estimator]() {
      State thread_state;
      thread_state.x = 4;
      thread_state.y = 5;
      thread_state.z = 6;
      EXPECT_TRUE(estimator.save_state(thread_state));
    });

    assignment_operator_condition_variable.wait(
        lock, []() { return assignment_operator_data.is_in_operator; });

    State this_state;
    this_state.x = -4;
    this_state.y = -5;
    this_state.z = -6;

    EXPECT_FALSE(estimator.load_state(&this_state));
    EXPECT_EQ(this_state.x, -4);
    EXPECT_EQ(this_state.y, -5);
    EXPECT_EQ(this_state.z, -6);

    assignment_operator_data.freeze_in_operator = false;
    assignment_operator_condition_variable.notify_one();
  }
  thread.join();

  {
    State validate_state;
    EXPECT_TRUE(estimator.load_state(&validate_state));
    EXPECT_EQ(validate_state.x, 4);
    EXPECT_EQ(validate_state.y, 5);
    EXPECT_EQ(validate_state.z, 6);
  }
}

TEST(TimeTruth, ObservationTimeEstimator) {
  TimeTruth time_truth;
  gnss_converters::ObservationTimeEstimator *estimator;
  ObservationTimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  uint32_t tow_ms;

  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(state, &tow_ms));

  // push invalid value
  estimator->push(WEEK_SECS * SECS_MS);
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(state, &tow_ms));

  // push valid value just before week roll over
  estimator->push(WEEK_SECS * SECS_MS - 1);
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(estimator->get_estimate(state, &tow_ms));
  EXPECT_EQ(tow_ms, WEEK_SECS * SECS_MS - 1);

  // roll over week
  estimator->push(0 * SECS_MS);
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(estimator->get_estimate(state, &tow_ms));
  EXPECT_EQ(tow_ms, 0 * SECS_MS);

  // roll week backwards
  estimator->push(WEEK_SECS * SECS_MS - 1);
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(estimator->get_estimate(state, &tow_ms));
  EXPECT_EQ(tow_ms, WEEK_SECS * SECS_MS - 1);

  // roll over week + try pushing new TOW outside the tolerance region
  for (size_t i = 0;
       i < gnss_converters::kObservationEstimatorMaximumTowMsTimeMismatchCount;
       ++i) {
    estimator->push(
        gnss_converters::kObservationEstimatorMaximumTowMsTimeDifference);
    EXPECT_TRUE(estimator->load_state(&state));
    EXPECT_TRUE(estimator->get_estimate(state, &tow_ms));
    EXPECT_EQ(tow_ms, WEEK_SECS * SECS_MS - 1);
  }

  estimator->push(
      gnss_converters::kObservationEstimatorMaximumTowMsTimeDifference);
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(estimator->get_estimate(state, &tow_ms));
  EXPECT_EQ(tow_ms,
            gnss_converters::kObservationEstimatorMaximumTowMsTimeDifference);

  // reset estimator
  estimator->reset();
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(state, &tow_ms));
}

TEST(TimeTruth, EphemerisTimeEstimatorEmpty) {
  gps_time_t gps_time;
  TimeTruth time_truth;
  gnss_converters::EphemerisTimeEstimator *estimator;
  EphemerisTimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(state, &gps_time, kEphemerisTimeTowMs));
}

TEST(TimeTruth, EphemerisTimeEstimatorBasicUsage) {
  gps_time_t gps_time;
  TimeTruth time_truth;
  gnss_converters::EphemerisTimeEstimator *estimator;
  EphemerisTimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  for (const auto &ephemeris_time : kGpsEphemerisTimes) {
    estimator->push(ephemeris_time.gnss_signal, ephemeris_time.gps_time);
  }
  for (const auto &ephemeris_time : kGalEphemerisTimes) {
    estimator->push(ephemeris_time.gnss_signal, ephemeris_time.gps_time);
  }
  for (const auto &ephemeris_time : kBdsEphemerisTimes) {
    estimator->push(ephemeris_time.gnss_signal, ephemeris_time.gps_time);
  }

  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(estimator->get_estimate(state, &gps_time, kEphemerisTimeTowMs));
  EXPECT_LT(abs(gpsdifftime(&gps_time, &kEphemerisTimeGps)), kDoubleTolerance);

  estimator->reset();

  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(state, &gps_time, kEphemerisTimeTowMs));
}

TEST(TimeTruth, EphemerisTimeEstimatorMinimumSizeCheck) {
  static_assert(ARRAY_SIZE(kGpsEphemerisTimes) >=
                gnss_converters::kEphemerisEstimatorMinimumSignalsToQualify);
  static_assert(ARRAY_SIZE(kGalEphemerisTimes) >=
                gnss_converters::kEphemerisEstimatorMinimumSignalsToQualify);
  static_assert(ARRAY_SIZE(kBdsEphemerisTimes) >=
                gnss_converters::kEphemerisEstimatorMinimumSignalsToQualify);

  static constexpr size_t kSignalCount =
      gnss_converters::kEphemerisEstimatorMinimumSignalsToQualify - 1;

  gps_time_t gps_time;
  TimeTruth time_truth;
  gnss_converters::EphemerisTimeEstimator *estimator;
  EphemerisTimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  for (size_t i = 0; i < kSignalCount; ++i) {
    estimator->push(kGpsEphemerisTimes[i].gnss_signal,
                    kGpsEphemerisTimes[i].gps_time);
  }
  for (size_t i = 0; i < kSignalCount; ++i) {
    estimator->push(kGalEphemerisTimes[i].gnss_signal,
                    kGalEphemerisTimes[i].gps_time);
  }
  for (size_t i = 0; i < kSignalCount; ++i) {
    estimator->push(kBdsEphemerisTimes[i].gnss_signal,
                    kBdsEphemerisTimes[i].gps_time);
  }

  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(state, &gps_time, kEphemerisTimeTowMs));

  estimator->push(kGpsEphemerisTimes[0].gnss_signal,
                  kGpsEphemerisTimes[0].gps_time);

  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(state, &gps_time, kEphemerisTimeTowMs));

  estimator->push(kGpsEphemerisTimes[kSignalCount].gnss_signal,
                  kGpsEphemerisTimes[kSignalCount].gps_time);

  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(estimator->get_estimate(state, &gps_time, kEphemerisTimeTowMs));
  EXPECT_LT(abs(gpsdifftime(&gps_time, &kEphemerisTimeGps)), kDoubleTolerance);
}

TEST(TimeTruth, EphemerisTimeEstimatorTooManyOutliers) {
  gps_time_t gps_time;
  TimeTruth time_truth;
  gnss_converters::EphemerisTimeEstimator *estimator;
  EphemerisTimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  for (size_t i = 0; i < ARRAY_SIZE(kBdsEphemerisTimes); ++i) {
    if (i % 2 == 0) {
      estimator->push(kBdsEphemerisTimes[i].gnss_signal,
                      kBdsEphemerisTimes[i].gps_time);
    } else {
      gps_time = kBdsEphemerisTimes[i].gps_time;
      add_secs(&gps_time, DAY_SECS);
      estimator->push(kBdsEphemerisTimes[i].gnss_signal, gps_time);
    }
  }

  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(estimator->get_estimate(state, &gps_time, kEphemerisTimeTowMs));
  EXPECT_EQ(gps_time.wn, kEphemerisTimeWn);
  EXPECT_DOUBLE_EQ(gps_time.tow, kEphemerisTimeTowMs / SECS_MS);
}

TEST(TimeTruth, EphemerisTimeEstimatorCachedData) {
  gps_time_t gps_time;
  TimeTruth time_truth;
  gnss_converters::EphemerisTimeEstimator *estimator;
  EphemerisTimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  for (const auto &ephemeris_time : kGalEphemerisTimes) {
    gps_time = ephemeris_time.gps_time;
    add_secs(&gps_time, -WEEK_SECS / 2);
    estimator->push(ephemeris_time.gnss_signal, gps_time);
  }

  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(state, &gps_time, kEphemerisTimeTowMs));
}

TEST(TimeTruth, EphemerisTimeEstimatorWeekRollover) {
  static constexpr EphemerisTimeEstimatorInput ephemeris_input[] = {
      {{1, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{2, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{3, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{4, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{5, static_cast<code_t>(0)}, {1000, 2193}},
      {{6, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{7, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{8, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{9, static_cast<code_t>(0)}, {1000, 2193}},
      {{10, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{12, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{13, static_cast<code_t>(0)}, {1000, 2193}},
      {{14, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{15, static_cast<code_t>(0)}, {1000, 2193}},
      {{16, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{17, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{18, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{19, static_cast<code_t>(0)}, {1000, 2193}},
      {{20, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{21, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{22, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2186}},
      {{23, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{24, static_cast<code_t>(0)}, {1000, 2193}},
      {{25, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{26, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{27, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{29, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{30, static_cast<code_t>(0)}, {1000, 2193}},
      {{31, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
      {{32, static_cast<code_t>(0)}, {WEEK_SECS - 10, 2192}},
  };

  gps_time_t gps_time;
  TimeTruth time_truth;
  gnss_converters::EphemerisTimeEstimator *estimator;
  EphemerisTimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  for (const auto &ephemeris_time : ephemeris_input) {
    estimator->push(ephemeris_time.gnss_signal, ephemeris_time.gps_time);
  }

  const gps_time_t before_roll_over{WEEK_SECS - 1, 2192};
  const gps_time_t after_roll_over{0, 2193};

  EXPECT_TRUE(estimator->load_state(&state));

  EXPECT_TRUE(estimator->get_estimate(
      state, &gps_time, before_roll_over.tow * SECS_MS));
  EXPECT_LT(abs(gpsdifftime(&gps_time, &before_roll_over)), kDoubleTolerance);

  EXPECT_TRUE(
      estimator->get_estimate(state, &gps_time, after_roll_over.tow * SECS_MS));
  EXPECT_LT(abs(gpsdifftime(&gps_time, &after_roll_over)), kDoubleTolerance);
}

TEST(TimeTruth, EphemerisTimeEstimatorPushTooManySignals) {
  static constexpr uint16_t kMaxSignals =
      gnss_converters::EphemerisTimeEstimator::State::kSatellites;
  static constexpr code_t kGpsCode = static_cast<code_t>(0);
  static constexpr code_t kGalCode = static_cast<code_t>(12);
  static constexpr code_t kBdsCode = static_cast<code_t>(14);

  static constexpr gps_time_t kGpsTime = {10000.0, 2000};

  gps_time_t gps_time;
  TimeTruth time_truth;
  gnss_converters::EphemerisTimeEstimator *estimator;
  EphemerisTimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  for (uint16_t i = 0; i < 2 * kMaxSignals; ++i) {
    estimator->push(gnss_signal_t{i, kGpsCode}, kGpsTime);
    estimator->push(gnss_signal_t{i, kGalCode}, kGpsTime);
    estimator->push(gnss_signal_t{i, kBdsCode}, kGpsTime);
  }

  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(
      estimator->get_estimate(state, &gps_time, kGpsTime.tow * SECS_MS));
  EXPECT_LT(abs(gpsdifftime(&gps_time, &kGpsTime)), kDoubleTolerance);
}

TEST(TimeTruth, EphemerisTimeEstimatorPushInvalidSignals) {
  constexpr uint16_t kMaxSignals =
      gnss_converters::EphemerisTimeEstimator::State::kSatellites;

  gps_time_t gps_time;
  TimeTruth time_truth;
  gnss_converters::EphemerisTimeEstimator *estimator;
  EphemerisTimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  estimator->push(gnss_signal_t{0, code_t::CODE_INVALID}, {0, 0});
}

TEST(TimeTruth, EphemerisTimeEstimatorInvalidTime) {
  static constexpr gps_time_t kGpsTime = {WN_UNKNOWN, TOW_UNKNOWN};

  gps_time_t gps_time;
  TimeTruth time_truth;
  gnss_converters::EphemerisTimeEstimator *estimator;
  EphemerisTimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  for (const auto &ephemeris_time : kGpsEphemerisTimes) {
    estimator->push(ephemeris_time.gnss_signal, kGpsTime);
  }

  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(state, &gps_time, kEphemerisTimeTowMs));
}

TEST(TimeTruth, Rtcm1013TimeEstimator) {
  TimeTruth time_truth;
  gnss_converters::Rtcm1013TimeEstimator *estimator;
  Rtcm1013TimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  gps_time_t gps_time_t;
  int8_t leap_seconds;

  // load empty dataset
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(state, &gps_time_t, &leap_seconds, 0));

  // push invalid gps time
  estimator->push({WEEK_SECS, INT16_MAX}, 18);
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(
      state, &gps_time_t, &leap_seconds, WEEK_SECS * SECS_MS - 1));

  // push normal values just within range
  estimator->push({12345, 2192}, 18);
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(estimator->get_estimate(
      state,
      &gps_time_t,
      &leap_seconds,
      12345 * SECS_MS +
          gnss_converters::kRtcm1013EstimatorGpsTimeTowMsTolerance));

  // push normal values just within range
  estimator->push({12345, 2192}, 18);
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(
      state,
      &gps_time_t,
      &leap_seconds,
      12345 * SECS_MS +
          gnss_converters::kRtcm1013EstimatorGpsTimeTowMsTolerance + 1));

  // load empty dataset
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(estimator->get_estimate(
      state, &gps_time_t, &leap_seconds, 12345 * SECS_MS));
  estimator->reset();
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(
      state, &gps_time_t, &leap_seconds, 12345 * SECS_MS));
}

TEST(TimeTruth, UbxLeapTimeEstimator) {
  TimeTruth time_truth;
  gnss_converters::UbxLeapTimeEstimator *estimator;
  UbxLeapTimeEstimator::State state;

  ASSERT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &estimator));

  gps_time_t gps_time_t;
  int8_t leap_seconds;

  // load empty dataset
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(state, &gps_time_t, &leap_seconds, 0));

  // push invalid gps time
  estimator->push({WEEK_SECS, INT16_MAX}, 18);
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(
      state, &gps_time_t, &leap_seconds, WEEK_SECS * SECS_MS - 1));

  // push normal values just within range
  estimator->push({12345, 2192}, 18);
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(estimator->get_estimate(
      state,
      &gps_time_t,
      &leap_seconds,
      12345 * SECS_MS +
          gnss_converters::kUbxLeapEstimatorGpsTimeTowMsTolerance));

  // push normal values just within range
  estimator->push({12345, 2192}, 18);
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(
      state,
      &gps_time_t,
      &leap_seconds,
      12345 * SECS_MS +
          gnss_converters::kUbxLeapEstimatorGpsTimeTowMsTolerance + 1));

  // load empty dataset
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_TRUE(estimator->get_estimate(
      state, &gps_time_t, &leap_seconds, 12345 * SECS_MS));
  estimator->reset();
  EXPECT_TRUE(estimator->load_state(&state));
  EXPECT_FALSE(estimator->get_estimate(
      state, &gps_time_t, &leap_seconds, 12345 * SECS_MS));
}

TEST(TimeTruth, CalculateTowMs) {
  // if the value is greater than 1 than we will need to add additional test
  // cases for the missed code coverage
  static_assert(gnss_converters::kTotalTowMsEstimationPerSource == 1, "");

  static constexpr size_t kMaximumEstimates =
      gnss_converters::kTotalTowMsEstimationPerSource * TIME_TRUTH_SOURCE_COUNT;
  static constexpr uint32_t kTowMsTolerance =
      gnss_converters::kGetLatestTimeMaximumTowMsRange;

  using EstimatesType =
      gnss_converters::StaticVector<std::pair<uint32_t, TimeTruthSource>,
                                    kMaximumEstimates>;

  uint32_t tow_ms;
  TimeTruthState tow_ms_state;

  // empty estimates
  {
    EstimatesType estimates;
    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_NONE);
  }

  // single local estimates
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(12345), TIME_TRUTH_SOURCE_LOCAL)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms, 12345);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BEST);
  }

  // single remote estimates
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(12345), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms, 12345);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BEST);
  }

  // matching local/remote estimates within tolerance - remote is ahead
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(12345), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(std::make_pair(
        UINT32_C(12345) + kTowMsTolerance, TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms, UINT32_C(12345) + kTowMsTolerance);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BEST);
  }

  // matching local/remote estimates within tolerance - local is ahead
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(std::make_pair(
        UINT32_C(12345) + kTowMsTolerance, TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(12345), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms, UINT32_C(12345) + kTowMsTolerance);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BEST);
  }

  // matching local/remote estimates within tolerance - local is ahead rollover
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(kTowMsTolerance - 1, TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(std::make_pair(
        UINT32_C(WEEK_SECS * SECS_MS - 1), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms, kTowMsTolerance - 1);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BEST);
  }

  // matching local/remote estimates within tolerance - remote is ahead rollover
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(std::make_pair(
        UINT32_C(WEEK_SECS * SECS_MS - 1), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(kTowMsTolerance - 1, TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms, kTowMsTolerance - 1);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BEST);
  }

  // local/remote mismatch with remote "ahead"
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(std::make_pair(
        UINT32_C(WEEK_SECS * SECS_MS - 1), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(kTowMsTolerance, TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms, UINT32_C(WEEK_SECS * SECS_MS - 1));
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BAD);
  }

  // local/remote mismatch with local "ahead"
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(kTowMsTolerance, TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(std::make_pair(
        UINT32_C(WEEK_SECS * SECS_MS - 1), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms, kTowMsTolerance);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BAD);
  }

  // two local source matching
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(kTowMsTolerance - 1, TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(std::make_pair(
        UINT32_C(WEEK_SECS * SECS_MS - 1), TIME_TRUTH_SOURCE_LOCAL)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms, kTowMsTolerance - 1);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BEST);
  }

  // two remote source matching
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(kTowMsTolerance - 1, TIME_TRUTH_SOURCE_REMOTE)));
    EXPECT_TRUE(estimates.push_back(std::make_pair(
        UINT32_C(WEEK_SECS * SECS_MS - 1), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms, kTowMsTolerance - 1);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BEST);
  }

  // two local source - unmatched
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(kTowMsTolerance, TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(std::make_pair(
        UINT32_C(WEEK_SECS * SECS_MS - 1), TIME_TRUTH_SOURCE_LOCAL)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_NONE);
  }

  // two remote source - unmatched
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(kTowMsTolerance, TIME_TRUTH_SOURCE_REMOTE)));
    EXPECT_TRUE(estimates.push_back(std::make_pair(
        UINT32_C(WEEK_SECS * SECS_MS - 1), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_tow_ms(
        estimates, kTowMsTolerance, &tow_ms, &tow_ms_state);

    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_NONE);
  }
}

TEST(TimeTruth, CalculateLeapSeconds) {
  static constexpr size_t kMaximumEstimates =
      gnss_converters::kTotalLeapSecondEstimationPerSource *
      TIME_TRUTH_SOURCE_COUNT;

  using EstimatesType =
      gnss_converters::StaticVector<std::pair<int8_t, TimeTruthSource>,
                                    kMaximumEstimates>;

  int8_t leap_seconds;
  TimeTruthState leap_seconds_state;

  // empty estimates
  {
    EstimatesType estimates;
    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_NONE);
  }

  // single local estimates
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds, INT8_C(18));
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_BEST);
  }

  // single remote estimates
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds, INT8_C(19));
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_BEST);
  }

  // matching local/remote
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds, INT8_C(19));
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_BEST);
  }

  // local/remote mismatch
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds, INT8_C(18));
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_BAD);
  }

  // two local source matching
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds, INT8_C(18));
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_BEST);
  }

  // two remote source matching
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_REMOTE)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds, INT8_C(18));
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_BEST);
  }

  // two local source - unmatched
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_LOCAL)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_NONE);
  }

  // two remote source - unmatched
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_REMOTE)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_NONE);
  }

  // two local source match against remote
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds, INT8_C(18));
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_BEST);
  }

  // two local source match against each other, remote is different
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds, INT8_C(18));
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_GOOD);
  }

  // two remote source match against each other, loca is different
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_REMOTE)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds, INT8_C(19));
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_GOOD);
  }

  // two local sources mismatch and one remote source, all disagreeing
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(20), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds, INT8_C(20));
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_BAD);
  }

  // two local and two remote sources, two pairs cross match but they disagree
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_REMOTE)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_NONE);
  }

  // three local and one remote sources, loca/remote pair matched and other two
  // local pairs matched
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(18), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(INT8_C(19), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_leap_seconds(
        estimates, &leap_seconds, &leap_seconds_state);

    EXPECT_EQ(leap_seconds, INT8_C(19));
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_BEST);
  }
}

TEST(TimeTruth, CalculateWeekNumber) {
  static constexpr size_t kMaximumEstimates =
      gnss_converters::kTotalWeekNumberEstimationPerSource *
      TIME_TRUTH_SOURCE_COUNT;

  using EstimatesType =
      gnss_converters::StaticVector<std::pair<uint16_t, TimeTruthSource>,
                                    kMaximumEstimates>;

  uint16_t wn;
  TimeTruthState wn_state;

  // empty estimates
  {
    EstimatesType estimates;
    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_NONE);
  }

  // single local estimates
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2192));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BEST);
  }

  // single remote estimates
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2193));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BEST);
  }

  // matching local/remote
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2193));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BEST);
  }

  // local/remote mismatch
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2192));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BAD);
  }

  // two local source matching
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2192));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BEST);
  }

  // two remote source matching
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_REMOTE)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2192));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BEST);
  }

  // two local source - unmatched
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_LOCAL)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_NONE);
  }

  // two remote source - unmatched
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_REMOTE)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_NONE);
  }

  // two local source match against remote
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2192));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BEST);
  }

  // two local source match against each other, remote is different
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2192));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_GOOD);
  }

  // two remote source match against each other, loca is different
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_REMOTE)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2193));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_GOOD);
  }

  // two local sources mismatch and one remote source, all disagreeing
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2194), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2194));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BAD);
  }

  // two local and two remote sources, two pairs cross match but they disagree
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_REMOTE)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_NONE);
  }

  // three local and one remote sources, local/remote pair matched and other two
  // local pairs matched
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2193));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BEST);
  }

  // three local and two remote sources, two locals match one remote and
  // remaining local/remote pairs match each other
  {
    EstimatesType estimates;
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2192), TIME_TRUTH_SOURCE_REMOTE)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_LOCAL)));
    EXPECT_TRUE(estimates.push_back(
        std::make_pair(UINT32_C(2193), TIME_TRUTH_SOURCE_REMOTE)));

    gnss_converters::calculate_week_number(estimates, &wn, &wn_state);

    EXPECT_EQ(wn, UINT32_C(2192));
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BEST);
  }
}

TEST(TimeTruth, TimeTruthRequestEstimators) {
  gnss_converters::TimeTruth time_truth;

  for (size_t source_index = 0; source_index < TIME_TRUTH_SOURCE_COUNT;
       ++source_index) {
    auto source = static_cast<TimeTruthSource>(source_index);
    gnss_converters::ObservationTimeEstimator *observation_estimator;
    gnss_converters::EphemerisTimeEstimator *ephemeris_estimator;
    gnss_converters::Rtcm1013TimeEstimator *rtcm_1013_estimator;
    gnss_converters::UbxLeapTimeEstimator *ubx_leap_estimator;

    EXPECT_TRUE(time_truth.request_estimator(source, &observation_estimator));
    EXPECT_NE(observation_estimator, nullptr);
    EXPECT_FALSE(time_truth.request_estimator(source, &observation_estimator));

    EXPECT_TRUE(time_truth.request_estimator(source, &ephemeris_estimator));
    EXPECT_NE(ephemeris_estimator, nullptr);
    EXPECT_FALSE(time_truth.request_estimator(source, &ephemeris_estimator));

    EXPECT_TRUE(time_truth.request_estimator(source, &rtcm_1013_estimator));
    EXPECT_NE(rtcm_1013_estimator, nullptr);
    EXPECT_FALSE(time_truth.request_estimator(source, &rtcm_1013_estimator));

    EXPECT_TRUE(time_truth.request_estimator(source, &ubx_leap_estimator));
    EXPECT_NE(ubx_leap_estimator, nullptr);
    EXPECT_FALSE(time_truth.request_estimator(source, &ubx_leap_estimator));
  }
}

/**
 * Standard Ubx Rover and Ntrip correction service setup
 */
TEST(TimeTruth, GetLatestTimeNormalSetup) {
  gnss_converters::TimeTruth time_truth;
  uint16_t wn;
  TimeTruthState wn_state;
  uint32_t tow_ms;
  TimeTruthState tow_ms_state;
  int8_t leap_seconds;
  TimeTruthState leap_seconds_state;
  gnss_converters::TimeTruthCache cache;

  gnss_converters::ObservationTimeEstimator *local_observation;
  gnss_converters::UbxLeapTimeEstimator *local_ubx_leap;
  gnss_converters::EphemerisTimeEstimator *local_ephemeris;

  EXPECT_TRUE(time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL,
                                           &local_observation));
  EXPECT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &local_ubx_leap));
  EXPECT_TRUE(
      time_truth.request_estimator(TIME_TRUTH_SOURCE_LOCAL, &local_ephemeris));

  gnss_converters::ObservationTimeEstimator *remote_observation;
  gnss_converters::Rtcm1013TimeEstimator *remote_rtcm_1013;
  gnss_converters::EphemerisTimeEstimator *remote_ephemeris;

  EXPECT_TRUE(time_truth.request_estimator(TIME_TRUTH_SOURCE_REMOTE,
                                           &remote_observation));
  EXPECT_TRUE(time_truth.request_estimator(TIME_TRUTH_SOURCE_REMOTE,
                                           &remote_rtcm_1013));
  EXPECT_TRUE(time_truth.request_estimator(TIME_TRUTH_SOURCE_REMOTE,
                                           &remote_ephemeris));

  static constexpr gps_time_t actual_time = {kEphemerisTimeTowMs / SECS_MS,
                                             kEphemerisTimeWn};

  gps_time_t local_ubx_leap_time = actual_time;
  add_secs(&local_ubx_leap_time, -2.5 * WEEK_SECS);

  gps_time_t remote_rtcm_1013_time = actual_time;
  add_secs(&remote_rtcm_1013_time,
           -gnss_converters::kRtcm1013EstimatorGpsTimeTowMsTolerance / SECS_MS);

  // validate empty dataset
  {
    time_truth.get_latest_time(&wn,
                               &wn_state,
                               &tow_ms,
                               &tow_ms_state,
                               &leap_seconds,
                               &leap_seconds_state,
                               &cache);
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_NONE);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_NONE);
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_NONE);
  }

  // push two leap second information, for the local source we will push data
  // that is 2.5 weeks old and for the remote source we push up-to-date
  // information. both sets of data will provide invalid leap second data. no
  // time deduction will be made as there are no observation estimates.
  {
    local_ubx_leap->push(local_ubx_leap_time, 50);
    remote_rtcm_1013->push(remote_rtcm_1013_time, 50);

    time_truth.get_latest_time(&wn,
                               &wn_state,
                               &tow_ms,
                               &tow_ms_state,
                               &leap_seconds,
                               &leap_seconds_state,
                               &cache);
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_NONE);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_NONE);
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_NONE);
  }

  // push local ephemeris data, values are cached 2.5 weeks in the past. still
  // unable to generate results until observation TOW is passed in.
  {
    for (size_t i = 0; i < ARRAY_SIZE(kGpsEphemerisTimes); ++i) {
      if (i % 2 == 0) {
        gps_time_t gps_time = kGpsEphemerisTimes[i].gps_time;
        add_secs(&gps_time, -2.5 * WEEK_SECS);
        local_ephemeris->push(kGpsEphemerisTimes[i].gnss_signal, gps_time);
      }
    }

    time_truth.get_latest_time(&wn,
                               &wn_state,
                               &tow_ms,
                               &tow_ms_state,
                               &leap_seconds,
                               &leap_seconds_state,
                               &cache);
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_NONE);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_NONE);
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_NONE);
  }

  // finally push in some observation TOW, this should now allow us to get some
  // estimates for TOW/leap seconds. the TOW estimate will be considered BEST
  // since it's the best we can deduce given the information provided (since
  // there is only one local observation). the WN will be categorized as BEST
  // because although the local leap estimator and the ephemeris estimator both
  // provided values 2.5 weeks in to the past, they were discarded because they
  // were too far away from the current TOW. therefor our best estimate is the
  // data coming from the remote leap second estimator. the leap second
  // information will be considered BEST as the WN/TOW combination is available
  // in the leap second lookup table even though the leap second information was
  // wrong.
  {
    local_observation->push(kEphemerisTimeTowMs);

    time_truth.get_latest_time(&wn,
                               &wn_state,
                               &tow_ms,
                               &tow_ms_state,
                               &leap_seconds,
                               &leap_seconds_state,
                               &cache);
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BEST);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BEST);
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_BEST);

    EXPECT_EQ(wn, kEphemerisTimeWn);
    EXPECT_EQ(tow_ms, kEphemerisTimeTowMs);
    EXPECT_EQ(leap_seconds, 18);
  }

  // push the exact same ephemeris signals but this time 5000 weeks into the
  // future. likewise push the local/remote leap second information 5000 weeks
  // into the future, at this point the WN should move forward. the leap second
  // information this time will be coming from the leap second estimator since
  // the leap second lookup table is far into the past.
  {
    static constexpr int16_t kWnShifted = 5000;

    gps_time_t shifted_local_ubx_leap_time = local_ubx_leap_time;
    gps_time_t shifted_remote_rtcm_1013_time = remote_rtcm_1013_time;

    shifted_local_ubx_leap_time.wn += kWnShifted;
    shifted_remote_rtcm_1013_time.wn += kWnShifted;

    local_ubx_leap->push(shifted_local_ubx_leap_time, 50);
    remote_rtcm_1013->push(shifted_remote_rtcm_1013_time, 50);

    for (size_t i = 0; i < ARRAY_SIZE(kGpsEphemerisTimes); ++i) {
      if (i % 2 == 0) {
        gps_time_t gps_time = kGpsEphemerisTimes[i].gps_time;
        gps_time.wn += kWnShifted;
        local_ephemeris->push(kGpsEphemerisTimes[i].gnss_signal, gps_time);
      }
    }

    time_truth.get_latest_time(&wn,
                               &wn_state,
                               &tow_ms,
                               &tow_ms_state,
                               &leap_seconds,
                               &leap_seconds_state,
                               &cache);
    EXPECT_EQ(wn_state, TIME_TRUTH_STATE_BEST);
    EXPECT_EQ(tow_ms_state, TIME_TRUTH_STATE_BEST);
    EXPECT_EQ(leap_seconds_state, TIME_TRUTH_STATE_BEST);

    EXPECT_EQ(wn, kWnShifted + kEphemerisTimeWn);
    EXPECT_EQ(tow_ms, kEphemerisTimeTowMs);
    EXPECT_EQ(leap_seconds, 50);
  }
}
