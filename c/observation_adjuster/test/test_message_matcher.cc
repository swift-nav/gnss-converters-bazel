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

#include <gtest/gtest.h>
#include <observation_adjuster/message_matcher.h>

namespace obs_adjuster {

TEST(MessageMatcher, TrivialMatch) {
  // Given... an empty matcher instance
  MessageMatcher matcher;

  TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> obs_array;
  obs_array.timestamp_.wn = 234;
  obs_array.timestamp_.tow = 4 * 24 * 3600 + 13 * 3600 + 5 * 60 + 54;

  // When... adding observations from all three sources
  matcher.add_obs(StreamType::VRS_CORR, obs_array);
  matcher.add_obs(StreamType::STATION_OBS, obs_array);
  matcher.add_obs(StreamType::STATION_CORR, obs_array);

  // Then... when asking for matching samples, one is found
  MatchingTriplet sample;
  gps_time_t timestamp = GPS_TIME_UNKNOWN;
  ASSERT_TRUE(matcher.find_match(&sample, &timestamp));
  EXPECT_EQ(timestamp.wn, obs_array.timestamp_.wn);
  EXPECT_FLOAT_EQ(timestamp.tow, obs_array.timestamp_.tow);

  // When... asking for additional matches
  // Then... none are found
  ASSERT_FALSE(matcher.find_match(&sample, &timestamp));
}

TEST(MessageMatcher, ClearsEarlierSamples) {
  // Given... an empty matcher instance
  MessageMatcher matcher;

  TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> obs_array_t1;
  obs_array_t1.timestamp_.wn = 234;
  obs_array_t1.timestamp_.tow = 4 * 24 * 3600 + 13 * 3600 + 5 * 60 + 54;

  // When... adding observations from two sources for t1..
  matcher.add_obs(StreamType::STATION_OBS, obs_array_t1);
  matcher.add_obs(StreamType::STATION_CORR, obs_array_t1);

  TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> obs_array_t2 = obs_array_t1;
  obs_array_t2.timestamp_.tow += 0.5;  // Half a second later than t1

  // .. and from all three for t2
  matcher.add_obs(StreamType::VRS_CORR, obs_array_t2);
  matcher.add_obs(StreamType::STATION_OBS, obs_array_t2);
  matcher.add_obs(StreamType::STATION_CORR, obs_array_t2);

  // Then... there is a match for t2
  MatchingTriplet sample;
  gps_time_t timestamp = GPS_TIME_UNKNOWN;
  ASSERT_TRUE(matcher.find_match(&sample, &timestamp));
  ASSERT_EQ(timestamp.wn, obs_array_t2.timestamp_.wn);
  ASSERT_FLOAT_EQ(timestamp.tow, obs_array_t2.timestamp_.tow);

  // When... adding the missing observation for t1
  matcher.add_obs(StreamType::VRS_CORR, obs_array_t1);

  // Then... there is still no match at t1
  ASSERT_FALSE(matcher.find_match(&sample, &timestamp));
}

TEST(MessageMatcher, IncreasingTime) {
  // Given... an empty matcher instance
  MessageMatcher matcher;

  TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> obs_array_t1;
  obs_array_t1.timestamp_.wn = 234;
  obs_array_t1.timestamp_.tow = 4 * 24 * 3600 + 13 * 3600 + 5 * 60 + 54;
  TimestampedSbpObsArray<MAX_OBS_PER_EPOCH> obs_array_t2 = obs_array_t1;
  obs_array_t2.timestamp_.tow += 0.5;  // Half a second later than t1

  // When... adding observations from three sources for both t1 and t2..
  matcher.add_obs(StreamType::VRS_CORR, obs_array_t1);
  matcher.add_obs(StreamType::STATION_OBS, obs_array_t1);
  matcher.add_obs(StreamType::STATION_CORR, obs_array_t1);
  matcher.add_obs(StreamType::VRS_CORR, obs_array_t2);
  matcher.add_obs(StreamType::STATION_OBS, obs_array_t2);
  matcher.add_obs(StreamType::STATION_CORR, obs_array_t2);

  // Then... there is a match for t1..
  MatchingTriplet sample;
  gps_time_t timestamp = GPS_TIME_UNKNOWN;
  ASSERT_TRUE(matcher.find_match(&sample, &timestamp));
  ASSERT_EQ(timestamp.wn, obs_array_t1.timestamp_.wn);
  ASSERT_FLOAT_EQ(timestamp.tow, obs_array_t1.timestamp_.tow);

  // .. and subsequently for t2
  ASSERT_TRUE(matcher.find_match(&sample, &timestamp));
  ASSERT_EQ(timestamp.wn, obs_array_t2.timestamp_.wn);
  ASSERT_FLOAT_EQ(timestamp.tow, obs_array_t2.timestamp_.tow);
}

}  // namespace obs_adjuster
