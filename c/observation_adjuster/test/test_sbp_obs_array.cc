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
#include <observation_adjuster/sbp_obs_array.h>

TEST(SbpObsArray, AddAndClear) {
  // Given an empty initialized array
  obs_adjuster::SbpObsArray<5> obs_array_;
  // Then it's reported size is zero
  ASSERT_EQ(0, obs_array_.size());

  // When adding an element
  ASSERT_TRUE(obs_array_.add(sbp_packed_obs_content_t{}));
  // Then the reported size is 1
  ASSERT_EQ(1, obs_array_.size());

  // When clearing the contents
  obs_array_.clear();
  // Then the size is again zero
  ASSERT_EQ(0, obs_array_.size());
}

TEST(SbpObsArray, TooMany) {
  // Given... an array filled up to maximum number of elements
  obs_adjuster::SbpObsArray<5> obs_array_;
  for (std::size_t i = 0; i < obs_array_.max_size(); ++i) {
    ASSERT_TRUE(obs_array_.add(sbp_packed_obs_content_t{}));
  }

  // When... adding one more element
  // Then... It returns false
  ASSERT_FALSE(obs_array_.add(sbp_packed_obs_content_t{}));
}

TEST(SbpObsArray, Get) {
  obs_adjuster::SbpObsArray<5> obs_array_;

  sbp_packed_obs_content_t obs_content{};
  obs_content.flags = 13;
  obs_content.cn0 = 23;
  ASSERT_TRUE(obs_array_.add(obs_content));

  obs_content.flags = 9;
  obs_content.cn0 = 25;
  ASSERT_TRUE(obs_array_.add(obs_content));

  sbp_packed_obs_content_t retrieved_obs{};
  ASSERT_TRUE(obs_array_.get(0, &retrieved_obs));
  EXPECT_EQ(retrieved_obs.flags, 13);
  EXPECT_EQ(retrieved_obs.cn0, 23);

  ASSERT_TRUE(obs_array_.get(1, &retrieved_obs));
  EXPECT_EQ(retrieved_obs.flags, 9);
  EXPECT_EQ(retrieved_obs.cn0, 25);

  ASSERT_FALSE(obs_array_.get(2, &retrieved_obs));
}

TEST(SbpObsArray, Set) {
  obs_adjuster::SbpObsArray<5> obs_array_;

  sbp_packed_obs_content_t obs_content{};
  obs_content.flags = 7;

  ASSERT_FALSE(obs_array_.set(0, obs_content));

  ASSERT_TRUE(obs_array_.add(obs_content));

  obs_content.flags = 9;
  ASSERT_TRUE(obs_array_.set(0, obs_content));
  sbp_packed_obs_content_t retrieved_obs{};
  ASSERT_TRUE(obs_array_.get(0, &retrieved_obs));
  EXPECT_EQ(retrieved_obs.flags, 9);

  ASSERT_FALSE(obs_array_.get(1, &retrieved_obs));
}
