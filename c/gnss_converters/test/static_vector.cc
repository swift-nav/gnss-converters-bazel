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

#include <gnss-converters/internal/static_vector.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

class TestStaticVector : public testing::Test {
 protected:
  struct InstanceTracker {
    size_t value;
    static size_t instances;
    int dummy_data;

    InstanceTracker() noexcept : InstanceTracker(0) {}

    explicit InstanceTracker(size_t value) : value(value), dummy_data(123) {
      ++instances;
    }

    InstanceTracker(const InstanceTracker &other) noexcept
        : InstanceTracker(0) {
      value = other.value;
    }

    InstanceTracker(InstanceTracker &&other) noexcept : InstanceTracker(0) {
      value = other.value;
    }

    InstanceTracker &operator=(const InstanceTracker &other) noexcept {
      value = other.value;
      return *this;
    }

    InstanceTracker &operator=(InstanceTracker &&other) noexcept {
      value = other.value;
      return *this;
    }

    ~InstanceTracker() {
      EXPECT_EQ(dummy_data, 123);
      dummy_data = 0;
      --instances;
    };
  };

 public:
  TestStaticVector() { InstanceTracker::instances = 0; }
};

size_t TestStaticVector::InstanceTracker::instances = 0;

TEST_F(TestStaticVector, DefaultConstructor) {
  static constexpr size_t kCapacity = 5;

  gnss_converters::StaticVector<InstanceTracker, kCapacity> vector;
  EXPECT_TRUE(vector.empty());
  EXPECT_EQ(vector.size(), 0);
  EXPECT_EQ(vector.capacity(), kCapacity);
  EXPECT_NE(vector.data(), nullptr);
  EXPECT_EQ(InstanceTracker::instances, 0);
}

TEST_F(TestStaticVector, BasicEmplaceBack) {
  static constexpr size_t kCapacity = 10;

  gnss_converters::StaticVector<size_t, kCapacity> vector;

  for (size_t i = 0; i < kCapacity; ++i) {
    EXPECT_TRUE(vector.emplace_back(i));
  }

  EXPECT_FALSE(vector.emplace_back(0u));

  for (size_t i = 0; i < kCapacity; ++i) {
    EXPECT_EQ(vector[i], i);
  }

  EXPECT_FALSE(vector.empty());
  EXPECT_EQ(vector.size(), kCapacity);
  EXPECT_EQ(vector.capacity(), kCapacity);
  EXPECT_NE(vector.data(), nullptr);

  vector.clear();

  EXPECT_TRUE(vector.empty());
  EXPECT_EQ(vector.size(), 0);
  EXPECT_EQ(vector.capacity(), kCapacity);
  EXPECT_NE(vector.data(), nullptr);
}

TEST_F(TestStaticVector, BasicReferencePushBack) {
  static constexpr size_t kCapacity = 10;
  const InstanceTracker kInstanceTracker(2);

  gnss_converters::StaticVector<InstanceTracker, kCapacity> vector;
  for (size_t i = 0; i < kCapacity; ++i) {
    EXPECT_TRUE(vector.push_back(kInstanceTracker));
  }

  EXPECT_FALSE(vector.push_back(kInstanceTracker));
  EXPECT_EQ(InstanceTracker::instances, kCapacity + 1);

  for (size_t i = 0; i < kCapacity; ++i) {
    EXPECT_EQ(vector[i].value, kInstanceTracker.value);
  }

  vector.clear();

  EXPECT_EQ(InstanceTracker::instances, 1);
  EXPECT_TRUE(vector.empty());
  EXPECT_EQ(vector.size(), 0);
  EXPECT_EQ(vector.capacity(), kCapacity);
  EXPECT_NE(vector.data(), nullptr);
}

TEST_F(TestStaticVector, BasicMovePushBack) {
  static constexpr size_t kCapacity = 10;

  gnss_converters::StaticVector<InstanceTracker, kCapacity> vector;
  for (size_t i = 0; i < kCapacity; ++i) {
    EXPECT_TRUE(vector.push_back(InstanceTracker(i)));
  }

  EXPECT_FALSE(vector.push_back(InstanceTracker(0)));
  EXPECT_EQ(InstanceTracker::instances, kCapacity);

  for (size_t i = 0; i < kCapacity; ++i) {
    EXPECT_EQ(vector[i].value, i);
  }

  vector.clear();

  EXPECT_EQ(InstanceTracker::instances, 0);
  EXPECT_TRUE(vector.empty());
  EXPECT_EQ(vector.size(), 0);
  EXPECT_EQ(vector.capacity(), kCapacity);
  EXPECT_NE(vector.data(), nullptr);
}

TEST_F(TestStaticVector, BasicPopBack) {
  static constexpr size_t kCapacity = 10;

  gnss_converters::StaticVector<InstanceTracker, kCapacity> vector;

  for (size_t i = 0; i < kCapacity; ++i) {
    vector.emplace_back(InstanceTracker(0));
  }

  for (size_t i = 0; i < kCapacity; ++i) {
    EXPECT_EQ(vector.size(), kCapacity - i);
    EXPECT_TRUE(vector.pop_back());
    EXPECT_EQ(vector.size(), kCapacity - i - 1);
    EXPECT_EQ(vector.capacity(), kCapacity);
  }

  EXPECT_FALSE(vector.pop_back());
}

TEST_F(TestStaticVector, DestructionPopBack) {
  static constexpr size_t kCapacity = 10;

  auto ptr = std::make_shared<int>(0);
  gnss_converters::StaticVector<std::shared_ptr<int>, kCapacity> vector;

  for (size_t i = 0; i < kCapacity; ++i) {
    vector.emplace_back(ptr);
  }

  EXPECT_EQ(vector.front().use_count(), kCapacity + 1);

  for (size_t i = 0; i < kCapacity; ++i) {
    EXPECT_TRUE(vector.pop_back());

    if (i + 1 < kCapacity) {
      EXPECT_EQ(vector.front().use_count(), kCapacity - i);
    }
  }

  EXPECT_FALSE(vector.pop_back());
}

TEST_F(TestStaticVector, HalfFilledArray) {
  static constexpr size_t kCapacity = 10;

  gnss_converters::StaticVector<InstanceTracker, kCapacity> vector;

  for (size_t i = 0; i < kCapacity / 2; ++i) {
    EXPECT_TRUE(vector.push_back(InstanceTracker(i)));
  }

  EXPECT_FALSE(vector.empty());
  EXPECT_EQ(vector.size(), kCapacity / 2);
  EXPECT_EQ(vector.capacity(), kCapacity);
  EXPECT_EQ(InstanceTracker::instances, kCapacity / 2);

  for (size_t i = 0; i < kCapacity / 2; ++i) {
    EXPECT_EQ(vector[i].value, i);
  }

  EXPECT_TRUE(vector.pop_back());

  EXPECT_EQ(vector.front().value, 0);
  EXPECT_EQ(vector.back().value, kCapacity / 2 - 2);

  EXPECT_FALSE(vector.empty());
  EXPECT_EQ(vector.size(), kCapacity / 2 - 1);
  EXPECT_EQ(vector.capacity(), kCapacity);
  EXPECT_EQ(InstanceTracker::instances, kCapacity / 2 - 1);

  EXPECT_DEBUG_DEATH(vector[kCapacity / 2 - 1], "");
}

TEST_F(TestStaticVector, IntegrationWithStdAlgorithms) {
  constexpr size_t kCapacity = 10;

  gnss_converters::StaticVector<int, kCapacity> vector;
  for (size_t i = 0; i < kCapacity; ++i) {
    vector.emplace_back(0);
  }

  std::generate(
      std::begin(vector), std::end(vector), [i = 0]() mutable { return i++; });
  for (size_t i = 0; i < kCapacity; ++i) {
    EXPECT_EQ(vector[i], i);
  }
  std::for_each(std::cbegin(vector),
                std::cend(vector),
                [i = 0](const int &entry) mutable { EXPECT_EQ(entry, i++); });

  std::generate(std::rbegin(vector), std::rend(vector), [i = 0]() mutable {
    return i++;
  });
  for (size_t i = 0; i < kCapacity; ++i) {
    EXPECT_EQ(vector[i], kCapacity - i - 1);
  }
  std::for_each(std::crbegin(vector),
                std::crend(vector),
                [i = 0](const int &entry) mutable { EXPECT_EQ(entry, i++); });
}

TEST_F(TestStaticVector, ConstVectorAccess) {
  constexpr size_t kCapacity = 10;

  gnss_converters::StaticVector<int, kCapacity> vector;
  for (size_t i = 0; i < kCapacity; ++i) {
    vector.emplace_back(0);
  }

  std::generate(
      std::begin(vector), std::end(vector), [i = 0]() mutable { return i++; });

  gnss_converters::StaticVector<int, kCapacity> &const_vector = vector;

  EXPECT_NE(const_vector.data(), nullptr);

  EXPECT_EQ(const_vector.front(), 0);
  EXPECT_EQ(const_vector.back(), kCapacity - 1);

  EXPECT_EQ(const_vector[0], 0);
  EXPECT_EQ(const_vector[const_vector.size() - 1], kCapacity - 1);
}
