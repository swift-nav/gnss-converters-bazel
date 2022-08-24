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
#include <observation_adjuster/epoch_adjuster.h>
#include <swiftnav/signal.h>

namespace obs_adjuster {

class TestEpochAdjuster : public ::testing::Test {
 public:
  TestEpochAdjuster() = default;
  virtual ~TestEpochAdjuster() = default;

  class ObsSample {
   public:
    ObsSample()
        : pseudorange_(0.0),
          carrier_phase_(0.0),
          doppler_(0.0),
          lock_time_(0),
          cn0_(0) {}
    ObsSample(const gnss_signal_t &sid,
              const double pseudorange,
              const double carrier_phase,
              const double doppler)
        : sid_(sid),
          pseudorange_(pseudorange),
          carrier_phase_(carrier_phase),
          doppler_(doppler),
          lock_time_(0),
          cn0_(0){};
    virtual ~ObsSample(){};

    u32 encoded_pseudorange() const { return std::round(pseudorange_ / 0.02); }

    sbp_carrier_phase_t encoded_carrier() const {
      sbp_carrier_phase_t L{};
      L.i = std::floor(carrier_phase_);
      L.f = std::round(std::fmod(carrier_phase_, 1.0) * 256.0);
      return L;
    }

    sbp_doppler_t encoded_doppler() const {
      sbp_doppler_t D{};
      D.i = std::floor(doppler_);
      D.f = std::round(std::fmod(doppler_, 1.0) * 256.0);
      return D;
    }

    sbp_v4_gnss_signal_t encoded_sid() const {
      sbp_v4_gnss_signal_t sbp_sid{static_cast<u8>(sid_.sat),
                                   static_cast<u8>(sid_.code)};
      return sbp_sid;
    }

    gnss_signal_t sid_;
    double pseudorange_;
    double carrier_phase_;
    double doppler_;
    uint8_t lock_time_;
    uint8_t cn0_;
  };

  void add_obs(const ObsSample &sample,
               SbpObsArray<MAX_OBS_PER_EPOCH> *obs_array) {
    assert(obs_array != nullptr);
    sbp_packed_obs_content_t obs{};

    obs.sid = sample.encoded_sid();
    obs.P = sample.encoded_pseudorange();
    obs.L.i = sample.encoded_carrier().i;
    obs.L.f = sample.encoded_carrier().f;
    obs.D.i = sample.encoded_doppler().i;
    obs.D.f = sample.encoded_doppler().f;
    obs.lock = sample.lock_time_;
    obs.cn0 = sample.cn0_;
    obs.flags = 0x0F;  // Psr, carrier, doppler and half cycle amb valid

    obs_array->add(obs);
  }

  ObsSample calculate_expected_sample(const ObsSample &base_obs_sample,
                                      const ObsSample &base_corr_sample,
                                      const ObsSample &vrs_corr_sample) {
    // base_obs + (vrs_corr - base_corr)
    ObsSample expected_sample(
        base_obs_sample.sid_,
        base_obs_sample.pseudorange_ +
            (vrs_corr_sample.pseudorange_ - base_corr_sample.pseudorange_),
        base_obs_sample.carrier_phase_ +
            (vrs_corr_sample.carrier_phase_ - base_corr_sample.carrier_phase_),
        base_obs_sample.doppler_ +
            (vrs_corr_sample.doppler_ - base_corr_sample.doppler_));
    // Lowest lock time
    uint8_t lowest_lock_time = base_obs_sample.lock_time_;
    if (base_corr_sample.lock_time_ < lowest_lock_time) {
      lowest_lock_time = base_corr_sample.lock_time_;
    }
    if (vrs_corr_sample.lock_time_ < lowest_lock_time) {
      lowest_lock_time = vrs_corr_sample.lock_time_;
    }

    // Lowest cn0
    uint8_t lowest_cn0 = base_obs_sample.cn0_;
    if (base_corr_sample.cn0_ < lowest_cn0) {
      lowest_cn0 = base_corr_sample.cn0_;
    }
    if (vrs_corr_sample.cn0_ < lowest_cn0) {
      lowest_cn0 = vrs_corr_sample.cn0_;
    }

    expected_sample.lock_time_ = lowest_lock_time;
    expected_sample.cn0_ = lowest_cn0;

    return expected_sample;
  }

  void verify_correct_adjustment(
      const ObsSample &base_obs_sample,
      const ObsSample &base_corr_sample,
      const ObsSample &vrs_corr_sample,
      const sbp_packed_obs_content_t &adjusted_sample) {
    ASSERT_EQ(base_obs_sample.sid_, base_corr_sample.sid_);
    ASSERT_EQ(base_obs_sample.sid_, vrs_corr_sample.sid_);
    ASSERT_EQ(base_obs_sample.sid_.sat, adjusted_sample.sid.sat);
    ASSERT_EQ(base_obs_sample.sid_.code, adjusted_sample.sid.code);

    const ObsSample expected_sample = calculate_expected_sample(
        base_obs_sample, base_corr_sample, vrs_corr_sample);

    EXPECT_EQ(adjusted_sample.sid.sat, expected_sample.encoded_sid().sat);
    EXPECT_EQ(adjusted_sample.sid.code, expected_sample.encoded_sid().code);
    EXPECT_EQ(adjusted_sample.flags, 0x0F);
    EXPECT_EQ(adjusted_sample.P, expected_sample.encoded_pseudorange());
    EXPECT_EQ(adjusted_sample.L.i, expected_sample.encoded_carrier().i);
    // Because of numerical effects when the fractional parts of cycles
    // are truncated into the .f part inside the EpochAdjuster we can't
    // expect the value in expected sample and adjusted_sample to be exactly
    // equal. Allowing 2.0 in difference means less than 0.1 cycle which is good
    // enough.
    EXPECT_NEAR(adjusted_sample.L.f, expected_sample.encoded_carrier().f, 2.0);
    EXPECT_EQ(adjusted_sample.D.i, expected_sample.encoded_doppler().i);
    EXPECT_NEAR(adjusted_sample.D.f, expected_sample.encoded_doppler().f, 2.0);
    EXPECT_EQ(adjusted_sample.lock, expected_sample.lock_time_);
    EXPECT_EQ(adjusted_sample.cn0, expected_sample.cn0_);
  }

  SbpObsArray<MAX_OBS_PER_EPOCH> base_obs;
  SbpObsArray<MAX_OBS_PER_EPOCH> base_corr;
  SbpObsArray<MAX_OBS_PER_EPOCH> vrs_corr;
  SbpObsArray<MAX_OBS_PER_EPOCH> adjusted_vrs;
};

TEST_F(TestEpochAdjuster, EmptyObs) {
  EpochAdjuster adjuster;
  ASSERT_FALSE(adjuster.adjust(base_obs, base_corr, vrs_corr, &adjusted_vrs));
}

TEST_F(TestEpochAdjuster, HandlesNull) {
  EpochAdjuster adjuster;
  ASSERT_FALSE(adjuster.adjust(base_obs, base_corr, vrs_corr, nullptr));
}

TEST_F(TestEpochAdjuster, IdenticalObs) {
  EpochAdjuster adjuster;

  ObsSample s1({1, CODE_GPS_L1CA}, 4567.8, 456.2, 986.3);
  s1.lock_time_ = 78;
  s1.cn0_ = 43;

  add_obs(s1, &base_obs);
  add_obs(s1, &base_corr);
  add_obs(s1, &vrs_corr);

  ASSERT_TRUE(adjuster.adjust(base_obs, base_corr, vrs_corr, &adjusted_vrs));

  ASSERT_EQ(adjusted_vrs.size(), 1);

  sbp_packed_obs_content_t adj_sample{};
  ASSERT_TRUE(adjusted_vrs.get(0, &adj_sample));

  ASSERT_NO_FATAL_FAILURE(verify_correct_adjustment(s1, s1, s1, adj_sample));
}

TEST_F(TestEpochAdjuster, SingleObs) {
  EpochAdjuster adjuster;

  gnss_signal_t sig{5, CODE_GAL_E1B};

  ObsSample s1(sig, 4567.8, 456.2, 986.3);
  s1.lock_time_ = 78;
  s1.cn0_ = 43;
  add_obs(s1, &base_obs);

  ObsSample s2(sig, 922234.3, 8325.6, 32.4);
  s2.lock_time_ = 20;
  s2.cn0_ = 49;
  add_obs(s2, &base_corr);

  ObsSample s3(sig, 65432.1, 3476.6, 134.2);
  s3.lock_time_ = 45;
  s3.cn0_ = 33;
  add_obs(s3, &vrs_corr);

  ASSERT_TRUE(adjuster.adjust(base_obs, base_corr, vrs_corr, &adjusted_vrs));

  ASSERT_EQ(adjusted_vrs.size(), 1);

  sbp_packed_obs_content_t adj_sample{};
  ASSERT_TRUE(adjusted_vrs.get(0, &adj_sample));

  ASSERT_NO_FATAL_FAILURE(verify_correct_adjustment(s1, s2, s3, adj_sample));
}

TEST_F(TestEpochAdjuster, MultipleUnorderedObs) {
  EpochAdjuster adjuster;

  std::vector<gnss_signal_t> signals{{1, CODE_GPS_L1CA},
                                     {23, CODE_GAL_E7I},
                                     {7, CODE_GPS_L2P},
                                     {13, CODE_GAL_E1B},
                                     {3, CODE_GAL_E1B},
                                     {9, CODE_GPS_L1CA}};

  std::map<uint64_t, std::map<StreamType, ObsSample>> samples;

  // Lambda function to calculate a unique key given a sat nr and signal code
  std::function<uint64_t(const gnss_signal_t &)> sat_to_map_key(
      [](const gnss_signal_t &sid) {
        const uint64_t key =
            static_cast<uint64_t>(code_to_constellation(sid.code)) * 10000 +
            sid.sat;
        return key;
      });

  for (std::size_t i = 0; i < signals.size(); ++i) {
    std::size_t base_obs_index = i;
    std::size_t base_corr_index = (i + 2) % signals.size();
    std::size_t vrs_corr_index = (i + 5) % signals.size();

    ObsSample base_obs_sample(signals.at(base_obs_index),
                              4567.8 + i * 13.2,
                              456.2 - i * 2.3,
                              986.3 + i * 0.2);
    base_obs_sample.lock_time_ = 78 + i;
    base_obs_sample.cn0_ = 37 + i;
    add_obs(base_obs_sample, &base_obs);
    samples[sat_to_map_key(base_obs_sample.sid_)][StreamType::STATION_OBS] =
        base_obs_sample;

    ObsSample base_corr_sample(signals.at(base_corr_index),
                               8932.1 + i * 45.7,
                               9923.3 - i * 13.9,
                               145.7 - i * 0.3);
    base_obs_sample.lock_time_ = 55 + i;
    base_obs_sample.cn0_ = 39 + i;
    add_obs(base_corr_sample, &base_corr);
    samples[sat_to_map_key(base_corr_sample.sid_)][StreamType::STATION_CORR] =
        base_corr_sample;

    ObsSample vrs_corr_sample(signals.at(vrs_corr_index),
                              8822.4 + i * 91.3,
                              567.1 - i * 94.5,
                              -135.6 + i * 0.1);
    base_obs_sample.lock_time_ = 88 + i;
    base_obs_sample.cn0_ = 38 + i;
    add_obs(vrs_corr_sample, &vrs_corr);
    samples[sat_to_map_key(vrs_corr_sample.sid_)][StreamType::VRS_CORR] =
        vrs_corr_sample;
  }

  SbpObsArray<MAX_OBS_PER_EPOCH> adjusted_vrs;
  ASSERT_TRUE(adjuster.adjust(base_obs, base_corr, vrs_corr, &adjusted_vrs));

  ASSERT_EQ(adjusted_vrs.size(), signals.size());

  // Check that all signals are present in the adjusted array, and have correct
  // (adjusted) values
  for (const gnss_signal_t &sig : signals) {
    bool sig_present = false;
    sbp_packed_obs_content_t adj_sample{};
    for (std::size_t i = 0; i < adjusted_vrs.size(); i++) {
      ASSERT_TRUE(adjusted_vrs.get(i, &adj_sample));
      if (adj_sample.sid.sat == sig.sat && adj_sample.sid.code == sig.code) {
        sig_present = true;
        break;
      }
    }
    ASSERT_TRUE(sig_present)
        << "Code " << static_cast<s32>(sig.code) << " sat "
        << static_cast<s32>(sig.sat) << " not present in adjusted list";
    ASSERT_NO_FATAL_FAILURE(verify_correct_adjustment(
        samples.at(sat_to_map_key(sig)).at(StreamType::STATION_OBS),
        samples.at(sat_to_map_key(sig)).at(StreamType::STATION_CORR),
        samples.at(sat_to_map_key(sig)).at(StreamType::VRS_CORR),
        adj_sample));
  }
}

}  // namespace obs_adjuster
