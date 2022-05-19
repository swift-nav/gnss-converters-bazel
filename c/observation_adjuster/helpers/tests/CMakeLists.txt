swift_add_test(observation-adjuster-helpers-tests
        UNIT_TEST
        SRCS test_obs_unpacker.cc test_sbp_obs_array.cc
        LINK gtest gtest_main swiftnav observation_adjuster)