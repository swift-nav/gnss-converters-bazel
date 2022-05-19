swift_add_test(unit-test-observation-adjuster
        UNIT_TEST
        SRCS test_epoch_adjuster.cc
             test_message_matcher.cc
             test_observation_adjuster.cc
             test_sbp_unpacker.cc
             test_sbp_packer.cc
             test_sbp_obs_array.cc
             test_obs_unpacker.cc
        INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/../include ${CMAKE_CURRENT_SOURCE_DIR}/../src/include
        LINK gtest gtest_main observation_adjuster)
