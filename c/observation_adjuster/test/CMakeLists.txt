swift_add_test(unit-test-observation-adjuster
  UNIT_TEST
  SRCS
    test_epoch_adjuster.cc
    test_message_matcher.cc
    test_observation_adjuster.cc
    test_sbp_unpacker.cc
    test_sbp_packer.cc
    test_sbp_obs_array.cc
    test_obs_unpacker.cc
  INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../src/include
  LINK
    gtest
    gtest_main
    observation_adjuster
)

swift_add_test(test_observation_adjuster_pedantic
  UNIT_TEST
  SRCS
    check_pedantic.cc
  LINK
    observation_adjuster
)
swift_set_language_standards(test_observation_adjuster_pedantic)
swift_set_compile_options(test_observation_adjuster_pedantic ADD -pedantic)

swift_add_test(test_observation_adjuster_binding
  UNIT_TEST
  SRCS
    check_binding.c
  LINK
    observation_adjuster
)
