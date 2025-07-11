add_custom_target(do-test-afl-rtcm3tosbp
  ${PROJECT_SOURCE_DIR}/scripts/start_afl_test.sh 
    rtcm3tosbp 
    ${CMAKE_CURRENT_SOURCE_DIR}/testcases/rtcm3tosbp
    ${CMAKE_CURRENT_BINARY_DIR}/findings/rtcm3tosbp/
    $<TARGET_FILE:rtcm3tosbp> -w 2161:532800)
add_custom_target(collect-known-failures-rtcm3tosbp
  ${PROJECT_SOURCE_DIR}/scripts/collect_known_failures.sh
    ${CMAKE_CURRENT_SOURCE_DIR}/findings/rtcm3tosbp/
    )
add_custom_target(rerun-known-failures-rtcm3tosbp
  COMMAND cargo run -- --crash_dir ${CMAKE_CURRENT_SOURCE_DIR}/findings/rtcm3tosbp/ --cmd_args "$<TARGET_FILE:rtcm3tosbp> -w 2161:532800"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/afl-runner
    )
add_custom_target(minimise-afl-data-rtcm3tosbp
  ${PROJECT_SOURCE_DIR}/scripts/afl-simplify-testcases.sh ${CMAKE_CURRENT_SOURCE_DIR}/findings/rtcm3tosbp ${CMAKE_CURRENT_SOURCE_DIR}/tmp-rtcm-simplified $<TARGET_FILE:rtcm3tosbp> -w 2161:532800
    )

add_custom_target(do-test-afl-ubx2sbp 
  ${PROJECT_SOURCE_DIR}/scripts/start_afl_test.sh 
    ubx2sbp 
    ${CMAKE_CURRENT_SOURCE_DIR}/testcases/ubx2sbp 
    ${CMAKE_CURRENT_BINARY_DIR}/findings/ubx2sbp
    $<TARGET_FILE:ubx2sbp>)
add_custom_target(collect-known-failures-ubx2sbp
  ${PROJECT_SOURCE_DIR}/scripts/collect_known_failures.sh
    ${CMAKE_CURRENT_SOURCE_DIR}/findings/ubx2sbp/
    )
add_custom_target(rerun-known-failures-ubx2sbp
  COMMAND cargo run -- --crash_dir ${CMAKE_CURRENT_SOURCE_DIR}/findings/ubx2sbp/ --cmd_args "$<TARGET_FILE:ubx2sbp>"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/afl-runner
    )
add_custom_target(minimise-afl-data-ubx2sbp
  ${PROJECT_SOURCE_DIR}/scripts/afl-simplify-testcases.sh ${CMAKE_CURRENT_SOURCE_DIR}/findings/ubx2sbp ${CMAKE_CURRENT_SOURCE_DIR}/tmp-ubx-simplified $<TARGET_FILE:ubx2sbp>
    )

add_custom_target(do-test-afl-sbp2sbp-decoder
  ${PROJECT_SOURCE_DIR}/scripts/start_afl_test.sh
    sbp2sbp_decoder
    ${CMAKE_CURRENT_SOURCE_DIR}/testcases/sbp2sbp_decoder
    ${CMAKE_CURRENT_BINARY_DIR}/findings/sbp2sbp_decoder/
    $<TARGET_FILE:sbp2sbp_decoder>)
add_custom_target(collect-known-failures-sbp2sbp-decoder
  ${PROJECT_SOURCE_DIR}/scripts/collect_known_failures.sh
    ${CMAKE_CURRENT_SOURCE_DIR}/findings/sbp2sbp_decoder/
    )
add_custom_target(rerun-known-failures-sbp2sbp-decoder
  COMMAND cargo run -- --crash_dir ${CMAKE_CURRENT_SOURCE_DIR}/findings/sbp2sbp_decoder/ --cmd_args "$<TARGET_FILE:sbp2sbp_decoder>"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/afl-runner
    )
add_custom_target(minimise-afl-data-sbp2sbp-decoder
  ${PROJECT_SOURCE_DIR}/scripts/afl-simplify-testcases.sh ${CMAKE_CURRENT_SOURCE_DIR}/findings/sbp2sbp_decoder ${CMAKE_CURRENT_SOURCE_DIR}/tmp-sbp-simplified $<TARGET_FILE:sbp2sbp_decoder>
    )

add_custom_target(do-test-afl-sbp2sbp-encoder
  ${PROJECT_SOURCE_DIR}/scripts/start_afl_test.sh
    sbp2sbp_encoder
    ${CMAKE_CURRENT_SOURCE_DIR}/testcases/sbp2sbp_encoder
    ${CMAKE_CURRENT_BINARY_DIR}/findings/sbp2sbp_encoder/
    $<TARGET_FILE:sbp2sbp_encoder>)
add_custom_target(collect-known-failures-sbp2sbp-encoder
  ${PROJECT_SOURCE_DIR}/scripts/collect_known_failures.sh
    ${CMAKE_CURRENT_SOURCE_DIR}/findings/sbp2sbp_encoder/
    )
add_custom_target(rerun-known-failures-sbp2sbp-encoder
  COMMAND cargo run -- --crash_dir ${CMAKE_CURRENT_SOURCE_DIR}/findings/sbp2sbp_encoder/ --cmd_args "$<TARGET_FILE:sbp2sbp_encoder>"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/afl-runner
    )
add_custom_target(minimise-afl-data-sbp2sbp-encoder
  ${PROJECT_SOURCE_DIR}/scripts/afl-simplify-testcases.sh ${CMAKE_CURRENT_SOURCE_DIR}/findings/sbp2sbp_encoder ${CMAKE_CURRENT_SOURCE_DIR}/tmp-sbp-simplified $<TARGET_FILE:sbp2sbp_encoder>
    )

swift_add_tool(make-afl-testcases
  SOURCES
    make_afl_testcases.c
    make_ubx_testcases.c
    make_rtcm_testcases.c
    make_sbp_decoder_testcases.c
    make_sbp_encoder_testcases.c

    helpers/sbp/sbp_data.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)
target_include_directories(make-afl-testcases PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(make-afl-testcases PRIVATE swiftnav::ubx swiftnav::rtcm swiftnav::sbp)
