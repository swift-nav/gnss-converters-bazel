add_subdirectory(parser)

add_library(nov2sbp_main nov2sbp.cc send_sbp_obs_messages.cc swiftnav_conversion_helpers.cc)

target_include_directories(nov2sbp_main PRIVATE "${CMAKE_CURRENT_LIST_DIR}/../../third_party/PROJ/src")
target_link_libraries(nov2sbp_main novatel-parser swiftnav sbp proj)

# Build SBP conversion tool.
add_executable(nov2sbp nov2sbp_main.cc)
target_link_libraries(nov2sbp nov2sbp_main)

target_compile_options(nov2sbp PRIVATE ${STARLING_COMPILE_OPTIONS} -Wno-strict-aliasing -Wno-effc++ -Wno-conversion -Wno-extern-c-compat)

install(TARGETS nov2sbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS nov2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
