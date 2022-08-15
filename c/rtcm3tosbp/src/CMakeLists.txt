swift_add_tool_library(rtcm3tosbp_main
  SOURCES
    rtcm3tosbp.c
    time_truth.cc
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)
swift_add_tool(rtcm3tosbp
  SOURCES
    rtcm3tosbp_main.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)

target_link_libraries(rtcm3tosbp_main PUBLIC swiftnav::gnss_converters)
target_link_libraries(rtcm3tosbp PRIVATE swiftnav::rtcm3tosbp_main)
target_include_directories(rtcm3tosbp_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(rtcm3tosbp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

install(TARGETS rtcm3tosbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS rtcm3tosbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
