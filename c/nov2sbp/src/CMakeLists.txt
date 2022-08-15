swift_add_tool_library(nov2sbp_main
  SOURCES
    nov2sbp.cc
    send_sbp_obs_messages.cc
    swiftnav_conversion_helpers.cc
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)
target_link_libraries(nov2sbp_main PRIVATE swiftnav::novatel-parser swiftnav::swiftnav swiftnav::sbp)

# Build SBP conversion tool.
swift_add_tool(nov2sbp
  SOURCES
    nov2sbp_main.cc
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)
target_link_libraries(nov2sbp PRIVATE swiftnav::nov2sbp_main)

target_compile_options(nov2sbp_main PRIVATE -UNDEBUG)
target_compile_options(nov2sbp PRIVATE -UNDEBUG)

install(TARGETS nov2sbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS nov2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
