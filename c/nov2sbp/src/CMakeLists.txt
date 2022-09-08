swift_add_tool_library(nov2sbp_library
  SOURCES
    nov2sbp.cc
    send_sbp_obs_messages.cc
    swiftnav_conversion_helpers.cc
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)

swift_add_tool(nov2sbp
  SOURCES
    nov2sbp_main.cc
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)

set_target_properties(nov2sbp_library PROPERTIES
  OUTPUT_NAME nov2sbp
)

target_link_libraries(nov2sbp_library PRIVATE swiftnav::novatel-parser swiftnav::swiftnav swiftnav::sbp)
target_link_libraries(nov2sbp PRIVATE swiftnav::nov2sbp_library)

target_include_directories(nov2sbp_library PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(nov2sbp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

target_compile_options(nov2sbp_library PRIVATE -UNDEBUG)
target_compile_options(nov2sbp PRIVATE -UNDEBUG)

install(TARGETS nov2sbp_library DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS nov2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
