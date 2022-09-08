swift_add_tool_library(ubx2sbp_library
  SOURCES
    time_truth.cc
    ubx2sbp.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)
swift_add_tool(ubx2sbp
  SOURCES
    ubx2sbp_main.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)

set_target_properties(ubx2sbp_library PROPERTIES
  OUTPUT_NAME ubx2sbp
)
target_link_libraries(ubx2sbp_library PUBLIC swiftnav::gnss_converters)
target_link_libraries(ubx2sbp PRIVATE swiftnav::ubx2sbp_library)
target_include_directories(ubx2sbp_library PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(ubx2sbp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

install(TARGETS ubx2sbp_library DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS ubx2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
