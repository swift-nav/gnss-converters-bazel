swift_add_tool_library(ubx2sbp_main
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

target_link_libraries(ubx2sbp_main PUBLIC swiftnav::gnss_converters)
target_link_libraries(ubx2sbp PRIVATE swiftnav::ubx2sbp_main)
target_include_directories(ubx2sbp_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(ubx2sbp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

install(TARGETS ubx2sbp_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS ubx2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
