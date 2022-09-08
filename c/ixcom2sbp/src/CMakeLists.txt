swift_add_tool_library(ixcom2sbp_library
  SOURCES
    ixcom2sbp.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)
swift_add_tool(ixcom2sbp
  SOURCES
    ixcom2sbp_main.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)

set_target_properties(ixcom2sbp_library PROPERTIES
  OUTPUT_NAME ixcom2sbp
)

target_link_libraries(ixcom2sbp_library PRIVATE swiftnav::gnss_converters)
target_link_libraries(ixcom2sbp PRIVATE swiftnav::ixcom2sbp_library)

target_include_directories(ixcom2sbp_library PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(ixcom2sbp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

install(TARGETS ixcom2sbp_library DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS ixcom2sbp DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
