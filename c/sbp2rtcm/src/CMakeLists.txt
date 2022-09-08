swift_add_tool_library(sbp2rtcm_library
  C_EXTENSIONS_ON
  SOURCES
    sbp2rtcm.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)
swift_add_tool(sbp2rtcm
  SOURCES
    sbp2rtcm_main.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)

set_target_properties(sbp2rtcm_library PROPERTIES
  OUTPUT_NAME sbp2rtcm
)
target_link_libraries(sbp2rtcm_library PUBLIC swiftnav::gnss_converters)
target_link_libraries(sbp2rtcm PRIVATE swiftnav::sbp2rtcm_library)
target_include_directories(sbp2rtcm_library PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(sbp2rtcm PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

install(TARGETS sbp2rtcm_library DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS sbp2rtcm DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
