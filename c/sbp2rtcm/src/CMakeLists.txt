swift_add_tool_library(sbp2rtcm_main
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

target_link_libraries(sbp2rtcm_main PUBLIC swiftnav::gnss_converters)
target_link_libraries(sbp2rtcm PRIVATE swiftnav::sbp2rtcm_main)

install(TARGETS sbp2rtcm_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS sbp2rtcm DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
