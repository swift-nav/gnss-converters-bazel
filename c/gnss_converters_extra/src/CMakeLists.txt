swift_add_library(gnss_converters_extra
  SOURCES
    logging.c
    sbp_conv.c
  REMOVE_COMPILE_OPTIONS
    -pedantic
    -Wconversion
)

target_compile_options(gnss_converters_extra PRIVATE "-UNDEBUG")

target_link_libraries(gnss_converters_extra PUBLIC swiftnav::gnss_converters swiftnav::rtcm swiftnav::sbp swiftnav::swiftnav)
target_include_directories(gnss_converters_extra PUBLIC ${PROJECT_SOURCE_DIR}/gnss_converters_extra/include)

install(TARGETS gnss_converters_extra DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/gnss_converters_extra/include/gnss-converters-extra
  DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/
)
