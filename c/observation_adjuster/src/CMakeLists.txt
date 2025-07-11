unset(extra_library_options)
if (WIN32 AND MINGW)
  list(APPEND extra_library_options
    REMOVE_COMPILE_OPTIONS
      -Wall
      -Wformat
  )
endif()

swift_add_library(observation_adjuster
  SOURCES
    epoch_adjuster.cc
    message_matcher.cc
    observation_adjuster.cc
    binding.cc
    sbp_unpacker.cc
    sbp_packer.cc
    obs_unpacker.cc
    ${extra_library_options}
)

target_compile_options(observation_adjuster PRIVATE "-UNDEBUG")

if (WIN32 AND MINGW)
  target_compile_definitions(observation_adjuster
    PRIVATE
      __STDC_FORMAT_MACROS=1
  )
endif()

target_link_libraries(observation_adjuster
  PUBLIC
    swiftnav::gnss_converters
    swiftnav::gnss_converters_extra
    swiftnav::sbp
    swiftnav::swiftnav
)
target_include_directories(observation_adjuster
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

install(TARGETS observation_adjuster DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
