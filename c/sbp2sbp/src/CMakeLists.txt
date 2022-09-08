swift_add_tool_library(sbp2sbp_decoder_library
  SOURCES
    sbp2sbp_decoder.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
    )
swift_add_tool(sbp2sbp_decoder
  SOURCES
    sbp2sbp_decoder_main.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
    )

set_target_properties(sbp2sbp_decoder_library PROPERTIES
  OUTPUT_NAME sbp2sbp_decoder
)
target_link_libraries(sbp2sbp_decoder_library PUBLIC swiftnav::sbp swiftnav::swiftnav)
target_link_libraries(sbp2sbp_decoder PRIVATE swiftnav::sbp2sbp_decoder_library)
target_include_directories(sbp2sbp_decoder_library PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(sbp2sbp_decoder PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
install(TARGETS sbp2sbp_decoder_library DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS sbp2sbp_decoder DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})

swift_add_tool_library(sbp2sbp_encoder_library
  SOURCES
    sbp2sbp_encoder.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
    )
swift_add_tool(sbp2sbp_encoder
  SOURCES
    sbp2sbp_encoder_main.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
    )

set_target_properties(sbp2sbp_encoder_library PROPERTIES
  OUTPUT_NAME sbp2sbp_encoder
)
target_link_libraries(sbp2sbp_encoder_library PUBLIC swiftnav::sbp swiftnav::swiftnav)
target_link_libraries(sbp2sbp_encoder PRIVATE swiftnav::sbp2sbp_encoder_library)
target_include_directories(sbp2sbp_encoder_library PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(sbp2sbp_encoder PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
install(TARGETS sbp2sbp_encoder_library DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS sbp2sbp_encoder DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
