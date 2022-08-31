swift_add_tool_library(sbp2sbp_decoder_main
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

target_link_libraries(sbp2sbp_decoder_main PUBLIC swiftnav::sbp swiftnav::swiftnav)
target_link_libraries(sbp2sbp_decoder PRIVATE swiftnav::sbp2sbp_decoder_main)
target_include_directories(sbp2sbp_decoder_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(sbp2sbp_decoder PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
install(TARGETS sbp2sbp_decoder_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS sbp2sbp_decoder DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})

swift_add_tool_library(sbp2sbp_encoder_main
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

target_link_libraries(sbp2sbp_encoder_main PUBLIC swiftnav::sbp swiftnav::swiftnav)
target_link_libraries(sbp2sbp_encoder PRIVATE swiftnav::sbp2sbp_encoder_main)
target_include_directories(sbp2sbp_encoder_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
target_include_directories(sbp2sbp_encoder PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)
install(TARGETS sbp2sbp_encoder_main DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(TARGETS sbp2sbp_encoder DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})
