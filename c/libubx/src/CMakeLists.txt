set(libubx_HEADERS
  ${PROJECT_SOURCE_DIR}/libubx/include/ubx/ubx_messages.h
  ${PROJECT_SOURCE_DIR}/libubx/include/ubx/encode.h
  ${PROJECT_SOURCE_DIR}/libubx/include/ubx/decode.h
  )

swift_add_library(ubx
  SOURCES
    decode.c
    encode.c
  REMOVE_COMPILE_OPTIONS
    -Wconversion
)

target_link_libraries(ubx PUBLIC swiftnav::swiftnav)
target_include_directories(ubx PUBLIC ${PROJECT_SOURCE_DIR}/libubx/include)
target_include_directories(ubx PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/)

install(TARGETS ubx DESTINATION ${CMAKE_INSTALL_FULL_LIBDIR})
install(FILES ${libubx_HEADERS} DESTINATION ${CMAKE_INSTALL_FULL_INCLUDEDIR}/ubx)
