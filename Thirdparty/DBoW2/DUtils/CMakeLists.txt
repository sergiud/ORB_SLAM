set (HDRS_DUTILS
  Random.h
  Timestamp.h
  ${CMAKE_CURRENT_BINARY_DIR}/DUtils_export.h
)

set (SRCS_DUTILS
  Random.cpp
  Timestamp.cpp
)

add_library (DUtils ${HDRS_DUTILS} ${SRCS_DUTILS})

#target_include_directories (DUtils PUBLIC ${OpenCV_INCLUDE_DIRS})

target_include_directories (DUtils PUBLIC
  $<INSTALL_INTERFACE:include/DUtils>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

#target_link_libraries (DUtils PUBLIC ${OpenCV_LIBS})

set_target_properties (DUtils PROPERTIES SOVERSION ${MAJOR_VERSION})
set_target_properties (DUtils PROPERTIES VERSION ${VERSION})
set_target_properties (DUtils PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties (DUtils PROPERTIES VISIBILITY_INLINES_HIDDEN ON)

generate_export_header (DUtils EXPORT_FILE_NAME
  ${CMAKE_CURRENT_BINARY_DIR}/DUtils_export.h)

install (TARGETS DUtils
  EXPORT dutils-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_package_config_file (dutils-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/dutils-config.cmake
  INSTALL_DESTINATION lib/cmake/DUtils
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)
write_basic_package_version_file (dutils-config-version.cmake
  VERSION ${VERSION} COMPATIBILITY SameMajorVersion)

export (TARGETS DUtils NAMESPACE DUtils:: FILE dutils-targets.cmake)
export (PACKAGE DUtils)

install (EXPORT dutils-targets NAMESPACE DUtils:: DESTINATION
  lib/cmake/DUtils)
install (FILES
  ${CMAKE_CURRENT_BINARY_DIR}/dutils-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/dutils-config-version.cmake
  DESTINATION lib/cmake/DUtils
)

install (FILES ${HDRS_DUTILS} DESTINATION include/DUtils)
