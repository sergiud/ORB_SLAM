set (HDRS_DBOW2
  BowVector.h
  FORB.h
  FClass.h
  FeatureVector.h
  ScoringObject.h
  TemplatedVocabulary.h
  ${CMAKE_CURRENT_BINARY_DIR}/DBoW2_export.h
)

set (SRCS_DBOW2
  BowVector.cpp
  FORB.cpp
  FeatureVector.cpp
  ScoringObject.cpp
)

add_library (DBoW2 ${HDRS_DBOW2} ${SRCS_DBOW2})

target_include_directories (DBoW2 PUBLIC
  $<INSTALL_INTERFACE:include/DBoW2>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

target_link_libraries (DBoW2 PUBLIC opencv_core Boost::random DUtils)

set_target_properties (DBoW2 PROPERTIES SOVERSION ${MAJOR_VERSION})
set_target_properties (DBoW2 PROPERTIES VERSION ${VERSION})
set_target_properties (DBoW2 PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties (DBoW2 PROPERTIES VISIBILITY_INLINES_HIDDEN ON)

generate_export_header (DBoW2 EXPORT_FILE_NAME
  ${CMAKE_CURRENT_BINARY_DIR}/DBoW2_export.h)

install (TARGETS DBoW2
  EXPORT dbow2-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_package_config_file (dbow2-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/dbow2-config.cmake
  INSTALL_DESTINATION lib/cmake/DBoW2
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)
write_basic_package_version_file (dbow2-config-version.cmake
  VERSION ${VERSION} COMPATIBILITY SameMajorVersion)

export (TARGETS DBoW2 NAMESPACE DBoW2:: FILE dbow2-targets.cmake)
export (PACKAGE DBoW2)

install (EXPORT dbow2-targets NAMESPACE DBoW2:: DESTINATION
  lib/cmake/DBoW2)
install (FILES
  ${CMAKE_CURRENT_BINARY_DIR}/dbow2-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/dbow2-config-version.cmake
  DESTINATION lib/cmake/DBoW2
)

install (FILES ${HDRS_DBOW2} DESTINATION include/DBoW2)
