# SPDX-FileCopyrightText: 2016-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
#
# SPDX-License-Identifier: BSD-3-Clause

add_library (DBoW2)
add_library (DBoW2::DBoW2 ALIAS DBoW2)

generate_export_header (DBoW2
  EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/DBoW2_export.h
)

target_sources (DBoW2
  PRIVATE
    BowVector.cpp
    FORB.cpp
    FeatureVector.cpp
    ScoringObject.cpp
  PUBLIC
    FILE_SET HEADERS
    BASE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${CMAKE_CURRENT_BINARY_DIR}
    FILES
      BowVector.h
      FClass.h
      FORB.h
      FeatureVector.h
      ScoringObject.h
      TemplatedVocabulary.h
      ${CMAKE_CURRENT_BINARY_DIR}/DBoW2_export.h
)

# The vocabulary owns its scoring object through std::make_unique.
target_compile_features (DBoW2 PUBLIC cxx_std_14)

target_link_libraries (DBoW2 PUBLIC opencv_core)

set_target_properties (DBoW2 PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  SOVERSION ${PROJECT_VERSION_MAJOR}
  VERSION ${PROJECT_VERSION}
  VISIBILITY_INLINES_HIDDEN ON
)

install (TARGETS DBoW2
  EXPORT dbow2-targets
  FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/DBoW2
)

configure_package_config_file (dbow2-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/dbow2-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DBoW2
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
write_basic_package_version_file (
  ${CMAKE_CURRENT_BINARY_DIR}/dbow2-config-version.cmake
  COMPATIBILITY SameMajorVersion
)

export (EXPORT dbow2-targets
  NAMESPACE DBoW2::
  FILE dbow2-targets.cmake
)
install (EXPORT dbow2-targets
  NAMESPACE DBoW2::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DBoW2
)
install (FILES
  ${CMAKE_CURRENT_BINARY_DIR}/dbow2-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/dbow2-config-version.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DBoW2
)
