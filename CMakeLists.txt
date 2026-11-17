# SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required (VERSION 3.28...4.5)
# The SuiteSparse of g2o finds OpenMP for C as well.
project (ORB_SLAM
  VERSION 2.0.0
  LANGUAGES C CXX
)

find_package (Boost 1.72 NO_MODULE REQUIRED COMPONENTS program_options)
find_package (Eigen3 3.4.1...5 REQUIRED NO_MODULE)
find_package (g2o 2017.7.30 REQUIRED core types_sba types_slam3d types_sim3)
find_package (ng-log 0.9.0 REQUIRED COMPONENTS experimental)
find_package (OpenCV 5.0 NO_MODULE REQUIRED core calib imgproc highgui features)
find_package (Threads REQUIRED)

include (CMakePackageConfigHelpers)
include (CTest)
include (GenerateExportHeader)
include (GNUInstallDirs)

set (CMAKE_DEBUG_POSTFIX _debug)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library (ORB_SLAM)
add_library (ORB_SLAM::ORB_SLAM ALIAS ORB_SLAM)

generate_export_header (ORB_SLAM
  EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM_export.h
)

target_sources (ORB_SLAM
  PRIVATE
    src/Converter.cc
    src/Frame.cc
    src/FramePublisher.cc
    src/Initializer.cc
    src/KeyFrame.cc
    src/KeyFrameDatabase.cc
    src/LocalMapping.cc
    src/LoopClosing.cc
    src/Map.cc
    src/MapPoint.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/Optimizer.cc
    src/PnPsolver.cc
    src/Sampling.cc
    src/Sim3Solver.cc
    src/Tracking.cc
  PUBLIC
    FILE_SET HEADERS
    BASE_DIRS
      include
      ${CMAKE_CURRENT_BINARY_DIR}
    FILES
      include/Converter.h
      include/Frame.h
      include/FramePublisher.h
      include/Initializer.h
      include/KeyFrame.h
      include/KeyFrameDatabase.h
      include/LocalMapping.h
      include/LoopClosing.h
      include/Map.h
      include/MapPoint.h
      include/ORBVocabulary.h
      include/ORBextractor.h
      include/ORBmatcher.h
      include/Optimizer.h
      include/PnPsolver.h
      include/Sampling.h
      include/Sim3Solver.h
      include/Tracking.h
      ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM_export.h
)

# The public headers use std::stop_token, while the sources format output
# with std::println.
target_compile_features (ORB_SLAM PUBLIC cxx_std_20 PRIVATE cxx_std_23)

target_compile_definitions (ORB_SLAM PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:_CRT_SECURE_NO_WARNINGS>
  $<$<CXX_COMPILER_ID:MSVC>:_SCL_SECURE_NO_WARNINGS>
)

# The optimizer instantiates more templates than an object file of MSVC
# holds by default.
target_compile_options (ORB_SLAM PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/bigobj>
)

target_link_libraries (ORB_SLAM
  PUBLIC
    DBoW2::DBoW2
    Eigen3::Eigen
    g2o::types_sba
    g2o::types_sim3
    opencv_core
    opencv_features
  PRIVATE
    g2o::core
    g2o::types_slam3d
    ng-log::experimental
    opencv_calib
    opencv_imgproc
    Threads::Threads
)

set_target_properties (ORB_SLAM PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  SOVERSION ${PROJECT_VERSION_MAJOR}
  VERSION ${PROJECT_VERSION}
  VISIBILITY_INLINES_HIDDEN ON
)

add_executable (run_ORB_SLAM src/run_ORB_SLAM.cc)

target_compile_features (run_ORB_SLAM PRIVATE cxx_std_23)

target_link_libraries (run_ORB_SLAM PRIVATE
  Boost::program_options
  ng-log::experimental
  opencv_highgui
  ORB_SLAM::ORB_SLAM
  Threads::Threads
)

add_subdirectory (Thirdparty/DBoW2)

if (PROJECT_IS_TOP_LEVEL AND BUILD_TESTING)
  add_subdirectory (tests)
endif (PROJECT_IS_TOP_LEVEL AND BUILD_TESTING)

install (TARGETS ORB_SLAM
  EXPORT orb_slam-targets
  FILE_SET HEADERS DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ORB_SLAM
)
install (TARGETS run_ORB_SLAM)

set (g2o_DEPENDENCY "find_dependency (g2o ${g2o_VERSION})")

configure_package_config_file (cmake_modules/orb_slam-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/orb_slam-config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ORB_SLAM
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)
write_basic_package_version_file (
  ${CMAKE_CURRENT_BINARY_DIR}/orb_slam-config-version.cmake
  COMPATIBILITY SameMajorVersion
)

export (EXPORT orb_slam-targets
  NAMESPACE ORB_SLAM::
  FILE orb_slam-targets.cmake
)
install (EXPORT orb_slam-targets
  NAMESPACE ORB_SLAM::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ORB_SLAM
)
install (FILES
  ${CMAKE_CURRENT_BINARY_DIR}/orb_slam-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/orb_slam-config-version.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ORB_SLAM
)
