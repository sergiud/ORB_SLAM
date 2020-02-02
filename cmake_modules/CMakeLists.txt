cmake_minimum_required (VERSION 3.1)

if (POLICY CMP0063)
  cmake_policy (SET CMP0063 NEW)
endif (POLICY CMP0063)

project (ORB_SLAM)

list (APPEND CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/g2o/cmake_modules)
list (APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

set (MAJOR_VERSION 1)
set (MINOR_VERSION 0)
set (VERSION_PATCH 0)

set (SHORT_VERSION "${MAJOR_VERSION}.${MINOR_VERSION}")

if (NOT VERSION_PATCH EQUAL 0)
  set (SHORT_VERSION "${SHORT_VERSION}.${VERSION_PATCH}")
endif (NOT VERSION_PATCH EQUAL 0)

set (VERSION "${MAJOR_VERSION}.${MINOR_VERSION}.${VERSION_PATCH}")

include (CheckCXXCompilerFlag)
include (CMakePackageConfigHelpers)
include (GenerateExportHeader)

find_package (Boost 1.55 REQUIRED atomic chrono date_time filesystem log
  program_options random thread system)
find_package (g2o 2017.7.30 REQUIRED core types_sba solver_cholmod types_slam3d
  types_sim3)
find_package (OpenCV 4.0 REQUIRED core calib3d imgproc highgui features2d)
find_package (TBB)
find_package (PkgConfig)

if (TBB_FOUND)
  set (TBB_DEPENDENCY "find_dependency (TBB ${TBB_VERSION})")

  # Create a proxy library called 'tbb' for OpenCV to link to. This is
  # especially needed under Windows if OpenCV was compiled with TBB support
  # enabled.
  if (NOT TARGET tbb)
    add_library (tbb UNKNOWN IMPORTED)

    if (TBB_LIBRARY_DEBUG)
      set_property (TARGET tbb APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
      set_target_properties (tbb PROPERTIES IMPORTED_LOCATION_DEBUG
        ${TBB_LIBRARY_DEBUG})
    endif (TBB_LIBRARY_DEBUG)

    if (TBB_LIBRARY_RELEASE)
      set_property (TARGET tbb APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
      set_target_properties (tbb PROPERTIES IMPORTED_LOCATION_RELEASE
        ${TBB_LIBRARY_RELEASE})
    endif (TBB_LIBRARY_RELEASE)
  endif (NOT TARGET tbb)
endif (TBB_FOUND)

add_definitions (-DBOOST_ALL_NO_LIB)
add_definitions (-DBOOST_FILESYSTEM_NO_DEPRECATED)

# Visual C++ compiler flags
check_cxx_compiler_flag (/bigobj HAVE_BIGOBJ)

if (HAVE_BIGOBJ)
  add_compile_options (/bigobj)
endif (HAVE_BIGOBJ)

if (MSVC)
  add_definitions (-D_CRT_SECURE_NO_WARNINGS)
  add_definitions (-D_SCL_SECURE_NO_WARNINGS)
endif (MSVC)

set (CMAKE_DEBUG_POSTFIX _debug)
set (CMAKE_POSITION_INDEPENDENT_CODE ON)

if (NOT DEFINED Boost_USE_STATIC_LIBS OR NOT Boost_USE_STATIC_LIBS)
  #add_definitions (-DBOOST_ALL_DYN_LINK)
endif (NOT DEFINED Boost_USE_STATIC_LIBS OR NOT Boost_USE_STATIC_LIBS)

if (PKGCONFIG_FOUND)
  pkg_check_modules (EIGEN3 REQUIRED eigen3>=3.2)
else (PKGCONFIG_FOUND)
  find_package (Eigen3 3.2 REQUIRED)
  set (EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif (PKGCONFIG_FOUND)

set (ORB_SLAM_HDRS
  ../include/Converter.h
  ../include/Frame.h
  ../include/FramePublisher.h
  ../include/Initializer.h
  ../include/KeyFrame.h
  ../include/KeyFrameDatabase.h
  ../include/LocalMapping.h
  ../include/LoopClosing.h
  ../include/Map.h
  ../include/MapPoint.h
  ../include/MapPublisher.h
  ../include/ORBVocabulary.h
  ../include/ORBextractor.h
  ../include/ORBmatcher.h
  ../include/Optimizer.h
  ../include/PnPsolver.h
  ../include/Sim3Solver.h
  ../include/Tracking.h
  ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM_export.h
)

add_library (ORB_SLAM
  ${ORB_SLAM_HDRS}
  ../src/Converter.cc
  ../src/Frame.cc
  ../src/FramePublisher.cc
  ../src/Initializer.cc
  ../src/KeyFrame.cc
  ../src/KeyFrameDatabase.cc
  ../src/LocalMapping.cc
  ../src/LoopClosing.cc
  ../src/Map.cc
  ../src/MapPoint.cc
  ../src/MapPublisher.cc
  ../src/ORBextractor.cc
  ../src/ORBmatcher.cc
  ../src/Optimizer.cc
  ../src/PnPsolver.cc
  ../src/Sim3Solver.cc
  ../src/Tracking.cc
)

set_target_properties (ORB_SLAM PROPERTIES SOVERSION ${MAJOR_VERSION})
set_target_properties (ORB_SLAM PROPERTIES VERSION ${VERSION})
set_target_properties (ORB_SLAM PROPERTIES CXX_VISIBILITY_PRESET hidden)
set_target_properties (ORB_SLAM PROPERTIES VISIBILITY_INLINES_HIDDEN ON)

if (MSVC)
  target_compile_definitions (ORB_SLAM PRIVATE _SCL_SECURE_NO_WARNINGS)
endif (MSVC)

target_include_directories (ORB_SLAM PUBLIC
  $<INSTALL_INTERFACE:include/ORB_SLAM>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/DBoW2
  ${CMAKE_CURRENT_BINARY_DIR}/DBoW2)

target_include_directories (ORB_SLAM SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries (ORB_SLAM PUBLIC Boost::thread g2o::solver_cholmod
  g2o::core g2o::types_sba g2o::types_slam3d g2o::types_sim3 ${OpenCV_LIBS}
  DBoW2 DUtils)

if (MSVC)
  set_property (DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS _SCL_SECURE_NO_WARNINGS)
endif (MSVC)

add_executable (run_ORB_SLAM
  ../src/run_ORB_SLAM.cc
)

target_link_libraries (run_ORB_SLAM PRIVATE ORB_SLAM Boost::log Boost::program_options Boost::thread)

generate_export_header (ORB_SLAM EXPORT_FILE_NAME
    ${CMAKE_CURRENT_BINARY_DIR}/ORB_SLAM_export.h)

install (TARGETS ORB_SLAM
  EXPORT orb_slam-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

set (g2o_DEPENDENCY "find_dependency (g2o ${g2o_VERSION})")

configure_package_config_file (orb_slam-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/orb_slam-config.cmake
  INSTALL_DESTINATION lib/cmake/ORB_SLAM
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)
write_basic_package_version_file (orb_slam-config-version.cmake
  VERSION ${VERSION} COMPATIBILITY SameMajorVersion)

export (TARGETS ORB_SLAM NAMESPACE ORB_SLAM:: FILE orb_slam-targets.cmake)
export (PACKAGE ORB_SLAM)

install (EXPORT orb_slam-targets NAMESPACE ORB_SLAM:: DESTINATION
  lib/cmake/ORB_SLAM)
install (FILES
  ${CMAKE_CURRENT_BINARY_DIR}/orb_slam-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/orb_slam-config-version.cmake
  DESTINATION lib/cmake/ORB_SLAM
)

install (FILES ${ORB_SLAM_HDRS} DESTINATION include/ORB_SLAM)
