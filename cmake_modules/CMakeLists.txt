cmake_minimum_required (VERSION 3.1)
project (ORB_SLAM)

list (APPEND CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/g2o/cmake_modules)
list (APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

include (CMakePackageConfigHelpers)
include (GenerateExportHeader)

find_package (Boost 1.55 REQUIRED atomic chrono date_time filesystem program_options thread system)
find_package (g2o 0.1 REQUIRED)
find_package (OpenCV 2.4 REQUIRED core calib3d imgproc highgui features2d)
find_package (TBB)
find_package (PkgConfig)

if (PKGCONFIG_FOUND)
  pkg_check_modules (EIGEN3 REQUIRED eigen3>=3.2)
else (PKGCONFIG_FOUND)
  find_package (Eigen3 3.2 REQUIRED)
  set (EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif (PKGCONFIG_FOUND)

set (ORB_SLAM_HDRS
  ../include/Converter.h
  ../include/EdgeSE3ProjectXYZ.h
  ../include/EdgeSim3ProjectXYZ.h
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
  ../Thirdparty/DBoW2/DBoW2/BowVector.cpp
  ../Thirdparty/DBoW2/DBoW2/BowVector.h
  ../Thirdparty/DBoW2/DBoW2/FClass.h
  ../Thirdparty/DBoW2/DBoW2/FORB.cpp
  ../Thirdparty/DBoW2/DBoW2/FORB.h
  ../Thirdparty/DBoW2/DBoW2/FeatureVector.cpp
  ../Thirdparty/DBoW2/DBoW2/FeatureVector.h
  ../Thirdparty/DBoW2/DBoW2/ScoringObject.cpp
  ../Thirdparty/DBoW2/DBoW2/ScoringObject.h
  ../Thirdparty/DBoW2/DBoW2/TemplatedVocabulary.h
  ../Thirdparty/DBoW2/DUtils/Random.cpp
  ../Thirdparty/DBoW2/DUtils/Random.h
  ../Thirdparty/DBoW2/DUtils/Timestamp.cpp
  ../Thirdparty/DBoW2/DUtils/Timestamp.h
  ${ORB_SLAM_HDRS}
  ../src/Converter.cc
  ../src/EdgeSE3ProjectXYZ.cc
  ../src/EdgeSim3ProjectXYZ.cc
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

set_target_properties (ORB_SLAM PROPERTIES CXX_STANDARD 11)
set_target_properties (ORB_SLAM PROPERTIES CXX_STANDARD_REQUIRED ON)

if (MSVC)
  target_compile_definitions (ORB_SLAM PRIVATE _SCL_SECURE_NO_WARNINGS)
endif (MSVC)

target_include_directories (ORB_SLAM PUBLIC
  $<INSTALL_INTERFACE:include/ORB_SLAM>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
)

target_include_directories (ORB_SLAM SYSTEM PUBLIC ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries (ORB_SLAM PUBLIC ${Boost_LIBRARIES} g2o::solver_cholmod
  g2o::core g2o::types_sba g2o::types_slam3d g2o::types_sim3 ${OpenCV_LIBS})

if (TBB_FOUND)
  target_link_libraries (ORB_SLAM PUBLIC ${TBB_LIBRARIES})
endif (TBB_FOUND)

if (MSVC)
  set_property (DIRECTORY APPEND PROPERTY COMPILE_DEFINITIONS _SCL_SECURE_NO_WARNINGS)
endif (MSVC)

add_executable (run_ORB_SLAM
  ../src/run_ORB_SLAM.cc
)

set_target_properties (run_ORB_SLAM PROPERTIES CXX_STANDARD 11)
set_target_properties (run_ORB_SLAM PROPERTIES CXX_STANDARD_REQUIRED ON)

target_link_libraries (run_ORB_SLAM PRIVATE ORB_SLAM)

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
  INSTALL_DESTINATION lib/cmake/ORB_SLAM)
write_basic_package_version_file (orb_slam-config-version.cmake
  VERSION 1.0 COMPATIBILITY SameMajorVersion)

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
