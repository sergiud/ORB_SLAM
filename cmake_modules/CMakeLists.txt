cmake_minimum_required (VERSION 2.8)
project (ORB_SLAM)

list (APPEND CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/g2o/cmake_modules)

find_package (Boost 1.55 REQUIRED chrono filesystem program_options thread system)
find_package (Cholmod REQUIRED)
find_package (G2O REQUIRED)
find_package (OpenCV 2.4 REQUIRED core calib3d imgproc highgui features2d)
find_package (PkgConfig)

if (PKGCONFIG_FOUND)
  pkg_check_modules (EIGEN3 REQUIRED eigen3>=3.2)
endif (PKGCONFIG_FOUND)

add_library (ORB_SLAM
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

find_library (DBOW2_LIBRARY DBoW2 DOC "DBoW2 library")
mark_as_advanced (DBOW2_LIBRARY)

add_library (DBoW2 UNKNOWN IMPORTED)

set_target_properties (DBoW2 PROPERTIES IMPORTED_LOCATION ${DBOW2_LIBRARY})

target_include_directories (ORB_SLAM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)

target_include_directories (ORB_SLAM SYSTEM PUBLIC ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} ${G2O_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS}
  ${CHOLMOD_INCLUDE_DIR})
target_link_libraries (ORB_SLAM PUBLIC ${Boost_LIBRARIES} ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${G2O_TYPES_SIM3}
  ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SBA} ${OpenCV_LIBS} DBoW2
  ${CHOLMOD_LIBRARIES})

set_target_properties (ORB_SLAM PROPERTIES CXX_STANDARD 11
  CXX_STANDARD_REQUIRED ON)

add_executable (run_ORB_SLAM
  ../src/run_ORB_SLAM.cc
)

target_link_libraries (run_ORB_SLAM PRIVATE ORB_SLAM)
