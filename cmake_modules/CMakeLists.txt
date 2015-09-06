cmake_minimum_required (VERSION 3.0)
project (ORB_SLAM)

list (APPEND CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/../Thirdparty/g2o/cmake_modules)
list (APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

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

if (MSVC)
  target_compile_definitions (ORB_SLAM PRIVATE _SCL_SECURE_NO_WARNINGS)
endif (MSVC)

target_include_directories (ORB_SLAM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)

target_include_directories (ORB_SLAM SYSTEM PUBLIC ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries (ORB_SLAM PUBLIC ${Boost_LIBRARIES} g2o::solver_cholmod
  g2o::core g2o::types_sba g2o::types_slam3d g2o::types_sim3 ${OpenCV_LIBS})

if (TBB_FOUND)
  target_link_libraries (ORB_SLAM PUBLIC ${TBB_LIBRARIES})
endif (TBB_FOUND)

add_executable (run_ORB_SLAM
  ../src/run_ORB_SLAM.cc
)

target_link_libraries (run_ORB_SLAM PRIVATE ORB_SLAM)
