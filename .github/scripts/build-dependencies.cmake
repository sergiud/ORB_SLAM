# SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later

# Builds the dependencies that a CI runner lacks as packages and installs them
# into one prefix:
#
#   cmake -D PREFIX=<dir> -D BUILD_TYPE=<config> -D "PACKAGES=<name>;..."
#         -P build-dependencies.cmake
#
# PACKAGES names the dependencies in the order they are built, a dependency
# before the packages that use it. The CMAKE_GENERATOR, CMAKE_GENERATOR_PLATFORM
# and CMAKE_<LANG>_COMPILER_LAUNCHER environment variables choose the
# generator, the platform and the compiler launcher of every build. The caches
# of the workflows include the hash of this file in their keys, so that
# changing a version or an option here rebuilds the dependencies.

cmake_minimum_required (VERSION 3.28...4.5)

set (Boost_URL
  https://github.com/boostorg/boost/releases/download/boost-1.92.0/boost-1.92.0-cmake.tar.xz)
set (Boost_SHA256
  9bed76128d4e46755dbe818487788c6fceb6f72b378f4daa49b7e1e600d9088d)
set (Boost_OPTIONS
  -DBOOST_INCLUDE_LIBRARIES=program_options
  -DBUILD_TESTING=OFF
)

set (Eigen3_URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz)
set (Eigen3_SHA256
  e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec)
set (Eigen3_OPTIONS
  -DBUILD_TESTING=OFF
  -DCMAKE_Fortran_COMPILER=
  -DEIGEN_BUILD_BLAS=OFF
  -DEIGEN_BUILD_DOC=OFF
  -DEIGEN_BUILD_LAPACK=OFF
)

# The g2o releases name themselves 1.0.0 and changed the interface that
# ORB-SLAM uses, which this branch keeps.
set (g2o_URL
  https://github.com/sergiud/g2o/archive/5a979a9ec1f08b2ab6aa3308860eb466ca196bc8.tar.gz)
set (g2o_SHA256
  32d852ad4e0b0e18daab7961eeaacb0ab5b9e3bf5062d838a991934203e2f395)
# The bundled CSparse keeps the build independent of the sparse solvers a runner
# provides, since disabling it requires an installed CXSparse instead.
set (g2o_OPTIONS
  -DBUILD_APPS=OFF
  -DBUILD_CSPARSE=ON
  -DBUILD_EXAMPLES=OFF
  -DBUILD_TESTING=OFF
)

# The experimental interface of ng-log is not released yet.
set (ng-log_URL
  https://github.com/ng-log/ng-log/archive/bba6055e23921a3a8e13928e0be8dc3f49e2ceff.tar.gz)
set (ng-log_SHA256
  8013869a5ba580074d8292760ca7ea1a549dee7a0d0f16637106df6074280c22)
set (ng-log_OPTIONS
  -DBUILD_EXAMPLES=OFF
  -DBUILD_TESTING=OFF
  -DWITH_EXPERIMENTAL=ON
  -DWITH_MODULES=OFF
)

# The modules are those ORB-SLAM and its tests use and the ones they depend on.
# The bundled image libraries let the tests read back the MJPEG videos they
# write, whatever the runner provides.
set (OpenCV_URL https://github.com/opencv/opencv/archive/refs/tags/5.1.0.tar.gz)
set (OpenCV_SHA256
  638c1dae7e37e45afea21aa217dc667ad7cf57acc66cdf652c074f1e0e5ad6d2)
set (OpenCV_OPTIONS
  -DBUILD_JAVA=OFF
  -DBUILD_JPEG=ON
  -DBUILD_LIST=calib,core,features,geometry,highgui,imgcodecs,imgproc,videoio
  -DBUILD_PERF_TESTS=OFF
  -DBUILD_PNG=ON
  -DBUILD_TESTS=OFF
  -DBUILD_ZLIB=ON
  -DBUILD_opencv_apps=OFF
  -DOPENCV_INSTALL_BINARIES_PREFIX=
  -DWITH_FFMPEG=OFF
  -DWITH_GSTREAMER=OFF
  -DWITH_IPP=OFF
  -DWITH_ITT=OFF
  -DWITH_OPENCL=OFF
  -DWITH_OPENEXR=OFF
  -DWITH_OPENJPEG=OFF
  -DWITH_PROTOBUF=OFF
  -DWITH_TIFF=OFF
  -DWITH_WEBP=OFF
)

foreach (_variable IN ITEMS PREFIX BUILD_TYPE PACKAGES)
  if (NOT ${_variable})
    message (FATAL_ERROR "${_variable} is empty but must be set with -D")
  endif (NOT ${_variable})
endforeach (_variable)

cmake_path (ABSOLUTE_PATH PREFIX NORMALIZE)
set (_work ${PREFIX}-work)

# An executable searches its run path only for the libraries it links
# directly, so an installed library looks for the libraries it loads, such as
# g2o_core for g2o_stuff, next to itself.
if (CMAKE_HOST_APPLE)
  set (_rpath -DCMAKE_INSTALL_RPATH=@loader_path)
elseif (CMAKE_HOST_UNIX)
  set (_rpath -DCMAKE_INSTALL_RPATH=$ORIGIN)
endif (CMAKE_HOST_APPLE)

function (run)
  execute_process (COMMAND ${ARGN} COMMAND_ECHO STDOUT COMMAND_ERROR_IS_FATAL ANY)
endfunction (run)

foreach (_package IN LISTS PACKAGES)
  if (NOT DEFINED ${_package}_URL)
    message (FATAL_ERROR "Unknown package '${_package}'")
  endif (NOT DEFINED ${_package}_URL)

  set (_archive ${_work}/${_package}.archive)
  file (DOWNLOAD ${${_package}_URL} ${_archive}
    EXPECTED_HASH SHA256=${${_package}_SHA256}
    SHOW_PROGRESS
    TLS_VERIFY ON
  )

  set (_source ${_work}/${_package}-source)
  file (REMOVE_RECURSE ${_source})
  file (ARCHIVE_EXTRACT INPUT ${_archive} DESTINATION ${_source})
  # Each archive holds a single directory named after the release.
  file (GLOB _root LIST_DIRECTORIES ON ${_source}/*)

  set (_build ${_work}/${_package}-build)
  run (${CMAKE_COMMAND} -S ${_root} -B ${_build}
    -DCMAKE_BUILD_TYPE=${BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${PREFIX}
    -DCMAKE_PREFIX_PATH=${PREFIX}
    ${_rpath}
    ${${_package}_OPTIONS}
  )
  run (${CMAKE_COMMAND} --build ${_build} --config ${BUILD_TYPE} --parallel)
  run (${CMAKE_COMMAND} --install ${_build} --config ${BUILD_TYPE})
endforeach (_package)

file (REMOVE_RECURSE ${_work})
