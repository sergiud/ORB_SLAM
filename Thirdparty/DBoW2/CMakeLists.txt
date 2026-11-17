# SPDX-FileCopyrightText: 2015 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
# SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
#
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required (VERSION 3.28...4.5)
project (DBoW2
  VERSION 1.0.0
  LANGUAGES CXX
)

find_package (OpenCV NO_MODULE REQUIRED core)

include (CMakePackageConfigHelpers)
include (GenerateExportHeader)
include (GNUInstallDirs)

# An enclosing project decides on testing for both.
if (PROJECT_IS_TOP_LEVEL)
  include (CTest)
endif (PROJECT_IS_TOP_LEVEL)

add_subdirectory (DBoW2)

if (BUILD_TESTING)
  add_subdirectory (tests)
endif (BUILD_TESTING)
