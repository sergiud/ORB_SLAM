# SPDX-FileCopyrightText: 2015 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
# SPDX-FileCopyrightText: 2015-2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
#
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required (VERSION 3.15)
project (DBoW2)

set (MAJOR_VERSION 1)
set (MINOR_VERSION 0)
set (VERSION_PATCH 0)

set (SHORT_VERSION "${MAJOR_VERSION}.${MINOR_VERSION}")

if (NOT VERSION_PATCH EQUAL 0)
  set (SHORT_VERSION "${SHORT_VERSION}.${VERSION_PATCH}")
endif (NOT VERSION_PATCH EQUAL 0)

set (VERSION "${MAJOR_VERSION}.${MINOR_VERSION}.${VERSION_PATCH}")

include (CMakePackageConfigHelpers)
include (GenerateExportHeader)

add_subdirectory (DBoW2)

if (BUILD_TESTING)
  add_subdirectory (tests)
endif (BUILD_TESTING)
