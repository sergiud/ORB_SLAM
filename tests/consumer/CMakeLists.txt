# SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later

# Builds a program against the installed ORB-SLAM package the way other
# projects use it.

cmake_minimum_required (VERSION 3.28...4.5)
# The SuiteSparse of g2o finds OpenMP for C as well.
project (consumer LANGUAGES C CXX)

find_package (orb_slam 2.0 REQUIRED)

include (CTest)

add_executable (consumer main.cc)

target_compile_features (consumer PRIVATE cxx_std_23)
# libstdc++ provides the output of std::print to the Windows console in a
# library of its own.
target_link_libraries (consumer PRIVATE ORB_SLAM::ORB_SLAM
  $<$<AND:$<PLATFORM_ID:Windows>,$<CXX_COMPILER_ID:GNU>>:stdc++exp>)

add_test (NAME consumer COMMAND consumer)
