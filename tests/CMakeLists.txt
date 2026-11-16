# SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later

foreach (_test matcher optimizer sampling threads tracking)
  add_executable (test_${_test} test_${_test}.cc Check.h Frames.h Scene.h
    Storage.h)
  target_compile_features (test_${_test} PRIVATE cxx_std_23)
  target_link_libraries (test_${_test} PRIVATE ORB_SLAM)
endforeach (_test)

add_test (NAME ORB_SLAM/threads COMMAND test_threads)
# A worker that ignores the stop request never returns.
set_tests_properties (ORB_SLAM/threads PROPERTIES TIMEOUT 60)

add_test (NAME ORB_SLAM/matcher COMMAND test_matcher)
add_test (NAME ORB_SLAM/optimizer COMMAND test_optimizer)
add_test (NAME ORB_SLAM/sampling COMMAND test_sampling)
add_test (NAME ORB_SLAM/tracking COMMAND test_tracking)

# The vocabulary is read first and reported with its path, before the
# missing calibration and video end the run.
add_test (NAME ORB_SLAM/cli/path-with-spaces
  COMMAND run_ORB_SLAM --no-video
    --vocabulary "dir with spaces/vocabulary.txt"
    --calibration missing.yml
    --input missing.avi
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set_tests_properties (ORB_SLAM/cli/path-with-spaces PROPERTIES
  PASS_REGULAR_EXPRESSION "reading vocabulary \"dir with spaces/vocabulary.txt\""
  TIMEOUT 60)

# An invalid command line ends the program with an error message, not an
# uncaught exception.
foreach (_case
    "invalid-value|--features|abc|the argument \\('abc'\\) for option '--features' is invalid"
    "repeated-path|--vocabulary|first.txt|--vocabulary|second.txt|option '--vocabulary' cannot be specified more than once")
  string (REPLACE "|" ";" _fields "${_case}")
  list (POP_FRONT _fields _name)
  list (POP_BACK _fields _error)
  string (REPLACE ";" "|" _arguments "${_fields}")

  add_test (NAME ORB_SLAM/cli/${_name}
    COMMAND ${CMAKE_COMMAND}
      -DPROGRAM=$<TARGET_FILE:run_ORB_SLAM>
      "-DARGUMENTS=${_arguments}"
      "-DERROR=^error: ${_error}"
      -P ${CMAKE_CURRENT_SOURCE_DIR}/ExpectError.cmake)
  set_tests_properties (ORB_SLAM/cli/${_name} PROPERTIES TIMEOUT 60)
endforeach (_case)

add_executable (make_fixtures make_fixtures.cc)
target_compile_features (make_fixtures PRIVATE cxx_std_23)
target_link_libraries (make_fixtures PRIVATE opencv_core opencv_videoio)

add_test (NAME ORB_SLAM/cli/fixtures COMMAND make_fixtures
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set_tests_properties (ORB_SLAM/cli/fixtures PROPERTIES
  FIXTURES_SETUP black_video)

# The program stops its worker threads once the video ends. A thread that
# ignores the request keeps the program from exiting.
add_test (NAME ORB_SLAM/cli/stops-at-end
  COMMAND run_ORB_SLAM --no-video
    --vocabulary missing.txt
    --calibration calibration.yml
    --input black.avi
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set_tests_properties (ORB_SLAM/cli/stops-at-end PROPERTIES
  FIXTURES_REQUIRED black_video
  TIMEOUT 120)
