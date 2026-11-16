# SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later

foreach (_test frame initializer keyframe mappoint matcher optimizer pnpsolver sampling sim3solver threads
    tracking)
  add_executable (test_${_test} test_${_test}.cc Check.h Frames.h Scene.h
    Storage.h)
  target_compile_features (test_${_test} PRIVATE cxx_std_23)
  target_link_libraries (test_${_test} PRIVATE ORB_SLAM)
endforeach (_test)

add_test (NAME ORB_SLAM/threads COMMAND test_threads)
# A worker that ignores the stop request never returns.
set_tests_properties (ORB_SLAM/threads PROPERTIES TIMEOUT 60)

add_test (NAME ORB_SLAM/frame COMMAND test_frame)
add_test (NAME ORB_SLAM/initializer COMMAND test_initializer)
add_test (NAME ORB_SLAM/keyframe COMMAND test_keyframe)
# Keyframes erased at the same time that deadlock never finish.
set_tests_properties (ORB_SLAM/keyframe PROPERTIES TIMEOUT 120)
add_test (NAME ORB_SLAM/mappoint COMMAND test_mappoint)
add_test (NAME ORB_SLAM/matcher COMMAND test_matcher)
add_test (NAME ORB_SLAM/optimizer COMMAND test_optimizer)
add_test (NAME ORB_SLAM/pnpsolver COMMAND test_pnpsolver)
add_test (NAME ORB_SLAM/sampling COMMAND test_sampling)
add_test (NAME ORB_SLAM/sim3solver COMMAND test_sim3solver)
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
target_link_libraries (make_fixtures PRIVATE ORB_SLAM opencv_core opencv_videoio)

add_test (NAME ORB_SLAM/cli/fixtures COMMAND make_fixtures
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set_tests_properties (ORB_SLAM/cli/fixtures PROPERTIES
  FIXTURES_SETUP black_video)

# The program stops its worker threads once the video ends. A thread that
# ignores the request keeps the program from exiting.
add_test (NAME ORB_SLAM/cli/stops-at-end
  COMMAND run_ORB_SLAM --no-video
    --vocabulary vocabulary.txt
    --calibration calibration.yml
    --input black.avi
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set_tests_properties (ORB_SLAM/cli/stops-at-end PROPERTIES
  FIXTURES_REQUIRED black_video
  TIMEOUT 120)

# Inputs that cannot be read end the program with an error that names them,
# rather than with a crash or a run on an empty vocabulary.
foreach (_case
    "missing-vocabulary|--vocabulary|missing.txt|--calibration|calibration.yml|--input|black.avi|cannot read vocabulary \"missing.txt\""
    "missing-calibration|--vocabulary|vocabulary.txt|--calibration|missing.yml|--input|black.avi|cannot read calibration \"missing.yml\""
    "uncalibrated|--vocabulary|vocabulary.txt|--calibration|uncalibrated.yml|--input|black.avi|camera intrinsic matrix must be 3x3, got 0x0")
  string (REPLACE "|" ";" _fields "${_case}")
  list (POP_FRONT _fields _name)
  list (POP_BACK _fields _error)
  string (REPLACE ";" "|" _arguments "${_fields}")

  add_test (NAME ORB_SLAM/cli/${_name}
    COMMAND ${CMAKE_COMMAND}
      -DPROGRAM=$<TARGET_FILE:run_ORB_SLAM>
      "-DARGUMENTS=--no-video|${_arguments}"
      # OpenCV and DBoW2 report the failure on their own first.
      "-DERROR=error: ${_error}"
      -P ${CMAKE_CURRENT_SOURCE_DIR}/ExpectError.cmake
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  set_tests_properties (ORB_SLAM/cli/${_name} PROPERTIES
    FIXTURES_REQUIRED black_video
    TIMEOUT 60)
endforeach (_case)
