# SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
#
# SPDX-License-Identifier: BSD-3-Clause

include (CheckCXXCompilerFlag)

add_executable (test_vocabulary test_vocabulary.cpp)
target_link_libraries (test_vocabulary PRIVATE DBoW2)

# Hardened builds initialize local variables with zero. The test does the
# same, so that a read of an uninitialized value behaves the same in every
# run.
check_cxx_compiler_flag (-ftrivial-auto-var-init=zero
  HAVE_TRIVIAL_AUTO_VAR_INIT_ZERO)

if (HAVE_TRIVIAL_AUTO_VAR_INIT_ZERO)
  target_compile_options (test_vocabulary PRIVATE
    -ftrivial-auto-var-init=zero)
endif (HAVE_TRIVIAL_AUTO_VAR_INIT_ZERO)

add_test (NAME DBoW2/vocabulary COMMAND test_vocabulary
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
# A loader that waits for the end of a stream that never ends hangs instead
# of failing.
set_tests_properties (DBoW2/vocabulary PROPERTIES TIMEOUT 30)
