# SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later

# Runs PROGRAM with the name of a case that violates a precondition and
# checks that the program terminates with a status matching STATUS after
# reporting the failed check. The program writes the text that the failed
# check contains to the standard output before it runs the case.
#
# cmake -DPROGRAM=<file> -DCASE=<name> -DSTATUS=<regex>
#       -P ExpectCheckFailure.cmake

execute_process (
  COMMAND "${PROGRAM}" "${CASE}"
  RESULT_VARIABLE _result
  OUTPUT_VARIABLE _expected
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_VARIABLE _error)

if (NOT _result MATCHES "${STATUS}")
  message (FATAL_ERROR
    "exit status \"${_result}\", expected a match of \"${STATUS}\"\n${_error}")
endif (NOT _result MATCHES "${STATUS}")

string (FIND "${_error}" "Check failed: ${_expected}" _position)

if (_expected STREQUAL "" OR _position EQUAL -1)
  message (FATAL_ERROR
    "error \"${_error}\", expected \"Check failed: ${_expected}\"")
endif (_expected STREQUAL "" OR _position EQUAL -1)
