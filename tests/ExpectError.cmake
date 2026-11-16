# SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
#
# SPDX-License-Identifier: GPL-3.0-or-later

# Runs PROGRAM with ARGUMENTS, which separates the arguments with |, and
# checks that it exits with status 1 and writes an error matching ERROR.
#
# cmake -DPROGRAM=<file> -DARGUMENTS=<arg>|<arg>... -DERROR=<regex>
#       -P ExpectError.cmake

string (REPLACE "|" ";" _arguments "${ARGUMENTS}")

execute_process (
  COMMAND "${PROGRAM}" ${_arguments}
  RESULT_VARIABLE _result
  OUTPUT_QUIET
  ERROR_VARIABLE _error)

if (NOT _result STREQUAL "1")
  message (FATAL_ERROR "exit status \"${_result}\", expected \"1\"\n${_error}")
endif (NOT _result STREQUAL "1")

if (NOT _error MATCHES "${ERROR}")
  message (FATAL_ERROR "error \"${_error}\", expected a match of \"${ERROR}\"")
endif (NOT _error MATCHES "${ERROR}")
