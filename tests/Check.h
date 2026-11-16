// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ORB_SLAM_TESTS_CHECK_H
#define ORB_SLAM_TESTS_CHECK_H

#include <cstdio>
#include <cstdlib>
#include <print>
#include <string_view>

namespace ORB_SLAM::tests {

//! Number of checks that failed so far.
inline int failures = 0;

//! Reports @p what as a failure unless @p condition holds.
inline void check(bool condition, std::string_view what)
{
    if (!condition) {
        std::println(stderr, "FAIL: {}", what);
        ++failures;
    }
}

//! Returns the exit status of a test program from the failed checks.
inline int exitStatus()
{
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

} // namespace ORB_SLAM::tests

#endif // ORB_SLAM_TESTS_CHECK_H
