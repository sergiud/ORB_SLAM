// SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef ORB_SLAM_TESTS_STORAGE_H
#define ORB_SLAM_TESTS_STORAGE_H

#include <cstddef>
#include <cstring>
#include <span>

namespace ORB_SLAM::tests {

//! Byte pattern that differs from the zero of value initialization.
inline constexpr unsigned char storagePattern = 0xAB;

/**
 * Fills storage for an object with a pattern before the object is
 * constructed in it, so that members the constructor leaves uninitialized
 * show the pattern.
 *
 * Compilers remove stores to storage before an object's lifetime begins as
 * dead, so the fill calls through a pointer they cannot see through.
 */
inline void fillWithPattern(std::span<std::byte> storage)
{
    static void* (*volatile const fill)(void*, int, std::size_t) = std::memset;
    fill(storage.data(), storagePattern, storage.size());
}

} // namespace ORB_SLAM::tests

#endif // ORB_SLAM_TESTS_STORAGE_H
