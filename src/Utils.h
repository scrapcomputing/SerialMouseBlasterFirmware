//-*- C++ -*-
//
// Copyright (C) 2023 Scarp Computing
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2, or (at your option) any later
// version.
// GCC is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
// You should have received a copy of the GNU General Public License
// along with GCC; see the file LICENSE.  If not see
// <http://www.gnu.org/licenses/>.

#ifndef __SRC_UTILS_H__
#define __SRC_UTILS_H__

#include <cstdint>
#include <iostream>

#define DUMP_METHOD __attribute__((noinline)) __attribute__((__used__))

static inline void sleep_ns(uint32_t ns) {
  for (int i = 0, e = ns / 8; i != e; ++i)
    asm volatile("nop\n"); /* 8ns each 1 cycle @125MHz */
}

template <typename T>
__attribute__((noreturn)) static inline void dieBase(const T &Val) {
  std::cerr << Val << "\n";
  exit(1);
}

/// Print arguments and exit with exit code 1.
template <typename T, typename... Ts>
__attribute__((noreturn)) static inline void dieBase(const T &Val1,
                                                     const Ts... Vals) {
  std::cerr << Val1;
  dieBase(Vals...);
}

/// Exit the program, reporting a bug.
#define die(...)                                                               \
  dieBase("Error in ", __FILE__, ":", __LINE__, " ", __FUNCTION__,             \
          "(): ", __VA_ARGS__, "\n")

#define unreachable(...)                        \
  dieBase("Unreachable in ", __FILE__, ":", __LINE__, " ", __FUNCTION__, \
          "(): ", __VA_ARGS__, "\n")

#endif // __SRC_UTILS_H__
