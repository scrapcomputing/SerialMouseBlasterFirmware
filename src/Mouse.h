//-*- C++ -*-
//
// The main data structure that tracks the mouse state.
//
// Copyright (C) 2023 Scrap Computing
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

#ifndef __SRC_MOUSE_H__
#define __SRC_MOUSE_H__

#include "tusb.h"
#include <algorithm>
#include <limits>
#include <sstream>

/// Holds the mouse state as reported by TinyUSB.
class Mouse {
  /// X movement.
  int8_t X = 0;
  /// Y movment.
  int8_t Y = 0;
  /// Mouse wheel movement.
  int8_t Wheel = 0;
  /// Left mouse button.
  bool Left = false;
  /// Right mouse button.
  bool Right = false;
  /// Middle mouse button.
  bool Middle = false;

public:
  /// Creates an empty, non-ready mouse state.
  Mouse() = default;
  /// Read TinyUSB's \p Report and populate state variables.
  void import(const hid_mouse_report_t *Report) {
    X = std::clamp(Report->x, std::numeric_limits<int8_t>::min(),
                   std::numeric_limits<int8_t>::max());
    Y = std::clamp(Report->y, std::numeric_limits<int8_t>::min(),
                   std::numeric_limits<int8_t>::max());
    Wheel = std::clamp(Report->wheel, std::numeric_limits<int8_t>::min(),
                       std::numeric_limits<int8_t>::max());
    Left = Report->buttons & MOUSE_BUTTON_LEFT;
    Right = Report->buttons & MOUSE_BUTTON_RIGHT;
    Middle = Report->buttons & MOUSE_BUTTON_MIDDLE;
  }
  /// The first byte for the Microsoft mouse: 01 LR Y[7:6] X[7:6]
  uint8_t getFirstByteMS() const {
    return (uint8_t)0b01000000 | ((uint8_t)Left << 5) | ((uint8_t)Right << 4) |
           (((uint8_t)Y & 0b11000000) >> 4) | (((uint8_t)X & 0b11000000) >> 6);
  }
  /// The second byte of the Microsft mouse: 00 X[5:0]
  uint8_t getSecondByteMS() const { return (uint8_t)X & 0b00111111; }
  /// The third byte of the Microsoft mouse: 00 Y[5:0]
  uint8_t getThirdByteMS() const { return (uint8_t)Y & 0b00111111; }
  /// Bit-wise dump of \p Byte.
  void dumpByte(uint8_t Byte) const {
    std::stringstream SS;
    for (int i = 7; i >= 0; --i)
      SS << ((Byte & 1 << i) == 0 ? '0' : '1');
    printf("%s ", SS.str().c_str());
  }
  /// Debug dump.
  void dump() const {
    printf("L:%d R:%d M:%d (X=%d Y=%d) Wheel=%d\n", Left, Right, Middle, X, Y,
           Wheel);
    dumpByte(getFirstByteMS());
    dumpByte(getSecondByteMS());
    dumpByte(getThirdByteMS());
    printf("\n");
  }
};

#endif // __SRC_MOUSE_H__

