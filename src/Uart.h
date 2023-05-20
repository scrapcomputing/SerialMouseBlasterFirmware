//-*- C++ -*-
//
// The Pico's UART.
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

#ifndef __SRC_UART_H__
#define __SRC_UART_H__

#include "Utils.h"
#include <config.h>
#include <hardware/uart.h>
#include <vector>

class Uart {
  /// The UART instance (either uart0 or uart1).
  uart_inst *UInst = nullptr;
  /// The baud rate actually set, which may differe from the requested.
  uint32_t ActualBaudrate = 0;

public:
  enum class Instance {
    Uart0,
    Uart1,
  };
  enum class Parity {
    None,
    Even,
    Odd,
  };
  static uart_parity_t GetParity(Parity Parity) {
    switch (Parity) {
    case Parity::None:
      return UART_PARITY_NONE;
    case Parity::Even:
      return UART_PARITY_EVEN;
    case Parity::Odd:
      return UART_PARITY_ODD;
    }
    unreachable("Bad Parity");
  }
  static const char *GetParityStr(Parity Parity) {
    switch (Parity) {
    case Parity::None:
      return "UART_PARITY_NONE";
    case Parity::Even:
      return "UART_PARITY_EVEN";
    case Parity::Odd:
      return "UART_PARITY_ODD";
    }
    unreachable("Bad Parity");
  }
  Uart(Instance UartInstance, uint32_t RequestedBaudrate, uint32_t DataBits,
       uint32_t StopBits, Parity Parity, bool FlowControl) {
    UInst = UartInstance == Instance::Uart0 ? uart0 : uart1;
    ActualBaudrate = uart_init(UInst, RequestedBaudrate);
    uart_set_hw_flow(UInst, /*cts=*/FlowControl, /*rts=*/FlowControl);
    uart_set_format(UInst, /*data_bits=*/DataBits, /*stop_bits=*/StopBits,
                    /*parity=*/GetParity(Parity));
    uart_set_fifo_enabled(UInst, false);
#if ENABLE_SERIAL_DBG == 1
    std::cerr << "Actual Baudrate=" << ActualBaudrate << "\n";
    std::cerr << "DataBits=" << DataBits << "\n";
    std::cerr << "StopBits=" << StopBits << "\n";
    std::cerr << "Parity=" << GetParityStr(Parity) << "\n";
    std::cerr << "FlowControl=" << FlowControl << "\n";
#endif
  }
  ~Uart() { uart_deinit(UInst); }
  void write(uint8_t Byte) { uart_write_blocking(UInst, &Byte, 1); }
  void write(const std::vector<uint8_t> &Bytes) {
    uart_write_blocking(UInst, Bytes.data(), Bytes.size());
  }
  void sync() { uart_tx_wait_blocking(UInst); }
};


#endif // __SRC_UART_H__
