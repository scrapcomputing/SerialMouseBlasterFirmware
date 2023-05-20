//-*- C++ -*-
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

#include "Mouse.h"
#include "Pico.h"
#include "Uart.h"
#include "bsp/board.h"
#include "tusb.h"
#include <chrono>
#include <config.h>
#include <hardware/watchdog.h>

static constexpr uint32_t RTS = 6;

static Mouse MouseState;

/// A simple guard class that makes sure we call tuh_hid_receive_report()
class TuhHidReceiveReportGuard {
  uint8_t DevAddr;
  uint8_t Instance;

public:
  TuhHidReceiveReportGuard(uint8_t DevAddr, uint8_t Instance)
      : DevAddr(DevAddr), Instance(Instance) {}
  ~TuhHidReceiveReportGuard() {
    bool ReportOK = tuh_hid_receive_report(DevAddr, Instance);
    if (!ReportOK)
      std::cerr << "Error in receive report\n";
  }
};

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance,
                      uint8_t const *descr_report, uint16_t desc_len) {
  TuhHidReceiveReportGuard ReceiveReportGuard(dev_addr, instance);
  if (tuh_hid_interface_protocol(dev_addr, instance) == HID_ITF_PROTOCOL_MOUSE)
    std::cerr << "Mouse mount!\n";
}

void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  if (tuh_hid_interface_protocol(dev_addr, instance))
    std::cerr << "Mouse umount\n";
}

static Pico *PicoPtr = nullptr;
static Uart *UartPtr = nullptr;

static void initMouse() {
#if ENABLE_SERIAL_DBG == 1
  std::cerr << "initMouse()\n";
#endif
  // Wait 14ms until we send the data.
  // Send 'M'
  sleep_ms(14);
  UartPtr->write('M');
#if ENABLE_SERIAL_DBG == 1
  std::cerr << "Sent 'M'\n";
#endif
}

void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance,
                                const uint8_t *report, uint16_t len) {
  TuhHidReceiveReportGuard ReceiveReportGuard(dev_addr, instance);
  if (tuh_hid_interface_protocol(dev_addr, instance) ==
      HID_ITF_PROTOCOL_MOUSE) {
    // Populate MouseState with the new mouse state.
    MouseState.import(reinterpret_cast<const hid_mouse_report_t *>(report));
#if ENABLE_SERIAL_DBG == 1
    printf("mouse data: ");
    MouseState.dump();
#endif

    // Send data
    UartPtr->write(MouseState.getFirstByteMS());
    UartPtr->write(MouseState.getSecondByteMS());
    UartPtr->write(MouseState.getThirdByteMS());
#if ENABLE_SERIAL_DBG == 1
    std::cerr << "Sent packets\n";
#endif

    // Blink LED to show activity.
#if PICO_LED == 1
    PicoPtr->ledON();
    // Keep this short to save energy.
    sleep_ns(50);
    PicoPtr->ledOFF();
#endif
  }
}

static void main_loop(Pico &Pico) {
  UartPtr->write('M');
  // TinyUSB seems to be flaky so enable watchdog.
  // We must have updated the watchdong within 1000ms.
  watchdog_enable(1000, 1);

  auto LastTime = std::chrono::high_resolution_clock::now();

  uint32_t LedCnt = 0;
  while (true) {
    // Check if PC asked for mouse reset.
    // According to Linux Programmer's Manual, man mouse (4):
    // The mouse driver drops RTS to low and raises it again.
    // About 14ms later the mouse sends 0x4d ('M') on the data line.
    // After a further 63ms a Microsoft-compatible 3-button mouse
    // will send 0x33 ('3').
    static bool LastRTS = false;
    bool NewRTS = Pico.getGPIO(RTS);
#if ENABLE_SERIAL_DBG == 1
    if (NewRTS != LastRTS)
      std::cerr << "RTS = " << NewRTS << "\n";
#endif
    if (NewRTS && !LastRTS) {
      initMouse();
#if ENABLE_SERIAL_DBG == 1
      std::cerr << "RTS 0->1 detected\n";
#endif
    }
    LastRTS = NewRTS;

#if PICO_LED == 1
    // LED pulse to show that we are alive. Blinks about every second.
    if (LedCnt++ % 40 == 0) {
      Pico.ledON();
      // Keep this short to save energy.
      sleep_ns(250);
      Pico.ledOFF();
    }
#endif
    sleep_ms(USB_POLL_PERIOD);
    // Update the watchdog to notify it that we are alive.
    watchdog_update();
    // Ask tinyUSB for data.
    tuh_task();
  }
}

// Hack for linking error: undefined reference to `__dso_handle'
static void *__dso_handle = 0;

int main() {
  (void)__dso_handle;
  Pico Pico(/*WakeUpGPIO=*/2, PICO_FREQ, PICO_VOLT);
  Pico.initGPIO(RTS, GPIO_IN, "RTS");

  PicoPtr = &Pico;
  tusb_init();
  // GPIO 4,5 for UART1.
  UartPtr = Pico.createUart(Uart::Instance::Uart1, /*UartGPIO=*/4,
                            /*RequestedBaudrate=*/1200,
                            /*DataBits=*/7,
                            /*StopBits=*/1, /*Parity=*/Uart::Parity::None,
                            /*FlowControl=*/false);
  main_loop(Pico);
  return 0;
}
