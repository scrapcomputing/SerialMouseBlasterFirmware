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

#include "Pico.h"
#include "Utils.h"
#include "hardware/structs/scb.h"
#include <iomanip>
#include "hardware/pll.h"

#ifndef NDEBUG
static uint32_t AlreadySetMask = 0u;
#endif // NDEBUG

PinRange::PinRange(uint32_t From, uint32_t To) : From(From), To(To) {
  Mask = ((1u << (To + 1 - From)) - 1) << From;
#ifndef NDEBUG
  if ((AlreadySetMask & Mask) != 0u)
    die("Some pins in range ", From, " to ", To, " are already set!");
  AlreadySetMask |= Mask;
#endif // NDEBUG
}

void PinRange::dump(std::ostream &OS) const {
  auto Flags = OS.flags();
  if (From == To)
    OS << std::setw(5) << std::setfill(' ') << From;
  else
    OS << std::setw(2) << From << "-" << std::setw(2) << To;
  OS << " Mask=";
  OS << "0x" << std::setw(8) << std::setfill('0') << std::hex << Mask;
  OS.flags(Flags);
}

void PinRange::dump() const { dump(std::cerr); }

void Pico::setFreqAndVoltage() {
  // Set the voltage if required.
  if (Voltage)
    vreg_set_voltage(*Voltage);
  if (Frequency == 0) {
    clock_stop(clk_adc);
    clock_stop(clk_rtc);
    // rosc_disable();

    // The following code is taken from `set_sys_clock_48mhz()`.
    // I am inlining it here in case we tune it even more.

    // Set frequency to 48MHz by default. Lower than breaks USB.

    // CLK_SYS from PLL_USB
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // CLK_REF from XOSC
    clock_configure(clk_ref,
                    CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC,
                    0, // No aux mux
                    12 * MHZ,
                    12 * MHZ);

    // CLK peri from PLL_USB
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);
    // CLK usb from PLL_USB
    clock_configure(clk_usb,
                    0,
                    CLOCKS_CLK_USB_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    48 * MHZ,
                    48 * MHZ);

    // We don't need PLL_SYS anymore
    pll_deinit(pll_sys);
  }
  else
    // Some over/underclocking if needed.
    set_sys_clock_khz(Frequency, true);
}

Pico::Pico(std::optional<uint32_t> WakeUpGPIO, uint32_t Frequency,
           std::optional<vreg_voltage> Voltage)
    : WakeUpGPIO(WakeUpGPIO), Frequency(Frequency), Voltage(Voltage) {
  setFreqAndVoltage();
  // Initialize stdio so that we can print debug messages.
  stdio_init_all();
  // Initialize the Pico LED.
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  // Initialize pin used to wake up from deep sleep.
  if (WakeUpGPIO) {
    initGPIO(PinRange(*WakeUpGPIO), GPIO_IN, "WakeUp");
  }
  // Wait for a bit otherwise this does not show up during serial debug.
  sleep_ms(500);
  std::cerr << "+---------------------------------+\n";
  std::cerr << "|        SerialMouseBlaster       |\n";
  std::cerr << "+---------------------------------+\n";
  std::cerr << "clk_sys = " << frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS)
            << "KHz\n";
  std::cerr << "clk_usb = " << frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_USB)
            << "KHz\n";
  std::cerr << "clk_peri = " << frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI)
            << "KHz\n";
  std::cerr << "clk_ref = " << frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_REF)
            << "KHz\n";
  std::cerr << "clk_adc = " << frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_ADC)
            << "KHz\n";
  std::cerr << "clk_rtc = " << frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_RTC)
            << "KHz\n";
}

// WARNING: This is not working as expected! Do not use!
void Pico::lowPwrSleep_ms(uint32_t ms) {
  // Make sure we send/receive any outstanding data because when the frequencies
  // are lower, UART may not work properly.
  if (Uart0)
    Uart0->sync();
  if (Uart1)
    Uart1->sync();
  // // To get the PLL values use:
  // // src/rp2_common/hardware_clocks/scripts/vcocalc.py --input <INPUT> <OUTPUT>
  // // For example, for a 48MHz input and a 15MHzoutput: vocalc.py --input 48 15
  // uint vco_freq = 750 * MHZ;
  // uint post_div1 = 7;
  // uint post_div2 = 7;        // 15.3MHZ
  // // Set the USB PLL to 15.3MHz
  // pll_init(pll_usb, 1, vco_freq, post_div1, post_div2);

  // Connect CLK_SYS to USB PLL and use a divider to get 15MHz out of 48MHz
  clock_configure(clk_sys,
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                  48 * MHZ,
                  15 * MHZ);
  // Check the frequency:
  // auto freq_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);

  // Sleep
  sleep_ms(ms);

  // Restore the CLK_SYS divider
  clock_configure(clk_sys,
                  CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                  48 * MHZ,
                  48 * MHZ);

  // // Restore USB PLL to 48MHz
  // vco_freq = 1440 * MHZ;
  // post_div1 = 6;
  // post_div2 = 5;        // 48.0MHZ
  // pll_init(pll_usb, 1, vco_freq, post_div1, post_div2);
}

Uart *Pico::createUart(Uart::Instance Instance, uint32_t UartGPIO,
                       uint32_t RequestedBaudrate, uint32_t DataBits,
                       uint32_t StopBits, Uart::Parity Parity,
                       bool FlowControl) {
  // Initialize GPIO pins.
  gpio_set_function(UartGPIO, GPIO_FUNC_UART);
  gpio_set_function(UartGPIO + 1, GPIO_FUNC_UART);

  std::unique_ptr<Uart> *UartPtr;
  std::optional<uint32_t> *UartGPIOPtr;
  switch (Instance) {
  case Uart::Instance::Uart0:
    UartPtr = &Uart0;
    UartGPIOPtr = &Uart0GPIO;
    break;
  case Uart::Instance::Uart1:
    UartPtr = &Uart1;
    UartGPIOPtr = &Uart1GPIO;
    break;
  }
  *UartPtr = std::make_unique<Uart>(Instance, RequestedBaudrate, DataBits,
                                    StopBits, Parity, FlowControl);
  *UartGPIOPtr = UartGPIO;
  return UartPtr->get();
}

void Pico::wakeUpUarts() {
  if (Uart0) {
    gpio_set_function(*Uart0GPIO, GPIO_FUNC_UART);
    gpio_set_function(*Uart0GPIO + 1, GPIO_FUNC_UART);
  }
  if (Uart1) {
    gpio_set_function(*Uart1GPIO, GPIO_FUNC_UART);
    gpio_set_function(*Uart1GPIO + 1, GPIO_FUNC_UART);
  }
}

void Pico::initGPIO(const PinRange &Pins, int Direction, const char *Descr) {
  std::cout << "Setting up GPIO " << std::setw(5) << std::setfill(' ') << Descr
            << " ";
  Pins.dump(std::cout);
  std::cout << " " << (Direction == GPIO_IN ? "IN" : "OUT") << "\n";
  for (uint32_t Pin = Pins.getFrom(), E = Pins.getTo(); Pin <= E; ++Pin) {
    gpio_init(Pin);
    gpio_set_dir(Pin, Direction);
  }
}
