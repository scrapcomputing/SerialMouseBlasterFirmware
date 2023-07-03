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
#include "hardware/pll.h"
#include "hardware/structs/rosc.h"
#include "hardware/structs/scb.h"
#include <iomanip>

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

static uint32_t my_set_sys_clock(uint32_t vco_freq, uint32_t post_div1,
                             uint32_t post_div2) {
  clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48 * MHZ,
                  48 * MHZ);
  pll_init(pll_sys, 1, vco_freq, post_div1, post_div2);
  uint32_t freq = vco_freq / (post_div1 * post_div2);
  // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
  clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                  CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, freq, freq);
  return freq;
}

// Thse rosc_* functions are taken from pico-extras.
inline static void rosc_clear_bad_write(void) {
    hw_clear_bits(&rosc_hw->status, ROSC_STATUS_BADWRITE_BITS);
}

inline static bool rosc_write_okay(void) {
    return !(rosc_hw->status & ROSC_STATUS_BADWRITE_BITS);
}

inline static void rosc_write(io_rw_32 *addr, uint32_t value) {
    rosc_clear_bad_write();
    assert(rosc_write_okay());
    *addr = value;
    assert(rosc_write_okay());
};

static void rosc_disable(void) {
  uint32_t tmp = rosc_hw->ctrl;
  tmp &= (~ROSC_CTRL_ENABLE_BITS);
  tmp |= (ROSC_CTRL_ENABLE_VALUE_DISABLE << ROSC_CTRL_ENABLE_LSB);
  rosc_write(&rosc_hw->ctrl, tmp);
  // // Wait for stable to go away
  // while (rosc_hw->status & ROSC_STATUS_STABLE_BITS)
  //   ;
}


void Pico::setFreqAndVoltage() {
  // This sets the system clocks/plls for the lowest power consumption.
  if (Frequency == 0) {
    // Stop unused clock generators. This saves about 0.1mA.
    clock_stop(clk_adc);
    clock_stop(clk_rtc);
    clock_stop(clk_gpout0);
    clock_stop(clk_gpout1);
    clock_stop(clk_gpout2);
    clock_stop(clk_gpout3);

    // Set PLL_USB to use a low VCO frequency to save power.
    // ./src/rp2_common/hardware_clocks/scripts/vcocalc.py -l --input 48 --vco-max 1000 48
    // Requested: 48.0 MHz
    // Achieved: 48.0 MHz
    // REFDIV: 1
    // FBDIV: 16 (VCO = 768.0 MHz)
    // PD1: 4
    // PD2: 4
    pll_init(pll_usb, 1, /*VCO=*/768 * MHZ, /*PD1=*/4, /*PD2=*/4);

    // The following code is based on code taken from `set_sys_clock_48mhz()`.

    // Set frequency to 48MHz by default. Lower than that breaks USB.

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

    // CLK peri from PLL_USB and run at 48MHz
    // NOTE: Running it off XOSC at 12MHz saves 0.2mA but UART does not
    // seem to work properly
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

    // Disabling rosc hangs the system, strange! I don't thing there is any
    // clock generator running off it.
    // rosc_disable();
    // rosc_write(&rosc_hw->dormant, ROSC_DORMANT_VALUE_DORMANT);
  }
  else
    // Some over/underclocking if needed.
    set_sys_clock_khz(Frequency, true);

  // Set the voltage if required.
  if (Voltage)
    vreg_set_voltage(*Voltage);
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
void Pico::lowpwrsleep_ms(uint32_t ms) {
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

  // // Connect CLK_SYS to USB PLL and use a divider to get 20MHz out of 48MHz
  // clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
  //                 CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48 * MHZ,
  //                 20 * MHZ);

  // clock_configure(clk_sys,
  //                 CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
  //                 CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
  //                 125 * MHZ,
  //                 125 * MHZ);


  // // Connect CLK_SYS to PLL_SYS which runs at 15MHz
  // clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
  //                 CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, 125 * MHZ,
  //                 15 * MHZ);

  // Check the frequency:
  // auto freq_peri = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_PERI);

  // 20MHz
  // set_sys_clock_pll(980 * MHZ, 7, 7);

// ./src/rp2_common/hardware_clocks/scripts/vcocalc.py -l --input 125 --vco-max 750 40
// Requested: 40.0 MHz
// Achieved: 41.666666666666664 MHz
// REFDIV: 3
// FBDIV: 18 (VCO = 750.0 MHz)
// PD1: 6
// PD2: 3
  // uint32_t freq = my_set_sys_clock(750 * MHZ, 6, 3);

  // Sleep
  sleep_ms(ms);

  // ./src/rp2_common/hardware_clocks/scripts/vcocalc.py -l --input 125 --vco-max 750 48
  // Requested: 48.0 MHz
  // Achieved: 46.875 MHz
  // REFDIV: 3
  // FBDIV: 18 (VCO = 750.0 MHz)
  // PD1: 4
  // PD2: 4
  // freq = my_set_sys_clock(750 * MHZ, 4, 4);

  // 48MHz
  // set_sys_clock_pll(1200 * MHZ, 5, 5);

  // // // Restore the CLK_SYS divider
  // clock_configure(clk_sys,
  //                 CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
  //                 CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
  //                 125 * MHZ,
  //                 48 * MHZ);

  // // Restore CLK_SYS from PLL_USB at 48MHz
  // clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
  //                 CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB, 48 * MHZ,
  //                 48 * MHZ);

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
