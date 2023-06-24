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

#ifndef __SRC_PICO_H__
#define __SRC_PICO_H__

#include "Uart.h"
#include "Utils.h"
#include "hardware/clocks.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include <iostream>
#include <memory>
#include <optional>
#include <map>

/// A range of GPIO pins. This is useful because it also creates the
/// corresponding mask which is used for setting and resetting pins.
class PinRange {
  uint32_t From = 0;
  uint32_t To = 0;
  uint32_t Mask = 0;

public:
  /// \p From is the first pin and \p To is the last pin in the reange.
  PinRange(uint32_t From, uint32_t To);
  /// Use this constructor to create a single pin.
  PinRange(uint32_t Pin) : PinRange(Pin, Pin) {}
  uint32_t getFrom() const { return From; }
  uint32_t getTo() const { return To; }
  uint32_t getMask() const { return Mask; }
  void dump(std::ostream &OS) const;
  DUMP_METHOD void dump() const;
};

class Pico {
  /// The GPIO pin used to wake up from deep sleep.
  std::optional<uint32_t> WakeUpGPIO;
  uint32_t Frequency;
  std::optional<vreg_voltage> Voltage;
  std::unique_ptr<Uart> Uart0;
  std::optional<uint32_t> Uart0GPIO;
  std::unique_ptr<Uart> Uart1;
  std::optional<uint32_t> Uart1GPIO;

  void setFreqAndVoltage();
  void wakeUpUarts();

public:
  // Some valid frequencies: 225000, 250000, 270000, 280000, 290400
  // Voltages: <pico-sdk>/src/rp2_common/hardware_vreg/include/hardware/vreg.h
  //           Examples:  VREG_VOLTAGE_0_85 0.85v
  //                      VREG_VOLTAGE_1_30 1.30v
  Pico(std::optional<uint32_t> WakeUpGPIO, uint32_t Frequency = 125000,
       std::optional<vreg_voltage> Voltage = std::nullopt);
  /// Sleep for \p ms but also lower frequencies to use less power.
  /// NOTE: This is for testing and is not currently in use.
  void lowpwrsleep_ms(uint32_t ms);
  /// Create a uart instance owned by the Pico object and returns a raw pointer
  /// to it.
  Uart *createUart(Uart::Instance Instance, uint32_t UartGPIO,
                   uint32_t RequestedBaudrate, uint32_t DataBits,
                   uint32_t StopBits, Uart::Parity Parity, bool FlowControl);
  /// \Returns the uart that has been created for \p Instance, or null.
  Uart *getUart(Uart::Instance Instance) {
    switch(Instance) {
    case Uart::Instance::Uart0:
      return Uart0.get();
    case Uart::Instance::Uart1:
      return Uart1.get();
    }
  }
  /// Sets \p Pins to GPIO_OUT.
  inline void setGPIODirectionOut(const PinRange &Pins) {
    gpio_set_dir_out_masked(Pins.getMask());
  }
  /// Sets \p Pins to GPIO_IN.
  inline void setGPIODirectionIn(const PinRange &Pins) {
    gpio_set_dir_in_masked(Pins.getMask());
  }
  /// Sets \p Pins to \p Value.
  inline void setGPIOBits(const PinRange &Pins, uint32_t Value) {
    gpio_put_masked(Pins.getMask(), Value << Pins.getFrom());
  }
  /// Direction can be GPIO_IN GPIO_OUT.
  void initGPIO(const PinRange &Pins, int Direction, const char *Descr);
  /// \Returns the status of all GPIO pins.
  inline uint32_t readGPIO() const { return gpio_get_all(); }
  /// \Returns the state of \p GPIO.
  inline bool getGPIO(uint32_t GPIO) const { return gpio_get(GPIO); }
  /// Sets \p GPIO to \p Value.
  inline void setGPIO(uint32_t GPIO, bool Value) { gpio_put(GPIO, Value); }
  void ledSet(bool State) { gpio_put(PICO_DEFAULT_LED_PIN, State); }
  void ledON() { gpio_put(PICO_DEFAULT_LED_PIN, 1); }
  void ledOFF() { gpio_put(PICO_DEFAULT_LED_PIN, 0); }
  inline void clear(uint32_t Mask) { gpio_clr_mask(Mask); }
  inline void set(uint32_t Mask) { gpio_set_mask(Mask); }
  // void sleep();
};

#endif // __SRC_PICO_H__

