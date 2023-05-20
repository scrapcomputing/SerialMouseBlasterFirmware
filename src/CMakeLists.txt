cmake_minimum_required(VERSION 3.13)

# For a debug build
# -----------------
# $ mkdir build && cd build && cmake -DENABLE_SERIAL_DBG=1 ../src/ && make -j
#
# This will place the firmware into: build/SerialMouseBlasterFirmware.uf2
#
# This build uses USB CNC instead of the Pico's UART pins.
# This means that as long as the Pico is connected to your PC via USB
# you can connect to it using /dev/ttyACM0.
# For example: minicom -b 115200 -D /dev/ttyACM0. Serial connection: 115200 8N1


set(PICO_SDK_PATH "$ENV{HOME}/pico-sdk/")
# set(PICO_EXTRAS_PATH "$ENV{HOME}/pico-extras/")

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)
# include(pico_extras_import.cmake)

project(SerialMouseBlasterFirmware)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()
include_directories("${PICO_SDK_PATH}/src/common/pico_stdlib/include")
include_directories("${PICO_SDK_PATH}/src/common/pico_base/include")
include_directories("${PICO_SDK_PATH}/src/rp2_common/hardware_rtc/include")
# include_directories("${PICO_EXTRAS_PATH}/src/rp2_common/pico_sleep/include")
include_directories("${PROJECT_BINARY_DIR}/") # for build/config.h

set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Werror ${EXTRA_DBG_FLAGS}")

# rest of your project
file(GLOB SOURCES *.c *.cpp *.h *.def)
add_executable(SerialMouseBlasterFirmware ${SOURCES})

target_link_libraries(SerialMouseBlasterFirmware
  pico_stdlib
  tinyusb_host
  tinyusb_board
  tinyusb_device
  )
  # hardware_sleep

target_include_directories(SerialMouseBlasterFirmware PUBLIC "${CMAKE_CURRENT_LIST_DIR}")

set_property(TARGET SerialMouseBlasterFirmware PROPERTY CXX_STANDARD 17)
set_property(TARGET SerialMouseBlasterFirmware PROPERTY CXX_STANDARD_REQUIRED ON)

# # Load program from RAM
# pico_set_binary_type(SerialMouseBlasterFirmware no_flash)

message("")
message("+---------------+")
message("| Configuration |")
message("+---------------+")

# Use -DENABLE_SERIAL_DBG=1 to enable debug messages in serial debugging
if (ENABLE_SERIAL_DBG STREQUAL "1")
  pico_enable_stdio_usb(SerialMouseBlasterFirmware 0)
  pico_enable_stdio_uart(SerialMouseBlasterFirmware 1)
else ()
  set(ENABLE_SERIAL_DBG 0)
  pico_enable_stdio_usb(SerialMouseBlasterFirmware 0)
  pico_enable_stdio_uart(SerialMouseBlasterFirmware 0)
endif ()
message("ENABLE_SERIAL_DBG = ${ENABLE_SERIAL_DBG}")

# Use -DPICO_FREQ to override the default frequency 133000
# *** Please note that not all frequency values are valid
#     Some good values: 125000 133000 225000 270000
#     0 sets it to 48MHz
if (NOT DEFINED PICO_FREQ)
  set(PICO_FREQ 0)
endif ()
message("PICO_FREQ = ${PICO_FREQ} (KHz)")

# Use -DPICO_VOLT to override the default voltage VREG_VOLTAGE_DEFAULT (1.10v)
# List: <pico-sdk>/src/rp2_common/hardware_vreg/include/hardware/vreg.h
#      Examples:  VREG_VOLTAGE_0_85 (0.85v)
#                 VREG_VOLTAGE_1_30 (1.30v)
if (NOT DEFINED PICO_VOLT)
  set(PICO_VOLT VREG_VOLTAGE_DEFAULT)
endif ()
message("PICO_VOLT = ${PICO_VOLT}")

# Use -DPICO_LED=0 to disable the use of Pico's LED. Constant ON LED consumes about 2mA.
if (NOT DEFINED PICO_LED)
  set (PICO_LED 1)
endif ()
message("PICO_LED = ${PICO_LED}")

# Use -DUSB_POLL_PERIOD to set the USB polling period in milliseconds.
# The default is 25ms which corresponds to 40Hz
if (NOT DEFINED USB_POLL_PERIOD)
  set(USB_POLL_PERIOD 25)
endif ()
message("USB_POLL_PERIOD = ${USB_POLL_PERIOD} (ms)")


# End of configuration
message("")

configure_file (
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_BINARY_DIR}/config.h"
  )

# Create map/bin/hex/uf2 in addition to ELF.
pico_add_extra_outputs(SerialMouseBlasterFirmware)
