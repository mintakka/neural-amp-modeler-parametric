# Servo rig firmware (Pico SDK). Build for a Pico (default) or Pico 2:
#   cmake -B build [-DPICO_BOARD=pico2] && cmake --build build
# then drag build/servo_rig.uf2 onto the board in BOOTSEL mode.
cmake_minimum_required(VERSION 3.13)

if(NOT DEFINED PICO_SDK_PATH AND NOT DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{HOME}/Projects/pico/pico-sdk)
endif()
if(NOT DEFINED picotool_DIR AND EXISTS $ENV{HOME}/Projects/pico/picotool-install/picotool)
    set(picotool_DIR $ENV{HOME}/Projects/pico/picotool-install/picotool)
endif()
if(NOT DEFINED PICO_BOARD)
    set(PICO_BOARD pico)
endif()
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(servo_rig C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(servo_rig main.cpp)
target_link_libraries(servo_rig pico_stdlib pico_flash hardware_pwm hardware_clocks hardware_flash)
pico_enable_stdio_usb(servo_rig 1)
pico_enable_stdio_uart(servo_rig 0)
pico_add_extra_outputs(servo_rig)
