cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(mpu_motion_driver_example)

pico_sdk_init()

add_executable(mpu_motion_driver_example
    mpu_example.c
)

add_subdirectory(libraries)
# Add pico_stdlib library, add more if used
target_link_libraries(mpu_motion_driver_example pico_stdlib pico_mpu6050 pico_mpu_dmp pico_float)

# enable usb output, disable uart output
pico_enable_stdio_usb(mpu_motion_driver_example 1)
pico_enable_stdio_uart(mpu_motion_driver_example 0)

# Need to generate UF2 file for upload to RP2040
pico_add_extra_outputs(mpu_motion_driver_example)