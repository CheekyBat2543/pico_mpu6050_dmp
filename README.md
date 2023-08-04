# RP Pico MPU6050 Driver Library

RP Pico port of Invensense's MPU driver.
This library port is still a work in progress and interrupt related functionalities are still not tested

# How To Add to Your Project:

-> Copy eMPL folder into your source folder.

-> Add the subdirctory to your CMakeLists.txt file with "add_subdirectory(eMPL)".

-> Add "pico_mpu6050" and "pico_mpu_dmp" to your "target_link_libraries".
