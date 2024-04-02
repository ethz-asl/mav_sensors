#[[
BSD 3-Clause License

Copyright (c) 2024, ETH Zurich, Autonomous Systems Lab, Mariano Biasio, Rik Girod

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
]]

cmake_minimum_required(VERSION 3.0.2)
project(mav_sensors_drivers)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED roscpp lpp mav_sensors_core)

catkin_package(
  INCLUDE_DIRS include lib/BMP3-Sensor-API lib/BMI08x-Sensor-API
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_BMP3 ${PROJECT_NAME}_BMI08x
  CATKIN_DEPENDS roscpp lpp mav_sensors_core
)

add_compile_definitions(MODE_LPP)

###########
## Build ##
###########

include_directories(
 include 
 lib/BMP3-Sensor-API
 lib/BMI08x-Sensor-API
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/imu/adis16448.cpp
        src/barometer/bmp390.cpp
        src/imu/bmi088.cpp
        src/radar/xwr18xx_mmw_demo.cpp)

add_library(${PROJECT_NAME}_BMP3
        lib/BMP3-Sensor-API/bmp3.c)

add_library(${PROJECT_NAME}_BMI08x
        lib/BMI08x-Sensor-API/bmi08a.c
        lib/BMI08x-Sensor-API/bmi08g.c
        lib/BMI08x-Sensor-API/bmi08xa.c)

target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_BMP3 ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
