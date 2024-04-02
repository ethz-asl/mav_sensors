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
project(mav_sensors_demo)

add_compile_options(-std=c++17)
add_compile_definitions(MODE_ROSLOG)

find_package(catkin REQUIRED COMPONENTS
  lpp
  mav_sensors_core
  mav_sensors_drivers
  roscpp
)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES mav_sensors_demo
        CATKIN_DEPENDS lpp mav_sensors_core mav_sensors_drivers roscpp
        DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_xwr18xx_mmw_demo src/xwr18xx_mmw_demo_node.cpp)
target_link_libraries(${PROJECT_NAME}_xwr18xx_mmw_demo ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_bmi088_demo src/bmi088_demo.cpp)
target_link_libraries(${PROJECT_NAME}_bmi088_demo ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_bmp390_demo src/bmp390_demo.cpp)
target_link_libraries(${PROJECT_NAME}_bmp390_demo ${catkin_LIBRARIES})