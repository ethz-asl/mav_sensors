cmake_minimum_required(VERSION 3.0.2)
project(mav_sensors_drivers)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED roscpp lpp mav_sensors_core)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mav_sensors_drivers
  CATKIN_DEPENDS roscpp lpp mav_sensors_core
)

add_compile_definitions(MODE_LPP)

###########
## Build ##
###########

include_directories(
 include lib/BMP3-Sensor-API
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/barometer/bmp390.cpp
        src/radar/xwr18xx_mmw_demo.cpp
        )

add_library(${PROJECT_NAME}_BMP3
        lib/BMP3-Sensor-API/bmp3.c)

target_link_libraries(${PROJECT_NAME} PUBLIC ${PROJECT_NAME}_BMP3 ${catkin_LIBRARIES})