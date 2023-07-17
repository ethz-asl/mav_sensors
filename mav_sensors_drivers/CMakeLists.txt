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