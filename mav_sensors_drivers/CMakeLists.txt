cmake_minimum_required(VERSION 3.0.2)
project(mav_sensors_drivers)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        lpp
        mav_sensors_core
)

add_compile_definitions(MODE_LPP)

###########
## Build ##
###########

if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/BMP3-Sensor-API/)
    set(MAV_SENSORS_BMP3_SUPPORT ON)
    set(MAV_SENSORS_OPTIONAL_SRC_FILES ${MAV_SENSORS_OPTIONAL_SRC_FILES} src/barometer/bmp390.cpp)
    set(MAV_SENSORS_OPTIONAL_INCLUDE_DIRS ${MAV_SENSORS_OPTIONAL_INCLUDE_DIRS} lib/BMP3-Sensor-API)
    message(STATUS "Found BMP3-Sensor-API")
else()
    message(WARNING "BMP3-Sensor-API not found. BMP3 sensor will not be supported.")
endif()

if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/BMI08x-Sensor-API/)
    set(MAV_SENSORS_BMI08X_SUPPORT ON)
    set(MAV_SENSORS_OPTIONAL_SRC_FILES ${MAV_SENSORS_OPTIONAL_SRC_FILES} src/imu/bmi088.cpp)
    set(MAV_SENSORS_OPTIONAL_INCLUDE_DIRS ${MAV_SENSORS_OPTIONAL_INCLUDE_DIRS} lib/BMI08x-Sensor-API)
    message(STATUS "Found BMI08x-Sensor-API")
else()
    message(WARNING "BMI08x-Sensor-API not found. BMI08x sensor will not be supported.")
endif()

catkin_package(
        INCLUDE_DIRS include ${MAV_SENSORS_OPTIONAL_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_BMP3 ${PROJECT_NAME}_BMI08x
        CATKIN_DEPENDS roscpp lpp mav_sensors_core
)

include_directories(
 include
 ${MAV_SENSORS_OPTIONAL_INCLUDE_DIRS}
 ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/imu/adis16448.cpp
        src/radar/xwr18xx_mmw_demo.cpp
        ${MAV_SENSORS_OPTIONAL_SRC_FILES})

if (MAV_SENSORS_BMP3_SUPPORT)
add_library(${PROJECT_NAME}_BMP3
        lib/BMP3-Sensor-API/bmp3.c)
set(MAV_SENSOR_LIBRARIES ${MAV_SENSOR_LIBRARIES} ${PROJECT_NAME}_BMP3)
endif()

if (MAV_SENSORS_BMI08X_SUPPORT)
add_library(${PROJECT_NAME}_BMI08x
        lib/BMI08x-Sensor-API/bmi08a.c
        lib/BMI08x-Sensor-API/bmi08g.c
        lib/BMI08x-Sensor-API/bmi08xa.c)
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC ${MAV_SENSOR_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
