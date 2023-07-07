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

add_library(${PROJECT_NAME}_BMP3 lib/BMP3-Sensor-API/bmp3.c)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_BMP3)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

## Mark cpp header files for installation

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mav_sensors_drivers.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
