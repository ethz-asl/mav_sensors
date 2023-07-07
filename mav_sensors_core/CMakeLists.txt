cmake_minimum_required(VERSION 3.0.2)
project(mav_sensors_core)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS roscpp lpp)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES mav_sensors_core
        CATKIN_DEPENDS roscpp lpp
)

###########
## Build ##
###########
include_directories(
        include lib/BMP3-Sensor-API
                ${catkin_INCLUDE_DIRS}
)

add_compile_definitions(MODE_ROSLOG)

add_library(${PROJECT_NAME}
        src/sensor_config.cpp
        src/sensor.cpp
        src/driver.cpp

        src/protocols/common/PosixFilesystem.cpp
        src/protocols/Serial.cpp
        src/protocols/Spi.cpp
        src/protocols/Gpio.cpp

        src/sensor_types/FluidPressure.cpp
        )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(${PROJECT_NAME}_node src/main.cpp)

#target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

#add_executable(xwr18xx_mmw_demo src/nodes/xwr18xx_mmw_demo_node.cpp)
#target_link_libraries(xwr18xx_mmw_demo ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
 install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mav_sensors.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
