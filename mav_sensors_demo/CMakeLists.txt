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

if (MAV_SENSORS_BMI08X_SUPPORT)
add_executable(${PROJECT_NAME}_bmi088_demo src/bmi088_demo.cpp)
target_link_libraries(${PROJECT_NAME}_bmi088_demo ${catkin_LIBRARIES})
endif ()

if (MAV_SENSORS_BMP3_SUPPORT)
add_executable(${PROJECT_NAME}_bmp390_demo src/bmp390_demo.cpp)
target_link_libraries(${PROJECT_NAME}_bmp390_demo ${catkin_LIBRARIES})
endif ()