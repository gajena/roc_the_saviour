cmake_minimum_required(VERSION 2.8.3)
project(controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  nav_msgs
  )
include_directories(${catkin_INCLUDE_DIRS})

add_executable(controller src/controller.cpp )
target_link_libraries(controller
    ${catkin_LIBRARIES} ${aruco_LIBS})
    
add_dependencies(controller ${catkin_EXPORTED_TARGETS})

add_executable(vin_controller src/vin_controller.cpp )
target_link_libraries(vin_controller
    ${catkin_LIBRARIES} ${aruco_LIBS})
    
add_dependencies(vin_controller ${catkin_EXPORTED_TARGETS})
