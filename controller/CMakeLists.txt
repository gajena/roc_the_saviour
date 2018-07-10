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
  dynamixel_workbench_msgs
  px_comm
  )

catkin_package()
include_directories(${catkin_INCLUDE_DIRS})

add_executable(vin_mission_control src/vin_mission_control.cpp )
target_link_libraries(vin_mission_control
    ${catkin_LIBRARIES} ${aruco_LIBS})

add_executable(vin_trajectory_track_controller src/vin_trajectory_track_controller.cpp )
target_link_libraries(vin_trajectory_track_controller
    ${catkin_LIBRARIES} ${aruco_LIBS})

add_executable(vin_controller src/vin_controller.cpp )
target_link_libraries(vin_controller
    ${catkin_LIBRARIES})

add_executable(vin_velocity_controller src/vin_velocity_controller.cpp )
target_link_libraries(vin_velocity_controller
    ${catkin_LIBRARIES} ${aruco_LIBS})

add_executable(controller src/controller.cpp )
target_link_libraries(controller
    ${catkin_LIBRARIES} ${aruco_LIBS})

#add_executable(vin_px4_controller src/vin_px4_controller.cpp )
#target_link_libraries(vin_px4_controller
 #   ${catkin_LIBRARIES} ${aruco_LIBS})

