cmake_minimum_required(VERSION 2.8.3)
project(warehouse_quad)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  cmake_modules
  px_comm

)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED COMPONENTS common io)

catkin_package(
   DEPENDS OpenCV Boost Eigen PCL
  INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp
)
include_directories(include
					${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(distance_sensor src/distance_sensor.cpp )
target_link_libraries(distance_sensor
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES})
