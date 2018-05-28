cmake_minimum_required(VERSION 2.8.3)
project(warehouse_quad)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  cmake_modules


)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
#find_package(PCL REQUIRED COMPONENTS common io)

catkin_package(
   DEPENDS OpenCV Boost Eigen
  INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp
)
include_directories(include
					${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(main src/main.cpp src/ekf.cpp src/sonar.cpp)
target_link_libraries(main
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES})
