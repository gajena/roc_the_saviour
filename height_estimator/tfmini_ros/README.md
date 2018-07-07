# tfmini_ros
#### Benewake mini infrared range sensor TFmini ROS package

##### Package Information
* Node:				tfmini_ros_node
* Published Topics:
  * /tfmini_ros_node/TFmini (sensor_msgs/Range)
  > The distance of object detected. 
      
> Note: This node won't publish topic if no object exists within TFmini's measurement range, and the behavior can be changed in file 
      /src/TFmini_ros_node.cpp
      
Quick Start
```shell
$ cd tfmini_ros/src
$ catkin_make
$ roslaunch tfmini_ros tfmini.launch
```
