#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>

geometry_msgs::PoseStamped setpoint;

float distance_tfmini;

void tfminicb(const sensor_msgs::Range::ConstPtr &msg)
{
	distance_tfmini = msg->range;
}

geometry_msgs::PoseStamped mocap_pose;
void mocapcb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	mocap_pose = *msg;
}



int main (int argc, char **argv)
{
    ros::init(argc, argv, "distance_sensor");
    ros::NodeHandle nh;

    ros::Subscriber tfmini_pub = nh.subscribe<sensor_msgs::Range>("/tfmini_ros_node/TFmini", 100, tfminicb);
    // ros::Subscriber mocap_sub  = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 100 , mocapcb);
	// ros::Publisher dist_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",100);

    ros::Rate loop_rate(50);

    while ( ros::ok() )
    {
		setpoint.pose.position.z = distance_tfmini;
		// setpoint.pose.position.x = mocap_pose.pose.position.x;
  //               setpoint.pose.position.y = mocap_pose.pose.position.y;
        setpoint.header.stamp = ros::Time::now();
		setpoint.header.frame_id = "quadPose";
		
		// dist_pub.publish(setpoint);
		ros::spinOnce();
        loop_rate.sleep();
	}
return 0;
}
