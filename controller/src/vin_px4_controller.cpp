#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
using namespace std;

/*flags for detection of aruco and threshold check*/
int odom_detected_flag = 0,cross_flag=0;
float x=0, y=0, x_des = 0, y_des=0;

float roll_p_perc,roll_i_perc,roll_d_perc,offset_perc;
float max_roll_p_perc=0,max_roll_d_perc=0,max_roll_i_perc=0;

float sp_thresh = 1.1, err_sum_x = 0.0, err_sum_y = 0.0;
float yaw = 5.7;

tf::Quaternion q;
geometry_msgs::TwistWithCovarianceStamped speed_;
geometry_msgs::PoseStamped setpoint;
// mavros_msgs::PositionTarget set_raw;
float dist;

void odomcb(const nav_msgs::Odometry::ConstPtr& msg)
{
	
    speed_.twist=msg.twist;
  	odom_detected_flag = 1;
}

void distcb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    dist = msg->pose.position.z;
    
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "vin_px4_controller");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/rovio/odometry", 10, odomcb);
    ros::Subscriber dist_sub = nh.subscribe<geometry_msgs::PoseStamped>("/pose", 100,distcb);
    
	ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/mavros/vision_speed/speed_twist_cov",10);
    // ros::Publisher set_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate loop_rate(30);
    
    float x_prev, y_prev;
    int i=0;

    while ( ros::ok() )
    {
	
		speed_.header.stamp = ros::Time::now();
        setpoint.header.stamp = ros::Time::now();
               
        if( odom_detected_flag == 1)        
        {
            
            mocap_pub.publish(speed_);
        }
        // mocap.pose.position.z = dist;
        // mocap_pub.publish(mocap);
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
