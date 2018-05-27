/*-----------------------------------------------------------------------------------------------------------------
----* Subscriber	/aruco_single/pose - position feedback from aruco marker
										 and y - vertically down 
										 and x --  90 degree couter clockwise.--------------------------------------

----* Publisher 	/mavros/mocap/pose - publish processed value from PID controller as euler angle setpoint
					/mavros/setpoint_position/local - publisher for altitude setpoint and yaw angle setpoint-------
------------------------------------------------------------------------------------------------------------------*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/PositionTarget.h>
using namespace std;

/*flags for detection of aruco and threshold check*/
int aruco_detected_flag = 0,cross_flag=0;
float aruco_x=0, aruco_y=0,quad_x,quad_y,enu_quad_x,enu_quad_y, x_des = 0, y_des=0;

float roll_p_perc,roll_i_perc,roll_d_perc,offset_perc;
float max_roll_p_perc=0,max_roll_d_perc=0,max_roll_i_perc=0;

float sp_thresh = 1.1, err_sum_x = 0.0, err_sum_y = 0.0;
float yaw = 5.7;

tf::Quaternion q;
geometry_msgs::PoseStamped mocap;
geometry_msgs::PoseStamped setpoint;
mavros_msgs::PositionTarget set_raw;
float dist;

void arucocb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	aruco_y = (msg->pose.position.y);
  	aruco_x = (msg->pose.position.x);

  	aruco_detected_flag = 1;
}

void distcb(const sensor_msgs::Range::ConstPtr& msg)
{
	dist = msg->range;
  	
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::Subscriber aruco_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 10, arucocb);
    ros::Subscriber dist_sub = nh.subscribe<sensor_msgs::Range>("/tfmini_ros_node/TFmini", 100,distcb);

	ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",10);
    ros::Publisher set_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate loop_rate(30);
    
    float x_prev, y_prev;
    int i=0;

    while ( ros::ok() )
    {
    	
	
		mocap.header.stamp = ros::Time::now();
        ///set_raw.header.stamp = ros::Time::now();
        ///set_raw.coordinate_frame = 1;
        ///set_raw.type_mask = 0b0000000000000000;

		if( aruco_detected_flag == 1)        
		{
			quad_x = -aruco_x;
			quad_y = -aruco_y;
			enu_quad_x = -quad_y;
			enu_quad_y = -quad_x;
	        mocap.pose.position.x = x;
	        mocap.pose.position.y = y;
			mocap.pose.position.z = dist;

	       	//set_pub.publish(set_raw);
        	mocap_pub.publish(mocap);
	   }

        ros::spinOnce();
        loop_rate.sleep();

   }
return 0; 
    }
