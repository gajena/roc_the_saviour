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
float x=0, y=0, x_des = 0, y_des=0;

float roll_p_perc,roll_i_perc,roll_d_perc,offset_perc;
float max_roll_p_perc=0,max_roll_d_perc=0,max_roll_i_perc=0;

float sp_thresh = 1.1, err_sum_x = 0.0, err_sum_y = 0.0;
float yaw = 5.7;

tf::Quaternion q;
geometry_msgs::PoseStamped mocap;
geometry_msgs::PoseStamped setpoint;
// mavros_msgs::PositionTarget set_raw;
float dist;

void arucocb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	x = (msg->pose.position.x);
  	y = (msg->pose.position.y);

  	aruco_detected_flag = 1;
}

void distcb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    dist = msg->pose.position.z;
    
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::Subscriber aruco_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 10, arucocb);
    ros::Subscriber dist_sub = nh.subscribe<geometry_msgs::PoseStamped>("/pose", 100,distcb);
    
	ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",10);
    // ros::Publisher set_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate loop_rate(30);
    
    float x_prev, y_prev;
    int i=0;

    while ( ros::ok() )
    {
    	float k_p , k_i, k_d, offset;
    	nh.getParam("/controller/k_p", k_p);
		nh.getParam("/controller/k_d", k_d);
    	nh.getParam("/controller/k_i", k_i);
    	nh.getParam("/controller/offset", offset);
	
		mocap.header.stamp = ros::Time::now();
        setpoint.header.stamp = ros::Time::now();
               
        if( aruco_detected_flag == 1)        
        {
            err_sum_x = err_sum_x + x;
            err_sum_y = err_sum_y + y;

            if (i==0)
            {
                x_prev=x;
                y_prev=y;
            }
            else
            { 
                mocap.pose.position.y = (x-x_des)*k_p + (err_sum_x)*0.03*k_i + (x-x_prev)*30*k_d + offset;//roll
                mocap.pose.position.x = (y-y_des)*k_p + (err_sum_y)*0.03*k_i + (y-y_prev)*30*k_d;//pitch
            } 

            setpoint.pose.position.z = 1.0f;

           // mocap.pose.position.x =  0;
           // mocap.pose.position.y = 0.03;
            // mocap.pose.position.z = dist;
           
            cout<<"debug="<<mocap.pose.position.x<<mocap.pose.position.y<<endl;

            x_prev=x;
            y_prev=y;

            i=i+1;

            q.setRPY(0, 0, yaw);

            setpoint.pose.orientation.z = q.z();
            setpoint.pose.orientation.w = q.w();

            if ( mocap.pose.position.x > sp_thresh )
                mocap.pose.position.x = sp_thresh;
            else if ( mocap.pose.position.x < -sp_thresh )
                mocap.pose.position.x = -sp_thresh;
            if ( mocap.pose.position.y < -sp_thresh )
                mocap.pose.position.y = -sp_thresh;
            else if ( mocap.pose.position.y > sp_thresh )
                mocap.pose.position.y = sp_thresh;

            if ( cross_flag==1 )
            cout<<"Attitude Threshold reached"<<endl;
            if ( mocap.pose.position.x < -sp_thresh || mocap.pose.position.x > sp_thresh || mocap.pose.position.y < -sp_thresh || mocap.pose.position.y > sp_thresh )
            cross_flag= 1;

            setpoint_pub.publish(setpoint);
        }
        mocap.pose.position.z = dist;
        mocap_pub.publish(mocap);
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
