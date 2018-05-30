/*-----------------------------------------------------------------------------------------------------------------
----* Subscriber	/aruco_single/pose - position feedback from aruco marker
										 and y - vertically down 
										 and x --  90 degree couter clockwise.--------------------------------------

----* Publisher 	/mavros/mocap/pose - publish processed value from PID controller as euler angle setpoint
					/mavros/setpoint_position/local - publisher for altitude setpoint and yaw angle setpoint.-------
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

float sp_thresh = 0.1, err_sum_x = 0.0, err_sum_y = 0.0;
float yaw = 5.7;

tf::Quaternion q;
geometry_msgs::PoseStamped mocap;
geometry_msgs::PoseStamped setpoint;
mavros_msgs::PositionTarget set_raw;
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

    ros::Subscriber aruco_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 1, arucocb);
    ros::Subscriber dist_sub = nh.subscribe<geometry_msgs::PoseStamped>("/pose", 100,distcb);

	ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",10);

    ros::Rate loop_rate(40);
    
    float x_prev, y_prev;
    int i=0;

    while ( ros::ok() )
    {
    	float roll_p,roll_i,roll_d, pitch_p, pitch_i, pitch_d,set_alt;
        
    	nh.getParam("/controller/roll_p", roll_p);
		nh.getParam("/controller/roll_d", roll_d);
    	nh.getParam("/controller/roll_i", roll_i);
        nh.getParam("/controller/pitch_p", pitch_p);
        nh.getParam("/controller/pitch_d", pitch_d);
        nh.getParam("/controller/pitch_i", pitch_i);
        nh.getParam("/controller/set_alt", set_alt);

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
                mocap.pose.position.y = (x-x_des)*roll_p + (err_sum_x)*0.03*roll_i + (x-x_prev)*30*roll_d;//roll
                mocap.pose.position.x = (y-y_des)*pitch_p + (err_sum_y)*0.03*pitch_i + (y-y_prev)*30*pitch_d;//pitch
            } 

            if(abs(mocap.pose.position.y)>0)
            {
                roll_p_perc = abs((x-x_des)*roll_p/mocap.pose.position.y)*100;
                roll_i_perc = abs((err_sum_x)*0.03*roll_i/mocap.pose.position.y)*100;
                roll_d_perc = abs((x-x_prev)*30*roll_d/mocap.pose.position.y)*100;
                // offset_perc = abs(offset/mocap.pose.position.y)*100;
            }

            if(roll_p_perc>max_roll_p_perc)
                max_roll_p_perc = roll_p_perc;
            if(roll_i_perc>max_roll_i_perc)
                max_roll_i_perc = roll_i_perc;
            if(roll_d_perc>max_roll_d_perc)
                max_roll_d_perc = roll_d_perc;

            cout<<"Roll sp = "<<mocap.pose.position.y<<endl;
            cout<<"X = "<<x<<endl;
            cout<<"Roll"<<endl;
            cout<<"P % = "<<roll_p_perc<<endl;
            cout<<"I % = "<<roll_i_perc<<endl;
            cout<<"D % = "<<roll_d_perc<<endl;
            // cout<<"offset % = "<<offset_perc<<endl<<endl;
            cout<<"MAX P % = "<<max_roll_p_perc<<endl;
            cout<<"MAX I % = "<<max_roll_i_perc<<endl;
            cout<<"MAX D % = "<<max_roll_d_perc<<endl<<endl<<endl;

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

            if ( mocap.pose.position.x < -sp_thresh || mocap.pose.position.x > sp_thresh || mocap.pose.position.y < -sp_thresh || mocap.pose.position.y > sp_thresh )
            cross_flag= 1;

            if ( cross_flag==1 )
            cout<<"Attitude Threshold reached"<<endl;

            //cout<<"aruco_detected"<<endl<<"pitch = "<<mocap.pose.position.x<<endl<< "roll = "<< mocap.pose.position.y<<endl;
            //cout<<"aruco_x = "<<x<<endl<<"aruco_y = "<<y<<endl;
        }
        
        setpoint.pose.position.z = set_alt;
        
        mocap.pose.position.z = dist;
        
        setpoint_pub.publish(setpoint);
        mocap_pub.publish(mocap);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
