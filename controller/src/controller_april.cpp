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
#include "geometry_msgs/PoseArray.h"
#include "mavros_msgs/AttitudeTarget.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Range.h>
#include <mavros_msgs/PositionTarget.h>
using namespace std;

int april_detected_flag = 0,cross_flag=0;
float x=0, y=0, x_des = 0, y_des=0;

float roll_p_perc,roll_i_perc,roll_d_perc,offset_perc;
float max_roll_p_perc=0,max_roll_d_perc=0,max_roll_i_perc=0;

float sp_thresh = 1.1, err_sum_x = 0.0, err_sum_y = 0.0;
float yaw = 5.7;

tf::Quaternion q;
geometry_msgs::PoseStamped mocap;
geometry_msgs::PoseStamped tag_pose;
geometry_msgs::PoseStamped setpoint;
mavros_msgs::PositionTarget set_raw;
float dist;

void aprilcb(const geometry_msgs::PoseArray::ConstPtr& msg)
{   
    //cout<<"Showing array size"<<endl;
    
    if(msg->poses.size()==1)
    {   
        x = msg->poses[0].position.x;
        y = msg->poses[0].position.y;
        tag_pose.header.stamp = ros::Time::now();
        tag_pose.pose.position.x = x;
        tag_pose.pose.position.y = y;
	    april_detected_flag = 1;



}

	//cout<<msg->poses[0]<<endl;

  
}

void distcb(const sensor_msgs::Range::ConstPtr& msg)
{
	dist = msg->range;
  	
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;
ros::Publisher tag_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/tag_pose", 10); 
    ros::Subscriber april_sub = nh.subscribe<geometry_msgs::PoseArray>("/tag_detections_pose", 10, aprilcb);
    ros::Subscriber dist_sub = nh.subscribe<sensor_msgs::Range>("/tfmini_ros_node/TFmini", 100,distcb);

	ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",10);
    


    ros::Rate loop_rate(30);
    
    float x_prev, y_prev;
    int i=0;

    while ( ros::ok() )
    {
    	float k_p , k_i, k_d, offset, reset;
    	nh.getParam("/controller/k_p", k_p);
		nh.getParam("/controller/k_d", k_d);
    	nh.getParam("/controller/k_i", k_i);
    	nh.getParam("/controller/offset", offset);
	nh.getParam("/controller/reset", reset);
	
		mocap.header.stamp = ros::Time::now();
        setpoint.header.stamp = ros::Time::now();
               
        if( april_detected_flag == 1)        
        {

            //mavros_msgs::SetMode offb_set_mode;
            //offb_set_mode.request.custom_mode = "OFFBOARD";
            //mavros_msgs::CommandBool arm_cmd;
            //arm_cmd.request.value = true;
            err_sum_x = err_sum_x + x;
            err_sum_y = err_sum_y + y;
		if (reset ==1)
		err_sum_x =0;

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

          

           
            //cout<<"diff="<<(x-x_prev)<<endl;
            if(abs(mocap.pose.position.y)>0)
            {
                roll_p_perc = abs((x-x_des)*k_p/mocap.pose.position.y)*100;
                roll_i_perc = abs((err_sum_x)*0.03*k_i/mocap.pose.position.y)*100;
                roll_d_perc = abs((x-x_prev)*30*k_d/mocap.pose.position.y)*100;
                offset_perc = abs(offset/mocap.pose.position.y)*100;
            }

            if(roll_p_perc>max_roll_p_perc)
                max_roll_p_perc = roll_p_perc;
            if(roll_i_perc>max_roll_i_perc)
                max_roll_i_perc = roll_i_perc;
            if(roll_d_perc>max_roll_d_perc)
                max_roll_d_perc = roll_d_perc;

            cout<<"X = "<<x<<endl;
            cout<<"Roll"<<endl;
            cout<<"P % = "<<roll_p_perc<<endl;
            cout<<"I % = "<<roll_i_perc<<endl;
            cout<<"D % = "<<roll_d_perc<<endl;
            cout<<"offset % = "<<offset_perc<<endl<<endl;
            cout<<"MAX P % = "<<max_roll_p_perc<<endl;
            cout<<"MAX I % = "<<max_roll_i_perc<<endl;
            cout<<"MAX D % = "<<max_roll_d_perc<<endl<<endl<<endl;

            x_prev=x;

            i=i+1;






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
		tag_pose_pub.publish(tag_pose);
        }
q.setRPY(0, 0 , yaw);
setpoint.pose.orientation.z = q.z();
setpoint.pose.orientation.w = q.w();	
 setpoint.pose.position.z = 1.2f;
 setpoint_pub.publish(setpoint);
        mocap.pose.position.z = dist;
        mocap_pub.publish(mocap);
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}
