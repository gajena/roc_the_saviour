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
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
using namespace std;

/*flags for detection of aruco and threshold check*/
int odom_detected_flag = 0,cross_flag=0, vel_cross_flag = 0;
float x = 0, y = 0, x_des = 0, y_des = 0, sp_thresh = 0.3, err_sum_x = 0.0, err_sum_y = 0.0, yaw = 5.7, dist;
float vel_x = 0, vel_y = 0, vel_thresh = 1.0;
string mode_;

tf::Quaternion q;
geometry_msgs::PoseStamped mocap;
geometry_msgs::PoseStamped setpoint;

void odomcb(const nav_msgs::Odometry::ConstPtr& msg)
{
    x = (msg->pose.pose.position.x);
    y = (msg->pose.pose.position.y);
    vel_x = (msg->twist.twist.linear.x); 
    vel_y = -(msg->twist.twist.linear.y); 

    odom_detected_flag = 1;
}

void distcb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    dist = msg->pose.position.z;
    
}

void statecb(const mavros_msgs::State::ConstPtr& msg)
{
    mode_ = msg->mode;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "vin_velocity_controller");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/rovio/odometry", 10, odomcb);
    ros::Subscriber dist_sub = nh.subscribe<geometry_msgs::PoseStamped>("/pose", 100,distcb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100,statecb);
    
    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",10);
   
    ros::Rate loop_rate(30);
    
    float vel_x_prev, vel_y_prev;
    int i=0;

    while ( ros::ok() )
    {
        float pos_k_p, vel_k_p , vel_k_i, vel_k_d, set_alt;
        nh.getParam("/vin_velocity_controller/pos_k_p", pos_k_p);
        nh.getParam("/vin_velocity_controller/vel_k_p", vel_k_p);
        nh.getParam("/vin_velocity_controller/vel_k_d", vel_k_d);
        nh.getParam("/vin_velocity_controller/set_alt", set_alt);
        
        if(mode_=="OFFBOARD")
            nh.getParam("/vin_velocity_controller/vel_k_i", vel_k_i);
        else
        {
            vel_k_i = 0;
            err_sum_y = 0;
            err_sum_x = 0;
        }
        mocap.header.stamp = ros::Time::now();
        setpoint.header.stamp = ros::Time::now();
               
        if( odom_detected_flag == 1)        
        {
            err_sum_x = err_sum_x + vel_x;
            err_sum_y = err_sum_y + vel_y;

            if (i==0)
            {
                vel_x_prev = vel_x;
                vel_y_prev = vel_y;
            }
            else
            { 
                float vel_sp_x = (x_des - x)*pos_k_p;
                float vel_sp_y = (y_des - y)*pos_k_p;
               //  vel_sp_y = -0.0f;
               // vel_sp_x = 0.0f; 

                if ( vel_sp_x < -vel_thresh || vel_sp_x > vel_thresh || vel_sp_y < -vel_thresh || vel_sp_y > vel_thresh )
                    vel_cross_flag= 1;

                if ( vel_sp_y > vel_thresh )
                    vel_sp_y = vel_thresh;
                else if ( vel_sp_y < -vel_thresh )
                    vel_sp_y = -vel_thresh;

                if ( vel_sp_x > vel_thresh )
                    vel_sp_x = vel_thresh;
                else if ( vel_sp_x < -vel_thresh )
                    vel_sp_x = -vel_thresh;

                if ( vel_cross_flag==1 )
                    cout<<"vel Threshold reached"<<endl;

                mocap.pose.position.y = (vel_y - vel_sp_y)*vel_k_p + (err_sum_y)*0.03*vel_k_i + (vel_y - vel_y_prev)*30*vel_k_d;//roll
                mocap.pose.position.x = (vel_x - vel_sp_x)*vel_k_p + (err_sum_x)*0.03*vel_k_i + (vel_x - vel_x_prev)*30*vel_k_d;//pitch
            } 
            setpoint.pose.position.z = set_alt;

            vel_x_prev = vel_x;
            vel_y_prev = vel_y;

            i=i+1;

            q.setRPY(0, 0, yaw);

            setpoint.pose.orientation.z = q.z();
            setpoint.pose.orientation.w = q.w();

            if ( mocap.pose.position.x < -sp_thresh || mocap.pose.position.x > sp_thresh || mocap.pose.position.y < -sp_thresh || mocap.pose.position.y > sp_thresh )
            cross_flag= 1;

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

            cout<<"pitch = "<<mocap.pose.position.x<<std::endl<<"roll = "<<mocap.pose.position.y<<endl;
            setpoint_pub.publish(setpoint);
        }
        mocap.pose.position.z = dist;
        mocap_pub.publish(mocap);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
