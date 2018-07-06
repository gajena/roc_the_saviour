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
#include <sensor_msgs/Imu.h>
#include <math.h>

using namespace std;

/*flags for detection of aruco and threshold check*/
int odom_detected_flag = 0,cross_flag=0, vel_cross_flag = 0, init_imu_flag = 0, yaw_reset = 0;
float x = 0, y = 0, x_des = 0, y_des = 0, sp_thresh = 0.3, err_sum_x = 0.0, err_sum_y = 0.0, yaw_sp = 5.7, dist, err_sum_pos_x = 0,err_sum_pos_y = 0;
float vel_x = 0, vel_y = 0, vel_thresh = 1.0,vel_sp_x = 0,vel_sp_y = 0;
double imu_yaw,yaw_init = -5.7;

string mode_;

tf::Quaternion q;
geometry_msgs::PoseStamped mocap;
geometry_msgs::PoseStamped setpoint;
geometry_msgs::PoseStamped vel_sp;

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

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    if(init_imu_flag==0)
    {
       tf::Quaternion q1(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf::Matrix3x3 m(q1);
        double r, p;
        m.getRPY(r,p,imu_yaw);
        init_imu_flag=1;
    }  
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "vin_velocity_controller");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/rovio/odometry", 10, odomcb);
    ros::Subscriber dist_sub = nh.subscribe<geometry_msgs::PoseStamped>("/pose", 100,distcb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100,statecb);
    ros::Subscriber sub1 = nh.subscribe("/mavros/imu/data",100, imuCallback);

    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",10);
    ros::Publisher vel_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/velocity_sp", 10);
   
    ros::Rate loop_rate(30);
    
    float vel_x_prev, vel_y_prev;
    int i=0;


    while ( ros::ok() )
    {
        float yaw_traj,pos_k_p,pos_k_i, vel_x_k_p , vel_x_k_i, vel_x_k_d, vel_y_k_p , vel_y_k_i, vel_y_k_d, set_alt, vel_set_y,vel_set_x;
        nh.getParam("/vin_velocity_controller/pos_k_p", pos_k_p);
        
        nh.getParam("/vin_velocity_controller/vel_x_k_p", vel_x_k_p);
        nh.getParam("/vin_velocity_controller/vel_x_k_d", vel_x_k_d);
        nh.getParam("/vin_velocity_controller/vel_y_k_p", vel_y_k_p);
        nh.getParam("/vin_velocity_controller/vel_y_k_d", vel_y_k_d);
        nh.getParam("/vin_velocity_controller/set_alt", set_alt);
        nh.getParam("/vin_velocity_controller/x_des", x_des);
        nh.getParam("/vin_velocity_controller/y_des", y_des);
	    nh.getParam("/vin_velocity_controller/vel_set_y", vel_set_y);        
	    nh.getParam("/vin_mission_control/yaw_reset", yaw_reset);
	    nh.getParam("/vin_velocity_controller/vel_set_x", vel_set_x);
	    nh.getParam("/vin_velocity_controller/yaw_traj", yaw_traj);
        mocap.header.stamp = ros::Time::now();
        setpoint.header.stamp = ros::Time::now();
        vel_sp.header.stamp = ros::Time::now();

               
        if( odom_detected_flag == 1)        
        {               

            

            if (i==0)
            {
                vel_x_prev = vel_x;
                vel_y_prev = vel_y;
            }
            else
            {   
                err_sum_pos_x = err_sum_pos_x + (x_des-x);
                err_sum_pos_y = err_sum_pos_y + (y_des-y);

                if(mode_=="OFFBOARD")
                {
                    nh.getParam("/vin_velocity_controller/pos_k_i", pos_k_i);
                }
                else
                {

                    pos_k_i = 0;
                    err_sum_pos_y = 0;
                    err_sum_pos_x = 0;
                }

                float vel_sp_x_world = (x_des - x)*pos_k_p + (err_sum_pos_x)*0.03*pos_k_i;
                float vel_sp_y_world = (y_des - y)*pos_k_p + (err_sum_pos_y)*0.03*pos_k_i;
                vel_sp_x = cos(yaw_traj)*vel_sp_x_world + sin(yaw_traj)* vel_sp_y_world;
                vel_sp_y = cos(yaw_traj)*vel_sp_y_world - sin(yaw_traj)* vel_sp_x_world;

                err_sum_x = err_sum_x + (vel_x - vel_sp_x);
                err_sum_y = err_sum_y + (vel_y - vel_sp_y);

               
                if(mode_=="OFFBOARD")
                {
                    nh.getParam("/vin_velocity_controller/vel_x_k_i", vel_x_k_i);
                    nh.getParam("/vin_velocity_controller/vel_y_k_i", vel_y_k_i);
                }
                else
                {
                    vel_x_k_i = 0;
                    vel_y_k_i = 0;
                    err_sum_y = 0;
                    err_sum_x = 0;
                }

                 cout<<"Error sum  = "<<err_sum_x*0.03*vel_x_k_i <<" , "<<err_sum_y*0.03*vel_y_k_i <<endl;
                // cout<<"0.03 * KI"<<0.03*vel_k_i;
                //    vel_sp_y = vel_set_y;
                //    vel_sp_x = vel_set_x; 

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

                

                mocap.pose.position.y = ((vel_y - vel_sp_y)*vel_y_k_p + (err_sum_y)*0.03*vel_y_k_i + (vel_y - vel_y_prev)*30*vel_y_k_d);//roll
                mocap.pose.position.x = ((vel_x - vel_sp_x)*vel_x_k_p + (err_sum_x)*0.03*vel_x_k_i + (vel_x - vel_x_prev)*30*vel_x_k_d);//pitch

                vel_sp.pose.position.x = vel_sp_x;
                vel_sp.pose.position.y = -vel_sp_y;
            } 
            setpoint.pose.position.z = set_alt;

            vel_x_prev = vel_x;
            vel_y_prev = vel_y;

            i=i+1;

            if(yaw_reset == 1)
            {
                
                yaw_init = imu_yaw;
                nh.setParam("/vin_mission_control/yaw_reset", 0);
            }
            yaw_sp = yaw_init+yaw_traj;
            q.setRPY(0, 0, yaw_sp);

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
        vel_sp_pub.publish(vel_sp);
        mocap_pub.publish(mocap);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
