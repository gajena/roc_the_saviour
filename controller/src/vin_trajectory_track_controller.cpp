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
#include <visualization_msgs/MarkerArray.h>

using namespace std;

/*flags for detection of msgs and threshold check*/
int odom_detected_flag = 0, cross_flag = 0, vel_cross_flag = 0, init_imu_flag = 0, traj_marker_size = 0, trajectory_size = 0;
float x = 0, y = 0, x_des = 0, y_des = 0, sp_thresh = 0.3, err_sum_x = 0.0, err_sum_y = 0.0, yaw = 5.7, dist, err_sum_pos_x = 0, err_sum_pos_y = 0;
float vel_x = 0, vel_y = 0, vel_thresh = 1.0, vel_sp_x = 0, vel_sp_y = 0;
double imu_yaw;
string mode_;

tf::Quaternion q;
geometry_msgs::PoseStamped mocap;
geometry_msgs::PoseStamped setpoint;
geometry_msgs::PoseStamped vel_sp,pos_sp;
visualization_msgs::MarkerArray traj;

void odomcb(const nav_msgs::Odometry::ConstPtr &msg);
void distcb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void statecb(const mavros_msgs::State::ConstPtr &msg);
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
void traj_cb(const visualization_msgs::MarkerArray::ConstPtr &msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vin_trajecoty_track_controller");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/rovio/odometry", 10, odomcb);
    ros::Subscriber dist_sub = nh.subscribe<geometry_msgs::PoseStamped>("/pose", 100, distcb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 100, statecb);
    ros::Subscriber traj_sub = nh.subscribe<visualization_msgs::MarkerArray>("trajectory_traject", 10, traj_cb);
    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 100, imuCallback);

    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 10);
    ros::Publisher vel_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/velocity_sp", 10);
    ros::Publisher pos_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/position_sp", 10);


    ros::Rate loop_rate(30);

    float vel_x_prev, vel_y_prev;
    int i = 0;

    while (ros::ok())
    {
       float pos_k_p,pos_k_i, vel_x_k_p , vel_x_k_i, vel_x_k_d, vel_y_k_p , vel_y_k_i, vel_y_k_d, set_alt;
        nh.getParam("/vin_trajectory_track_controller/pos_k_p", pos_k_p);
        nh.getParam("/vin_trajectory_track_controller/vel_x_k_p", vel_x_k_p);
        nh.getParam("/vin_trajectory_track_controller/vel_x_k_d", vel_x_k_d);
        nh.getParam("/vin_trajectory_track_controller/vel_y_k_p", vel_y_k_p);
        nh.getParam("/vin_trajectory_track_controller/vel_y_k_d", vel_y_k_d);
        nh.getParam("/vin_trajectory_track_controller/set_alt", set_alt);
        // nh.getParam("/vin_trajectory_track_controller/x_des", x_des);
        // nh.getParam("/vin_trajectory_track_controller/y_des", y_des);

        mocap.header.stamp = ros::Time::now();
        setpoint.header.stamp = ros::Time::now();
        vel_sp.header.stamp = ros::Time::now();
        pos_sp.header.stamp = ros::Time::now();

        for(int ii=0; ii < trajectory_size; ii++)
        {
            float x_sp = traj.markers[traj_marker_size - 1].points[ii].x;
            float y_sp = traj.markers[traj_marker_size - 1].points[ii].y;
            
            if ( (x_sp - 0.1) < x && (x_sp + 0.1) > x ) 
                x_des = traj.markers[traj_marker_size - 1].points[ii+1].x; 
            if ( (y_sp - 0.1) < y && (y_sp + 0.1) > y )
                y_des = traj.markers[traj_marker_size - 1].points[ii+1].y;
            cout<<"traj"<<x_des<<","<<y_des<<endl;
        }
        
        pos_sp.pose.position.x = x_des;
        pos_sp.pose.position.y = y_des;

        if (odom_detected_flag == 1)
        {

            if (i == 0)
            {
                vel_x_prev = vel_x;
                vel_y_prev = vel_y;
            }
            else
            {
                err_sum_pos_x = err_sum_pos_x + (x_des - x);
                err_sum_pos_y = err_sum_pos_y + (y_des - y);

                if (mode_ == "OFFBOARD")
                {
                    nh.getParam("/vin_trajectory_track_controller/pos_k_i", pos_k_i);
                }
                else
                {
                    pos_k_i = 0;
                    err_sum_pos_y = 0;
                    err_sum_pos_x = 0;
                }

                cout << "Error sum pos = " << err_sum_pos_x * 0.03 * pos_k_i << " , " << err_sum_pos_y * 0.03 * pos_k_i << endl;


                vel_sp_x = (x_des - x) * pos_k_p + (err_sum_pos_x)*0.03 * pos_k_i;
                vel_sp_y = (y_des - y) * pos_k_p + (err_sum_pos_y)*0.03 * pos_k_i;

                err_sum_x = err_sum_x + (vel_x - vel_sp_x);
                err_sum_y = err_sum_y + (vel_y - vel_sp_y);

                if(mode_=="OFFBOARD")
                {
                    nh.getParam("/vin_trajectory_track_controller/vel_x_k_i", vel_x_k_i);
                    nh.getParam("/vin_trajectory_track_controller/vel_y_k_i", vel_y_k_i);
                }
                else
                {
                    vel_x_k_i = 0;
                    vel_y_k_i = 0;
                    err_sum_y = 0;
                    err_sum_x = 0;
                }

                // vel_sp_y = -0.0f;
                // vel_sp_x = 0.0f;

                if (vel_sp_x < -vel_thresh || vel_sp_x > vel_thresh || vel_sp_y < -vel_thresh || vel_sp_y > vel_thresh)
                    vel_cross_flag = 1;

                if (vel_sp_y > vel_thresh)
                    vel_sp_y = vel_thresh;
                else if (vel_sp_y < -vel_thresh)
                    vel_sp_y = -vel_thresh;

                if (vel_sp_x > vel_thresh)
                    vel_sp_x = vel_thresh;
                else if (vel_sp_x < -vel_thresh)
                    vel_sp_x = -vel_thresh;

                if (vel_cross_flag == 1)
                    cout << "vel Threshold reached" << endl;

                mocap.pose.position.y = (vel_y - vel_sp_y)*vel_y_k_p + (err_sum_y)*0.03*vel_y_k_i + (vel_y - vel_y_prev)*30*vel_y_k_d;//roll
                mocap.pose.position.x = (vel_x - vel_sp_x)*vel_x_k_p + (err_sum_x)*0.03*vel_x_k_i + (vel_x - vel_x_prev)*30*vel_x_k_d;//pitch

                vel_sp.pose.position.x = vel_sp_x;
                vel_sp.pose.position.y = vel_sp_y;
            }
            setpoint.pose.position.z = set_alt;

            vel_x_prev = vel_x;
            vel_y_prev = vel_y;

            i = i + 1;

            yaw = imu_yaw;

            q.setRPY(0, 0, yaw);

            setpoint.pose.orientation.z = q.z();
            setpoint.pose.orientation.w = q.w();

            if (mocap.pose.position.x < -sp_thresh || mocap.pose.position.x > sp_thresh || mocap.pose.position.y < -sp_thresh || mocap.pose.position.y > sp_thresh)
                cross_flag = 1;

            if (mocap.pose.position.x > sp_thresh)
                mocap.pose.position.x = sp_thresh;
            else if (mocap.pose.position.x < -sp_thresh)
                mocap.pose.position.x = -sp_thresh;
            if (mocap.pose.position.y < -sp_thresh)
                mocap.pose.position.y = -sp_thresh;
            else if (mocap.pose.position.y > sp_thresh)
                mocap.pose.position.y = sp_thresh;

            if (cross_flag == 1)
                cout << "Attitude Threshold reached" << endl;

            cout << "pitch = " << mocap.pose.position.x << std::endl
                 << "roll = " << mocap.pose.position.y << endl;
            setpoint_pub.publish(setpoint);
        }
        mocap.pose.position.z = dist;
        mocap_pub.publish(mocap);
        vel_sp_pub.publish(vel_sp);
        pos_sp_pub.publish(pos_sp);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void odomcb(const nav_msgs::Odometry::ConstPtr &msg)
{
    x = (msg->pose.pose.position.x);
    y = (msg->pose.pose.position.y);
    vel_x = (msg->twist.twist.linear.x);
    vel_y = -(msg->twist.twist.linear.y);

    odom_detected_flag = 1;
}

void distcb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    dist = msg->pose.position.z;
}

void statecb(const mavros_msgs::State::ConstPtr &msg)
{
    mode_ = msg->mode;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    if (init_imu_flag == 0)
    {
        tf::Quaternion q1(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf::Matrix3x3 m(q1);
        double r, p;
        m.getRPY(r, p, imu_yaw);
        init_imu_flag = 1;
    }
}

void traj_cb(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    traj = *msg;
    traj_marker_size = traj.markers.size();
    trajectory_size = traj.markers[traj_marker_size - 1].points.size();
}
