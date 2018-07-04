#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "mavros_msgs/AttitudeTarget.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Int32.h>
using namespace std;

/*flags for detection of msgs and threshold check*/
int odom_detected_flag = 0, cross_flag = 0, vel_cross_flag = 0, init_imu_flag = 0, aruco_detected_flag = 0;
float x = 0, y = 0, x_des = 0, y_des = 0, err_sum_x = 0.0, err_sum_y = 0.0, yaw = 5.7, err_sum_pos_x = 0, err_sum_pos_y = 0;
float vel_x = 0, vel_y = 0, vel_thresh = 1.0, vel_sp_x = 0, dist, sp_thresh = 0.3, vel_sp_y = 0, object_x, object_y, pick_goal_x_cb, pick_goal_y_cb;
int  index_x = 0, index_y = 0,yaw_reset = 0, off_flag =1, grip_status = 0, traj_marker_size = 0, trajectory_size = 0, take_off_flag = 0;
double imu_yaw;
string mode_;

tf::Quaternion q;
geometry_msgs::PoseStamped mocap, setpoint, vel_sp, pos_sp;
visualization_msgs::MarkerArray traj;
std_msgs::Int32 gripper_pos, mission_reset_flag;

void odomcb(const nav_msgs::Odometry::ConstPtr &msg);
void distcb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void statecb(const mavros_msgs::State::ConstPtr &msg);
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
void traj_cb(const visualization_msgs::MarkerArray::ConstPtr &msg);
void gripper_state_cb(const std_msgs::Int32::ConstPtr &msg);
void arucocb(const geometry_msgs::PoseStamped::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vin_mission_control");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/rovio/odometry", 10, odomcb);
    ros::Subscriber dist_sub = nh.subscribe("/pose", 100, distcb);
    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 100, statecb);
    ros::Subscriber traj_sub = nh.subscribe("trajectory_traject", 10, traj_cb);
    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 100, imuCallback);
    ros::Subscriber gripper_sub = nh.subscribe("/gripper/grip_status", 10, gripper_state_cb);
    ros::Subscriber aruco_sub = nh.subscribe("/aruco_single/pose", 10, arucocb);

    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 10);
    ros::Publisher vel_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/velocity_sp", 10);
    ros::Publisher pos_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/position_sp", 10);
    ros::Publisher gripper_sp_pub = nh.advertise<std_msgs::Int32>("/gripper/position", 10);
    ros::Publisher mission_reset_flag_pub = nh.advertise<std_msgs::Int32>("/mission_reset_flag", 10);
    
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode take_off_mode, land_set_mode, offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    land_set_mode.request.custom_mode = "AUTO.LAND";
    take_off_mode.request.custom_mode = "AUTO.TAKEOFF";

    mission_reset_flag.data = 0;

    double timer_=0, timer_land =0;
    float vel_x_prev, vel_y_prev, x_prev, y_prev;
    int i = 0;

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        float pos_k_p, pos_k_d,pos_k_i, vel_x_k_p , vel_x_k_i, vel_x_k_d, vel_y_k_p , vel_y_k_i, vel_y_k_d, set_alt;
        nh.getParam("/vin_mission_control/pos_k_p", pos_k_p);
        nh.getParam("/vin_mission_control/pos_k_d", pos_k_d);
        nh.getParam("/vin_mission_control/vel_x_k_p", vel_x_k_p);
        nh.getParam("/vin_mission_control/vel_x_k_d", vel_x_k_d);
        nh.getParam("/vin_mission_control/vel_y_k_p", vel_y_k_p);
        nh.getParam("/vin_mission_control/vel_y_k_d", vel_y_k_d);
        nh.getParam("/vin_mission_control/set_alt", set_alt);
        nh.getParam("/vin_mission_control/yaw_reset", yaw_reset);

        mocap.header.stamp = ros::Time::now();
        setpoint.header.stamp = ros::Time::now();
        vel_sp.header.stamp = ros::Time::now();
        pos_sp.header.stamp = ros::Time::now();

        if( (mission_reset_flag.data ==1 || take_off_flag == 1) && (mode_=="AUTO.LAND" || mode_=="AUTO.TAKEOFF" ))   
        {
            if(mode_=="AUTO.LAND")
                take_off_flag = 2;
            
            
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("OFFBOARD enabled");
                off_flag=1;
                index_x = 0;
                index_y = 0;
            }
        }
        //cout<<"take_off_flag = "<<take_off_flag<<endl;
        if(mode_=="OFFBOARD" && traj_marker_size>0)
        {
            if( off_flag ==1)
            {
            timer_ = ros::Time::now().toSec();
            off_flag =0;
            }

            if(ros::Time::now().toSec()-timer_ < 10 )
            {
                x_des = traj.markers[traj_marker_size - 1].points[0].x;
                y_des = traj.markers[traj_marker_size - 1].points[0].y;
                
                /*if(take_off_flag == 0 ||take_off_flag == 2 )
                {
                    if( set_mode_client.call(take_off_mode) && take_off_mode.response.mode_sent )
                    {
                        take_off_flag = take_off_flag+1;
                        cout<<"take_off_mode"<<endl;

                    }
                    
                }
                if (ros::Time::now().toSec()-timer_ <5 ) 
                    setpoint.pose.position.z = 0.5;                                
		        else if (ros::Time::now().toSec()-timer_ <8 )
              	    setpoint.pose.position.z = 0.0+(ros::Time::now().toSec()-timer_)*0.1;
		        else */
		        setpoint.pose.position.z = 0.8;

                cout<<"Timer ="<<ros::Time::now().toSec()-timer_<<endl;
            }
            else
            {
                for(int ii=0; ii < trajectory_size; ii++)
                {
                    float x_sp = traj.markers[traj_marker_size - 1].points[ii].x;
                    float y_sp = traj.markers[traj_marker_size - 1].points[ii].y;
                    
                    if ( (x_sp - 0.06) < x && (x_sp + 0.06) > x && ii >= index_x && (ii)<trajectory_size && (y_sp - 0.06) < y && (y_sp + 0.06) > y && ii>= index_y ) 
                    {
                        index_x = ii;
                        x_des = traj.markers[traj_marker_size - 1].points[ii].x; 
                 
                        index_y = ii;
                        y_des = traj.markers[traj_marker_size - 1].points[ii].y;
                    }
                }

                float pick_goal_x = traj.markers[traj_marker_size - 1].points[trajectory_size-1].x;
                float pick_goal_y = traj.markers[traj_marker_size - 1].points[trajectory_size-1].y;

                if(aruco_detected_flag == 1 &&  mission_reset_flag.data == 0)
                {
                    pick_goal_x = pick_goal_x_cb; 
                    pick_goal_y = pick_goal_y_cb; 

                    x_des = pick_goal_x;
                    y_des = pick_goal_y;

                }

                if ((pick_goal_x - 0.1) < x && (pick_goal_x + 0.1) > x && (pick_goal_y - 0.1) < y && (pick_goal_y + 0.1) > y )
                {    
                    cout<<"inside landing"<<endl;

                    if((ros::Time::now().toSec() - timer_land) >= 4.0)
                    {
                        setpoint.pose.position.z = 0.8-(ros::Time::now().toSec()-timer_land-4.0)*0.15;
                        cout<<"landing"<<endl;

                        if( (ros::Time::now().toSec()-timer_land >7) )
                        {
                            if( set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent )
                            {
                                ROS_INFO("land enabled");
                                ros::Duration(3).sleep();

                                if(grip_status == 0)
                                {
                                    gripper_pos.data = 1;
cout<<"gripped"<<endl;
                                    gripper_sp_pub.publish(gripper_pos);
                                    ros::Duration(4).sleep();

                                    if(  mission_reset_flag.data == 0)
                                        mission_reset_flag.data = 1;
                                    else
                                        mission_reset_flag.data = 2;

                                    mission_reset_flag_pub.publish(mission_reset_flag);
                                    //off_flag = 1
                                }
                                else if (grip_status == 1)
                                {
                                    gripper_pos.data = 0;
                                    gripper_sp_pub.publish(gripper_pos);
                                    ros::Duration(4).sleep();
                                    mission_reset_flag.data = 2;
                                    mission_reset_flag_pub.publish(mission_reset_flag);
                                }
                                
                            }
                        }
                        
                    }
                }
                else
                timer_land = ros::Time::now().toSec();
            } 
        }

        cout<<"traj"<<x_des<<","<<x<<","<<y_des<<","<<y<<endl<<","<<index_x<<","<<index_y<<endl;
        
        pos_sp.pose.position.x = x_des;
        pos_sp.pose.position.y = y_des;

        if (odom_detected_flag == 1)
        {
            if (i == 0)
            {
                vel_x_prev = vel_x;
                vel_y_prev = vel_y;
                x_prev = x;
                y_prev = y;
            }
            else
            {
                err_sum_pos_x = err_sum_pos_x + (x_des - x);
                err_sum_pos_y = err_sum_pos_y + (y_des - y);

                if (mode_ == "OFFBOARD")
                {
                    nh.getParam("/vin_mission_control/pos_k_i", pos_k_i);
                }
                else
                {
                    pos_k_i = 0;
                    err_sum_pos_y = 0;
                    err_sum_pos_x = 0;
                }

                //cout << "Error sum pos = " << err_sum_pos_x * 0.03 * pos_k_i << " , " << err_sum_pos_y * 0.03 * pos_k_i << endl;


                vel_sp_x = (x_des - x) * pos_k_p + (err_sum_pos_x)*0.03 * pos_k_i + (x - x_prev)*30*pos_k_d;
                vel_sp_y = (y_des - y) * pos_k_p + (err_sum_pos_y)*0.03 * pos_k_i + (y - y_prev)*30*pos_k_d;
		        //cout<<"pos_d = "<<(y - y_prev)*30*pos_k_d<<endl;
                err_sum_x = err_sum_x + (vel_x - vel_sp_x);
                err_sum_y = err_sum_y + (vel_y - vel_sp_y);

                if(mode_=="OFFBOARD")
                {
                    nh.getParam("/vin_mission_control/vel_x_k_i", vel_x_k_i);
                    nh.getParam("/vin_mission_control/vel_y_k_i", vel_y_k_i);
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

            vel_x_prev = vel_x;
            vel_y_prev = vel_y;
            x_prev = x;
            y_prev = y;

            i = i + 1;

            if(yaw_reset == 1)
            {
                yaw = imu_yaw;
                nh.setParam("/vin_mission_control/yaw_reset", 0);
            }

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

            //cout << "pitch = " << mocap.pose.position.x << std::endl
                 //<< "roll = " << mocap.pose.position.y << endl;
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
        tf::Quaternion q1(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        tf::Matrix3x3 m(q1);
        double r, p;
        m.getRPY(r, p, imu_yaw);
}

void traj_cb(const visualization_msgs::MarkerArray::ConstPtr &msg)
{
    traj = *msg;
    traj_marker_size = traj.markers.size();
    trajectory_size = traj.markers[traj_marker_size - 1].points.size();
}

void gripper_state_cb(const std_msgs::Int32::ConstPtr &msg)
{
    grip_status = msg->data;
    cout<<"grip_status = "<<grip_status<<endl;
}

void arucocb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    object_x = -(msg->pose.position.y); 
    object_y = -(msg->pose.position.x);
    pick_goal_x_cb = x + object_x;
    pick_goal_y_cb = y + object_y;

    aruco_detected_flag = 1;
}

