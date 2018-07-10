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
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <px_comm/OpticalFlow.h>

using namespace std;

/*flags for detection of msgs and threshold check*/
int odom_detected_flag = 0, cross_flag = 0, vel_cross_flag = 0, init_imu_flag = 0, object_yaw_flag = 0, tfmini_flag = 0;
int aruco_detected_flag = 0, landing_flag = 0, update_set_alt_flag = 0,  arucocb_count = 0;
float x = 0, y = 0, x_des = 0, y_des = 0, err_sum_x = 0.0, err_sum_y = 0.0, yaw_sp = 5.7, err_sum_pos_x = 0, err_sum_pos_y = 0;
float vel_x = 0, vel_y = 0, vel_thresh = 1.0, vel_sp_x = 0, z_dist, att_sp_thresh = 0.3, traj_sp_threshold = 0.08;
float vel_sp_y = 0, object_x, object_y, pick_goal_x_cb, pick_goal_y_cb, landing_threshold = 0.1, yaw_init = 5.7;
int  index_x = 0, index_y = 0,yaw_reset = 0, off_flag =1, grip_status = 0, trajectory_size = 0, take_off_flag = 0, distcb_count = 0;
double x_dist, imu_yaw, yaw_set,yaw_traj, yaw_marker, yaw_sp_temp;

float landing_time = 7;
float takeoff_time = 5;
float landing_time_threshold = 4;
float yaw_alignment_time = 4;
float landing_height = 0.3;
float gripping_sleep_time = 3;
float land_mode_sleep_time = 3;
string mode_;

tf::Quaternion q;
geometry_msgs::PoseStamped mocap, setpoint, vel_sp, pos_sp;
geometry_msgs::PoseArray traj;
std_msgs::Int32 gripper_pos, mission_reset_flag;

void odomcb(const nav_msgs::Odometry::ConstPtr &msg);
void distcb(const geometry_msgs::PoseStamped::ConstPtr &msg);
void statecb(const mavros_msgs::State::ConstPtr &msg);
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
void traj_cb(const geometry_msgs::PoseArray::ConstPtr &msg);
void gripper_state_cb(const std_msgs::Int32::ConstPtr &msg);
void arucocb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void flowcb(const px_comm::OpticalFlow::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vin_mission_control");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/rovio/odometry", 10, odomcb);
    ros::Subscriber dist_sub = nh.subscribe("/pose", 100, distcb);
    ros::Subscriber state_sub = nh.subscribe("/mavros/state", 100, statecb);
    ros::Subscriber traj_sub = nh.subscribe("/trajectory_with_yaw", 10, traj_cb);
    ros::Subscriber imu_sub = nh.subscribe("/mavros/imu/data", 100, imuCallback);
    ros::Subscriber gripper_sub = nh.subscribe("/gripper/grip_status", 10, gripper_state_cb);
    ros::Subscriber aruco_sub = nh.subscribe("/aruco_single/filtered_pose", 10, arucocb);
    ros::Subscriber flow_sub = nh.subscribe("/px4flow/opt_flow", 10, flowcb);

    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 10);
    ros::Publisher vel_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/velocity_sp", 10);
    ros::Publisher pos_sp_pub = nh.advertise<geometry_msgs::PoseStamped>("/position_sp", 10);
    ros::Publisher gripper_sp_pub = nh.advertise<std_msgs::Int32>("/gripper/position", 10);
    ros::Publisher mission_reset_flag_pub = nh.advertise<std_msgs::Int32>("/mission_reset_flag", 10);
    
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    mavros_msgs::SetMode  land_set_mode, offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mission_reset_flag.data = 0;

    double timer_=0, timer_land =0;
    float vel_x_prev, vel_y_prev, x_prev, y_prev, set_alt_temp = 0.8;
    int i = 0;

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        float pos_k_p_x, pos_k_p_y, pos_k_d,pos_k_i, vel_x_k_p , vel_x_k_i, vel_x_k_d, vel_y_k_p , vel_y_k_i, vel_y_k_d, set_alt;
        nh.getParam("/vin_mission_control/pos_k_p", pos_k_p_x);
        nh.getParam("/vin_mission_control/pos_k_p", pos_k_p_y);
        nh.getParam("/vin_mission_control/pos_k_d", pos_k_d);
        nh.getParam("/vin_mission_control/vel_x_k_p", vel_x_k_p);
        nh.getParam("/vin_mission_control/vel_x_k_d", vel_x_k_d);
        nh.getParam("/vin_mission_control/vel_y_k_p", vel_y_k_p);
        nh.getParam("/vin_mission_control/vel_y_k_d", vel_y_k_d);
        nh.getParam("/vin_mission_control/set_alt", set_alt);
        nh.getParam("/vin_mission_control/yaw_reset", yaw_reset);

        if(update_set_alt_flag==0)
        {
            set_alt_temp = set_alt;
            update_set_alt_flag=1;
        }

        mocap.header.stamp = ros::Time::now();
        setpoint.header.stamp = ros::Time::now();
        vel_sp.header.stamp = ros::Time::now();
        pos_sp.header.stamp = ros::Time::now();

        if( (mission_reset_flag.data ==1) && (mode_=="AUTO.LAND"))   
        {
            
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                cout<<"OFFBOARD enabled";
                off_flag=1;
                index_x = 0;
                index_y = 0;
                set_alt_temp = set_alt;
            }
        }
        if(mode_=="OFFBOARD" && trajectory_size>0)
        {
            if( off_flag ==1)
            {
            timer_ = ros::Time::now().toSec();
            off_flag =0;
            }

            if(ros::Time::now().toSec()-timer_ < takeoff_time )
            {
                // x_des = traj.poses[0].position.x;
                // y_des = traj.poses[0].position.y;
                
                
		        setpoint.pose.position.z = set_alt;

                cout<<"Timer ="<<ros::Time::now().toSec()-timer_<<endl;
            }
            else
            {
                for(int ii=0; ii < trajectory_size; ii++)
                {
                    float x_sp = traj.poses[ii].position.x;
                    float y_sp = traj.poses[ii].position.y;

                    
    
                    
                    if ( (x_sp - traj_sp_threshold) < x && (x_sp + traj_sp_threshold) > x  && (y_sp - traj_sp_threshold) < y && (y_sp + traj_sp_threshold) > y && ii+1<trajectory_size ) 
                    {
                        // index_x = ii;
                        x_des = traj.poses[ii+1].position.x; 
                 
                        // index_y = ii;
                        y_des = traj.poses[ii+1].position.y;

                        tf::Quaternion q1(
                        traj.poses[ii+1].orientation.x,
                        traj.poses[ii+1].orientation.y,
                        traj.poses[ii+1].orientation.z,
                        traj.poses[ii+1].orientation.w);

                        tf::Matrix3x3 m(q1);

                        double r, p;
                        m.getRPY(r, p, yaw_traj);
                        yaw_sp = (yaw_init+yaw_traj);

                        cout<<"yaw_sp"<<yaw_sp<<endl<<"yaw_init="<<yaw_init<<endl<<"yaw_traj"<<yaw_traj<<endl<<endl;
                    }
                }

                float pick_goal_x = traj.poses[trajectory_size-1].position.x;
                float pick_goal_y = traj.poses[trajectory_size-1].position.y;

                if(aruco_detected_flag == 1 &&  mission_reset_flag.data == 0)
                {
                    pick_goal_x = pick_goal_x_cb; 
                    pick_goal_y = pick_goal_y_cb; 

                    x_des = pick_goal_x;
                    y_des = pick_goal_y;

                }

                if ((pick_goal_x - landing_threshold) < x && (pick_goal_x + landing_threshold) > x && (pick_goal_y - landing_threshold) < y && (pick_goal_y + landing_threshold) > y )
                {    
                    cout<<"Stabilizing over object"<<endl;
                    arucocb_count = arucocb_count + 1;

                    if((ros::Time::now().toSec() - timer_land) >= yaw_alignment_time)
                    {
                        if(object_yaw_flag == 0 )
                        {
                            yaw_sp_temp = imu_yaw-yaw_marker;
                            object_yaw_flag = 1;
                        }
                        if(mission_reset_flag.data == 0)
                            yaw_sp = yaw_sp_temp;
                        cout<<"Aligning yaw over object"<<endl;
                        
                        if((ros::Time::now().toSec() - timer_land) >= (landing_time_threshold+yaw_alignment_time))
                        {
                            landing_flag = 1;
                            setpoint.pose.position.z = set_alt_temp-(ros::Time::now().toSec()-timer_land-landing_time_threshold-yaw_alignment_time)*((set_alt_temp-landing_height)/landing_time);
                            
                            cout<<"Landing"<<endl;

                            if( (ros::Time::now().toSec()-timer_land) >(landing_time+landing_time_threshold+yaw_alignment_time) )
                            {
                                if( set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent )
                                {
                                    ROS_INFO("land enabled");
                                    ros::Duration(land_mode_sleep_time).sleep();
                                    if(grip_status == 0)
                                    {
                                        gripper_pos.data = 1;
                                        cout<<"gripped"<<endl;
                                        
                                    }
                                    else if (grip_status == 1)
                                    {
                                        gripper_pos.data = 0;
                                    }

                                    //CHANGED HERE
                                    cout<<"Previus mision reset "<<mission_reset_flag.data<<endl;
                                    if(mission_reset_flag.data == 0)
                                    {
                                        mission_reset_flag.data = 1;
                                        cout<<"reseting mission"<<endl;
                                    }
                                    else
                                    {
                                        mission_reset_flag.data = 2;
                                        cout<<"DONE!"<<endl;
                                        ros::Duration(10).sleep();
                                    }

                                    mission_reset_flag_pub.publish(mission_reset_flag);
                                    gripper_sp_pub.publish(gripper_pos);
                                    ros::Duration(gripping_sleep_time).sleep();

                                }
                            }
                        }   
                    }    
                }
                else
                {
                    timer_land = ros::Time::now().toSec();
                    set_alt_temp = setpoint.pose.position.z;
                }
            } 
        }

        cout<<"traj"<<x_des<<","<<x<<","<<y_des<<","<<y<<endl<<endl;
        
        pos_sp.pose.position.x = x_des;
        pos_sp.pose.position.y = y_des;

        if (odom_detected_flag == 1 && tfmini_flag == 1)
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


                float vel_sp_x_world = (x_des - x)*pos_k_p_x + (err_sum_pos_x)*0.03*pos_k_i;
                float vel_sp_y_world = (y_des - y)*pos_k_p_y + (err_sum_pos_y)*0.03*pos_k_i;
                vel_sp_x = cos(yaw_traj)*vel_sp_x_world + sin(yaw_traj)* vel_sp_y_world;
                vel_sp_y = cos(yaw_traj)*vel_sp_y_world - sin(yaw_traj)* vel_sp_x_world;

                
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

                mocap.pose.position.y = ((vel_y - vel_sp_y)*vel_y_k_p + (err_sum_y)*0.03*vel_y_k_i + (vel_y - vel_y_prev)*30*vel_y_k_d);//roll
                mocap.pose.position.x = ((vel_x - vel_sp_x)*vel_x_k_p + (err_sum_x)*0.03*vel_x_k_i + (vel_x - vel_x_prev)*30*vel_x_k_d);//pitch

                vel_sp.pose.position.x = vel_sp_x;
                vel_sp.pose.position.y = -vel_sp_y;
            }

            vel_x_prev = vel_x;
            vel_y_prev = vel_y;
            x_prev = x;
            y_prev = y;

            i = i + 1;

            if(yaw_reset == 1)
            {
                
                yaw_init = imu_yaw;
                yaw_sp = imu_yaw;
                nh.setParam("/vin_mission_control/yaw_reset", 0);
            }

            q.setRPY(0, 0, yaw_sp);

            setpoint.pose.orientation.z = q.z();
            setpoint.pose.orientation.w = q.w();

            if (mocap.pose.position.x < -att_sp_thresh || mocap.pose.position.x > att_sp_thresh || mocap.pose.position.y < -att_sp_thresh || mocap.pose.position.y > att_sp_thresh)
                cross_flag = 1;

            if (mocap.pose.position.x > att_sp_thresh)
                mocap.pose.position.x = att_sp_thresh;
            else if (mocap.pose.position.x < -att_sp_thresh)
                mocap.pose.position.x = -att_sp_thresh;
            if (mocap.pose.position.y < -att_sp_thresh)
                mocap.pose.position.y = -att_sp_thresh;
            else if (mocap.pose.position.y > att_sp_thresh)
                mocap.pose.position.y = att_sp_thresh;

            if (cross_flag == 1)
                cout << "Attitude Threshold reached" << endl;


            /*check for obstacles in front  */
            if(x_dist < 0.7 )
                mocap.pose.position.x = 0.2 * ( 0.7 - x_dist  );     

            // cout << "pitch = " << mocap.pose.position.x << std::endl
            //      << "roll = " << mocap.pose.position.y << endl;
            setpoint_pub.publish(setpoint);
             tfmini_flag = 0;
        }

        mocap.pose.position.z = z_dist;
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

    double vel_with_cov[36], pos_with_cov[36];

    for(int iii = 0; iii<36; iii++)
    {
        vel_with_cov[iii] = fabs(msg->twist.covariance[iii]);
        pos_with_cov[iii] = fabs(msg->pose.covariance[iii]);
    }
    sort(vel_with_cov,vel_with_cov+36);
    sort(pos_with_cov,pos_with_cov+36);
    if( vel_with_cov[35]<1 && pos_with_cov[35]<1)
            odom_detected_flag = 1;
        else
            odom_detected_flag = 0;
}

void distcb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    z_dist = msg->pose.position.z;
    distcb_count = distcb_count + 1;

    if(distcb_count > 10)
        tfmini_flag = 1;
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

void traj_cb(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    traj = *msg;
    trajectory_size = traj.poses.size();
}

void gripper_state_cb(const std_msgs::Int32::ConstPtr &msg)
{
    grip_status = msg->data;
}

void arucocb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double current_heading = yaw_init-imu_yaw;
    if(landing_flag==0 )
    {
        object_x = -(cos(current_heading)*(msg->pose.position.y)+sin(current_heading)*(msg->pose.position.x)); 
    	object_y = -(cos(current_heading)*(msg->pose.position.x)-sin(current_heading)*(msg->pose.position.y));
	    if(object_x>-0.2 && object_x < 0.2 && object_y <0.2 && object_y >-0.2 && arucocb_count <100)
    	{
	        pick_goal_x_cb = x + object_x;
    	    pick_goal_y_cb = y + object_y;
            aruco_detected_flag = 1;
            // cout<<"x = "<<object_x<<"y = "<<object_y<<"arucocb_count = "<<arucocb_count<<endl;

             tf::Quaternion q2(
                msg->pose.orientation.x,
                msg->pose.orientation.y,
                msg->pose.orientation.z,
                msg->pose.orientation.w);
        
                tf::Matrix3x3 m1(q2);
    
                double r, p;
                m1.getRPY(r, p, yaw_marker);
        }
        
    }
}

void flowcb(const px_comm::OpticalFlow::ConstPtr& msg)
{
    double x_dist_temp = msg->ground_distance;
    double x_dist_median [16];
    
    for( int i =0; i<16; i++)
        x_dist_median[i] = x_dist_temp;

    sort(x_dist_median,x_dist_median+10);

    if ((x_dist_temp > (x_dist_median[8] - 0.3)) && (x_dist_temp < (x_dist_median[8] + 0.3)))
        x_dist = x_dist_temp;
    
    if(x_dist < 0.1)
        x_dist = 2.0;
    
    // cout<<"x_dist = "<<x_dist<<endl;

}