/*-----------------------------------------------------------------------------------------------------------------
----* Subscriber	/aruco_single/pose - position feedback from aruco marker
										 and y - vertically down 
										 and x --  90 degree couter clockwise.--------------------------------------

----* Publisher 	/mavros/vision_pose/pose - publish processed value from PID controller as euler angle setpoint
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
#include <geometry_msgs/Vector3Stamped.h>

using namespace std;

/*flags for detection of aruco and threshold check*/
int aruco_detected_flag =0 ,cross_flag=0;
float x=0, y=0,x_des = 0, y_des=0;
float k_p = 0.03334, k_i = 0.0000, k_d = 0; //i=0.000034

float sp_thresh = 0.08, err_sum_x = 0.0, err_sum_y = 0.0;
float yaw = 5.7;

tf::Quaternion q;
geometry_msgs::PoseStamped vision_pose;
geometry_msgs::PoseStamped setpoint;
geometry_msgs::PoseStamped dist;
geometry_msgs::Vector3Stamped vel;

void arucocb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	x = (msg->pose.position.y);
  	y = (msg->pose.position.x);

  	aruco_detected_flag = 1;
}

float distance_tfmini;

void tfminicb(const sensor_msgs::Range::ConstPtr &msg)
{
	distance_tfmini = msg->range;
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle nh;

    ros::Subscriber aruco_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aruco_single/pose", 10, arucocb);
    ros::Subscriber tfmini_pub = nh.subscribe<sensor_msgs::Range>("/tfmini_ros_node/TFmini", 100, tfminicb);

	ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Vector3Stamped>("/mavros/vision_speed/speed_vector",10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate loop_rate(250);
    
    float x_prev = 0, y_prev = 0;
    int i=0;

    while ( ros::ok() )
    {

		if( aruco_detected_flag == 1)        
		{
			vision_pose.header.stamp = ros::Time::now();
	        setpoint.header.stamp = ros::Time::now();
			dist.header.stamp = ros::Time::now();
			vel.header.stamp = ros::Time::now();

			//mavros_msgs::SetMode offb_set_mode;
	        //offb_set_mode.request.custom_mode = "OFFBOARD";

	        //mavros_msgs::CommandBool arm_cmd;
	        //arm_cmd.request.value = true;

	       //  err_sum_x = err_sum_x + x;
	       //  err_sum_y = err_sum_y + y;

			if (i==0)
			{
				i=i+1;
			}
	       //   else
	       //   { 
		     	// vision_pose.pose.position.y = (x-x_des)*k_p + (err_sum_x)*k_i + (x-x_prev)*k_d + 0.02;//roll
	       //      vision_pose.pose.position.x = (y-y_des)*k_p + (err_sum_y)*k_i + (y-y_prev)*k_d;//pitch
	       //   } 
	        
	        setpoint.pose.position.z = 1.8f;
	        setpoint.pose.position.x = 0.0f;
	        setpoint.pose.position.y = 0.0f;
	        vision_pose.pose.position.x =  x;
	    	vision_pose.pose.position.y = y;
			vision_pose.pose.position.z = distance_tfmini;

	        	
			//vel.vector.x = (x-x_prev)/0.1;
			//vel.vector.y =  (y-y_prev)/0.1;

			x_prev=x;
		    y_prev=y;

	        q.setRPY(0, 0, yaw);

	        setpoint.pose.orientation.z = q.z();
			setpoint.pose.orientation.w = q.w();

	  //       if ( vision_pose.pose.position.x > sp_thresh )
			// 	vision_pose.pose.position.x=sp_thresh;
			// else if ( vision_pose.pose.position.x < -sp_thresh )
			// 	vision_pose.pose.position.x = -sp_thresh;
			// if ( vision_pose.pose.position.y < -sp_thresh )
			// 	vision_pose.pose.position.y = -sp_thresh;
			// else if ( vision_pose.pose.position.y > sp_thresh )
			// 	vision_pose.pose.position.y = sp_thresh;

			// if ( vision_pose.pose.position.x < -sp_thresh || vision_pose.pose.position.x > sp_thresh || vision_pose.pose.position.y < -sp_thresh || vision_pose.pose.position.y > sp_thresh )
			// 	cross_flag= 1;

			// if ( cross_flag==1 )
			// 	cout<<"Attitude Threshold reached"<<endl;
			
			// cout<<"aruco_detected"<<endl<<"pitch = "<<vision_pose.pose.position.x<<endl<< "roll = "<< vision_pose.pose.position.y<<endl;
			// cout<<"aruco_x = "<<x<<endl<<"aruco_y = "<<y<<endl;

			setpoint_pub.publish(setpoint);
	        vision_pub.publish(vision_pose);
	        //vel_pub.publish(vel);
		}
        
        else

        {
        	vision_pose.header.stamp = ros::Time::now();
        	vision_pose.pose.position.z = distance_tfmini;
        	vision_pub.publish(vision_pose);
        }
    	ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

