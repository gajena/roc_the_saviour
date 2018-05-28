#include <ros/ros.h>
#include <boost/thread.hpp>

#include "kalman.h"
int main(int argc, char **argv){

	ros::init(argc, argv, "detection");
	ros::NodeHandle nh;


	//call ekf for height and orientation estimation
	HMDETECTION::EKF ekf(nh);
	boost::thread track(&HMDETECTION::EKF::subscriber, &ekf);

	ros::Rate loopFreq(100);

	while (ros::ok()){
		loopFreq.sleep();
	}

	return 0;
}
