/*
 * kalman.h
 *
 *  Created on: 25-Oct-2017
 *      Author: ksg
 */

#ifndef KALMAN_H_
#define KALMAN_H_


#include <Eigen/Core>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <px_comm/OpticalFlow.h>
#include <geometry_msgs/PoseStamped.h>

#include "sonar.h"

using namespace std;
using namespace Eigen;

namespace HMDETECTION{
class EKF{
public:
	EKF(ros::NodeHandle nh);
	void subscriber();
	enum sensorState{WAITING=0, INITIALIZED=1, ACQUIRED=2};
	enum ekfState{EKF_WAITING=0, EKF_INITIALIZED=1, EKF_PREDICTION=2, EKF_UPDATE=3};

private:
	void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
	void magCallback(const sensor_msgs::MagneticField::ConstPtr& msg);
	void sonarCallback(const px_comm::OpticalFlow::ConstPtr& msg);
	void ekfInitialize();
	void ekfPrediction();
	void ekfUpdate();
	void ekfUpdateHeight(double sonarDistance);

	ros::Publisher anglesPub, accelefPub, posePub;

	ros::NodeHandle nh_;

	Vector3d accelbf;
	Vector3d magbf;
	Vector3d omegabf;

	VectorXd X, X_;
	MatrixXd sigma, sigma_;

	sensorState imuState, magState, sonarState;
	ekfState ekfCurrentState;

	int imuSeq, magSeq;

	double currentImutime, previousImutime, deltaImutime, initialtime;

	double roll, pitch, yaw;
	sonar sonarVal;

	ros::Time time;
};

}

#endif /* KALMAN_H_ */
