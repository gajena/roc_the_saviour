/*
 * sonar.h
 *
 *  Created on: 19-Jun-2017
 *      Author: krissh
 */

#ifndef LIDAR_IMU_INCLUDE_SONAR_H_
#define LIDAR_IMU_INCLUDE_SONAR_H_

#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

namespace HMDETECTION{
class sonar{
	public:
		sonar();
		double getSonarFilteredData(double distance);

	private:
		double sonarVal;
		double threshold;
		double set_count;
		VectorXf sonar_set;
		int first_val_check;
		int no_sonar_data;
};
}



#endif /* LIDAR_IMU_INCLUDE_SONAR_H_ */
