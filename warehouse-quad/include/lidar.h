/*
 * lidar.h
 *
 *  Created on: 19-Jun-2017
 *      Author: krissh
 */

#ifndef LIDAR_IMU_INCLUDE_lidar_H_
#define LIDAR_IMU_INCLUDE_lidar_H_

#include "ros/ros.h"
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace std;

namespace HMDETECTION
{
class lidar
{
  public:
	lidar();
	double getlidarFilteredData(double distance);

  private:
	double lidarVal;
	double threshold;
	double set_count;
	VectorXf lidar_set;
	int first_val_check;
	int no_lidar_data;
};
} // namespace HMDETECTION

#endif /* LIDAR_IMU_INCLUDE_lidar_H_ */
