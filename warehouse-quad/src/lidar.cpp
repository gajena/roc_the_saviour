#include "lidar.h"

using namespace Eigen;
using namespace std;

namespace HMDETECTION
{

lidar::lidar() : lidar_set(10), set_count(0.0), first_val_check(0), threshold(0.5), no_lidar_data(6.0)
{
}

double lidar::getlidarFilteredData(double distance)
{

	int i;
	VectorXf lidar_set_copy(10);

	if (set_count == 0)
	{
		lidar_set.setZero();
	}
	lidar_set_copy = lidar_set;
	if (set_count < 10)
	{
		//getting first 40 set of lidar data
		lidar_set_copy[set_count] = distance;
		lidar_set[set_count] = distance;
		set_count++;
		lidarVal = distance;
		if (set_count == 9)
		{
			first_val_check = 1;
		}
	}

	else
	{

		sort(lidar_set_copy.data(), lidar_set_copy.data() + lidar_set_copy.size()); //arranging the 400 set of data in increasing order
		//cout << lidar_set_copy[200] << "\t \t" << Px4flowData->ground_distance<<endl;

		if ((distance > (lidar_set_copy[5] - threshold)) && (distance < (lidar_set_copy[5] + threshold)))
		{ //checking if the current value is in a limit of the median value
			//if so then return the current lidar data
			lidarVal = distance;
			for (i = 0; i < 9; i++)
			{
				lidar_set[i] = lidar_set[i + 1]; // updating the lidar set for next 400 data
			}
			lidar_set[9] = distance;
			return lidarVal;
		}

		else
		{
			return no_lidar_data;
		}
	}
	return no_lidar_data;
}

} // namespace HMDETECTION
