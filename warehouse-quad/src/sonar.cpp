#include "sonar.h"

using namespace Eigen;
using namespace std;

namespace HMDETECTION{

sonar::sonar():sonar_set(10),set_count(0.0),first_val_check(0),threshold(0.5), no_sonar_data(6.0){

}

double sonar::getSonarFilteredData(double distance){

	int i;
	VectorXf sonar_set_copy(10);

	if (set_count==0){
		sonar_set.setZero();
	}
	sonar_set_copy = sonar_set;
	if (set_count < 10){
		//getting first 40 set of sonar data
		sonar_set_copy[set_count] = distance;
		sonar_set[set_count] = distance;
		set_count++;
		sonarVal = distance;
		if(set_count==9){
			first_val_check = 1;
		}
	}

	else{

		sort(sonar_set_copy.data(), sonar_set_copy.data()+sonar_set_copy.size()); //arranging the 400 set of data in increasing order
		//cout << sonar_set_copy[200] << "\t \t" << Px4flowData->ground_distance<<endl;

		if((distance > (sonar_set_copy[5]-threshold))&&(distance < (sonar_set_copy[5]+threshold))){ //checking if the current value is in a limit of the median value
			//if so then return the current sonar data
			sonarVal = distance;
			for(i=0;i<9;i++){
				sonar_set[i] = sonar_set[i+1]; // updating the sonar set for next 400 data
			}
			sonar_set[9] = distance;
			return sonarVal;
		}

		else{
			return no_sonar_data;
		}

	}
	return no_sonar_data;

}

}
