#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>

using namespace std;
using namespace std::chrono;

#include "AHRS.h"
#include "mpu9250.h"

void getTXT(string name);

vector<vector<float>> set_data;
vector<float> data_gyro;
vector<float> data_accel;

int main(){
	AHRS ahrs1 = AHRS();
	MPU9250 mpu1 = MPU9250();
	time_point<system_clock> start, end;
	duration<double> interval;
	float vector[7];

	
	//setting
	set_data.resize(8); //for 8 kinds of data
	data_accel.resize(3);
	data_gyro.resize(3);
	mpu1.mpu9250Initialize();
	

	while(1){
		start = system_clock::now();
		
		mpu1.mpu9250read_all(vector, 0);
		
		end = system_clock::now();
		interval = end - start;

		cout<<interval.count()<<" ";
		for(int i=0;i<7;i++){
			cout<<vector[i]<<" ";
		}
		cout<<endl;
	}
	/*
	//data collecting
	getTXT("new.txt");
	
	for(int i=0;i<set_data[0].size()-1;i++){
		for(int j=0;j<3;j++){
			data_accel[j] = set_data[j+1][i];
			data_gyro[j] = set_data[j+5][i];
		}
		ahrs1.attitude_update(data_gyro, (set_data[0][i]-time0)/10000.0;
		data_accel = ahrs1.frame_transformer(ahrs1.Q_0, data_accel);

		time0 = set_data[0][i];
		
		//print
		cout<<time0<<" [";
		for(int k=0;k<3;k++) cout<<ahrs1.Q_0[k]<<", ";
		cout<<ahrs1.Q_0[3]<<"]  \t  [";
		for(int k=0;k<2;k++) cout<<data_accel[k]<<", ";
		cout<<data_accel[2]<<"]"<<endl;
	}
	*/

	return 0;
}

void getTXT(string name){
	char buff[20];
	ifstream file;
	file.open(name);
	while(file.good()){
		for(int i=0;i<8;i++){
			file.getline(buff, 20, ',');
			set_data[i].push_back(atof(buff));
		}
	}
	file.close();
}


