#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

#include "AHRS.h"

void getTXT(string name);

vector<vector<float>> set_data;
vector<float> data_gyro;
vector<float> data_accel;

int main(){
	//setting
	float time0 = 0;
	AHRS ahrs1 = AHRS();
	set_data.resize(8); //for 8 kinds of data
	data_accel.resize(3);
	data_gyro.resize(3);
	cout<<"hello AHRS"<<endl;

	//data collecting
	getTXT("new.txt");
	
	for(int i=0;i<set_data[0].size()-1;i++){
		for(int j=0;j<3;j++){
			data_accel[j] = set_data[j+1][i];
			data_gyro[j] = set_data[j+5][i];
		}
		ahrs1.attitude_update(data_gyro, set_data[0][i]-time0);
		data_accel = ahrs1.frame_transformer(ahrs1.Q_0, data_accel);
		time0 = set_data[0][i];
		
		//print
		cout<<time0<<" [";
		for(int k=0;k<3;k++) cout<<ahrs1.Q_0[k]<<", ";
		cout<<ahrs1.Q_0[3]<<"] \t [";
		for(int k=0;k<2;k++) cout<<data_accel[k]<<", ";
		cout<<data_accel[2]<<"]"<<endl;
	}


	/*
	for(int i=0;i<set_data[0].size();i++){
		cout<<set_data[0][i]<<endl;
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


