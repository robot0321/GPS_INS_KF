#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <chrono>

using namespace std;
using namespace std::chrono;

#include "AHRS.h"
#include "mpu9250.h"

vector<vector<float>> getTXT(string name, int row);

int main(int argc, char* argv[]){
	//********* Declare Variables ***************
	//vector<vector<float>> set_data;
	//vector<float> data_accel;
	vector<float> data_gyro;
	vector<float> Euler_angle;
	double data_mpu[7];
	int time_limit=0;
	int sampling_time_limit=30;
	bool print_ones = true;


	//********* processing input value **********
	ofstream outFile;
	if(argc>2){
		if(string(argv[1])=="-m"){
			outFile.open(string(argv[2]),ios::out);
			if(argc>3){
				if(string(argv[3])=="-t"){
					time_limit = stoi(argv[4]);
				}
			}
		}else if(string(argv[1])=="-s"){
			sampling_time_limit = stoi(argv[2]);
		}
	}


	//********** Declare Objects **************
	AHRS ahrs = AHRS();
	MPU9250 mpu = MPU9250(sampling_time_limit, 16, 250, 80000, 200); //sampling time, range of acc, range of gyro, spi transmission speed, spi delay
	time_point<system_clock> start, end, loop_start;
	duration<double> interval, livetime;


	//*************** setting **************
	//set_data.resize(8); //for 8 kinds of data
	//data_accel.resize(3);
	data_gyro.resize(3);
	Euler_angle.resize(3);



	//*************** Loop ****************
	loop_start = system_clock::now();
	while(1){
		start = system_clock::now();
		mpu.mpu9250read_all(data_mpu, 0);
		
		end = system_clock::now();
		interval = end - start;
		livetime = end - loop_start;


		//*********** showing time(sec)/acc/temp/gyro data *************
		cout<<livetime.count()<<" "<<interval.count()<<" ";
		for(int i=0;i<7;i++) cout<<data_mpu[i]<<" ";
		cout<<endl;
		


		/*********************** Get Querternion *************************
		cout<<livetime.count()<<" "<<interval.count()<<" ";
		for(int i=0;i<3;i++) data_gyro[i] = (float)data_mpu[i+4];
		ahrs.attitude_update(data_gyro, interval.count());
		for(int i=0;i<4;i++) cout<<ahrs.Q_0[i]<<"/";
		cout<<endl;
		*/
		


		/************** Making data file in limited time ****************
		if(print_ones) cout<<"Collecting mpu data for \""<<time_limit<<"sec\" & making TXT file named \""<<string(argv[2])<<"\""<<endl; print_ones = false;
		if(argc>2 && string(argv[1])=="-m" && outFile.is_open()){
			outFile<<livetime.count()<<", "<<interval.count();
			for(int i=0;i<7;i++) outFile<<", "<<data_mpu[i];
			outFile<<endl;	
		}
		if(argc>3 && time_limit<double(livetime.count())){
				outFile.close();
				cout<<"\nMaking txt file (for "<<livetime.count()<<"sec) is done"<<endl;
				break;
		}
		*/



		/********************** Euler Angle ***********************
		//needed "Get Quarternion" first
		
		Euler_angle = ahrs.Qaurt2Euler(ahrs.Q_0);
		for(int i=0; i<3; i++) cout<<Euler_angle[i]<<" ";
		cout<<endl;
		*/
		
		
	}




	/*
	// *********data collecting***************
	set_data = getTXT("new.txt", 8);
	
	for(int i=0;i<set_data[0].size()-1;i++){
		for(int j=0;j<3;j++){
			data_accel[j] = set_data[j+1][i];
			data_gyro[j] = set_data[j+5][i];
		}
		ahrs.attitude_update(data_gyro, (set_data[0][i]-time0)/10000.0;
		data_accel = ahrs.frame_transformer(ahrs.Q_0, data_accel);

		time0 = set_data[0][i];
		
		//print
		cout<<time0<<" [";
		for(int k=0;k<3;k++) cout<<ahrs.Q_0[k]<<", ";
		cout<<ahrs.Q_0[3]<<"]  \t  [";
		for(int k=0;k<2;k++) cout<<data_accel[k]<<", ";
		cout<<data_accel[2]<<"]"<<endl;
	}
	*/

	return 0;
}



vector<vector<float>> getTXT(string name, int row){
	vector<vector<float>> set_data;
	set_data.resize(row);

	char buff[20];
	ifstream inFile;
	inFile.open(name);
	while(inFile.good()){
		for(int i=0;i<8;i++){
			inFile.getline(buff, 20, ',');
			set_data[i].push_back(atof(buff));
		}
	}
	inFile.close();

	return set_data;
}