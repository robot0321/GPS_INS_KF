#include <fstream>
#include <iostream>
#include <mpu9250.h>
#include <ctime>
//#include <signal.h>
//#include <cstdlib>
//#include <stdio.h>
//#include <unistd.h>

using namespace std;

//void signalHandler(sig_t s);
ofstream outFile("output.txt");

int main(int arg, char **argv){
	float accData[3];
	float gyroData[3];

	//signal(SIGINT, signalHandler); //Catch Keyboard Interrupt

	cout<<"Kalman Filter"<<endl;
	
	MPU9250 mpu = MPU9250();
	mpu.mpu9250Initialize();

	
	while(1){
		clock_t time = clock();
		cout<<time<<" ";
		outFile<<time<<"\t";
		
		mpu.mpu9250read_acc(accData);
		for(int i=0;i<3;i++){
			cout<<accData[i]<<" ";
			outFile<<accData[i]<<"\t";
		}
		mpu.mpu9250read_gyro(gyroData);
		for(int i=0;i<3;i++){
			cout<<gyroData[i]<<" ";
			outFile<<gyroData[i]<<"\t";
		}
		cout<<endl;
		outFile<<endl;
		


	}
}



/*
void signalHandler(int signum){
	cout<<"Interrupt signal "<<signum<<endl;
	try{
		outFile.close();
	} catch(int exception){
	}
	exit(signum);
}
*/

