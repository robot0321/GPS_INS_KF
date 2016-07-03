#include <iostream>
#include <mpu9250.h>

using namespace std;

int main(int arg, char **argv){
	float accData[3];
	float gyroData[3];
	
	cout<<"Kalman Filter"<<endl;
	
	MPU9250 mpu = MPU9250();
	mpu.mpu9250Initialize();
	
	while(1){
		mpu.mpu9250read_acc(accData);
		for(int i=0;i<3;i++){
			cout<<accData[i]<<" ";
		}
		mpu.mpu9250read_gyro(gyroData);
		for(int i=0;i<3;i++){
			cout<<gyroData[i]<<" ";
		}
		cout<<endl;
		


	}
}


