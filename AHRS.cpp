#include "AHRS.h"
#include <iostream>
#include <cmath>

using namespace std;

AHRS::AHRS(){

	Q_0.resize(sizeof(float)*4);
	Q_1.resize(sizeof(float)*4);
	Q_2.resize(sizeof(float)*4);
	w_0.resize(sizeof(float)*3);
	w_1.resize(sizeof(float)*3);
	w_2.resize(sizeof(float)*3);
	transforming_buffer.resize(sizeof(float)*3);

	Q_0[0]=1.0;Q_0[1]=0.0;Q_0[2]=0.0;Q_0[3]=0.0;
	w_0[0]=0.0;w_0[1]=0.0;w_0[2]=0.0;
	Q_1[0]=1.0;Q_1[1]=0.0;Q_1[2]=0.0;Q_1[3]=0.0;
	w_1[0]=0.0;w_1[1]=0.0;w_1[2]=0.0;
	Q_2[0]=1.0;Q_2[1]=0.0;Q_2[2]=0.0;Q_2[3]=0.0;
	w_2[0]=0.0;w_2[1]=0.0;w_2[2]=0.0;

	//Initial transforming to platform-frame
	/* inital gyro value must be all-zero, so it doesn't need to transform
	w_1 = frame_transformer(Q_1, w_1);
	w_2 = frame_transformer(Q_2, w_2);*/ 
}

AHRS::AHRS(vector<float> init_Q){
	for(int i=0;i<4;i++){
		Q_1[i] = init_Q[i];
		Q_2[i] = init_Q[i];
	}

}

AHRS::~AHRS(){
	 Q_0.clear();
	 Q_1.clear();
	 Q_2.clear();
	 w_0.clear();
	 w_1.clear();
	 w_2.clear();
	 transforming_buffer.clear();
}

void AHRS::attitude_update(vector<float> w_new, float dt){
	dt /= 1000.0;
	//Adams-Bashford method : Linear multistep(in this case, 2) method
	//w_1 & w_2 is already transformed (body to platform) gyro_value
	Q_0[0] = Q_1[0] - (3*(w_1[0]*Q_1[1] + w_1[1]*Q_1[2] + w_1[2]*Q_1[3]) - (w_2[0]*Q_2[1] + w_2[1]*Q_2[2] + w_2[2]*Q_2[3]))*dt/4;
	Q_0[1] = Q_1[1] + (3*(w_1[0]*Q_1[0] + w_1[2]*Q_1[2] - w_1[1]*Q_1[3]) - (w_2[0]*Q_2[0] + w_2[2]*Q_2[2] - w_2[1]*Q_2[3]))*dt/4;
	Q_0[2] = Q_1[2] + (3*(w_1[1]*Q_1[0] - w_1[2]*Q_1[1] + w_1[0]*Q_1[3]) - (w_2[1]*Q_2[0] - w_2[2]*Q_2[1] + w_2[0]*Q_2[3]))*dt/4;
	Q_0[3] = Q_1[3] + (3*(w_1[2]*Q_1[0] + w_1[1]*Q_1[1] - w_1[0]*Q_1[2]) - (w_2[2]*Q_2[0] + w_2[1]*Q_2[1] - w_2[0]*Q_2[2]))*dt/4;
	
	//norm
	float Q_sum = sqrt(Q_0[0]*Q_0[0] + Q_0[1]*Q_0[1] + Q_0[2]*Q_0[2] + Q_0[3]*Q_0[3]);
	for(int k=0;k<4;k++){
		Q_0[k] /= Q_sum;
	}

	// Preparing next function call : [0 => -1 => -2]
	w_0 = frame_transformer(Q_0, w_new); //priori transforming

	for(int i=0;i<3;i++){
		w_2[i] = w_1[i];
		w_1[i] = w_0[i];
	}

	for(int j=0;j<4;j++){
		Q_2[j] = Q_1[j];
		Q_1[j] = Q_0[j];
	}

	//renewed Attitude
}


vector<float> AHRS::frame_transformer(vector<float> Q, vector<float> v){
	//Quarternion is a frame-transformation alone.
	transforming_buffer[0] = (2*(Q[0]*Q[0]+Q[1]*Q[1])-1)*v[0] + 2*(Q[1]*Q[2]-Q[0]*Q[3])*v[1] + 2*(Q[1]*Q[3] + Q[0]*Q[2])*v[2];
	transforming_buffer[1] = 2*(Q[1]*Q[2] + Q[0]*Q[3])*v[0] + (2*(Q[0]*Q[0]+Q[2]*Q[2])-1)*v[1] + 2*(Q[2]*Q[3] - Q[0]*Q[1])*v[2];
	transforming_buffer[2] = 2*(Q[1]*Q[3] - Q[0]*Q[2])*v[0] + 2*(Q[2]*Q[3] + Q[0]*Q[1])*v[1] + (2*(Q[0]*Q[0]+Q[3]*Q[3])-1)*v[2];

	//return Q-transformed gyro
	return transforming_buffer;
} 
