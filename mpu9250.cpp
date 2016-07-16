#include "mpu9250.h"

#include <wiringPiSPI.h>
#include <wiringPi.h>
#include <chrono>
#include <cmath>
#include <iostream>

using namespace std;
using namespace std::chrono;

// MPU_Init_Data 변수에는 초기화할 값과 레지스터 주소가 들어감.
#define MPU_InitRegNum  15
#define SPI_transtime 200
#define PI 3.141592

MPU9250::MPU9250(){ //default constructor	
	accelero_range = 0x08;
	gyro_range = 0x18;
	spi_speed = 800000; //speed limit : 1M
	spi_delayMS = SPI_transtime; //min:100 //wait before SPI transmission
	gyro_scaler     = 0.0010652642;    // +-2000dps
	accelero_scaler = 0.0001220703125;    // +-4G, let g=9.8m/s^2
    temp_scaler = 0.0;

    //for(int i=0;i<3;i++){scaler[i] = accelero_scaler; scaler[i+4] = gyro_scaler;}
	//scaler[3] = temp_scaler;
    
    mpu9250Initialize(10);  //sampling_time_for_calibration

}

MPU9250::MPU9250(int sampling_time_for_calibration, int acc, int gyro, int spi_speed_edit, int spi_delay){
	switch(acc){
		case 2: accelero_range=0x00; 
		case 4: accelero_range=0x08; 
		case 8: accelero_range=0x10; 
		case 16: accelero_range=0x18; 
	}
    accelero_scaler = double(acc)/32786.0; //we don't know 'g(gravity accel)' so we will measure it later. but now, its unit is g

	switch(gyro){
		case 250: gyro_range=0x00; 
		case 500: gyro_range=0x08;
		case 1000: gyro_range=0x10;
		case 2000: gyro_range=0x18; 
	}
    gyro_scaler = double(gyro)/32768.0*PI/180.0; 

	spi_speed = spi_speed_edit;
	spi_delayMS = spi_delay;

    mpu9250Initialize(sampling_time_for_calibration);


}

MPU9250::~MPU9250(){
}

		 
void MPU9250::mpu9250Initialize(int sampling_time_for_calibration){
    unsigned char MPU_Init_Data[MPU_InitRegNum][2] ={ //15*2 array
        {0x80, MPUREG_PWR_MGMT_1}, // Reset
        {0x01, MPUREG_PWR_MGMT_1}, // Auto select clock source
        {0x00, MPUREG_PWR_MGMT_2}, // Acc & Gyro enable


        //********MPU setting************
        {gyro_range, MPUREG_GYRO_CONFIG}, 
        {accelero_range, MPUREG_ACCEL_CONFIG}, 
        //DLPF


        {0x30, MPUREG_INT_PIN_CFG},
        //{0x40, MPUREG_I2C_MST_CTRL},   // I2C Speed 348 kHz
        //{0x20, MPUREG_USER_CTRL},      // Enable AUX
        {0x20, MPUREG_USER_CTRL},       // I2C Master mode
        {0x0D, MPUREG_I2C_MST_CTRL}, //  I2C configuration multi-master  IIC 400KHz

        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR},  //Set the I2C slave addres of AK8963 and set for write.
        //{0x09, MPUREG_I2C_SLV4_CTRL},
        //{0x81, MPUREG_I2C_MST_DELAY_CTRL}, //Enable I2C delay

        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO}, // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL},  //Enable I2C and set 1 byte

        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG}, //I2C slave 0 register address from where to begin data transfer
        {0x12, MPUREG_I2C_SLV0_DO}, // Register value to continuous measurement in 16bit
        {0x81, MPUREG_I2C_SLV0_CTRL}  //Enable I2C and set 1 byte
    };

    // WiringPiSPI에 내장 함수로 SPI를 시작하려면 아래 함수를 불러야 함.
    // 라즈베리파이에는 2개의 SPI채널이 존재하는데 pxf mini에서는 0번채널에 barometer, 1번채널엔 mpu9250이 부착되어 있음.
    // MPU_CHANNEL 은 1로 되어 있음.
    // 500000은 SPI 통신속도임. MPU9250의 최대 SPI 속도는 1MHz이므로 이것보다 작게 설정해야 함.
    wiringPiSPISetup(MPU_CHANNEL, spi_speed);

    // 각 레지스터에 값을 씀
    for(int i=0;i<MPU_InitRegNum;i++) mpu9250Write(MPU_Init_Data[i][0], MPU_Init_Data[i][1]);
    offset_samplingNsetting(sampling_time_for_calibration);

}

// mpu9250으로부터 하나의 레지스터 값을 받아오는 함수
unsigned char MPU9250::mpu9250Read(unsigned char regNum){
    unsigned char buffer[2];

    buffer[0] = regNum | 0x80;
    buffer[1] = 0x00;
    wiringPiSPIDataRW(MPU_CHANNEL, buffer, 2);
    delayMicroseconds(spi_delayMS);

    return buffer[1];
}

// mpu9250으로부터 여러개의 값을 한번에 받아오는 함수
// regNum은 받기 시작할 주소이고, Bytes는 regNum으로부터 연속으로 받아올 바이트 수
// readBuf는 데이터를 저장할 변수. 이 때 주의할 점은 readBuf의 크기는 Bytes+1 보다 같거나 커야함.
// 왜냐하면 readBuf의 첫번째 항에는 번지수가 들어가고, 2번째부터 맨 뒤에까지는 받아온 데이터들이 저장되기 때문
void MPU9250::mpu9250Reads(unsigned char regNum, unsigned char* readBuf, int Bytes){
    readBuf[0] = regNum | 0x80;
    wiringPiSPIDataRW(MPU_CHANNEL, readBuf, Bytes + 1);
    delayMicroseconds(spi_delayMS);
}

// mpu9250에 값을 씀.
void MPU9250::mpu9250Write(unsigned char value, unsigned char regNum){
    unsigned char buffer[2];

    buffer[0] = regNum;
    buffer[1] = value;

    wiringPiSPIDataRW(MPU_CHANNEL, buffer, 2);
    delayMicroseconds(spi_delayMS);
}

// 가속도 받아오기.
void MPU9250::mpu9250read_acc(double* vector){
    unsigned char buffer[7];
    short temp;
    double data;

    // +-4G
    mpu9250Reads(0x3B, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    data = (double)temp*accelero_scaler;
    vector[0] = data;

    temp = ((short)buffer[3] << 8) | buffer[4];
    data = (double)temp*accelero_scaler;
    vector[1] = data;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    data = (double)temp*accelero_scaler;
    vector[2] = data;   
}

// 자기장 받아오기. 현재 작동 안함..
void MPU9250::mpu9250read_mag(double* vector){
    unsigned char buffer[7];
    short temp;

    mpu9250Write(AK8963_I2C_ADDR | 0x80, MPUREG_I2C_SLV0_ADDR); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250Write(AK8963_HXL, MPUREG_I2C_SLV0_REG); //I2C slave 0 register address from where to begin data transfer
    mpu9250Write(0x86, MPUREG_I2C_SLV0_CTRL); //Read 6 bytes from the magnetometer

    delayMicroseconds(spi_delayMS);

    mpu9250Reads(MPUREG_EXT_SENS_DATA_00, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    vector[0] = (double)temp;

    temp = ((short)buffer[3] << 8) | buffer[4];
    vector[1] = (double)temp;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    vector[2] = (double)temp; 
}


// 자이로 받아오기
void MPU9250::mpu9250read_gyro(double* vector, bool raw_enable=false){
    unsigned char buffer[7];
    short temp;
    double data;

    // +-2000dps
    mpu9250Reads(0x43, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    data = (double)temp*gyro_scaler;
    vector[0] = data - DATA_GYRO_OFFSET[0]*(1-raw_enable);

    temp = ((short)buffer[3] << 8) | buffer[4];
    data = (double)temp*gyro_scaler;
    vector[1] = data - DATA_GYRO_OFFSET[1]*(1-raw_enable);
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    data = (double)temp*gyro_scaler;
    vector[2] = data - DATA_GYRO_OFFSET[2]*(1-raw_enable);    
}

void MPU9250::mpu9250read_all(double* vector, bool raw_enable = false){
    gyro_scaler = (1-raw_enable)*gyro_scaler + raw_enable*1;
    accelero_scaler = (1-raw_enable)*accelero_scaler + raw_enable*1;
    
    mpu9250read_acc(vector);
    vector[3]=0;
    mpu9250read_gyro(vector+4, raw_enable);
    

    /*
    unsigned char buffer[15];

    short temp;
    double data;

    mpu9250Reads(0x3B, buffer, 14);
    for(int i=0;i<7;i++){
        temp = ((short)buffer[2*i+1] << 8) | buffer[2*i+2];
        data = (double)temp*gyro_scaler;
        vector[i] = data;
    }
    */
    /*
    unsigned char buffer[15];
    mpu9250Reads(0x3B, buffer, 14);
    for(int i=0;i<7;i++) vector[i] = ((double)(((short)buffer[2*i+1]<<8) | buffer[2*i+2]))*scaler[i];//(scaler[i]*(1-raw_enable) + 1*raw_enable);}
    */
}

void MPU9250::offset_samplingNsetting(int sampling_time_for_calibration){
    //offset sampling
    double data_vector[7];
    int data_storage[7] = {};
    double DATA_RAW_AVERAGE[7];
    int count = 0;
    double norm_accel;
    int time_buffer=0;

    time_point<system_clock> end, loop_start;
    duration<double> livetime;

    delayMicroseconds(20000);
    cout<<"Calibraion Time: "<<sampling_time_for_calibration<<" sec"<<endl;

    loop_start = system_clock::now();
    while(1){
        mpu9250read_all(data_vector, true);
        for(int i=0;i<7;i++) data_storage[i] += int(data_vector[i]); //you must check the data variance "not to be overflowed"
        count++;

        livetime = system_clock::now() - loop_start;
        if(int(livetime.count())>time_buffer){
            time_buffer+=1;
            cout<<"("<<time_buffer<<"/"<<sampling_time_for_calibration<<") sec..."<<endl;
        }    
        if(double(livetime.count()) >= sampling_time_for_calibration){
            for(int i=0;i<7;i++){
                DATA_RAW_AVERAGE[i] = double(data_storage[i])/double(count); //This is average (or expectation)
                //for(j=0;j<count;count++){DATA_GYRO_VAR[i] =     } //This if for variance to use in Gaussian Filter (e.g. Kalman Filter)
            }
            break;
        }
    }
    
    //accelero : configuration of value of 'g'
    //Explaination :: We just can get accelero sensor value as a factor of 'g'. 
    //                If we don't know what 'g' is on that surface, we can't get accurate acccelation value.
    //                The code below means, with assuming "stopped-initializing-states" (which is same as "1g-state")
    //                we can scaling it to our taste, especailly 9.8 as an example. 
    norm_accel = sqrt(pow(DATA_RAW_AVERAGE[0]*accelero_scaler,2) + pow(DATA_RAW_AVERAGE[1]*accelero_scaler,2) + pow(DATA_RAW_AVERAGE[2]*accelero_scaler,2));
    accelero_scaler *= 9.8/norm_accel;

    //gyro : configuration of a
    for(int i=0;i<3;i++) DATA_GYRO_OFFSET[i] = DATA_RAW_AVERAGE[i+4];
     
}