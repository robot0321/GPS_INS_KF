#include <wiringPiSPI.h>
#include <wiringPi.h>

#include "mpu9250.h"

// MPU_Init_Data 변수에는 초기화할 값과 레지스터 주소가 들어감.
#define MPU_InitRegNum  15

/*
class MPU9250{
private:
	float gyro_scaler;
	float accelero_scaler;
	float magneto_scaler;	

public:
	char gyro_range;
	char accelero_range;
	int spi_speed;
	int spi_delayMS;

	MPU9250();
	MPU9250(int acc, int gyro, int speed, int delay);
	~MPU9250();

	void mpu9250Initialize();
	void mpu9250Write(unsigned char value, unsigned char regNum);
	unsigned char mpu9250Read(unsigned char regNum);
	void mpu9250Reads(unsigned char regNum, unsigned char* regBuf, int Bytes);
	void mpu9250read_acc(float* vector);
	void mpu9250read_mag(float* vector);
	void mpu9250read_gyro(float* vector);

};
*/

MPU9250::MPU9250(){ //default constructor	
	accelero_range = 0x08;
	gyro_range = 0x18;
	spi_speed = 800000; //speed limit : 1M
	spi_delayMS = 10000; //wait before SPI transmission
	gyro_scaler = (float)0.00106526421440972222222;
	accelero_scaler = (float)0.001197509765625;
	//magneto_scaler = 
}

MPU9250::MPU9250(int acc=0x08, int gyro=0x18, int speed=800000,int delay = 1000){
	switch(acc){
		case 2: accelero_range=0x00; accelero_scaler=1;
		case 4: accelero_range=0x08; accelero_scaler=1;
		case 8: accelero_range=0x10; accelero_scaler=1;
		case 16: accelero_range=0x18; accelero_scaler=1;
	}

	switch(gyro){
		case 250: gyro_range=0x00; gyro_scaler=1;
		case 500: gyro_range=0x08; gyro_scaler=1;
		case 1000: gyro_range=0x10; gyro_scaler=1;
		case 2000: gyro_range=0x18; gyro_scaler=1;
	}

	spi_speed = speed;
	spi_delayMS = delay;
}

MPU9250::~MPU9250(){
}

		 
void MPU9250::mpu9250Initialize(){
    int i;

    unsigned char MPU_Init_Data[MPU_InitRegNum][2] ={ //15*2 array
        {0x80, MPUREG_PWR_MGMT_1}, // Reset
        {0x01, MPUREG_PWR_MGMT_1}, // Auto select clock source
        {0x00, MPUREG_PWR_MGMT_2}, // Acc & Gyro enable
        {gyro_range, MPUREG_GYRO_CONFIG}, // +-2000dps
        {accelero_range, MPUREG_ACCEL_CONFIG}, // +-4G
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
    for(i=0;i<MPU_InitRegNum;i++){
        mpu9250Write(MPU_Init_Data[i][0], MPU_Init_Data[i][1]);
    }

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
void MPU9250::mpu9250read_acc(float* vector){
    unsigned char buffer[7];
    short temp;
    float data;

    // +-4G
    mpu9250Reads(0x3B, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    data = (float)temp*gyro_scaler;
    vector[0] = data;

    temp = ((short)buffer[3] << 8) | buffer[4];
    data = (float)temp*gyro_scaler;
    vector[1] = data;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    data = (float)temp*gyro_scaler;
    vector[2] = data;   
}

// 자기장 받아오기. 현재 작동 안함..
void MPU9250::mpu9250read_mag(float* vector){
    unsigned char buffer[7];
    short temp;

    mpu9250Write(AK8963_I2C_ADDR | 0x80, MPUREG_I2C_SLV0_ADDR); //Set the I2C slave addres of AK8963 and set for read.
    mpu9250Write(AK8963_HXL, MPUREG_I2C_SLV0_REG); //I2C slave 0 register address from where to begin data transfer
    mpu9250Write(0x86, MPUREG_I2C_SLV0_CTRL); //Read 6 bytes from the magnetometer

    delayMicroseconds(spi_delayMS);

    mpu9250Reads(MPUREG_EXT_SENS_DATA_00, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    vector[0] = (float)temp;

    temp = ((short)buffer[3] << 8) | buffer[4];
    vector[1] = (float)temp;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    vector[2] = (float)temp; 
}


// 자이로 받아오기
void MPU9250::mpu9250read_gyro(float* vector){
    unsigned char buffer[7];
    short temp;
    float data;

    // +-2000dps
    mpu9250Reads(0x43, buffer, 6);
    temp = ((short)buffer[1] << 8) | buffer[2];
    data = (float)temp*accelero_scaler;
    vector[0] = data;

    temp = ((short)buffer[3] << 8) | buffer[4];
    data = (float)temp*accelero_scaler;
    vector[0] = data;
    
    temp = ((short)buffer[5] << 8) | buffer[6];
    data = (float)temp*accelero_scaler;
    vector[0] = data;    
}
