#g++ -std=c++11 -g -Wall mpu9250.cpp imuKF.cpp -I./ -lwiringPi -o KFimu
#g++ -std=c++11 -g -Wall AHRS.cpp -I./ -o AHRS
g++ -std=c++11 -g -Wall -ggdb AHRS.cpp main.cpp -I./ -o main


