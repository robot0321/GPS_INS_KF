#!/bin/bash
g++ -g -Wall mpu9250.cpp imuKF.cpp -I./ -lwiringPi -o KFimu && ./KFimu
