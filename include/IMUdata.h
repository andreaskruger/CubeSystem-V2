#ifndef IMUDATA_H
#define IMUDATA_H
#include <Arduino.h>
#include <IMUconfig.h>


void changeDigiPin(int pin);

void init_IMU1();

void init_IMU2();

void init_IMU3();

void init_IMU();

void readBuffer1();

void readBuffer2();

void readBuffer3();

void readIMU1();

void readIMU2();

void readIMU3();

void getAngle();

float IMU_angle_get_yaw(int IMU_unit);

float IMU_angle_get_pitch(int IMU_unit);

float IMU_angle_get_roll(int IMU_unit);

#endif