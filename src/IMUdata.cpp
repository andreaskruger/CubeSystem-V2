#include <Arduino.h>
#include <IMUconfig.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <cmath>


// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu1(0x68);
MPU6050 mpu2(0x68);
MPU6050 mpu3(0x68);

//IMU 1
bool dmpReady1 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus1;   // holds actual interrupt status byte from MPU
uint8_t devStatus1;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize1;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount1;     // count of all bytes currently in FIFO
uint8_t fifoBuffer1[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
VectorInt16 gyro[3];          // Gyro daa x,y,z
float yaw,pitch,roll;

//IMU 2
bool dmpReady2 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus2;   // holds actual interrupt status byte from MPU
uint8_t devStatus2;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize2;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount2;     // count of all bytes currently in FIFO
uint8_t fifoBuffer2[64]; // FIFO storage buffer

Quaternion q2;           // [w, x, y, z]         quaternion container
VectorInt16 aa2;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal2;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld2;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity2;    // [x, y, z]            gravity vector
float euler2[3];         // [psi, theta, phi]    Euler angle container
float ypr2[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float yaw2,pitch2,roll2;

//IMU 3
bool dmpReady3 = false;  // set true if DMP init was successful
uint8_t mpuIntStatus3;   // holds actual interrupt status byte from MPU
uint8_t devStatus3;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize3;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount3;     // count of all bytes currently in FIFO
uint8_t fifoBuffer3[64]; // FIFO storage buffer

Quaternion q3;           // [w, x, y, z]         quaternion container
VectorInt16 aa3;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal3;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld3;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity3;    // [x, y, z]            gravity vector
float euler3[3];         // [psi, theta, phi]    Euler angle container
float ypr3[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float yaw3,pitch3,roll3;

int IMU_pins[3] = {IMU_pin_1,IMU_pin_2,IMU_pin_3};

#define OUTPUT_READABLE_YAWPITCHROLL
#define numberOfIMU 1

#if i2CDEV_IMPLEMENTATION == I2C_ARDUINO_WIRE
    #include "Wire.h"
#endif

float IMU_angle_get_yaw(int IMU_unit){
    return yaw;
}

float IMU_angle_get_pitch(int IMU_unit){
    return pitch;
}

float IMU_angle_get_roll(int IMU_unit){
    return roll;
}

void changeDigiPin(int pin){
    if(pin == 1){
        digitalWrite(IMU_pin_1,LOW);
        digitalWrite(IMU_pin_2,HIGH);
        digitalWrite(IMU_pin_3,HIGH);
    }
    if(pin == 2){
        digitalWrite(IMU_pin_1,HIGH);
        digitalWrite(IMU_pin_2,LOW);
        digitalWrite(IMU_pin_3,HIGH);
    }
    if(pin == 3){
        digitalWrite(IMU_pin_1,HIGH);
        digitalWrite(IMU_pin_2,HIGH);
        digitalWrite(IMU_pin_3,LOW);
    }
}

void init_IMU1(){
    changeDigiPin(1);
    delay(100);
    Serial.println(F("Initializing I2C device 1..."));
    mpu1.initialize();
    // verify connection
    Serial.println(F("Testing device 1 connections..."));
    Serial.println(mpu1.testConnection() ? F("MPU6050 connection IMU 1 successful") : F("MPU6050 connection IMU 1 failed"));
    Serial.println(F("Initializing DMP 1..."));

    devStatus1 = mpu1.dmpInitialize();
    Serial.println("Dev status 1: "+devStatus1);

    mpu1.setXGyroOffset(85);
    mpu1.setYGyroOffset(20);
    mpu1.setZGyroOffset(-14);
    mpu1.setZAccelOffset(1550); 

    mpu1.CalibrateAccel(7);
    mpu1.CalibrateGyro(7);
    mpu1.PrintActiveOffsets();
    mpu1.setDMPEnabled(true);
    dmpReady1 = true;
    packetSize1 = mpu1.dmpGetFIFOPacketSize();
}
void init_IMU2(){
    changeDigiPin(2);
    delay(100);
    Serial.println(F("Initializing I2C device 2..."));
    mpu2.initialize();
    // verify connection
    Serial.println(F("Testing device 2 connections..."));
    Serial.println(mpu2.testConnection() ? F("MPU6050 connection IMU 2 successful") : F("MPU6050 connection IMU 2 failed"));
    Serial.println(F("Initializing DMP 2..."));

    devStatus2 = mpu2.dmpInitialize();
    Serial.println("Dev status 2: "+devStatus2);

    mpu2.setXGyroOffset(73);
    mpu2.setYGyroOffset(-35);
    mpu2.setZGyroOffset(2);
    mpu2.setZAccelOffset(1504); 

    mpu2.CalibrateAccel(7);
    mpu2.CalibrateGyro(7);
    mpu2.PrintActiveOffsets();
    mpu2.setDMPEnabled(true);
    dmpReady2 = true;
    packetSize2 = mpu2.dmpGetFIFOPacketSize();
}
void init_IMU3(){
    changeDigiPin(3);
    delay(100);
    Serial.println(F("Initializing I2C device 3..."));
    mpu3.initialize();
    // verify connection
    Serial.println(F("Testing device 3 connections..."));
    Serial.println(mpu3.testConnection() ? F("MPU6050 connection IMU 3 successful") : F("MPU6050 connection IMU 3 failed"));
    Serial.println(F("Initializing DMP 3..."));

    devStatus3 = mpu3.dmpInitialize();
    Serial.println("Dev status 3: "+devStatus3);

    mpu3.setXGyroOffset(34);
    mpu3.setYGyroOffset(-44);
    mpu3.setZGyroOffset(-47);
    mpu3.setZAccelOffset(11358); 

    mpu3.CalibrateAccel(7);
    mpu3.CalibrateGyro(7);
    mpu3.PrintActiveOffsets();
    mpu3.setDMPEnabled(true);
    dmpReady3 = true;
    packetSize3 = mpu3.dmpGetFIFOPacketSize();
}


void init_IMU() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    #if numberOfIMU == 1 || numberOfIMU == 2||  numberOfIMU == 3
        init_IMU1(); //Blake tape
    #endif
    #if numberOfIMU == 2 || numberOfIMU == 3
        init_IMU2(); //White tape
    #endif
        #if numberOfIMU == 3
        init_IMU3(); //No tape
    #endif

}

void readBuffer1(){
    changeDigiPin(1);
    mpu1.resetFIFO();
    fifoCount1 = mpu1.getFIFOCount();
    while (fifoCount1 < packetSize1){fifoCount1 = mpu1.getFIFOCount();}
    mpu1.getFIFOBytes(fifoBuffer1,packetSize1);
}
void readBuffer2(){
    changeDigiPin(2);
    mpu2.resetFIFO();
    fifoCount2 = mpu2.getFIFOCount();
    while (fifoCount2 < packetSize2){fifoCount2 = mpu2.getFIFOCount();}
    mpu2.getFIFOBytes(fifoBuffer2,packetSize2);
}
void readBuffer3(){
    changeDigiPin(3);
    mpu3.resetFIFO();
    fifoCount3 = mpu3.getFIFOCount();
    while (fifoCount3 < packetSize3){fifoCount3 = mpu3.getFIFOCount();}
    mpu3.getFIFOBytes(fifoBuffer3,packetSize3);
}

void readIMU1(){
    readBuffer1();
    mpu1.dmpGetQuaternion(&q, fifoBuffer1);
    mpu1.dmpGetGravity(&gravity, &q);
    mpu1.dmpGetYawPitchRoll(ypr, &q, &gravity);
    yaw = ypr[0] * 180/PI;
    roll = ypr[1] * 180/PI;
    pitch = ypr[2] * 180/PI;
}

void readIMU2(){
    readBuffer2();
    mpu2.dmpGetQuaternion(&q2, fifoBuffer2);
    mpu2.dmpGetGravity(&gravity2, &q2);
    mpu2.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
    yaw2 = ypr2[0] * 180/PI;
    roll2 = ypr2[1] * 180/PI;
    pitch2 = ypr2[2] * 180/PI;
}

void readIMU3(){
    readBuffer3();
    mpu3.dmpGetQuaternion(&q3, fifoBuffer3);
    mpu3.dmpGetGravity(&gravity3, &q3);
    mpu3.dmpGetYawPitchRoll(ypr3, &q3, &gravity3);
    yaw3 = ypr3[0] * 180/PI;
    roll3 = ypr3[1] * 180/PI;
    pitch3 = ypr3[2] * 180/PI;

}

void getAngle(){
    readIMU1();
    readIMU2();
    readIMU3();
}
