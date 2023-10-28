#include "Arduino.h"
#include "IMUdata.h"
#include "DC_Control.h"
#include "I2C_SCANNER.h"
#include "IMUconfig.h"
#include "PID.h"
#include "Wire.h"

/*Defines*/
#define system_runtime 20
#define DC
#define IMU1
//#define IMU2
//#define IMU3
//#define IMU_ALL
#define print_system_info

/*Variable declaration*/
double p = 0;
double i = 0;
double d = 0;
double main_currentTime;
double main_prevTime = 0;
double print_prevTime = 0;
double main_elapsTime = 0;
double runtime = 0;


/*Test functions*/

void print_system_IMU_info(int IMU_unit){
  Serial.println("-------------IMU details-------------");
  Serial.print("Yaw: ");
  Serial.print(IMU_angle_get_yaw(IMU_unit));
  Serial.print("  | Pitch: ");
  Serial.print(IMU_angle_get_pitch(IMU_unit));
  Serial.print("  | Roll: ");
  Serial.println(IMU_angle_get_roll(IMU_unit));
  Serial.println();
}

void print_system_PID_info(){
  Serial.println("-------------PID details-------------");
  Serial.print("Proportional: ");
  Serial.print(PID_getProportional());
  Serial.print("  | Derivative: ");
  Serial.print(PID_getDerivate());
  Serial.print("  | Integral: ");
  Serial.println(PID_getIntegral());
  Serial.println();
}

void print_system_SPEED_info(){
  Serial.println("-------------SPEED details-------------");
  Serial.print("Motor1 speed: ");
  Serial.print(DC_get_speed(1));
  Serial.print("  | Motor2 speed: ");
  Serial.print(DC_get_speed(2));
  Serial.print("  | Motor3 speed: ");
  Serial.println(DC_get_speed(3));
  Serial.println();
}

void print_system_TIME_info(double t){
  Serial.println("-------------TIME details-------------");
  Serial.print("Runtime: ");
  Serial.print(t);
  Serial.print("  | Time for PID1: ");
  Serial.print(PID_getEtime(1));
  Serial.print("  | Time for PID2: ");
  Serial.print(PID_getEtime(2));
  Serial.print("  | Time for PID3: ");
  Serial.println(PID_getEtime(3));
  Serial.println();
}



void setup(){
  Serial.begin(115200);
  while (!Serial){delay(10); }
  pinMode(IMU_pin_1,OUTPUT);
  pinMode(IMU_pin_2,OUTPUT);
  pinMode(IMU_pin_3,OUTPUT);
  digitalWrite(IMU_pin_1,HIGH);
  digitalWrite(IMU_pin_2,HIGH);
  digitalWrite(IMU_pin_3,HIGH);
  delay(1000);
  p = PID_getProportional();
  d = PID_getDerivate();
  i = PID_getIntegral();

  delay(2000);
  #ifdef DC
    DC_init(BREAK1, PWM1, DIR1, BREAK2, PWM2, DIR2, BREAK3, PWM2, DIR3);
  #endif
  #ifdef IMU1
    Wire.begin();
    init_IMU1();
  #endif
  #ifdef IMU2
    Wire.begin();
    init_IMU2();
  #endif
  #ifdef IMU3
    Wire.begin();
    init_IMU3();
  #endif
  #ifdef IMU_ALL
    IMU_init();
  #endif
}

void loop(){
  main_currentTime = millis();
  if(main_currentTime - main_prevTime >= system_runtime){    
    getAngle();
    DC_calculate_speed(IMU_angle_get_pitch(1), IMU_angle_get_pitch(2), IMU_angle_get_pitch(3));
    main_elapsTime = millis();
    runtime = main_elapsTime - main_currentTime;
    main_prevTime = main_currentTime;
  }
    #ifdef print_system_info
    if((main_currentTime - print_prevTime) >= 1000){
      print_system_IMU_info(1);
      //print_system_IMU_info(2);
      //print_system_IMU_info(3);
      print_system_SPEED_info();
      //print_system_TIME_info(runtime);
      print_prevTime  = main_currentTime;
    } 
    #endif
}