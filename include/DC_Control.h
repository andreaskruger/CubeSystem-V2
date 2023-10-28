#ifndef DC_CONTROL_H
#define DC_CONTROL_H
#include "Arduino.h"


/*
  5v || GND || BREAK || PWM || DIR 
  GRÖN BREAK
  VIt PWM
  BLÅ DIR

  DC Motorcontrol: 
  1: blå 14, vit 32, grön 17
  2: blå 12, vit 33, grön 16
  2: blå 13, vit 01, grön 04
*/

/*Defines*/
#define TIMER_BIT 8
#define BASE_FREQ 20000
#define PWM1_CH 1
#define PWM2_CH 2
#define PWM3_CH 3

#define BREAK1 17
#define PWM1 32
#define DIR1 14
#define DCmotor1 1

#define BREAK2 16
#define PWM2 33
#define DIR2 12
#define DCmotor2 2

#define BREAK3 4
#define PWM3 1
#define DIR3 13
#define DCmotor3 3

#define PID1 0
#define PID2 1
#define PID3 2
#define ANGLE_CUTOFPOINT 5

#define MaxSpeed 255

/*Prototypes*/

/**
 * @brief 
 * 
 */
void DC_balance_speed_1(float angle);

/**
 * @brief 
 * 
 */
void DC_balance_speed_2(float angle);

/**
 * @brief 
 * 
 */
void DC_balance_speed_3(float angle);

/**
 * @brief 
 * 
 * @param angle_1 
 * @param angle_2 
 * @param angle_3 
 */
void DC_calculate_speed(float angle_1, float angle_2, float angle_3);

/**
 * @brief 
 * 
 * @param motor 
 * @return float 
 */
float DC_get_speed(int motor);

/**
 * @brief 
 * 
 * @param motor 
 * @param angle 
 */
void DC_set_speed(int motor, float angle);

/**
 * @brief Initiates the DC motors controlling the flywheels
 * @note All pins are defined in "DC_Control.h"
 * 
 * @param break1 break pin motor 1
 * @param pwm1 pwm pin motor 1
 * @param dir1 direction pin motor 1
 * @param break2 break pin motor 2
 * @param pwm2 pwm pin motor 2
 * @param dir2 direction pin motor 2
 * @param break3 break pin motor 3
 * @param pwm3 pwm pin motor 3
 * @param dir3 direction pin motor 3
 * 
 */
void DC_init(uint8_t break1, uint8_t pwm1, uint8_t dir1, 
             uint8_t break2, uint8_t pwm2, uint8_t dir2,
             uint8_t break3, uint8_t pwm3, uint8_t dir3);


#endif