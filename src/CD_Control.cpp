#include "DC_Control.h"
#include "PID.h"

/*Variable declarations*/
float speed[3];

float DC_get_speed(int motor){
    return speed[motor - 1];
}

void DC_set_speed(int motor, float angle){
    speed[motor - 1] = PID_angle(angle, motor);
}

void DC_init(uint8_t break1, uint8_t pwm1, uint8_t dir1, 
             uint8_t break2, uint8_t pwm2, uint8_t dir2,
             uint8_t break3, uint8_t pwm3, uint8_t dir3){
    
    delay(100);
    pinMode(break1, OUTPUT);
    pinMode(dir1, OUTPUT);
    digitalWrite(break1, LOW);
    ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
    ledcAttachPin(pwm1, PWM1_CH );
  
    pinMode(break2, OUTPUT);
    pinMode(dir2, OUTPUT);
    digitalWrite(break2, LOW);
    ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
    ledcAttachPin(pwm2, PWM2_CH );

    pinMode(break3, OUTPUT);
    pinMode(dir3, OUTPUT);
    digitalWrite(break3, LOW);
    ledcSetup(PWM3_CH, BASE_FREQ, TIMER_BIT);
    ledcAttachPin(pwm3, PWM3_CH );

    delay(100);
    digitalWrite(DIR1,HIGH);
    digitalWrite(DIR2,HIGH);
    digitalWrite(DIR3,HIGH);
    delay(100);
}

void DC_calculate_speed(float angle_1, float angle_2, float angle_3){
    DC_set_speed((int)DCmotor1, angle_1);
    DC_set_speed((int)DCmotor2, angle_2);
    DC_set_speed((int)DCmotor3, angle_3);
    DC_balance_speed_1(angle_1);
    DC_balance_speed_2(angle_2);
    DC_balance_speed_3(angle_3); 
}

void setPWM(int channel, int value){
    value = 255 - value;
    ledcWrite(channel,value);
}

void DC_balance_speed_1(float angle){
    if(speed[0]<0){
        digitalWrite(DIR1, LOW);
        speed[0] = speed[0]*(-1);
    }else{
        digitalWrite(DIR1, HIGH);
    }
    if(abs(speed[0]) > MaxSpeed){speed[0] = MaxSpeed;}
    if(angle > ANGLE_CUTOFPOINT || angle < -ANGLE_CUTOFPOINT){speed[0] = 0;}
    if(speed[0] == 0){digitalWrite(BREAK1,LOW);}
    if(speed[0] != 0){digitalWrite(BREAK1,HIGH);}
    setPWM(PWM1_CH, speed[0]);
}

void DC_balance_speed_2(float angle){
    if(speed[1]<0){
        digitalWrite(DIR2, LOW);
        speed[1] = speed[1]*(-1);
    }else{
        digitalWrite(DIR2, HIGH);
    }
    if(abs(speed[1]) > MaxSpeed){speed[1] = MaxSpeed;}
    if(angle > ANGLE_CUTOFPOINT || angle < -ANGLE_CUTOFPOINT){speed[1] = 0;}
    if(speed[1] == 0){digitalWrite(BREAK2,LOW);}
    if(speed[1] != 0){digitalWrite(BREAK2,HIGH);}
    setPWM(PWM2_CH, speed[1]);
}

void DC_balance_speed_3(float angle){
    if(speed[2]<0){
        digitalWrite(DIR3, LOW);
        speed[2] = speed[2]*(-1);
    }else{
        digitalWrite(DIR3, HIGH);
    }
    if(abs(speed[2]) > MaxSpeed){speed[2] = MaxSpeed;}
    if(angle > ANGLE_CUTOFPOINT || angle < -ANGLE_CUTOFPOINT){speed[2] = 0;}
    if(speed[2] == 0){digitalWrite(BREAK3,LOW);}
    if(speed[2] != 0){digitalWrite(BREAK3,HIGH);}
    setPWM(PWM3_CH, speed[2]);
}