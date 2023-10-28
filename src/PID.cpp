#include "Arduino.h"


double kp = 160.0;
double kd = 10.5;
double ki = 0.01;
double output_k = 1.0;
double currentPIDTime[3] = {0.0,0.0,0.0};
double lastTime[3] = {0.0,0.0,0.0};
double lastError[3] = {0.0,0.0,0.0};
double eTime[3] = {0.0,0.0,0.0};
double cum[3] = {0.0,0.0,0.0};

double der = 0.0;
double prop = 0.0;
float baseLineAngle = 0.;

void PID_setProportional(double P){
    kp = P;
}
void PID_setIntegral(double I){
    ki = I;
}
void PID_setDerivate(double D){
    kd = D;
}
double PID_getProportional(){
    return kp;
}
double PID_getIntegral(){
    return ki;
}
double PID_getDerivate(){
    return kd;
}
double PID_getEtime(int i){
    return eTime[i - 1];
}

float PID_angle(float angle, int id){
    float speed;
    float error = baseLineAngle - angle;
    lastTime[id - 1] = currentPIDTime[id - 1];
    currentPIDTime[id - 1] = millis();
    eTime[id - 1] = currentPIDTime[id - 1] - lastTime[id - 1];
    cum[id - 1] += error*eTime[id - 1];
    der = ((error-lastError[id - 1])/eTime[id - 1]);
    speed = (kp * error + kd * der + ki * cum[id - 1])*output_k;
    lastError[id - 1] = error;
    if (abs(angle) < 0.1){speed = 0;}
    return speed;
}