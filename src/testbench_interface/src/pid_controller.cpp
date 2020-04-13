#include "testbench_interface/mainwindow.h"

PID_Algorithm::PID_Algorithm(const float& controller_frequency)
{
    last_margin = 0.0;
    last_I = 0.0;
    Kp = 0.0;
    Ki = 0.0;
    Kd = 0.0;
    I_limit = 10000.0;
    ctrl_freq = controller_frequency;
}

void PID_Algorithm::set_Kp(const float& kp) {Kp = kp;}
void PID_Algorithm::set_Ki(const float& ki) {Ki = ki;}
void PID_Algorithm::set_Kd(const float& kd) {Kd = kd;}
void PID_Algorithm::set_I_limit(const float& i_limit) {I_limit = i_limit;}
float PID_Algorithm::read_Kp() {return Kp;}
float PID_Algorithm::read_Ki() {return Ki;}
float PID_Algorithm::read_Kd() {return Kd;}
float PID_Algorithm::read_I_limit() {return I_limit;}
float PID_Algorithm::PID_calculate(const float& target, const float& feedback)
{
    float margin, P, I, D, PID_output;
    margin = feedback - target;
    last_I = limitation(last_I + margin/ctrl_freq, I_limit);
    P = Kp * margin;
    I = Ki * last_I;
    D = Kd * (margin - last_margin) / ctrl_freq;
    last_margin = margin;

    PID_output = - P - I - D;
    return PID_output;
}
