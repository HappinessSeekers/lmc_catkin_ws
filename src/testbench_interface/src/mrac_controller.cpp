#include "testbench_interface/mainwindow.h"

MRAC_Algorithm::MRAC_Algorithm(const float& controller_frequency) {
    theta1 = 0.0;
    theta2 = 0.0;
    theta3 = 0.0;
    gamma1 = 0.0;
    gamma2 = 0.0;
    gamma3 = 0.0;
    ym = 0.0;
    ym_dot = 0.0;
    ctrl_freq = controller_frequency;
}

bool MRAC_Algorithm::set_reference_model(const float& wn, const float& ksi) {
    if(wn > ctrl_freq/2.0) {return false;}
    else if(ksi < 0) {return false;}
    else {
        // Setting p21 & p22
        p21 = 1.0/2.0/wn/wn;
        p22 = 1.0/4.0/ksi/wn*(1.0+1.0/wn/wn);
        // Setting Ak & Bk
        Ak[0][0] = 1.0;
        Ak[0][1] = 1.0/ctrl_freq;
        Ak[1][0] = -wn*wn/ctrl_freq;
        Ak[1][1] = -2.0*ksi*wn/ctrl_freq + 1.0;
        Bk[0][0] = 0.0;
        Bk[1][0] = wn*wn/ctrl_freq;
        return true;
    }
}

bool MRAC_Algorithm::set_gamma1(const float& gamma_input) {
    if(gamma_input < 0) {return false;}
    else {
        gamma1 = gamma_input;
        return true;
    }
}

bool MRAC_Algorithm::set_gamma2(const float& gamma_input) {
    if(gamma_input < 0) {return false;}
    else {
        gamma2 = gamma_input;
        return true;
    }
}

bool MRAC_Algorithm::set_gamma3(const float& gamma_input) {
    if(gamma_input < 0) {return false;}
    else {
        gamma3 = gamma_input;
        return true;
    }
}

float MRAC_Algorithm::MRAC_calculate(const float& r, const float& y, const float& y_dot) {
    float u;
    float e, e_dot, delta, theta1_dot, theta2_dot, theta3_dot;
    // Reference Model Update
    ym = Ak[0][0]*ym + Ak[0][1]*ym_dot + Bk[0][0]*r;
    ym_dot = Ak[1][0]*ym + Ak[1][1]*ym_dot + Bk[1][0]*r;
    // Calculating Error
    e = y-ym;
    e_dot = y_dot-ym_dot;
    delta = p21*e+0*p22*e_dot;
    // MRAC Output Calculation
    theta1_dot=-gamma1*delta*r;
    theta2_dot=-gamma2*delta*y;
    theta3_dot=-gamma3*delta*y_dot;
    theta1 = theta1 + theta1_dot/ctrl_freq;
    theta2 = theta2 + theta2_dot/ctrl_freq;
    theta3 = theta3 + theta3_dot/ctrl_freq;
    u = theta1*r+theta2*y+theta3*y_dot;
    return u;
}

float MRAC_Algorithm::get_theta1() {return theta1;}
float MRAC_Algorithm::get_theta2() {return theta2;}
float MRAC_Algorithm::get_theta3() {return theta3;}
float MRAC_Algorithm::get_ym() {return ym;}
float MRAC_Algorithm::get_ym_dot() {return ym_dot;}
float MRAC_Algorithm::calculate_delta(const float& r, const float& y, const float& y_dot) {
    float e, e_dot, delta;
    // Calculating Error
    e = y-ym;
    e_dot = y_dot-ym_dot;
    delta = p21*e+p22*e_dot;
    return delta;
}