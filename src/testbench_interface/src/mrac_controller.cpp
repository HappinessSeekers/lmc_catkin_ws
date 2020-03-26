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



