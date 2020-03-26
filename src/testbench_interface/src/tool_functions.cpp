#include "testbench_interface/mainwindow.h"

float limitation(const float& value, const float& limit) {
    float upper_limit, lower_limit;
    upper_limit = fabs(limit);
    lower_limit = -upper_limit;

    if (value > upper_limit)
        return upper_limit;
    else if (value < lower_limit)
        return lower_limit;
    else
        return value;

}

bool reach_limit(const float& value, const float& limit) {
    float upper_limit, lower_limit;
    upper_limit = fabs(limit);
    lower_limit = -upper_limit;

    if (value > upper_limit)
        return true;
    else if (value < lower_limit)
        return true;
    else
        return false;
}

float loadMotorVoltage(const float& desired_force) {
    float p_desire = 30.0;   // preset in motor driver
    float desired_voltage = desired_force*135.0/40.0/14.3*p_desire/10.0;

    return desired_voltage;
}

void MainWindow::BLDC0_angle_protection(){
    if (roadwheel_angle > 400)
        control->stop();
    else if (roadwheel_angle < -400)
        control->stop();
}

void MainWindow::BLDC1_angle_protection(){
    if (steerwheel_angle > 400)
        control->stop();
    else if (steerwheel_angle < -400)
        control->stop();
}

// timeout function. NOT YET ACTIVE
void MainWindow::timeout_protection(){
    if (timeout_counter >= 10)
        control->stop();
}

