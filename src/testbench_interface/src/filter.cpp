#include "testbench_interface/mainwindow.h"

Filter_IIR_Butterworth_fs_100Hz_fc_4Hz::Filter_IIR_Butterworth_fs_100Hz_fc_4Hz()
{
    filter_init();
}

void Filter_IIR_Butterworth_fs_100Hz_fc_4Hz::filter_init() {
    a0 = 1.0;
    a1 = -1.6475;
    a2 = 0.7009;
    b0 = 1.0;
    b1 = 2.0;
    b2 = 1.0;
    scale = 0.0134;
    last_input = -1000000.0;
    last_last_input = -1000000.0;
    last_output = 0.0;
    last_last_output = 0.0;
}

float Filter_IIR_Butterworth_fs_100Hz_fc_4Hz::get_differential() {
    float differential = 0.0;
    differential = (last_output - last_last_output)*100;
    return differential;
}

float Filter_IIR_Butterworth_fs_100Hz_fc_4Hz::filter(const float& current_input) {
    float current_output = 0;

    if (last_last_input == -1000000.0 || last_last_output == -1000000.0)
    {current_output = current_input;}
    else
    {
        current_output += current_input * b0 * scale;
        current_output += last_input * b1 * scale;
        current_output += last_last_input * b2 * scale;
        current_output -= last_output * a1;
        current_output -= last_last_output * a2;
        current_output = current_output / a0;
    }

    last_last_input = last_input;
    last_input = current_input;
    last_last_output = last_output;
    last_output = current_output;

    return current_output;
}
