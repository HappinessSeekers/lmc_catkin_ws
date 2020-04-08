#ifndef FILTER_H
#define FILTER_H
class Filter_IIR_Butterworth_fs_100Hz_fc_4Hz
{
private:
    float a0, a1, a2, b0, b1, b2, scale;
    float last_input, last_last_input, last_output, last_last_output;
public:
    Filter_IIR_Butterworth_fs_100Hz_fc_4Hz();
    void filter_init();
    float get_differential();
    float filter(const float&);
};


#endif // FILTER_H
