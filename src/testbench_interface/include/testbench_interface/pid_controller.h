#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PID_Algorithm
{
private:
    float last_margin, last_I, Kp, Ki, Kd, I_limit;
    float ctrl_freq;
public:
    PID_Algorithm(const float&);
    void set_Kp(const float&);
    void set_Ki(const float&);
    void set_Kd(const float&);
    void set_I_limit(const float&);
    float PID_calculate(const float&, const float&);
    float read_Kp();
    float read_Ki();
    float read_Kd();
    float read_I_limit();
};



#endif // PID_CONTROLLER_H
