#ifndef MRAC_CONTROLLER_H
#define MRAC_CONTROLLER_H

class MRAC_Algorithm
{
private:
    float gamma1, gamma2, gamma3; // MRAC Adaptation Parameters.
    float ym, ym_dot; // Reference Model State.
    float p21, p22, Ak[2][2], Bk[2][1];
    float theta1, theta2, theta3; // MRAC Outputs.
    float ctrl_freq;
public:
    MRAC_Algorithm(const float&);
    bool set_reference_model(const float&, const float&);
    bool set_gamma1(const float&);
    bool set_gamma2(const float&);
    bool set_gamma3(const float&);
    float MRAC_calculate(const float&, const float&, const float&);
};



#endif // MRAC_CONTROLLER_H
