#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QString>
#include <QWidget>
#include <QVBoxLayout>
#include "ros/ros.h"
#include <string>
#include <cmath>
#include <iostream>
#include <ctime>
#include "testbench_interface/mrac_controller.h"
#include "testbench_interface/pid_controller.h"
#include "testbench_interface/filter.h"
#include "testbench_interface/tool_functions.h"
#include "testbench_interface/rviz.h"
#include "std_msgs/String.h"
#include <std_msgs/Empty.h>
#include "std_msgs/Float32.h"

namespace Ui {
class MainWindow;
}

class Control
{
public:
    Control(const float&);
    ~Control();
    float BLDC0_current;    // BLDC0_current: unit: A. roadwheel motor
    float BLDC1_current;    // BLDC1_current: unit: A. steerwheel motor
    bool clutch_state;    // clutch_state: 0 -> engaged;   1 -> disengaged.
    float loadmotor_voltage;    // loadmotor_voltage: unit: V
    bool ctrl_quit;    // ctrl_quit: 0 -> dont quit;   1 -> quit.s

    PID_Algorithm *steerwheelMotor_PID_controller;
    PID_Algorithm *roadwheelMotor_PID_controller;
    PID_Algorithm *loadMotor_PID_controller;

    MRAC_Algorithm *steerwheelMotor_MRAC_controller;
    MRAC_Algorithm *roadwheelMotor_MRAC_controller;

    void stop();
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    ros::NodeHandle n;
    ros::Subscriber roadwheel_angle_sub;
    ros::Subscriber steerwheel_angle_sub;
    ros::Subscriber BLDC0_sub;
    ros::Subscriber BLDC1_sub;
    ros::Subscriber Rackforce_feedback_sub;
    ros::Subscriber windows_matlab_sub;
    ros::Publisher loadMotor_pub;
    ros::Publisher clutch_pub;
    ros::Publisher BLDC0_current_pub;
    ros::Publisher BLDC1_current_pub;
    ros::Publisher windows_matlab_response_pub;

private slots:
    void on_btn_close_clicked();
    void on_btn_func_1_clicked();
    void on_btn_func_2_clicked();
    void on_btn_func_3_clicked();
    void on_btn_func_4_clicked();
    void on_btn_func_5_clicked();
    void on_btn_func_6_clicked();
    void on_btn_func_7_clicked();

    void on_btn_terminate_clicked();
    void onDisplayTimerOut();
    void onControlTimerOut();

private:
    Ui::MainWindow *ui;
    void init_variables();
    void init_ROS_and_ui_update();
    bool is_running();

public:
    void log_new_line(const QString&);
    void log_clearup(const QString&);
    void log_clearup();
    void txt_update();

    void BLDC0_angle_protection();
    void BLDC1_angle_protection();
    void timeout_protection();

    void roadwheel_angle_reciever_callback(const std_msgs::Float32& msg);
    void steerwheel_angle_reciever_callback(const std_msgs::Float32& msg);
    void BLDC0_current_reciever_callback(const std_msgs::Float32& msg);
    void BLDC1_current_reciever_callback(const std_msgs::Float32& msg);
    void rackforce_reciever_callback(const std_msgs::Float32& msg);
    void windows_matlab_cmd_callback(const std_msgs::Float32& msg);

    float BLDC0_current;    // BLDC near rack
    float BLDC1_current;    // BLDC near steering wheel
    float rackforce;    // rack force detected by the sensor, kN. Stretching: >0; Compressing: <0.
    float steerwheel_angle;    // column angle near steering wheel
    float roadwheel_angle;    // column angle near rack
    float control_frequency;   // This value can not be zero or below!
    int timeout_counter;
    float matlab_cmd_repo;
    bool matlab_cmd_reception;

    float steerwheel_anglar_velocity;    // column anglar velocity near steering wheel
    float roadwheel_anglar_velocity;    // column anglar velocity near rack

    Filter_IIR_Butterworth_fs_100Hz_fc_4Hz *steerwheel_av_filter;
    Filter_IIR_Butterworth_fs_100Hz_fc_4Hz *roadwheel_av_filter;

    QTimer *displayTimer, *controlTimer;
    Control *control;

    void system_disable();
    void lowermotor_sine_tuning_demo();
    void uppermotor_sine_tuning_demo();
    void angle_following_demo();
    void recover();
    void matlab_connection_test();
    void developer_mode();
    void joint_simulation_mode();

    // typedef void (MainWindow::*MemberFunctionPtr)();
    // MemberFunctionPtr active_function_ptr;
    void (MainWindow::*active_function_ptr)();
};

#endif // MAINWINDOW_H
