#include "testbench_interface/mainwindow.h"
#include "ui_mainwindow.h"
#include "ros/rostime_decl.h"
#include <cstdio>

time_t timep;
FILE *fp;
FILE *fp_mrac;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Testbench Interface");
    log_new_line("NODE Interface: Interface Initialized");

    init_variables();
    init_ROS_and_ui_update();

}

MainWindow::~MainWindow()
{
    delete control;
    delete ui;
    delete controlTimer;
    delete displayTimer;
    //Need to shut down ROS and return all things back to normal here.
}

void MainWindow::init_variables()
{
    //Testbench state
    BLDC0_current = 0.0;
    BLDC1_current = 0.0;
    rackforce = 0.0;
    steerwheel_angle = 0.0;
    roadwheel_angle = 0.0;
    control_frequency = 100;
    timeout_counter = 0;
    matlab_cmd_repo = 0.0;
    matlab_cmd_reception = false;
    //control state ptr
    control = new Control(control_frequency);
    //filter for anglar velocity calculation
    steerwheel_av_filter = new Filter_IIR_Butterworth_fs_100Hz_fc_4Hz;
    roadwheel_av_filter = new Filter_IIR_Butterworth_fs_100Hz_fc_4Hz;
    //QTimer ptr
    controlTimer = new QTimer();
    displayTimer = new QTimer();
    //PID controller
    control->steerwheelMotor_PID_controller->set_Kp(0.6);
    control->steerwheelMotor_PID_controller->set_Ki(3.2);
    control->steerwheelMotor_PID_controller->set_Kd(0.8);
    control->steerwheelMotor_PID_controller->set_I_limit(9.0);
    control->roadwheelMotor_PID_controller->set_Kp(0.6);
    control->roadwheelMotor_PID_controller->set_Ki(3.2);
    control->roadwheelMotor_PID_controller->set_Kd(0.8);
    control->roadwheelMotor_PID_controller->set_I_limit(9.0);
    control->loadMotor_PID_controller->set_Kp(0.2);
    control->loadMotor_PID_controller->set_Ki(2.2);
    control->loadMotor_PID_controller->set_Kd(0.0);
    control->loadMotor_PID_controller->set_I_limit(0.7);
    //MRAC controller
    control->steerwheelMotor_MRAC_controller->set_reference_model(50,1);
    control->steerwheelMotor_MRAC_controller->set_gamma1(0.3);
    control->steerwheelMotor_MRAC_controller->set_gamma2(0.3);
    control->steerwheelMotor_MRAC_controller->set_gamma3(0.00001);
    control->roadwheelMotor_MRAC_controller->set_reference_model(50,1);
    control->roadwheelMotor_MRAC_controller->set_gamma1(3);
    control->roadwheelMotor_MRAC_controller->set_gamma2(3);
    control->roadwheelMotor_MRAC_controller->set_gamma3(0.01);
    //function ptr
    active_function_ptr = NULL;

    char filename_str[1000] = {0};
    sprintf(filename_str,"rostime_%f.csv",ros::Time::now().toSec());
    std::cout << filename_str << std::endl;
    fp = fopen(filename_str,"a+");//设置记录文件的路径
    // fprintf(fp,"\n%s\n",ctime(&timep));
    fprintf(fp,"ros_time, steerwheel_angle, roadwheel_angle\n");

    char mractest_filename_str[1000] = {0};
    sprintf(mractest_filename_str,"mrac_%f.csv",ros::Time::now().toSec());
    std::cout << mractest_filename_str << std::endl;
    fp_mrac = fopen(mractest_filename_str,"a+");//设置记录文件的路径
    // fprintf(fp,"\n%s\n",ctime(&timep));
    fprintf(fp_mrac,"ros_time, r, y, y_dot, theta1, theta2, theta3\n");

    MainWindow::timeout_reset();

    log_new_line("NODE Interface: Variables Initialized");
}

void MainWindow::init_ROS_and_ui_update()
{
    // ROS init
    roadwheel_angle_sub = n.subscribe("roadWheelAngle", 1000, &MainWindow::roadwheel_angle_reciever_callback, this);
    steerwheel_angle_sub = n.subscribe("steerWheelAngle", 1000, &MainWindow::steerwheel_angle_reciever_callback, this);
    BLDC0_sub = n.subscribe("BLDC0_current_feedback", 1000, &MainWindow::BLDC0_current_reciever_callback, this);
    BLDC1_sub = n.subscribe("BLDC1_current_feedback", 1000, &MainWindow::BLDC1_current_reciever_callback, this);
    Rackforce_feedback_sub = n.subscribe("rackforce_feedback", 1000, &MainWindow::rackforce_reciever_callback, this);
    windows_matlab_sub = n.subscribe("windows_matlab_cmd", 1000, &MainWindow::windows_matlab_cmd_callback, this);
    loadMotor_pub = n.advertise<std_msgs::Float32>("loadMotor", 1000);
    clutch_pub = n.advertise<std_msgs::Float32>("clutch", 1000);
    BLDC0_current_pub = n.advertise<std_msgs::Float32>("BLDC0_current", 1000);
    BLDC1_current_pub = n.advertise<std_msgs::Float32>("BLDC1_current", 1000);
    windows_matlab_response_pub = n.advertise<std_msgs::Float32>("windows_matlab_response", 1000);

    // Initial display windows
    Rvizviewer* rvizviewer = new Rvizviewer(ui->rviz_window);

    // Scan timer at 100Hz for scanning data
    displayTimer->setInterval(40);
    connect(displayTimer, SIGNAL(timeout()), this, SLOT(onDisplayTimerOut()));
    displayTimer->start();

    // Scan timer at 100Hz for scanning data
    controlTimer->setInterval(1000/control_frequency);
    connect(controlTimer, SIGNAL(timeout()), this, SLOT(onControlTimerOut()));
    controlTimer->start();
    
    log_new_line("NODE Interface: ROS Initialized");
}

void MainWindow::onDisplayTimerOut() {txt_update();}

void MainWindow::onControlTimerOut() {

    // Update Anglar Velocity
    steerwheel_av_filter->filter(steerwheel_angle);
    roadwheel_av_filter->filter(roadwheel_angle);
    steerwheel_anglar_velocity = steerwheel_av_filter->get_differential();
    roadwheel_anglar_velocity = roadwheel_av_filter->get_differential();

    if (active_function_ptr == NULL) {
        // Do Nothing.
        system_disable();
    }
    else {
        // Execute function.
        (this->*active_function_ptr)();
    }
    // Send out message.
    std_msgs::Float32 loadmotor_cmd_msg;
    std_msgs::Float32 clutch_cmd_msg;
    std_msgs::Float32 BLDC0_cmd_msg;
    std_msgs::Float32 BLDC1_cmd_msg;
    std_msgs::Float32 matlab_rsps_msg;
    loadmotor_cmd_msg.data = control->loadmotor_voltage;
    loadMotor_pub.publish(loadmotor_cmd_msg);
    clutch_cmd_msg.data = control->clutch_state;
    clutch_pub.publish(clutch_cmd_msg);
    BLDC0_cmd_msg.data = control->BLDC0_current;
    BLDC0_current_pub.publish(BLDC0_cmd_msg);
    BLDC1_cmd_msg.data = -control->BLDC1_current; // Mind that this motor has opposite direction than others.
    BLDC1_current_pub.publish(BLDC1_cmd_msg);
    matlab_rsps_msg.data = matlab_cmd_repo;
    windows_matlab_response_pub.publish(matlab_rsps_msg);
    // log_new_line("control published");
    if (control->ctrl_quit == true) {active_function_ptr = NULL;}
    BLDC0_angle_protection();
    BLDC1_angle_protection();
    motor_timeout_check();
    // write information into csv file.
    fprintf(fp,"%f, %f, %f\n",ros::Time::now().toSec(),steerwheel_angle, roadwheel_angle);

    steeringwheel_motor_timeout_counter++;
    roadwheel_motor_timeout_counter++;

    ros::spinOnce();
}

//check if any function is running.
bool MainWindow::is_running(){
    if (active_function_ptr == NULL)
        return false;
    else
        return true;
}

// update all txt.
void MainWindow::txt_update(){
    if (matlab_cmd_reception == false)
        ui->txtDisp_Matlab->setPlainText("disconnected");
    else
        ui->txtDisp_Matlab->setPlainText(QString::number(matlab_cmd_repo));

    ui->txtDisp_Clutch->setPlainText(QString::number(control->clutch_state));
    ui->txtDisp_LoadMotor_CMD->setPlainText(QString::number(control->loadmotor_voltage));
    ui->txtDisp_RW_Angle->setPlainText(QString::number(roadwheel_angle));
    ui->txtDisp_RW_Motor_CMD->setPlainText(QString::number(control->BLDC0_current));
    ui->txtDisp_RW_Motor_RSPS->setPlainText(QString::number(BLDC0_current));
    ui->txtDisp_Rackforce_RSPS->setPlainText(QString::number(rackforce));
    ui->txtDisp_SW_Angle->setPlainText(QString::number(steerwheel_angle));
    ui->txtDisp_SW_Motor_CMD->setPlainText(QString::number(control->BLDC1_current));
    ui->txtDisp_SW_Motor_RSPS->setPlainText(QString::number(BLDC1_current));

    // log_new_line("data updated");
}

//
void MainWindow::log_new_line(const QString& str) {ui->txtDisp_info->appendPlainText(str);}
void MainWindow::log_clearup(const QString& str) {ui->txtDisp_info->setPlainText(str);}
void MainWindow::log_clearup() {ui->txtDisp_info->setPlainText("");}

//*********************************** CONTROL CLASS ********************************//

Control::Control(const float& controller_frequency)
{
    BLDC0_current = 0.0;
    BLDC1_current = 0.0;
    clutch_state = true;
    loadmotor_voltage = 0.0;
    ctrl_quit = false;

    steerwheelMotor_PID_controller = new PID_Algorithm(controller_frequency);
    roadwheelMotor_PID_controller = new PID_Algorithm(controller_frequency);
    loadMotor_PID_controller = new PID_Algorithm(controller_frequency);

    steerwheelMotor_MRAC_controller = new MRAC_Algorithm(controller_frequency);
    roadwheelMotor_MRAC_controller = new MRAC_Algorithm(controller_frequency);
}

Control::~Control(){
    delete steerwheelMotor_PID_controller;
    delete roadwheelMotor_PID_controller;
    delete loadMotor_PID_controller;
    delete steerwheelMotor_MRAC_controller;
    delete roadwheelMotor_MRAC_controller;
}

void Control::stop() {ctrl_quit = true;}


//*********************************** FUNCTIONS ********************************//

void MainWindow::system_disable() {
    control->BLDC0_current = 0;
    control->BLDC1_current = 0;
    control->loadmotor_voltage = 0;
    control->clutch_state = false;
}
void MainWindow::lowermotor_sine_tuning_demo() {
    double secs =ros::Time::now().toSec();
    float angle_target = 100*sin(0.5 * secs);
    control->BLDC0_current = limitation(control->roadwheelMotor_PID_controller->PID_calculate(angle_target, roadwheel_angle),15);
    control->BLDC1_current = 0;
    control->loadmotor_voltage = 0;
    control->clutch_state = false;
}
void MainWindow::uppermotor_sine_tuning_demo() {
    double secs =ros::Time::now().toSec();
    float angle_target = 100*sin(0.5 * secs);
    control->BLDC0_current = 0;
    control->BLDC1_current = limitation(control->steerwheelMotor_PID_controller->PID_calculate(angle_target, steerwheel_angle),15);
    control->loadmotor_voltage = 0;
    control->clutch_state = false;
}
void MainWindow::angle_following_demo() {
    control->BLDC0_current = limitation(control->roadwheelMotor_PID_controller->PID_calculate(steerwheel_angle, roadwheel_angle),15);
    control->BLDC1_current = 0;
    control->loadmotor_voltage = 0;
    control->clutch_state = true;
}
void MainWindow::recover() {
    control->BLDC0_current = limitation(control->roadwheelMotor_PID_controller->PID_calculate(0, roadwheel_angle),15);
    control->BLDC1_current = 0;
    control->loadmotor_voltage = 0;
    control->clutch_state = true;
    if ((steerwheel_angle - roadwheel_angle < 0.05)&&(steerwheel_angle - roadwheel_angle > -0.05)) {
        active_function_ptr = NULL;
    }
}
void MainWindow::matlab_connection_test() {
    float angle_target = 0;
    float rack_targetforce = matlab_cmd_repo;
    control->BLDC0_current = 0;//limitation(control->roadwheelMotor_PID_controller->PID_calculate(angle_target, roadwheel_angle),15);
    control->BLDC1_current = 0;
    control->loadmotor_voltage = loadMotorVoltage(rack_targetforce) + limitation(control->loadMotor_PID_controller->PID_calculate(rack_targetforce, rackforce),1.5);
    control->clutch_state = false;
}
void MainWindow::developer_mode() {
    double secs =ros::Time::now().toSec();
    float angle_target = 30*sin(0.5 * secs);
    float theta1 = control->steerwheelMotor_MRAC_controller->get_theta1();
    float theta2 = control->steerwheelMotor_MRAC_controller->get_theta2();
    float theta3 = control->steerwheelMotor_MRAC_controller->get_theta3();
    control->BLDC0_current = 0;
    // control->BLDC1_current = limitation(control->steerwheelMotor_PID_controller->PID_calculate(angle_target, steerwheel_angle),15);
    control->BLDC1_current = limitation(control->steerwheelMotor_MRAC_controller->MRAC_calculate(angle_target, steerwheel_angle, steerwheel_anglar_velocity),15);
    fprintf(fp_mrac,"%f, %f, %f, %f, %f, %f, %f\n", ros::Time::now().toSec(), angle_target, steerwheel_angle, steerwheel_anglar_velocity, theta1, theta2, theta3);
    control->loadmotor_voltage = 0;
    control->clutch_state = false;
}
void MainWindow::joint_simulation_mode() {
    float angle_target = 0;
    float rack_targetforce = matlab_cmd_repo;
    control->BLDC0_current = 0;//limitation(control->roadwheelMotor_PID_controller->PID_calculate(angle_target, roadwheel_angle),15);
    control->BLDC1_current = 0;
    control->loadmotor_voltage = loadMotorVoltage(rack_targetforce) + limitation(control->loadMotor_PID_controller->PID_calculate(rack_targetforce, rackforce),1.5);
    control->clutch_state = false;
}
//*********************************** CALLBACKS ********************************//
void MainWindow::roadwheel_angle_reciever_callback(const std_msgs::Float32& msg) {roadwheel_angle = msg.data;}
void MainWindow::steerwheel_angle_reciever_callback(const std_msgs::Float32& msg) {steerwheel_angle = msg.data;}
void MainWindow::BLDC0_current_reciever_callback(const std_msgs::Float32& msg) {
    BLDC0_current = msg.data;
    roadwheel_motor_timeout_counter = 0;
}
void MainWindow::BLDC1_current_reciever_callback(const std_msgs::Float32& msg) {
    BLDC1_current = msg.data;
    steeringwheel_motor_timeout_counter = 0;
}
void MainWindow::rackforce_reciever_callback(const std_msgs::Float32& msg) {rackforce = msg.data * 2;}
void MainWindow::windows_matlab_cmd_callback(const std_msgs::Float32& msg) {
    matlab_cmd_repo = msg.data;
    matlab_cmd_reception = true;
}

//*************************************SLOTS************************************//

void MainWindow::on_btn_close_clicked() {
    active_function_ptr = NULL;
    
    std_msgs::Float32 loadmotor_cmd_msg;
    std_msgs::Float32 clutch_cmd_msg;
    std_msgs::Float32 BLDC0_cmd_msg;
    std_msgs::Float32 BLDC1_cmd_msg;
    std_msgs::Float32 matlab_rsps_msg;
    loadmotor_cmd_msg.data = 0;
    loadMotor_pub.publish(loadmotor_cmd_msg);
    clutch_cmd_msg.data = false;
    clutch_pub.publish(clutch_cmd_msg);
    BLDC0_cmd_msg.data = 0;
    BLDC0_current_pub.publish(BLDC0_cmd_msg);
    BLDC1_cmd_msg.data = 0;
    BLDC1_current_pub.publish(BLDC1_cmd_msg);
    matlab_rsps_msg.data = matlab_cmd_reception;
    windows_matlab_response_pub.publish(matlab_rsps_msg);
    
    close();
}

void MainWindow::on_btn_terminate_clicked(){
    if (active_function_ptr == NULL)
        log_new_line("NOTE: No Functions Currently Running.");
    else
        log_new_line("NOTE: Termination Request Recieved.");
    active_function_ptr = NULL;
    //terminate any running functions.
    
}

void MainWindow::on_btn_func_1_clicked(){
    if (active_function_ptr == NULL)
    {
        timeout_reset();
        active_function_ptr = &MainWindow::lowermotor_sine_tuning_demo;
        log_new_line("NOTE: Loading Function 'lowermotor sine tuning demo'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_2_clicked(){
    if (active_function_ptr == NULL)
    {
        timeout_reset();
        active_function_ptr = &MainWindow::uppermotor_sine_tuning_demo;
        log_new_line("NOTE: Loading Function 'uppermotor sine tuning demo'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_3_clicked(){
    if (active_function_ptr == NULL)
    {
        timeout_reset();
        active_function_ptr = &MainWindow::angle_following_demo;
        log_new_line("NOTE: Loading Function 'angle following demo'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_4_clicked(){
    if (active_function_ptr == NULL)
    {
        timeout_reset();
        active_function_ptr = &MainWindow::recover;
        log_new_line("NOTE: Loading Function 'recover'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_5_clicked(){
    if (active_function_ptr == NULL)
    {
        timeout_reset();
        active_function_ptr = &MainWindow::matlab_connection_test;
        log_new_line("NOTE: Loading Function 'matlab connection test'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_6_clicked(){
    if (active_function_ptr == NULL)
    {
        timeout_reset();
        active_function_ptr = &MainWindow::developer_mode;
        log_new_line("NOTE: Loading Function 'developer mode'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_7_clicked(){
    if (active_function_ptr == NULL)
    {
        timeout_reset();
        active_function_ptr = &MainWindow::joint_simulation_mode;
        log_new_line("NOTE: Loading Function 'testbench joint simulation'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}



