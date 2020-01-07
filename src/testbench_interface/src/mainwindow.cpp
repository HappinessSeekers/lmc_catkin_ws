#include "testbench_interface/mainwindow.h"
#include "ui_mainwindow.h"
#include "ros/rostime_decl.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
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
    //function ptr
    active_function_ptr = NULL;

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
}

Control::~Control(){
    delete steerwheelMotor_PID_controller;
    delete roadwheelMotor_PID_controller;
    delete loadMotor_PID_controller;
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
    float angle_target = 0;
    float rack_targetforce = -1;
    control->BLDC0_current = 0;//limitation(control->roadwheelMotor_PID_controller->PID_calculate(angle_target, roadwheel_angle),15);
    control->BLDC1_current = 0;
    control->loadmotor_voltage = loadMotorVoltage(rack_targetforce) + limitation(control->loadMotor_PID_controller->PID_calculate(rack_targetforce, rackforce),1.5);
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
void MainWindow::roadwheel_angle_reciever_callback(const std_msgs::Float32& msg)
{
    roadwheel_angle = msg.data;
}
void MainWindow::steerwheel_angle_reciever_callback(const std_msgs::Float32& msg)
{
    steerwheel_angle = msg.data;
}
void MainWindow::BLDC0_current_reciever_callback(const std_msgs::Float32& msg)
{
    BLDC0_current = msg.data;
}
void MainWindow::BLDC1_current_reciever_callback(const std_msgs::Float32& msg)
{
    BLDC1_current = msg.data;
}
void MainWindow::rackforce_reciever_callback(const std_msgs::Float32& msg)
{
    rackforce = msg.data * 2;
}
void MainWindow::windows_matlab_cmd_callback(const std_msgs::Float32& msg)
{
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
        active_function_ptr = &MainWindow::lowermotor_sine_tuning_demo;
        log_new_line("NOTE: Loading Function 'lowermotor sine tuning demo'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_2_clicked(){
    if (active_function_ptr == NULL)
    {
        active_function_ptr = &MainWindow::uppermotor_sine_tuning_demo;
        log_new_line("NOTE: Loading Function 'uppermotor sine tuning demo'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_3_clicked(){
    if (active_function_ptr == NULL)
    {
        active_function_ptr = &MainWindow::angle_following_demo;
        log_new_line("NOTE: Loading Function 'angle following demo'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_4_clicked(){
    if (active_function_ptr == NULL)
    {
        active_function_ptr = &MainWindow::recover;
        log_new_line("NOTE: Loading Function 'recover'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_5_clicked(){
    if (active_function_ptr == NULL)
    {
        active_function_ptr = &MainWindow::matlab_connection_test;
        log_new_line("NOTE: Loading Function 'matlab connection test'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_6_clicked(){
    if (active_function_ptr == NULL)
    {
        active_function_ptr = &MainWindow::developer_mode;
        log_new_line("NOTE: Loading Function 'developer mode'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}

void MainWindow::on_btn_func_7_clicked(){
    if (active_function_ptr == NULL)
    {
        active_function_ptr = &MainWindow::joint_simulation_mode;
        log_new_line("NOTE: Loading Function 'testbench joint simulation'");
    }
    else
        log_new_line("NOTE: A Function Is Already Running!.");
}



