#include "drivers/serial_driver.h"


using namespace std;
/******** serial defination *************/

serial::Serial ser1; // 离合器       ttyS1
serial::Serial ser2; // 负载电机     ttyS2
serial::Serial ser3; // 力传感器测量  ttyS3

unsigned char buf3[5] = {'#','0','1','0','\r'};   //力传感器通道0读取buf写入
unsigned char buf1[5] = {0x5A,0x00,0x00,0x00,0xA5};  //离合器
unsigned char buf2[5] = {0x5A,0x01,0x09,0x30,0xA5};  //负载电机DA输出buf写入
float ser3_data = 0; 
size_t ser3_num = 0;
uint32_t ser3_count = 0;
unsigned char ser3_buffer[1024]; //读取的结果

/******** timeout protect defination *************/
uint32_t timeout_period = 40;
uint32_t loadMotor_timeout_count = 0;

/******************* serial function*****************/
void OpenSerial(serial::Serial &ser,std::string &port,uint32_t baud)
{
    try{
    ser.setPort(port); 
    ser.setBaudrate(baud); 
    serial::Timeout to= serial::Timeout::simpleTimeout(1000); 
    ser.setTimeout(to); 
    ser.open();
    }
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open " + port);         
    } 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM(port + "initialized"); 
    } 
    
 
}

/*****************力传感器*****************/
float Str2num(unsigned char* buf,uint8_t size)
{
    float num = 0;
    int i = 0;
    if (size < 5)  return 10;
    for (i = size -2; i > 1 ; i--)
    {   
        if (buf[i]>=48&&buf[i]<=57)
        {
        num = num/10;
        num += ((float)(buf[i]-48));        
        }
    }
    if (buf[1] == '-')
    {
        num = - num; 
    }
    return num;
}
/************** callback********************/

void loadMotorCallback(const std_msgs::Float32& msg)
{
    float temp = 0;
    int i = 0;
    loadMotor_timeout_count = 0;
    //ROS_INFO("loadMotor: %f",msg.data);
    if (ser2.isOpen())
    {   
        buf2[0]=0x5A;
        if(msg.data>0)
        {
            buf2[1] = 0x01;
            temp = msg.data;    
        }
        else if(msg.data == 0)
        {
            buf2[1] = 0x00;
            temp = 0;
        }
        else
        {
            buf2[1] = 0x02;
            temp = -msg.data;
        }
        buf2[2] = (uint8_t)(temp);
        temp = temp -buf2[2];
        buf2[3] = (uint8_t)(temp*10)*16 +(uint8_t)((temp*10-(uint8_t)(temp*10))*10);
        buf2[4] = 0xA5;        
        ser2.write(buf2,5);
    }
}

void clutchCallback(const std_msgs::Float32& msg)
{
    if (ser1.isOpen())
    {
        if(msg.data>0)
        {
            buf1[0] = 0x5A;
            buf1[1] = 0x01;
            buf1[2] = 0x05;
            buf1[3] = 0x00;
            buf1[4] = 0xA5;

        }
        else
        {
            buf1[0] = 0x5A;
            buf1[1] = 0x00;
            buf1[2] = 0x00;
            buf1[3] = 0x00;
            buf1[4] = 0xA5;

        }
        ser1.write(buf1,5);
    }

}


int main(int argc,char** argv)
{   
    std::string port1 = "/dev/ttyS1";
    std::string port2 = "/dev/ttyS2";
    std::string port3 = "/dev/ttyS3";  

/********** ros handle***********************/
    ros::init(argc,argv,"serial_driver");
    ros::NodeHandle n;
    ros::Subscriber loadMotor_sub = n.subscribe("loadMotor", 1000, loadMotorCallback);
    ros::Subscriber clutch_sub = n.subscribe("clutch", 1000, clutchCallback);
    ros::Publisher rackforce_feedback_pub = n.advertise<std_msgs::Float32>("rackforce_feedback", 1000);
    std_msgs::Float32 rackforce_feedback_msg;
/**************打开串口****************/
    OpenSerial(ser1,port1,19200);
    OpenSerial(ser2,port2,19200);
    OpenSerial(ser3,port3,9600);
/**************************************/
    ros::Rate loop_rate(200);
    while(ros::ok())
    {   
        /*********力传感器读取***************/ 
        ser3_count++;        
        if (ser3.isOpen())
        {
            if (ser3_count == 1)
            {
                ser3.write(buf3,5);        
            } 
            if (ser3_count == 9 && ser3.available()) 
            {
                ser3_num = ser3.read(ser3_buffer,ser3.available());                
                ser3_data = Str2num(ser3_buffer,ser3_num);
                //std::cout << ser3_data << std::endl;
                rackforce_feedback_msg.data = ser3_data;
                rackforce_feedback_pub.publish(rackforce_feedback_msg);
            }  
            if (ser3_count == 10)
            {
                ser3_count = 0;
            }

        }
        /**************loadMotor timeout protect**/
        loadMotor_timeout_count ++ ;
        if (loadMotor_timeout_count == timeout_period)
        {
            if (ser2.isOpen())
            {   
                buf2[0]=0x5A;
                buf2[1] = 0x00;
                buf2[2] = 0x00;
                buf2[3] = 0x00;
                buf2[4] = 0xA5;        
                ser2.write(buf2,5);
            }
        }
        /*****************************************/

        loop_rate.sleep();
        ros::spinOnce();
        
    }

}
