#include "ros/ros.h"
#include "usbcan/usbcan_driver.h"
#include "usbcan/can_proto.h"
#include <cstdio>
#include "std_msgs/Float32.h"
#include <time.h>

time_t timep;

 

FILE *fp;

VCI_CAN_OBJ transimit_obj[100];
VCI_CAN_OBJ receive_obj[1000];
uint32_t CAN_transimit_len = 0; 
uint8_t CAN_transimit_error_count = 0;
uint32_t CAN_receive_len = 0 ;
CAN_ID0C4 CAN_ID0C4_obj;
CAN_ID180 CAN_ID180_obj;
CAN_ID310 CAN_ID310_obj;
CAN_ID311 CAN_ID311_obj;
CAN_ID530 CAN_ID530_obj;
CAN_ID531 CAN_ID531_obj;

/******** timeout protect defination *************/
uint32_t timeout_period = 200;
uint32_t BLDC0_timeout_count = 0;
uint32_t BLDC1_timeout_count = 0;



void BLDC0_currentCallback(const std_msgs::Float32& msg)
{
	BLDC0_timeout_count = 0;
	CAN_ID310_obj.BLDC_current = msg.data;
	CAN_ID310_obj.Compress();
	CAN_ID310_obj.ReadCANData(transimit_obj[CAN_transimit_len].Data);
	transimit_obj[CAN_transimit_len].DataLen = 8;
	transimit_obj[CAN_transimit_len].ID = 0x310;
	transimit_obj[CAN_transimit_len].SendType=0;
	transimit_obj[CAN_transimit_len].RemoteFlag=0;
	transimit_obj[CAN_transimit_len].ExternFlag=0;
	CAN_transimit_len++;
	printf("BLDC0 OK\n");
	//ROS_INFO("%d",CAN_transimit_len);

}
void BLDC1_currentCallback(const std_msgs::Float32& msg)
{
	BLDC1_timeout_count = 0;
	CAN_ID311_obj.BLDC_current = msg.data;
	CAN_ID311_obj.Compress();
	CAN_ID311_obj.ReadCANData(transimit_obj[CAN_transimit_len].Data);
	transimit_obj[CAN_transimit_len].DataLen = 8;
	transimit_obj[CAN_transimit_len].ID = 0x311;
	transimit_obj[CAN_transimit_len].SendType=0;
	transimit_obj[CAN_transimit_len].RemoteFlag=0;
	transimit_obj[CAN_transimit_len].ExternFlag=0;
	CAN_transimit_len++;
	printf("BLDC1 OK\n");
	//ROS_INFO("%d",CAN_transimit_len);

}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"usbcan_driver");   
    ros::NodeHandle n;    
	ros::Rate loop_rate(1000);

	ros::Publisher steerWheelAngle_pub = n.advertise<std_msgs::Float32>("steerWheelAngle", 1000);
	ros::Publisher roadWheelAngle_pub = n.advertise<std_msgs::Float32>("roadWheelAngle", 1000);
	ros::Publisher BLDC0_current_feedback_pub = n.advertise<std_msgs::Float32>("BLDC0_current_feedback", 1000);
	ros::Publisher BLDC1_current_feedback_pub = n.advertise<std_msgs::Float32>("BLDC1_current_feedback", 1000);
	ros::Subscriber BLDC0_current_sub = n.subscribe("BLDC0_current",1000, BLDC0_currentCallback);
	ros::Subscriber BLDC1_current_sub = n.subscribe("BLDC1_current",1000, BLDC1_currentCallback);
	std_msgs::Float32 steerWheelAngle_msg;
	std_msgs::Float32 roadWheelAngle_msg;
	std_msgs::Float32 BLDC0_current_feedback_msg;
	std_msgs::Float32 BLDC1_current_feedback_msg;
	/****** for test data settoing*****/
	transimit_obj[0].ID=0;
	transimit_obj[0].SendType=0;
	transimit_obj[0].RemoteFlag=0;
	transimit_obj[0].ExternFlag=0;
	transimit_obj[0].DataLen=8;
	int i=0;	
	for(i = 0; i < transimit_obj[0].DataLen; i++)
	{
		transimit_obj[0].Data[i] = i;
	}
	/*******************************************/
	/**************CAN DEV_INDEX_0  OPEN *****************************/	
	if(VCI_OpenDevice(VCI_USBCAN2,DEV_INDEX_0,0)==1)  //打开设备
	{
		printf(">>open deivce success!\n");
		
	}
	else
	{	
		printf(">>open deivce error!\n");
		return 0;
	}
	/****************CAN DEV_INDEX_0 CAN_INDEX_1 paramter**************/

	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x00;/*波特率125 Kbps  0x03  0x1C 500kbps 0x00 0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式	
	/**************CAN DEV_INDEX_0  CAN_INDEX_1 start *****************************/
	if(VCI_InitCAN(VCI_USBCAN2,DEV_INDEX_0,CAN_INDEX_1,&config)!=1) //初始化设备通道1参数
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,DEV_INDEX_0);
		return 0;
	}

	if(VCI_StartCAN(VCI_USBCAN2,DEV_INDEX_0,CAN_INDEX_1)!=1) //打开通道1 
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,DEV_INDEX_0);
		return 0;
	}
	/**********************************************************************************/

	fp = fopen("1-11.csv","a+");//设置记录文件的路径
	time(&timep);
    fprintf(fp,"\n%s\n",ctime(&timep));

	/*********************************main loop******************************************/
	while (ros::ok())
	{
		
		/***************BLDC protect*****************************************/
		BLDC0_timeout_count++;
		if(BLDC0_timeout_count == timeout_period)
		{	
			CAN_ID310_obj.BLDC_current = 0;
			CAN_ID310_obj.Compress();
			CAN_ID310_obj.ReadCANData(transimit_obj[CAN_transimit_len].Data);
			transimit_obj[CAN_transimit_len].DataLen = 8;
			transimit_obj[CAN_transimit_len].ID = 0x310;
			transimit_obj[CAN_transimit_len].SendType=0;
			transimit_obj[CAN_transimit_len].RemoteFlag=0;
			transimit_obj[CAN_transimit_len].ExternFlag=0;
			CAN_transimit_len++;
			BLDC0_timeout_count = 0;
			printf("BLDC0 TIME OUT\n");
		}

		BLDC1_timeout_count++;
		if(BLDC1_timeout_count == timeout_period)
		{
			CAN_ID311_obj.BLDC_current = 0;
			CAN_ID311_obj.Compress();
			CAN_ID311_obj.ReadCANData(transimit_obj[CAN_transimit_len].Data);
			transimit_obj[CAN_transimit_len].DataLen = 8;
			transimit_obj[CAN_transimit_len].ID = 0x311;
			transimit_obj[CAN_transimit_len].SendType=0;
			transimit_obj[CAN_transimit_len].RemoteFlag=0;
			transimit_obj[CAN_transimit_len].ExternFlag=0;
			CAN_transimit_len++;
			BLDC1_timeout_count = 0;
			printf("BLDC1 TIME OUT\n");
		}

		/***************CAN transimit****************************************/
		if (CAN_transimit_len != 0)
		{
			if(VCI_Transmit(VCI_USBCAN2, DEV_INDEX_0, CAN_INDEX_1, transimit_obj, CAN_transimit_len) != -1)
			{
				CAN_transimit_len = 0;
				CAN_transimit_error_count = 0;
			}
			else 
			{
				CAN_transimit_error_count ++;
				if (CAN_transimit_error_count == 3)
				{
					VCI_ResetCAN(VCI_USBCAN2,DEV_INDEX_0, CAN_INDEX_1);//复位CAN1通道。
					usleep(100000);
					VCI_CloseDevice(VCI_USBCAN2,DEV_INDEX_0);	
					printf("usbcan transimit error");
					return 0;
				}

			}

		}
		/***************CAN receive****************************************/
		if ((CAN_receive_len = VCI_Receive(VCI_USBCAN2,DEV_INDEX_0,CAN_INDEX_1,receive_obj,1000,100)) > 0)
		{	
										
			for (int j=0;j<CAN_receive_len;j++){				
				switch(receive_obj[j].ID)
				{
					case 0xC4:{
						CAN_ID0C4_obj.GetCANData(receive_obj[j].Data,receive_obj[j].TimeStamp);
						CAN_ID0C4_obj.Extract();
						steerWheelAngle_msg.data = CAN_ID0C4_obj.steerWheelAngle;
						steerWheelAngle_pub.publish(steerWheelAngle_msg);						
						break;
					};
					case 0x180:{
						CAN_ID180_obj.GetCANData(receive_obj[j].Data,receive_obj[j].TimeStamp);
						CAN_ID180_obj.Extract();
						roadWheelAngle_msg.data = CAN_ID180_obj.roadWheelAngle;	
						roadWheelAngle_pub.publish(roadWheelAngle_msg);											
						break;
					}
					case 0x530:{
						CAN_ID530_obj.GetCANData(receive_obj[j].Data,receive_obj[j].TimeStamp);
						CAN_ID530_obj.Extract();
						BLDC0_current_feedback_msg.data = CAN_ID530_obj.BLDC_current;
						BLDC0_current_feedback_pub.publish(BLDC0_current_feedback_msg);
						break;
					}
					case 0x531:{
						CAN_ID531_obj.GetCANData(receive_obj[j].Data,receive_obj[j].TimeStamp);
						CAN_ID531_obj.Extract();
						BLDC1_current_feedback_msg.data = CAN_ID531_obj.BLDC_current;
						BLDC1_current_feedback_pub.publish(BLDC1_current_feedback_msg);
						break;
					}
					case 0x7F0:{
					fprintf(fp,"%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",receive_obj[j].Data[0],receive_obj[j].Data[1],receive_obj[j].Data[2],receive_obj[j].Data[3],receive_obj[j].Data[4],receive_obj[j].Data[5],receive_obj[j].Data[6],receive_obj[j].Data[7]);
					break;

					}
					default:break;
				}	
			}

		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	VCI_ResetCAN(VCI_USBCAN2,DEV_INDEX_0, CAN_INDEX_1);//复位CAN1通道。
	usleep(100000);
	VCI_CloseDevice(VCI_USBCAN2,DEV_INDEX_0);	
	printf("usbcan node stop");
	return 0;
}