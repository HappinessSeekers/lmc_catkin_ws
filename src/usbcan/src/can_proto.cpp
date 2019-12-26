#include "usbcan/can_proto.h"


void CAN_PROTO::ReadCANData(uint8_t* data)
{
    int i;
    for (i=0;i<8;i++)
    {
        data[i] = CAN_data[i];
    }
    
}

void CAN_PROTO::GetCANData(uint8_t* data,uint32_t time_data)
{
    int i;
    for (i=0;i<8;i++)
    {
        CAN_data[i] = data[i];
    }
    timeStamp = time_data;
}

void CAN_ID180::Extract(void)
{
    if (CAN_data[0]>0x80)
    {
        roadWheelAngle = ((float)(CAN_data[0]*256 + CAN_data[1]) - 65536)/10;
    }
    else {
        roadWheelAngle = ((float)(CAN_data[0]*256 + CAN_data[1]))/10;        
    }
    //printf("%f\n",roadWheelAngle);
}
void CAN_ID0C4::Extract(void)
{
    
    steerWheelAngle = ((float)(CAN_data[0]*256 + CAN_data[1])-0x80*256)/16;
    //printf("%f\n",steerWheelAngle);
}

void CAN_ID310::Extract(void)
{


}
void CAN_ID310::Compress(void)
{
    uint32_t temp = 0;
    if (BLDC_current >= 0)
    {
        CAN_data[3] = 0x00;
        temp = (uint32_t)(BLDC_current*1000);
    }
    else {
        CAN_data[3] = 0x01;
        temp = (uint32_t)(-BLDC_current*1000);
    }
    CAN_data[1] = (uint8_t)(temp/256);
    CAN_data[2] = (uint8_t)(temp-(uint8_t)(temp/256)*256);
    CAN_data[0] = 0x01;
}

void CAN_ID311::Extract(void)
{


}

void CAN_ID311::Compress(void)
{
    uint32_t temp = 0;
    if (BLDC_current >= 0)
    {
        CAN_data[3] = 0x00;
        temp = (uint32_t)(BLDC_current*1000);
    }
    else {
        CAN_data[3] = 0x01;
        temp = (uint32_t)(-BLDC_current*1000);
    }
    CAN_data[1] = (uint8_t)(temp/256);
    CAN_data[2] = (uint8_t)(temp-(uint8_t)(temp/256)*256);
    CAN_data[0] = 0x01;
}

void CAN_ID530::Extract(void)
{
    float temp = 0;
    temp = (float)(CAN_data[0]*256+CAN_data[1]);
    BLDC_current = temp/1000;
}

void CAN_ID530::Compress(void)
{


}

void CAN_ID531::Extract(void)
{
    float temp = 0;
    temp = (float)(CAN_data[0]*256+CAN_data[1]);
    BLDC_current = temp/1000;
}

void CAN_ID531::Compress(void)
{


}