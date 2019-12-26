#pragma once
#ifndef _CAN_PROTO_H_
#define _CAN_PROTO_H_

#include "ros/ros.h"



class CAN_PROTO{
    
    public:
    uint32_t timeStamp;
    uint32_t CAN_Id;
    uint8_t CAN_data[8];
    uint8_t RemoteFlag;
    uint8_t DataLen;//是否是远程帧
    uint8_t ExternFlag;//是否是扩展帧
    uint8_t SendType;
    
    void GetCANData(uint8_t* data,uint32_t time_data);
    void ReadCANData(uint8_t* data);

};

class CAN_ID0C4 : public CAN_PROTO{
    
    public:

    float steerWheelAngle;

    void Extract(void);


};

class CAN_ID180 : public CAN_PROTO{

    public:

    float roadWheelAngle;

    void Extract(void);

};
class CAN_ID310 : public CAN_PROTO{

    public:
    float BLDC_current;
    
    void Extract(void);
    void Compress(void);


};

class CAN_ID311 : public CAN_PROTO{

    public:
    float BLDC_current;

    void Extract(void);
    void Compress(void);
};

class CAN_ID530 : public CAN_PROTO{

    public:
    float BLDC_current;

    void Extract(void);
    void Compress(void);


};
class CAN_ID531 : public CAN_PROTO{

    public:
    float BLDC_current;
    
    void Extract(void);
    void Compress(void);

};


#endif 