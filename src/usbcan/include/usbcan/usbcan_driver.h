/******************** 
author: zhaojt

命名规则：（其实就是拍脑袋习惯）
function like: FunctionFirst
data like: dataFirst ID238_dataFirst
长意的连词符：　_
 
佛祖保佑～永无bug

*******************/

#ifndef _USBCAN_DRIVER_H_
#define _USBCAN_DRIVER_H_


#include "usbcan/controlcan.h"
#include <unistd.h>
/********usb-can definiations start ************************/

enum Status {start = 0 , stop = 1 ,error = 2};
#define BAUDRATE_500K_0 0x00
#define BAUDRATE_500K_1 0x1C
#define DEV_INDEX_0 0
#define DEV_INDEX_1 1
#define CAN_INDEX_1 0
#define CAN_INDEX_2 1
/********usb-can definiations end ************************/




#endif 