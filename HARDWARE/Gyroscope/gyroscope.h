/**********************************************************************
* 陀螺仪型号为：WT931和JY901配置方法类似，陀螺仪网站为www.wit-motion.com
* 支持波特率：2400，4800，9600，19200，38400，57600，115200，230400，460800，921600
* 优先配置为460800，一个起始位，8个数据位，1个停止位，一个字节共10位，传输速度为每秒46080字节，一个字节占用时间为21.7微秒，
* 陀螺仪返回数据一帧共11个字节，以0x55开头，第二个字节为输出类型标志，角度输出为0x53，接着是8个字节的数据，最后一个字节的SUM和校验
* 一帧需要时间为22*11 = 242微秒，时间较长，故不能一直在中断里等待接收数据，要配置为DMA模式。
************************************************************************/
#ifndef __GYROSCOPE_H__
#define __GYROSCOPE_H__


#include "sys.h"
#include <stm32f4xx_usart.h>
#include <stm32f4xx_rcc.h>
#include "delay.h"


void TestFIFO(void);
void WT931_Init(void);
void MPU_PrintMPUYaw(void);
void MPU_TestMPURate(void);
float ReadMpuYaw(void);
uint8_t MPU_IsUpdated(void);
float MPU_GetStartYaw(void);

void MPU_SetStartYaw(void);
void MPU_UserSetStartYaw(float Yaw);//强制设置初始角度

#endif

