/**********************************************************************
* �������ͺ�Ϊ��WT931��JY901���÷������ƣ���������վΪwww.wit-motion.com
* ֧�ֲ����ʣ�2400��4800��9600��19200��38400��57600��115200��230400��460800��921600
* ��������Ϊ460800��һ����ʼλ��8������λ��1��ֹͣλ��һ���ֽڹ�10λ�������ٶ�Ϊÿ��46080�ֽڣ�һ���ֽ�ռ��ʱ��Ϊ21.7΢�룬
* �����Ƿ�������һ֡��11���ֽڣ���0x55��ͷ���ڶ����ֽ�Ϊ������ͱ�־���Ƕ����Ϊ0x53��������8���ֽڵ����ݣ����һ���ֽڵ�SUM��У��
* һ֡��Ҫʱ��Ϊ22*11 = 242΢�룬ʱ��ϳ����ʲ���һֱ���ж���ȴ��������ݣ�Ҫ����ΪDMAģʽ��
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
void MPU_UserSetStartYaw(float Yaw);//ǿ�����ó�ʼ�Ƕ�

#endif

