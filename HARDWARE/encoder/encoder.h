#ifndef __ENCODER_H
#define __ENCODER_H	
#include "sys.h"

#define ENCODER_TIM_PERIOD (u16)(65535)   //���ɴ���65535 ��Ϊ��ʱ����16λ�ġ�


typedef struct 
{
  int16_t Wheel1;
  int16_t Wheel2;
  int16_t Wheel3;
  int16_t Wheel4;
}WheelSpeed_t;//1mm��ת��



void Encoder_InitAll(void);
void ReadEncoderLoop(void);

#endif
