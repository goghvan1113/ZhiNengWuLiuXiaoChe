#ifndef __ENCODER_H
#define __ENCODER_H	
#include "sys.h"

#define ENCODER_TIM_PERIOD (u16)(65535)   //不可大于65535 因为定时器是16位的。


typedef struct 
{
  int16_t Wheel1;
  int16_t Wheel2;
  int16_t Wheel3;
  int16_t Wheel4;
}WheelSpeed_t;//1mm的转数



void Encoder_InitAll(void);
void ReadEncoderLoop(void);

#endif
