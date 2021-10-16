#ifndef __POSITION_SPEED_CIRCLE_H__
#define __POSITION_SPEED_CIRCLE_H__

#include "PIDController.h"



void Speed_ControlLoop(void);
void SpeedC_Init(uint8_t PID_Type);
void SpeedC_SetTarget(float Wheel_1, float Wheel_2, float Wheel_3, float Wheel_4);
float SpeedC_GetTarget(uint8_t Wheel_x);
void SpeedC_SetPIDNumber(uint8_t Wheel_x,uint16_t KP, uint16_t KI, uint16_t KD);//USMART参数调试使用
void SpeedC_RealitySpeedUp(int16_t Wheel_1, int16_t Wheel_2, int16_t Wheel_3, int16_t Wheel_4);//编码器中更新速度
void SpeedC_SetOneRamp(uint8_t WhichWheel,float RampTime,float RampStep);//位置环里面，需要设置每一个轮子不同的斜坡
void SpeedC_ClearPID(void);
float SpeedC_GetRealSpeed(uint8_t Wheel_x);
void PID_DebugPID(void);//调试PID参数

void print(void);
#endif

