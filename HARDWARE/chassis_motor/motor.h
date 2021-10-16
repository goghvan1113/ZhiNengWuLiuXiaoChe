#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"


#define MOTOR1_SPEED	TIM8->CCR1
#define MOTOR2_SPEED	TIM8->CCR2
#define MOTOR3_SPEED	TIM8->CCR3
#define MOTOR4_SPEED	TIM8->CCR4

#define MOTOR1_OFF			PEout(9)=1;PEout(10)=1	
#define MOTOR1_R			PEout(9)=0;PEout(10)=1   /* 顺时针 */
#define MOTOR1_L			PEout(9)=1;PEout(10)=0 
		
#define MOTOR2_OFF			PEout(11)=1;PEout(12)=1	
#define MOTOR2_R			PEout(11)=1;PEout(12)=0 /* 顺时针 */
#define MOTOR2_L			PEout(11)=0;PEout(12)=1	

#define MOTOR3_OFF			PEout(13)=1;PEout(14)=1	
#define MOTOR3_R			PEout(13)=0;PEout(14)=1 /* 顺时针 */
#define MOTOR3_L			PEout(13)=1;PEout(14)=0
		
#define MOTOR4_OFF			PEout(15)=1;PBout(10)=1	
#define MOTOR4_R			PEout(15)=1;PBout(10)=0	 /* 顺时针 */
#define MOTOR4_L			PEout(15)=0;PBout(10)=1	
/////////////////////////////////////////////////



typedef struct 
{
	int16_t Wheel1; 
	int16_t Wheel2; 
	int16_t Wheel3; 
	int16_t Wheel4; //储存四个马达的目标电流值，不是占空比
}MotorCurrent_T	;

//配置相关io口函数

void MotorPWM_Init(void);
void MotorState_Init(void);
void SetWheelPulse(int16_t Wheel1, int16_t Wheel2,int16_t Wheel3,int16_t Wheel4);
void SetWheelCurrent(float Wheel1, float Wheel2,float Wheel3,float Wheel4);

#endif


