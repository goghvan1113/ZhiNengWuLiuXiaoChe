#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"


#define MOTOR1_SPEED	TIM8->CCR1
#define MOTOR2_SPEED	TIM8->CCR2
#define MOTOR3_SPEED	TIM8->CCR3
#define MOTOR4_SPEED	TIM8->CCR4

#define MOTOR1_R			PEout(8)=0;
#define MOTOR1_L			PEout(9)=1;
		
#define MOTOR2_R			PEout(10)=1; 
#define MOTOR2_L			PEout(10)=0;

#define MOTOR3_R			PEout(11)=0;
#define MOTOR3_L			PEout(11)=1;
		
#define MOTOR4_R			PEout(12)=1;
#define MOTOR4_L			PEout(12)=0;
/////////////////////////////////////////////////


void MotorTest(void);//ǰ�����Ҳ���/�������
typedef struct 
{
	int16_t Wheel1; 
	int16_t Wheel2; 
	int16_t Wheel3; 
	int16_t Wheel4; //�����ĸ�����Ŀ�����ֵ������ռ�ձ�
}MotorCurrent_T	;

//�������io�ں���

void MotorPWM_Init(void);
void MotorState_Init(void);
void SetWheelPulse(int16_t Wheel1, int16_t Wheel2,int16_t Wheel3,int16_t Wheel4);
void SetWheelCurrent(float Wheel1, float Wheel2,float Wheel3,float Wheel4);

#endif


