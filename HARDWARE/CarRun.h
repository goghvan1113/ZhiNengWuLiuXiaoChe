/*
* �ñ�������λ�û���þ���
* �⻷�ܵĿ��ƣ������ǽǶȻ��ͱ�����λ�û����ӵ�һ��Ȼ����Ʒ����ٶȽ��㣬�ٶȽ���󷢸�ÿһ������
*/

#ifndef __CARRUN_H__
#define __CARRUN_H__

#include "sys.h"
#include "SpeedSolving.h"

typedef struct
{
	float Target_X;//Ŀ����룬��λ��mm
	float Target_Y;//Ŀ����룬��λ��mm	
	float Target_Z;//Ŀ��Ƕȣ���λ�Ƕ�	
	int64_t AllPulse_1;	//1������ת��������
	int64_t AllPulse_2;	//2������ת��������
	int64_t AllPulse_3;	//3������ת��������
	int64_t AllPulse_4;	//4������ת��������
	
}WRun_PositionRdcord_t;//������¼


void PIDInit(void);
void WRun_Loop1(void);
void WRun_Update(int16_t Wheel_1,int16_t Wheel_2,int16_t Wheel_3,int16_t Wheel_4);
void SetWRun_Target(int circle);
void WRunPIDDebug(void);
SpeedSolvingOut4_t SimpleSpeedSolving_4(float Speed_x,float Speed_y,float Speed_z);

#endif



















