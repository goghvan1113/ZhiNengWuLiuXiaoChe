/*
* 用编码器做位置环获得距离
* 外环总的控制，陀螺仪角度环和编码器位置环叠加到一起，然后控制发给速度解算，速度解算后发给每一个轮子
*/

#ifndef __CARRUN_H__
#define __CARRUN_H__

#include "sys.h"
#include "SpeedSolving.h"

typedef struct
{
	float Target_X;//目标距离，单位是mm
	float Target_Y;//目标距离，单位是mm	
	float Target_Z;//目标角度，单位是度	
	int64_t AllPulse_1;	//1号轮子转过的脉冲
	int64_t AllPulse_2;	//2号轮子转过的脉冲
	int64_t AllPulse_3;	//3号轮子转过的脉冲
	int64_t AllPulse_4;	//4号轮子转过的脉冲
	
}WRun_PositionRdcord_t;//参数记录


void PIDInit(void);
void WRun_Loop1(void);
void WRun_Update(int16_t Wheel_1,int16_t Wheel_2,int16_t Wheel_3,int16_t Wheel_4);
void SetWRun_Target(int circle);
void WRunPIDDebug(void);
SpeedSolvingOut4_t SimpleSpeedSolving_4(float Speed_x,float Speed_y,float Speed_z);

#endif



















