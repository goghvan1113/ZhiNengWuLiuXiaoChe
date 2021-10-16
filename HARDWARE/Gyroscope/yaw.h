/**************************************
* 角度环控制
************************************/


#ifndef __YAW_H__
#define __YAW_H__


void YAW_ControlLoop(void);
float YAW_GetPidOut(void);
void YAW_Test(void);
void YAW_StartLoop(void);
void YAW_CloseLoop(void);
void YAW_TCK_RunWithSpeed(float Speed_x,float speed_y,char ArrayChar);//循迹并保持车身正直

//保持车身正直，设置X/Y轴的速度
void YAW_RunWithSpeed(float Speed_x,float speed_y);
void YAW_DoNothingWithSpeed(float Speed_x,float speed_y);//角度不做修正保持停止状态。

void UseTCKYaw_RunWithSpeed(float Speed_x,float speed_y,char ArrayChar);//循迹，并且用循迹灯条来设置车身正直。
void YAW_UseTCKYawTest(void);

void YAW_ClearMPU_PID(void);

void YAW_CheckDouble(void);//双灯校正陀螺仪角度。
void YAW_CheckDoubleTest(void);
void YAW_Test(void);

#endif


















