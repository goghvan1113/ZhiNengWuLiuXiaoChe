/**************************************
* �ǶȻ�����
************************************/


#ifndef __YAW_H__
#define __YAW_H__


void YAW_ControlLoop(void);
float YAW_GetPidOut(void);
void YAW_Test(void);
void YAW_StartLoop(void);
void YAW_CloseLoop(void);
void YAW_TCK_RunWithSpeed(float Speed_x,float speed_y,char ArrayChar);//ѭ�������ֳ�����ֱ

//���ֳ�����ֱ������X/Y����ٶ�
void YAW_RunWithSpeed(float Speed_x,float speed_y);
void YAW_DoNothingWithSpeed(float Speed_x,float speed_y);//�ǶȲ�����������ֹͣ״̬��

void UseTCKYaw_RunWithSpeed(float Speed_x,float speed_y,char ArrayChar);//ѭ����������ѭ�����������ó�����ֱ��
void YAW_UseTCKYawTest(void);

void YAW_ClearMPU_PID(void);

void YAW_CheckDouble(void);//˫��У�������ǽǶȡ�
void YAW_CheckDoubleTest(void);
void YAW_Test(void);

#endif


















