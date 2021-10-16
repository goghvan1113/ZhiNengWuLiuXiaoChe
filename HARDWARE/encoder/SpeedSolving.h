

#ifndef __SPEED_SOLVING_H__
#define __SPEED_SOLVING_H__

#include "SpeedControl.h"


#define SPEED_SOLVING_RAMP_TIME	4.0f	//��4��ʹб�´ﵽ����ֵ
#define SPEED_SOLVING_RAMP_STEP_MIN	0.25f

//��һ����Ժ���������


//���������ķ��
//�����ٶȽ��㣬�ο���https://blog.csdn.net/banzhuan133/article/details/69229922
//�����ٶȽ��㣬�ο�https://zhuanlan.zhihu.com/p/20282234
//���Ӱ�װ��ʽ����ǰ��������תʱ����������ǰ������ǰ��������תʱ����������ǰ����
//���궨�壺ǰ��ΪY����ǰ������ֱ����ΪX����ǰ����
// Speed_x��ʾ X ���˶����ٶȣ������ҷ��򣬶�������Ϊ����
// Speed_y��ʾ Y ���˶����ٶȣ���ǰ���򣬶�����ǰΪ����
// Speed_z��ʾ yaw ����ת�Ľ��ٶȣ�������ʱ��Ϊ��
//�����ֵ������ο�������������ģ��²���
#define SPEED_SOLVING4_DATA_L1	139.5f	//С������ߴ�İ������ת���ĵ�������ӵľ��롣��λ��mm
#define SPEED_SOLVING4_DATA_L2	135.0f	//С������ߴ�İ볤������ת���ĵ�ǰ�����ӵľ��롣��λ��mm
#define SPEED_SOLVING4_DATA_R_Y	30.000000f	//���ְ뾶(mm)��ͨ��λ�Ƽ�����İ뾶������뾶������Y����
#define SPEED_SOLVING4_DATA_R_X	30.000000f	//���ְ뾶(mm)��ͨ��λ�Ƽ�����İ뾶������뾶������X����
#define SPEED_SOLVING4_DATA_R_Z	30.000000f	//���ְ뾶(mm)��ͨ��λ�Ƽ�����İ뾶������뾶������Z����
#define SPEED_SOLVING4_DATA_R	30.000000f		//���Ҹ������ְ뾶�����Ҳ�����۰뾶
#define SPEED_SOLVING4_DATA_TAN_A	1.0f	//tan(a)�����ǶȰ�����������ֵ
#define SPEED_SOLVING4_DATA_MOTOR_N	375.0f	//������ת��
#define SPEED_SOLVING4_DATA_MOTOR_PULSE_N	98.0f	//������ת�ٶ�Ӧ�ı�����ÿ10��������
#define SPEED_SOLVING4_DATA_MOTOR_PWM	2000.0f	//PWM���ֵ
#define SPEED_SOLVING4_DATA_ENCODE_PULSE_MAX	1560.0f	//����תһȦ��Ӧ�ı�����������



typedef struct
{
	float Speed_1;
	float Speed_2;
	float Speed_3;
	float Speed_4;
	uint8_t UpData;
}SpeedSolvingOut4_t;


typedef struct
{
	float PositionX_mm;
	float PositionY_mm;
	float PositionZ_D;//��λ�Ƕ�
}PositionSolving_t;//λ�ý���

typedef struct
{
	float TargetSpeed_X;
	float TargetSpeed_Y;
	float TargetSpeed_Z;
}TargetSpeed_t;


/*
* �����βΣ�Speed_x��Speed_y��λ��Ϊm/s��Speed_z��λΪ��ÿ�룬����Ľ����ÿms��PWM����
*/
SpeedSolvingOut4_t SpeedSolving_4(float Speed_x,float Speed_y,float Speed_z);/*С��
* ������Ϊ1��2����ǰΪY�������򣬴�ֱ����ΪX�����������Ϊ����㣬�������ÿ�����ӵ�Ŀ��ת��
*/


/*
* �����������㣬����ÿ������ת�ı��������壬���С��X��Y��Z����ת���ľ���ͽǶȣ���λ��mm�Ͷ�
*/
PositionSolving_t PositionSolving_4(float Pulse_1,float Pulse_2,float Pulse_3,float Pulse_4);


/*
* ͬʱ����Ŀ��ֵ
*/
void SpeedSolving4_Set(void);


/*
* ��λ�û�������ٶ�֮�󣬻��о�׼����ʱ�䣬�����Ҫ������һ��б�º����ĺ����time����
* �־�������޷��Ļ���Ӧ�÷�������
*/
void SpeedSolving4_SetWithTime(float HowTime,SpeedSolvingOut4_t *TargetPulse);
TargetSpeed_t SpeedSolving_GetTargetSpeed(void);
float MyAbs(float a);


#endif











