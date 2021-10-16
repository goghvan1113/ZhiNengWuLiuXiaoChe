/**
 * @file������С�����˶�����
 * @brief��           
 * @details��
 * @author��
 * @date]��
 * @version��
 * @par History:
    version:
 */
#include "yaw.h"
#include "gyroscope.h"
#include "CarRun.h"
#include "usart1.h"
#include "tracking.h"

#define WheelPID_Yaw_KP	7.0f//���õ���7.0
#define WheelPID_Yaw_KI	0.0f//���õ���0.5
#define WheelPID_Yaw_KD	0.0f
#define WheelPID_Yaw_PID_OUTMAX	30
#define WheelPID_Yaw_PID_PRECISION	0.9//����

static uint8_t YAW_TCK_RunOpenFlag = 0;//��ѭ���ֱ��ֳ�����ֱ
static uint8_t YAW_UseTCKYaw_RunOpenFlag = 0;//��ѭ���ֱ��ֳ�����ֱ
static char YAW_TCK_WhichTCK = 'N';//�ĸ�ѭ��������ѭ��
static uint8_t YAW_CheckYawFlag = 0;//˫��У��FLAG

#define YAW_TCK_KP	5        //ͨ��������ȡX��Y��ƫ�Ƶ�KP
static float TargetSpeedx = 0;
static float TargetSpeedy = 0;


typedef struct 
{
	float Kp1;				
	float Ki1;				
	float Kd1;				
	float SpeedErr_now;		
	float SpeedErr_last;
	float SpeedErr_lastlast;
	float SpeedErr_all;	
	float pid_out;
	float pid_lastout;
	float TargetSpeed;
	float RealitySpeed;
	
}PID_Yaw_T;

#define PID_YAW_T_DEFAULT	{WheelPID_Yaw_KP,WheelPID_Yaw_KI,WheelPID_Yaw_KD,0,0,0,0,0,0,0,0}


////////////////////////////////////////������ѭ��������ýǶȵ�PID��������
#define PID_USE_TCK_YAW_KP 2.0		//pid������2.5��0.002��0.01
#define PID_USE_TCK_YAW_KI 0.002
#define PID_USE_TCK_YAW_KD 0.01
#define PID_USE_TCK_YAW_DEFAULT		{PID_USE_TCK_YAW_KP,PID_USE_TCK_YAW_KI,PID_USE_TCK_YAW_KD,0,0,0,0,0,0,0,0}
#define PID_USE_TCK_YAW_OUT_MAX	20


///////////////////////////////////////����˫��У��YAW���PID����
#define PID_YAW_DOUBLE_CHECK_KP	8.0
#define PID_YAW_DOUBLE_CHECK_KI	0.008
#define PID_YAW_DOUBLE_CHECK_KD	0.5
#define PID_YAW_DOUBLE_OUT_MAX	15.0
#define PID_YAW_DOUBLE_DEFAULT	{PID_YAW_DOUBLE_CHECK_KP,PID_YAW_DOUBLE_CHECK_KI,PID_YAW_DOUBLE_CHECK_KD,0,0,0,0,0,0,0,0}
static int32_t PID_CheckDoubleTime = 0;//У׼ʱ��У׼��ȷ1ms�ͼ�һ


PID_Yaw_T PID_Yaw = PID_YAW_T_DEFAULT;
PID_Yaw_T PID_UseTckYaw = PID_USE_TCK_YAW_DEFAULT;
PID_Yaw_T PID_YAW_DOUBLE = PID_YAW_DOUBLE_DEFAULT;

float YawJiaoDu = 0;

void YAW_PIDUpdata(void)
{
	PID_Yaw.SpeedErr_lastlast = PID_Yaw.SpeedErr_last;
	PID_Yaw.SpeedErr_last = PID_Yaw.SpeedErr_now;
	YawJiaoDu = ReadMpuYaw();
	PID_Yaw.SpeedErr_now = MPU_GetStartYaw() - YawJiaoDu;
	PID_Yaw.SpeedErr_all += PID_Yaw.SpeedErr_now;
	
	if(PID_Yaw.SpeedErr_now < WheelPID_Yaw_PID_PRECISION && PID_Yaw.SpeedErr_now > -WheelPID_Yaw_PID_PRECISION)
	{
		PID_Yaw.SpeedErr_now = 0;
		PID_Yaw.SpeedErr_all = 0;
		PID_Yaw.SpeedErr_last = 0;
	}		
	
	PID_Yaw.pid_out = PID_Yaw.Kp1 * PID_Yaw.SpeedErr_now + PID_Yaw.Kd1 * (PID_Yaw.SpeedErr_now - PID_Yaw.SpeedErr_last);
	PID_Yaw.pid_out += PID_Yaw.Ki1 * PID_Yaw.SpeedErr_all;
	if(PID_Yaw.pid_out > WheelPID_Yaw_PID_OUTMAX)
		PID_Yaw.pid_out = WheelPID_Yaw_PID_OUTMAX;
	else if(PID_Yaw.pid_out < -WheelPID_Yaw_PID_OUTMAX)
		PID_Yaw.pid_out = -WheelPID_Yaw_PID_OUTMAX;
}

void YAW_CheckYawPID_Up(void)
{//˫��У��
	PID_YAW_DOUBLE.SpeedErr_lastlast = PID_YAW_DOUBLE.SpeedErr_last;
	PID_YAW_DOUBLE.SpeedErr_last = PID_YAW_DOUBLE.SpeedErr_now;
	PID_YAW_DOUBLE.SpeedErr_now = TCK_DoubleCheckYaw();     //ͨ��ǰ��������ȡ���ĳ���ƫ��
	PID_YAW_DOUBLE.SpeedErr_all += PID_YAW_DOUBLE.SpeedErr_now;
	
	if(PID_YAW_DOUBLE.SpeedErr_now == 0)
		++ PID_CheckDoubleTime;
	else
		PID_CheckDoubleTime = 0;
	
	PID_YAW_DOUBLE.pid_out = PID_YAW_DOUBLE.Kp1 * PID_YAW_DOUBLE.SpeedErr_now + PID_YAW_DOUBLE.Kd1 * (PID_YAW_DOUBLE.SpeedErr_now - PID_YAW_DOUBLE.SpeedErr_last);
	PID_YAW_DOUBLE.pid_out += PID_YAW_DOUBLE.Ki1 * PID_YAW_DOUBLE.SpeedErr_all;
	
	if(PID_YAW_DOUBLE.pid_out > PID_YAW_DOUBLE_OUT_MAX)
		PID_YAW_DOUBLE.pid_out = PID_YAW_DOUBLE_OUT_MAX;
	else if(PID_YAW_DOUBLE.pid_out < -PID_YAW_DOUBLE_OUT_MAX)
		PID_YAW_DOUBLE.pid_out = -PID_YAW_DOUBLE_OUT_MAX;
}

void PID_UseTckYaw_Update(uint8_t WhichLED)
{
	PID_UseTckYaw.SpeedErr_lastlast = PID_UseTckYaw.SpeedErr_last;
	PID_UseTckYaw.SpeedErr_last = PID_UseTckYaw.SpeedErr_now;
	PID_UseTckYaw.SpeedErr_now = TCK_GetYawUseTck(WhichLED);  //ͨ��������ó��������X��Y��ĽǶ�
	PID_UseTckYaw.SpeedErr_all += PID_UseTckYaw.SpeedErr_now;
			
	PID_UseTckYaw.pid_out = PID_UseTckYaw.Kp1 * PID_UseTckYaw.SpeedErr_now + PID_UseTckYaw.Kd1 * (PID_UseTckYaw.SpeedErr_now - PID_UseTckYaw.SpeedErr_last);
	PID_UseTckYaw.pid_out += PID_UseTckYaw.Ki1 * PID_UseTckYaw.SpeedErr_all;
	if(PID_UseTckYaw.pid_out > PID_USE_TCK_YAW_OUT_MAX)
		PID_UseTckYaw.pid_out = PID_USE_TCK_YAW_OUT_MAX;
	else if(PID_UseTckYaw.pid_out < -PID_USE_TCK_YAW_OUT_MAX)
		PID_UseTckYaw.pid_out = -PID_USE_TCK_YAW_OUT_MAX;
	
	
}

void YAW_SetTargetYaw(float	Yaw)
{
	PID_Yaw.TargetSpeed = Yaw;
}

static int8_t YAW_LOOP_ENABLE = 0;
void YAW_CloseLoop(void)
{
	YAW_LOOP_ENABLE = 0;
}

void YAW_StartLoop(void)
{
	YAW_TCK_RunOpenFlag = 0;
	YAW_UseTCKYaw_RunOpenFlag = 0;
	YAW_CheckYawFlag = 0;
	YAW_LOOP_ENABLE = 1; 
}



void YAW_ControlLoop(void)
{
	if(MPU_IsUpdated() == 1 && YAW_LOOP_ENABLE == 1)
	{
		YAW_PIDUpdata();
		SimpleSpeedSolving_4(TargetSpeedx,TargetSpeedy,0);
	}	
	else if(MPU_IsUpdated() == 1 && YAW_TCK_RunOpenFlag == 1)  //YAW������ͨ�������Ǹ���
	{//ѭ�����ˡ�
		//�Ȼ���ĸ�ѭ��������ѭ�����ٻ��ƫ�˼����ƣ��ټ����Ӧ���������ٶȣ�
		float TCK_Float;
		switch(YAW_TCK_WhichTCK)
		{
			case 'F':
			{
				YAW_PIDUpdata();
				TCK_Float = TCK_GetXunJiBias_F();
				SimpleSpeedSolving_4(TargetSpeedx + TCK_Float * YAW_TCK_KP,TargetSpeedy,YAW_GetPidOut());
				break;
			}
			case 'B':
			{
				YAW_PIDUpdata();
				TCK_Float = TCK_GetXunJiBias_B();
				SimpleSpeedSolving_4(TargetSpeedx + TCK_Float * YAW_TCK_KP,TargetSpeedy,YAW_GetPidOut());			
				break;
			}
			case 'R':
			{
				YAW_PIDUpdata();
				TCK_Float = TCK_GetXunJiBias_R();
				SimpleSpeedSolving_4(TargetSpeedx ,TargetSpeedy + TCK_Float * YAW_TCK_KP,YAW_GetPidOut());			
				break;
			}
			case 'L':
			{
				YAW_PIDUpdata();
				TCK_Float = TCK_GetXunJiBias_L();
				SimpleSpeedSolving_4(TargetSpeedx,TargetSpeedy + TCK_Float * YAW_TCK_KP,YAW_GetPidOut());			
				break;
			}
			
		}
	}	
	else if(YAW_UseTCKYaw_RunOpenFlag == 1)   //YAW������ͨ����������
	{
		//�Ȼ���ĸ�ѭ��������ѭ�����ٻ��ƫ�˼����ƣ��ټ����Ӧ���������ٶȣ�
		float TCK_Float;
		//float TCK_FloatBiasYaw;
		switch(YAW_TCK_WhichTCK)
		{
			case 'F':
			{
				TCK_Float = TCK_GetXunJiBias_F();
				//TCK_FloatBiasYaw = TCK_GetYawUseTck('Y');
				PID_UseTckYaw_Update('Y');
				SimpleSpeedSolving_4(TargetSpeedx + TCK_Float * YAW_TCK_KP,TargetSpeedy,PID_UseTckYaw.pid_out);
				break;
			}
			case 'B':
			{
				TCK_Float = TCK_GetXunJiBias_B();
				//TCK_FloatBiasYaw = TCK_GetYawUseTck('Y');
				PID_UseTckYaw_Update('Y');
				SimpleSpeedSolving_4(TargetSpeedx + TCK_Float * YAW_TCK_KP,TargetSpeedy,PID_UseTckYaw.pid_out);			
				break;
			}
			case 'R':
			{
				TCK_Float = TCK_GetXunJiBias_R();
				//TCK_FloatBiasYaw = TCK_GetYawUseTck('X');
				PID_UseTckYaw_Update('X');
				SimpleSpeedSolving_4(TargetSpeedx ,TargetSpeedy + TCK_Float * YAW_TCK_KP,PID_UseTckYaw.pid_out);			
				break;
			}
			case 'L':
			{
				TCK_Float = TCK_GetXunJiBias_L();
				//TCK_FloatBiasYaw = TCK_GetYawUseTck('X');
				PID_UseTckYaw_Update('X');
				SimpleSpeedSolving_4(TargetSpeedx,TargetSpeedy + TCK_Float * YAW_TCK_KP,PID_UseTckYaw.pid_out);			
				break;
			}			
		}		
	}
	else if(YAW_CheckYawFlag == 1)
	{
		YAW_CheckYawPID_Up();
		PID_UseTckYaw_Update('Y');
		
		SimpleSpeedSolving_4(TargetSpeedx + PID_YAW_DOUBLE.pid_out,TargetSpeedy,PID_UseTckYaw.pid_out);		
	}
	
}

float YAW_GetPidOut(void)
{
	return PID_Yaw.pid_out;
}

uint8_t YAW_ClearMPU_PID_Dis = 0;
void YAW_ClearMPU_PID(void)
{
	YAW_ClearMPU_PID_Dis = 1;
	
}

//���ֳ�����ֱ������X/Y����ٶ�
void YAW_RunWithSpeed(float Speed_x,float speed_y)
{
	YAW_CloseLoop();
	YAW_TCK_RunOpenFlag = 0;
	YAW_UseTCKYaw_RunOpenFlag = 0;
	YAW_CheckYawFlag = 0;
	
	PID_Yaw.Kp1 = WheelPID_Yaw_KP ;
	PID_Yaw.Ki1 = WheelPID_Yaw_KI;//ѭ��ʱ�ѽǶȵ��ɵ�
	PID_Yaw.Kd1 = WheelPID_Yaw_KD;
	
//	if(YAW_ClearMPU_PID_Dis == 1)
//	{
//		PID_Yaw.Kp1 = 0;
//		PID_Yaw.Ki1 = 0;//ѭ��ʱ�ѽǶȵ��ɵ�
//		PID_Yaw.Kd1 = 0;		
//	}
	
//	PID_Yaw.Kp1 = 0 ;
//	PID_Yaw.Ki1 = 0;//ѭ��ʱ�ѽǶȵ��ɵ�
//	PID_Yaw.Kd1 = 0;
	
	TargetSpeedx = Speed_x;
	TargetSpeedy = speed_y;
	
	YAW_StartLoop();
}

void YAW_DoNothingWithSpeed(float Speed_x,float speed_y)//�ǶȲ�����������ֹͣ״̬��
{
	YAW_TCK_RunOpenFlag = 0;
	YAW_LOOP_ENABLE = 0;
	YAW_UseTCKYaw_RunOpenFlag = 0;
	YAW_CheckYawFlag = 0;
	
//	PID_Yaw.Kp1 = 0;
//	PID_Yaw.Ki1 = 0;
//	PID_Yaw.Kd1 = 0;
	
	TargetSpeedx = Speed_x;
	TargetSpeedy = speed_y;	
	SimpleSpeedSolving_4(0,0,0);
	delay_ms(2);
	SimpleSpeedSolving_4(0,0,0);
	YAW_TCK_WhichTCK = 0;	
	
}

//���ֳ�����ֱ������XY�ٶȣ�����ѭ������,��ѭ������ѭ���ģ�Ĭ�����ҷ���Ϊ�м��Ǹ����ں����ϣ�ǰ��ѭ��Ϊ�м���ڰ��ߣ������Աߵ��ں���
void YAW_TCK_RunWithSpeed(float Speed_x,float speed_y,char ArrayChar)
{
	YAW_TCK_RunOpenFlag = 1;
	YAW_LOOP_ENABLE = 0;
	YAW_UseTCKYaw_RunOpenFlag = 0;
	YAW_CheckYawFlag = 0;
	
	PID_Yaw.Kp1 = WheelPID_Yaw_KP;
	PID_Yaw.Ki1 = WheelPID_Yaw_KI;
	PID_Yaw.Kd1 = WheelPID_Yaw_KD;
	
//	PID_Yaw.Kp1 = 0;
//	PID_Yaw.Ki1 = 0;
//	PID_Yaw.Kd1 = 0;
	
	TargetSpeedx = Speed_x;
	TargetSpeedy = speed_y;	
		
	YAW_TCK_WhichTCK = ArrayChar;
	
}
void Yaw_Tck_RunFlagOpen(void)
{
	YAW_TCK_RunOpenFlag = 1;
	YAW_UseTCKYaw_RunOpenFlag = 0;
	YAW_LOOP_ENABLE = 0; 
	YAW_CheckYawFlag = 0;
}
void Yaw_Tck_RunFlagClose(void)
{
	YAW_TCK_RunOpenFlag = 0;
}


void YAW_Test(void)
{
	int32_t TimeTemp = 0;
	
//	SimpleSpeedSolving_4(-50,0,0);
//	delay_ms(3000);
//	SimpleSpeedSolving_4(0,0,0);
//	while(1);
	
	YAW_RunWithSpeed(30,0);
	while(1)
	{
//		YawJiaoDu = ReadMpuYaw();
		printf("V4:%.2f,%.2f,%.2f,%.2f,%.2f\n\r",SpeedC_GetTarget(1),SpeedC_GetTarget(2),SpeedC_GetTarget(3),SpeedC_GetTarget(4),YawJiaoDu);
		//printf("V4:%d,%d,%d,%d,%.2f\n\r",PID_GetRealSpeed(1),PID_GetRealSpeed(2),PID_GetRealSpeed(3),PID_GetRealSpeed(4),YawJiaoDu);
		//printf("V4:%d,%d,%d,%d\n\r",PID_GetAllStep(1),PID_GetAllStep(2),PID_GetAllStep(3),PID_GetAllStep(4));
//		printf("YO:%.6f,%.6f\n\r",YawJiaoDu,PID_Yaw.pid_out);//ʵ�ʽǶȣ��Ƕ�У�����
		++ TimeTemp;
		delay_ms(1);
		if(TimeTemp > 30000)
			break;
	}	
	TimeTemp = 0;
	YAW_RunWithSpeed(0,0);
//	SimpleSpeedSolving_4(0,0,0);
	while(1)
	{
		//printf("V4:%.2f,%.2f,%.2f,%.2f,%.2f\n\r",GetPIDTargetSpeed(1),GetPIDTargetSpeed(2),GetPIDTargetSpeed(3),GetPIDTargetSpeed(4),YawJiaoDu);
		//printf("V4:%d,%d,%d,%d,%.2f\n\r",PID_GetRealSpeed(1),PID_GetRealSpeed(2),PID_GetRealSpeed(3),PID_GetRealSpeed(4),YawJiaoDu);
		//printf("V4:%d,%d,%d,%d\n\r",PID_GetAllStep(1),PID_GetAllStep(2),PID_GetAllStep(3),PID_GetAllStep(4));
		printf("YO:%.6f,%.6f\n\r",YawJiaoDu,PID_Yaw.pid_out);//ʵ�ʽǶȣ��Ƕ�У�����
		++ TimeTemp;
		delay_ms(1);
		if(TimeTemp > 5000)
			break;
	}
	
	while(1)
		;
}


void YAW_UseTCKYawTest(void)
{//����ʹ��ѭ�����������Ƕȡ�
	
	int32_t Temp1 = 0;
	
	UseTCKYaw_RunWithSpeed(0,100,'F');
	while(1)
	{
		//���4�����ӵ�Ŀ���ٶ�
		//printf("V4:%.2f,%.2f,%.2f,%.2f\n\r",GetPIDTargetSpeed(1),GetPIDTargetSpeed(2),GetPIDTargetSpeed(3),GetPIDTargetSpeed(4));
		//���PID��ص�ֵ
		printf("PID:%f,%f\n\r",PID_UseTckYaw.SpeedErr_now,PID_UseTckYaw.pid_out);
		
		++ Temp1;
		if(Temp1 > 2000)
			break;
		delay_ms(1);
	}
	Temp1 = 0;
	UseTCKYaw_RunWithSpeed(0,0,'F');
	while(1)
	{
		//���4�����ӵ�Ŀ���ٶ�
		//printf("V4:%.2f,%.2f,%.2f,%.2f\n\r",GetPIDTargetSpeed(1),GetPIDTargetSpeed(2),GetPIDTargetSpeed(3),GetPIDTargetSpeed(4));
		
		printf("PID:%f,%f\n\r",PID_UseTckYaw.SpeedErr_now,PID_UseTckYaw.pid_out);
		//++ Temp1;
		if(Temp1 > 1000)
			break;
		delay_ms(1);
	}
	
	while(1)
		;
}

void UseTCKYaw_RunWithSpeed(float Speed_x,float speed_y,char ArrayChar)//ѭ����������ѭ�����������ó�����ֱ��
{
	YAW_TCK_RunOpenFlag = 0;
	YAW_LOOP_ENABLE = 0;
	YAW_UseTCKYaw_RunOpenFlag = 0;
	YAW_CheckYawFlag = 0;
	
	PID_UseTckYaw.Kp1 = PID_USE_TCK_YAW_KP;
	PID_UseTckYaw.Ki1 = PID_USE_TCK_YAW_KI;
	PID_UseTckYaw.Kd1 = PID_USE_TCK_YAW_KD;
	
	
	TargetSpeedx = Speed_x;
	TargetSpeedy = speed_y;	

	YAW_TCK_WhichTCK = ArrayChar;	
	YAW_UseTCKYaw_RunOpenFlag = 1;	
	
}

