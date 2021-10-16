/*
���Ҹ���ת��Ϊ500rpm
����ʱ��ߵ��ٶ�(ռ�ձ�Ϊ100%��PWM����
ÿ100ms�����Զ�ȡ��980�����壬ÿ��9800�����壬���9800/1560*60=377rpm	
*/

#include "SpeedControl.h"
#include "PIDController.h"
#include "led.h"
#include "encoder.h"
#include "usart1.h"
#include "motor.h"
#include "delay.h"



#define SpeedC_PRECISION 0	//���ƾ��ȣ���Ŀ���ٶ���ʵ���ٶ�С�ڴ�ֵʱ����Ϊû����ʹPID���ȶ�
#define SpeedC_ERRALL_MAX 1000	//����ERR_ALL���ֵ������ERR_ALL���ֵ���󣬻�ʹPID��Ӧ�������ȶ�
#define SpeedC_POSITION_OUT_MAX	375  //375//λ��ʽPID����޷�
#define SpeedC_INCREMENT_OUT_MAX	375  //375//����ʽPID����޷�
#define SpeedC_POSITION_STEP_MAX	100//����λ��ʽPID��󲽷������򲽷�̫�󣬱����һ�����5���ڶ���һ�������3000���Ե����̫�ã�
#define SpeedC_INCREMENT_STEP_MAX	100//����λ��ʽPID��󲽷������򲽷�̫�󣬱����һ�����5���ڶ���һ�������3000���Ե����̫�ã�



//������λ��ʽPID�Ĳ���
#define PID_KP_POSITION_1 13.000000f      //400ʱ�е����, 350Ҳ��                            
#define PID_KI_POSITION_1 0.200000f   //0.20000f
#define PID_KD_POSITION_1 0.000000f

#define PID_KP_POSITION_2 13.0000f                                   
#define PID_KI_POSITION_2 0.200000f
#define PID_KD_POSITION_2 0.000000f

#define PID_KP_POSITION_3 13.000000f                                   
#define PID_KI_POSITION_3 0.200000f
#define PID_KD_POSITION_3 0.000000f

#define PID_KP_POSITION_4 13.000000f                                   
#define PID_KI_POSITION_4 0.200000f
#define PID_KD_POSITION_4 0.000000f

//����������ʽPID�Ĳ���
#define PID_KP_INCREMENT_1 15.0	
#define PID_KI_INCREMENT_1 1.0		
#define PID_KD_INCREMENT_1 0.5

#define PID_KP_INCREMENT_2 15.0
#define PID_KI_INCREMENT_2 1.0
#define PID_KD_INCREMENT_2 0.5

#define PID_KP_INCREMENT_3 15.0
#define PID_KI_INCREMENT_3 1.0
#define PID_KD_INCREMENT_3 0.5

#define PID_KP_INCREMENT_4 15.0
#define PID_KI_INCREMENT_4 1.0
#define PID_KD_INCREMENT_4 0.5




typedef struct
{
	int16_t RealSpeed_1;
	int16_t RealSpeed_2;
	int16_t RealSpeed_3;
	int16_t RealSpeed_4;
	uint16_t RealSpeedUpdate;
	float TargetSpeed_1;
	float TargetSpeed_2;
	float TargetSpeed_3;
	float TargetSpeed_4;
	
}SpeedC_Speed_t;

SpeedC_Speed_t SpeedC_Speed; 

PID_t   SpeedC_PID_1;
PID_t   SpeedC_PID_2;
PID_t   SpeedC_PID_3;
PID_t   SpeedC_PID_4;

static uint8_t SpeedC_PIDWorkType = IncrementPID_e;

//1������ʽ��2��λ��ʽ
void SpeedC_Init(uint8_t PID_WorkType)
{
	SpeedC_PIDWorkType = PID_WorkType;
	
	PID_DefaultInit(&SpeedC_PID_1);
	PID_DefaultInit(&SpeedC_PID_2);
	PID_DefaultInit(&SpeedC_PID_3);
	PID_DefaultInit(&SpeedC_PID_4);
	
	if(PID_WorkType == IncrementPID_e)
	{
		SpeedC_PID_1.Kp1 = PID_KP_INCREMENT_1;
		SpeedC_PID_1.Ki1 = PID_KI_INCREMENT_1;
		SpeedC_PID_1.Kd1 = PID_KD_INCREMENT_1;
		SpeedC_PID_1.PID_OutMax = SpeedC_INCREMENT_OUT_MAX;
		SpeedC_PID_1.PID_OutStep = SpeedC_INCREMENT_STEP_MAX;		
		
		SpeedC_PID_2.Kp1 = PID_KP_INCREMENT_2;
		SpeedC_PID_2.Ki1 = PID_KI_INCREMENT_2;
		SpeedC_PID_2.Kd1 = PID_KD_INCREMENT_2;
		SpeedC_PID_2.PID_OutMax = SpeedC_INCREMENT_OUT_MAX;	
		SpeedC_PID_2.PID_OutStep = SpeedC_INCREMENT_STEP_MAX;
		
		SpeedC_PID_3.Kp1 = PID_KP_INCREMENT_3;
		SpeedC_PID_3.Ki1 = PID_KI_INCREMENT_3;
		SpeedC_PID_3.Kd1 = PID_KD_INCREMENT_3;
		SpeedC_PID_3.PID_OutMax = SpeedC_INCREMENT_OUT_MAX;
		SpeedC_PID_3.PID_OutStep = SpeedC_INCREMENT_STEP_MAX;
		
		SpeedC_PID_4.Kp1 = PID_KP_INCREMENT_4;
		SpeedC_PID_4.Ki1 = PID_KI_INCREMENT_4;
		SpeedC_PID_4.Kd1 = PID_KD_INCREMENT_4;
		SpeedC_PID_4.PID_OutMax = SpeedC_INCREMENT_OUT_MAX;
		SpeedC_PID_4.PID_OutStep = SpeedC_INCREMENT_STEP_MAX;
		
	}
	else if(PID_WorkType == PositionPID_e)
	{
		SpeedC_PID_1.Kp1 = PID_KP_POSITION_1;
		SpeedC_PID_1.Ki1 = PID_KI_POSITION_1;
		SpeedC_PID_1.Kd1 = PID_KD_POSITION_1;
		SpeedC_PID_1.PID_OutMax = SpeedC_POSITION_OUT_MAX;
		SpeedC_PID_1.PID_OutStep = SpeedC_POSITION_STEP_MAX;
		
		SpeedC_PID_2.Kp1 = PID_KP_POSITION_2;
		SpeedC_PID_2.Ki1 = PID_KI_POSITION_2;
		SpeedC_PID_2.Kd1 = PID_KD_POSITION_2;
		SpeedC_PID_2.PID_OutMax = SpeedC_POSITION_OUT_MAX;
		SpeedC_PID_2.PID_OutStep = SpeedC_POSITION_STEP_MAX;
		
		SpeedC_PID_3.Kp1 = PID_KP_POSITION_3;
		SpeedC_PID_3.Ki1 = PID_KI_POSITION_3;
		SpeedC_PID_3.Kd1 = PID_KD_POSITION_3;
		SpeedC_PID_3.PID_OutMax = SpeedC_POSITION_OUT_MAX;
		SpeedC_PID_3.PID_OutStep = SpeedC_POSITION_STEP_MAX;
		
		SpeedC_PID_4.Kp1 = PID_KP_POSITION_4;
		SpeedC_PID_4.Ki1 = PID_KI_POSITION_4;
		SpeedC_PID_4.Kd1 = PID_KD_POSITION_4;
		SpeedC_PID_4.PID_OutMax = SpeedC_POSITION_OUT_MAX;
		SpeedC_PID_4.PID_OutStep = SpeedC_POSITION_STEP_MAX;
	}
	
	SpeedC_PID_1.PID_ErrAllMax = SpeedC_ERRALL_MAX;
	SpeedC_PID_2.PID_ErrAllMax = SpeedC_ERRALL_MAX;
	SpeedC_PID_3.PID_ErrAllMax = SpeedC_ERRALL_MAX;
	SpeedC_PID_4.PID_ErrAllMax = SpeedC_ERRALL_MAX;
	
	SpeedC_PID_1.PID_WorkType = PID_WorkType;
	SpeedC_PID_2.PID_WorkType = PID_WorkType;
	SpeedC_PID_3.PID_WorkType = PID_WorkType;
	SpeedC_PID_4.PID_WorkType = PID_WorkType;
	
	SpeedC_PID_1.PID_Precision = SpeedC_PRECISION;
	SpeedC_PID_2.PID_Precision = SpeedC_PRECISION;
	SpeedC_PID_3.PID_Precision = SpeedC_PRECISION;
	SpeedC_PID_4.PID_Precision = SpeedC_PRECISION;
	
}
//
// target=100�ǵ��ת��, input=10ms��ȡ��������ֵ��out�ǵ��ת�������375
void Speed_ControlLoop(void)
{	
	
	PID_Update(&SpeedC_PID_1,SpeedC_Speed.RealSpeed_1);		
	PID_Update(&SpeedC_PID_2,SpeedC_Speed.RealSpeed_2);		
	PID_Update(&SpeedC_PID_3,SpeedC_Speed.RealSpeed_3);
	PID_Update(&SpeedC_PID_4,SpeedC_Speed.RealSpeed_4);

	if(SpeedC_PIDWorkType == IncrementPID_e)
	{
		PID_GetIncrementalPID(&SpeedC_PID_1);
		PID_GetIncrementalPID(&SpeedC_PID_2);
		PID_GetIncrementalPID(&SpeedC_PID_3);
		PID_GetIncrementalPID(&SpeedC_PID_4);
	}
	else if(SpeedC_PIDWorkType == PositionPID_e)
	{
		PID_GetPositionPID(&SpeedC_PID_1);
		PID_GetPositionPID(&SpeedC_PID_2);
		PID_GetPositionPID(&SpeedC_PID_3);
		PID_GetPositionPID(&SpeedC_PID_4);
	}	

	SetWheelCurrent(SpeedC_PID_1.PID_Out, SpeedC_PID_2.PID_Out, SpeedC_PID_3.PID_Out, SpeedC_PID_4.PID_Out);
}



void SpeedC_RealitySpeedUp(int16_t Wheel_1, int16_t Wheel_2, int16_t Wheel_3, int16_t Wheel_4)  //
{
	SpeedC_Speed.RealSpeed_1 = Wheel_1;
	SpeedC_Speed.RealSpeed_2 = Wheel_2;
	SpeedC_Speed.RealSpeed_3 = Wheel_3;
	SpeedC_Speed.RealSpeed_4 = Wheel_4;
	
}


void SpeedC_SetTarget(float Wheel_1, float Wheel_2, float Wheel_3, float Wheel_4)
{
	PID_SetTargetWithRamp(&SpeedC_PID_1,Wheel_1);
	PID_SetTargetWithRamp(&SpeedC_PID_2,Wheel_2);
	PID_SetTargetWithRamp(&SpeedC_PID_3,Wheel_3);
	PID_SetTargetWithRamp(&SpeedC_PID_4,Wheel_4);
}
//
float SpeedC_GetTarget(uint8_t Wheel_x)
{
	switch(Wheel_x)
	{
        case 1: return SpeedC_PID_1.PID_Target;
        case 2: return SpeedC_PID_2.PID_Target;
        case 3: return SpeedC_PID_3.PID_Target;
        case 4: return SpeedC_PID_4.PID_Target;
    }
    return 0;	
}
//
void SpeedC_SetOneRamp(uint8_t WhichWheel,float RampTime,float RampStep)
{//λ�û����棬��Ҫ����ÿһ�����Ӳ�ͬ��б��
	switch(WhichWheel)
	{
		case 1:
		{
			SpeedC_PID_1.RampTartgetTime = RampTime;
			SpeedC_PID_1.RampTartgetStep = RampStep;
			break;
		}
		
		case 2:
		{
			SpeedC_PID_2.RampTartgetTime = RampTime;
			SpeedC_PID_2.RampTartgetStep = RampStep;
			break;
		}
		
		case 3:
		{
			SpeedC_PID_3.RampTartgetTime = RampTime;
			SpeedC_PID_3.RampTartgetStep = RampStep;
			break;
		}
		
		case 4:
		{
			SpeedC_PID_4.RampTartgetTime = RampTime;
			SpeedC_PID_4.RampTartgetStep = RampStep;
			break;
		}		
	}	
}
//
void SpeedC_ClearPID(void)
{
	PID_Clear(&SpeedC_PID_1);
	PID_Clear(&SpeedC_PID_2);
	PID_Clear(&SpeedC_PID_3);
	PID_Clear(&SpeedC_PID_4);
}
//
float SpeedC_GetRealSpeed(uint8_t Wheel_x)
{
	switch(Wheel_x)
	{
		case 1:
			return SpeedC_Speed.RealSpeed_1;
		
		case 2:
			return SpeedC_Speed.RealSpeed_2;
		
		case 3:
			return SpeedC_Speed.RealSpeed_3;
		
		case 4:
			return SpeedC_Speed.RealSpeed_4;
		
	}
	return 0;
}


void print(void){
	printf("V,T,O:%d,%.1f,%.2f\n\r",SpeedC_Speed.RealSpeed_2,SpeedC_GetTarget(2),SpeedC_PID_2.PID_Out);
}


extern uint8_t UpdatePIDReality ;
void PID_DebugPID(void)
{
	int16_t Temp1 = 0;
	
//	
//	SpeedC_SetTarget(50,50,50,50);
//	SpeedC_SetOneRamp(1,0,2);
//	SpeedC_SetOneRamp(2,0,2);
//	SpeedC_SetOneRamp(3,0,2);
//	SpeedC_SetOneRamp(4,0,2);
	
	
	while(1)
	{
		if(UpdatePIDReality == 1)
		{
			++ Temp1;
									
//			//���1�����ӵ�Ŀ��ֵ����ǰ�ٶȡ�����ֵ
//			printf("V,T,O:%d,%.1f,%.2f\n\r",SpeedC_Speed.RealSpeed_1,SpeedC_GetTarget(1),SpeedC_PID_1.PID_Out);
			
			//���2�����ӵ�Ŀ��ֵ����ǰ�ٶȡ�����ֵ
			printf("V,T,O:%d,%.1f,%.2f\n\r",SpeedC_Speed.RealSpeed_2,SpeedC_GetTarget(2),SpeedC_PID_2.PID_Out);
			
//			//���3�����ӵ�Ŀ��ֵ����ǰ�ٶȡ�����ֵ
//			printf("V,T,O:%d,%.1f,%.2f\n\r",SpeedC_Speed.RealSpeed_3,SpeedC_GetTarget(3),SpeedC_PID_3.PID_Out);
//			
//			//���4�����ӵ�Ŀ��ֵ����ǰ�ٶȡ�����ֵ
//			printf("V,T,O:%d,%.1f,%.2f\n\r",SpeedC_Speed.RealSpeed_4,SpeedC_GetTarget(4),SpeedC_PID_4.PID_Out);

			
			//printf("V,T,O:%d,%d,%.2f,%d\n\r",WheelPID_1.RealitySpeed,GetPIDTargetSpeed(1),GetPidOut(1),WheelPID_1.SpeedErr_now);

			
			//
//			printf("V,T,O:%d,%d,%d,%d\n\r",SpeedC_Speed.RealSpeed_1,SpeedC_Speed.RealSpeed_1,SpeedC_Speed.RealSpeed_1,SpeedC_Speed.RealSpeed_1);
			
			//���4�����ӵ�ʵ���ٶ�
//		printf("V4:%d,%d,%d,%d,%.1f\n\r",SpeedC_Speed.RealSpeed_1,SpeedC_Speed.RealSpeed_2,SpeedC_Speed.RealSpeed_3,SpeedC_Speed.RealSpeed_4,SpeedC_GetTarget(1));
			//printf("V4:%d,%d,%d,%d\n\r",PID_GetAllStep(1),PID_GetAllStep(2),PID_GetAllStep(3),PID_GetAllStep(4));
			
			//���3�����������1���ֵ�ƫ��
			//printf("V4:%d,%d,%d\n\r",Temp2,Temp3,Temp4);
			
			//printf("V4:%d,%d,%d,%d\n\r",GetPIDTargetSpeed(1),GetPIDTargetSpeed(2),GetPIDTargetSpeed(3),GetPIDTargetSpeed(4));
			
			UpdatePIDReality = 0;
//			if(Temp1 >= 1500)			
//			//if(WheelPID_3.WheelSeppdAll >= 6000000)// && Temp1 == 0)
//			{
//				
//				Temp1 = 0;
//				//SetPIDTargetSpeed(0,0,0,0);
//				break;
//			}
		}
	}
//	
//	SpeedC_SetTarget(0,0,0,0);
//	//SetWheelCurrent(0,0,0,0);
//	printf("speed is 0\n");
//	Temp1 = 0;
//	while(1)
//	{
//		if(UpdatePIDReality == 1)
//		{
//			++ Temp1;
//			
//			
//			//���1�����ӵ�Ŀ��ֵ����ǰ�ٶȡ�����ֵ
////			printf("V,T,O:%d,%.1f,%.2f\n\r",WheelPID_1.RealitySpeed,GetPIDTargetSpeed(1),GetPidOut(1));
//			
////			//���2�����ӵ�Ŀ��ֵ����ǰ�ٶȡ�����ֵ
////			printf("V,T,O:%d,%.1f,%.2f\n\r",WheelPID_2.RealitySpeed,GetPIDTargetSpeed(2),GetPidOut(2));
////			
////			//���3�����ӵ�Ŀ��ֵ����ǰ�ٶȡ�����ֵ
////			printf("V,T,O:%d,%.1f,%.2f\n\r",WheelPID_3.RealitySpeed,GetPIDTargetSpeed(3),GetPidOut(3));
////			
////			//���4�����ӵ�Ŀ��ֵ����ǰ�ٶȡ�����ֵ
////			printf("V,T,O:%d,%.1f,%.2f\n\r",WheelPID_4.RealitySpeed,GetPIDTargetSpeed(4),GetPidOut(4));
//			
//			//printf("V,T,O:%d,%d,%.2f,%d\n\r",WheelPID_1.RealitySpeed,GetPIDTargetSpeed(1),GetPidOut(1),WheelPID_1.SpeedErr_now);
//			
//			//
//			//printf("V,T,O:%d,%d,%d,%d,%d,%.2f\n\r",WheelPID_1.RealitySpeed,WheelPID_2.RealitySpeed,WheelPID_3.RealitySpeed,WheelPID_4.RealitySpeed,GetPIDTargetSpeed(1),GetPidOut(1));
//			
//			//���4�����ӵ�ʵ���ٶ�
//		printf("V4:%d,%d,%d,%d,%.1f\n\r",SpeedC_Speed.RealSpeed_1,SpeedC_Speed.RealSpeed_2,SpeedC_Speed.RealSpeed_3,SpeedC_Speed.RealSpeed_4,SpeedC_GetTarget(1));
//			//printf("V4:%d,%d,%d,%d\n\r",PID_GetAllStep(1),PID_GetAllStep(2),PID_GetAllStep(3),PID_GetAllStep(4));
//			
//			//���4�����ӵ�Ŀ���ٶ�
//			//printf("V4:%d,%d,%d,%d\n\r",GetPIDTargetSpeed(1),GetPIDTargetSpeed(2),GetPIDTargetSpeed(3),GetPIDTargetSpeed(4));
//			UpdatePIDReality = 0;
//			if(Temp1 >= 1500)
//				break;
//		}
//	}

	
	
	while(1)
		delay_ms(10);
}




