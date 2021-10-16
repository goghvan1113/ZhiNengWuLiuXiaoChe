#include "CarRun.h"
#include "PIDController.h"
#include "SpeedControl.h"
#include "usart1.h"
#include "SpeedSolving.h"
#include "tracking.h"

#define PID_RAMP_STEP	2.000000f		//速度环斜坡函数步幅
#define MOT_MaxCurrent 90       //速度环输入的最大值


#define WHEEL_CODE_NUM	1560	//轮子轴转一圈的脉冲数
#define WRunRampStep	300
#define WRunRampTime	0


#define WRUN_PID_PRECISION	5
#define WRUN_PID_ERRALL_MAX	3120

//下面是PID参数
#define WRun_PID_KP_POSITION_1 0.08000f                                  
#define WRun_PID_KI_POSITION_1 0.00000f
#define WRun_PID_KD_POSITION_1 0.00000f

#define WRun_PID_KP_POSITION_2 0.004000f                                  
#define WRun_PID_KI_POSITION_2 0.000000f
#define WRun_PID_KD_POSITION_2 0.000000f

#define WRun_PID_KP_POSITION_3 0.004000f                                   
#define WRun_PID_KI_POSITION_3 0.000000f
#define WRun_PID_KD_POSITION_3 0.000000f

#define WRun_PID_KP_POSITION_4 0.004000f                                   
#define WRun_PID_KI_POSITION_4 0.000000f
#define WRun_PID_KD_POSITION_4 0.000000f


static uint8_t WRun_PID_Updata = 0;   //位置环PID是否更新


PID_t PID1;
PID_t PID2;
PID_t PID3;
PID_t PID4;

WRun_PositionRdcord_t PositionRdcord;


void PIDInit(void)
{
	PID_DefaultInit(&PID1);
	PID_DefaultInit(&PID2);
	PID_DefaultInit(&PID3);
	PID_DefaultInit(&PID4);
	
	PID1.Kp1=WRun_PID_KP_POSITION_1;
	PID1.Ki1=WRun_PID_KI_POSITION_1;
	PID1.Kd1=WRun_PID_KD_POSITION_1;
	PID1.RampTartgetTime=WRunRampTime;
	PID1.RampTartgetStep=WRunRampStep;
	PID1.PID_Precision=WRUN_PID_PRECISION;
	PID1.PID_ErrAllMax=WRUN_PID_ERRALL_MAX;
	PID1.PID_OutMax=MOT_MaxCurrent;
	
	PID2.Kp1=WRun_PID_KP_POSITION_2;
	PID2.Ki1=WRun_PID_KI_POSITION_2;
	PID2.Kd1=WRun_PID_KD_POSITION_2;
	PID2.RampTartgetTime=WRunRampTime;
	PID2.RampTartgetStep=WRunRampStep;
	PID2.PID_Precision=WRUN_PID_PRECISION;
	PID2.PID_ErrAllMax=WRUN_PID_ERRALL_MAX;
	PID2.PID_OutMax=MOT_MaxCurrent;
	
	PID3.Kp1=WRun_PID_KP_POSITION_3;
	PID3.Ki1=WRun_PID_KI_POSITION_3;
	PID3.Kd1=WRun_PID_KD_POSITION_3;
	PID3.RampTartgetTime=WRunRampTime;
	PID3.RampTartgetStep=WRunRampStep;
	PID3.PID_Precision=WRUN_PID_PRECISION;
	PID3.PID_ErrAllMax=WRUN_PID_ERRALL_MAX;
	PID3.PID_OutMax=MOT_MaxCurrent;
	
	PID4.Kp1=WRun_PID_KP_POSITION_4;
	PID4.Ki1=WRun_PID_KI_POSITION_4;
	PID4.Kd1=WRun_PID_KD_POSITION_4;
	PID4.RampTartgetTime=WRunRampTime;
	PID4.RampTartgetStep=WRunRampStep;
	PID4.PID_Precision=WRUN_PID_PRECISION;
	PID4.PID_ErrAllMax=WRUN_PID_ERRALL_MAX;
	PID4.PID_OutMax=MOT_MaxCurrent;
	
}
void WRun_Update(int16_t Wheel_1,int16_t Wheel_2,int16_t Wheel_3,int16_t Wheel_4)
{//更新位置环四个轮子脉冲数

	PositionRdcord.AllPulse_1 += Wheel_1;
	PositionRdcord.AllPulse_2 += Wheel_2;
	PositionRdcord.AllPulse_3 += Wheel_3;
	PositionRdcord.AllPulse_4 += Wheel_4;

}

void WRun_Loop1(void)
{
	PID_Update(&PID1,	PositionRdcord.AllPulse_1);
	PID_Update(&PID2,	PositionRdcord.AllPulse_2);
	PID_Update(&PID3,	PositionRdcord.AllPulse_3);
	PID_Update(&PID4,	PositionRdcord.AllPulse_4);
	
	PID_GetPositionPID(&PID1);
	PID_GetPositionPID(&PID4);
	PID_GetPositionPID(&PID2);
	PID_GetPositionPID(&PID3);

	SpeedC_SetTarget(PID1.PID_Out,PID2.PID_Out,PID3.PID_Out,PID4.PID_Out);
	WRun_PID_Updata = 1;
	
}


void WRun_SetWheelRunStep(uint8_t Wheel_x,float AllTurns,int16_t MaxSpeed,float AdvanceStopTurns)
{
	switch(Wheel_x)
	{
		case 1:
		{
			PID1.PID_OutMax = MaxSpeed;
			PID1.State_RampOrNormal = Normal_e;
			PID1.PID_Target = AllTurns * WHEEL_CODE_NUM;
			PID1.RampTarget = (AllTurns - AdvanceStopTurns) * WHEEL_CODE_NUM;
			break;
		}
		case 2:
		{
			PID2.PID_OutMax = MaxSpeed;
			PID2.State_RampOrNormal = Normal_e;
			PID2.PID_Target = AllTurns * WHEEL_CODE_NUM;
			PID2.RampTarget = (AllTurns - AdvanceStopTurns) * WHEEL_CODE_NUM;
			break;
		}
		case 3:
		{
			PID3.PID_OutMax = MaxSpeed;
			PID3.State_RampOrNormal = Normal_e;
			PID3.PID_Target = AllTurns * WHEEL_CODE_NUM;
			PID3.RampTarget = (AllTurns - AdvanceStopTurns) * WHEEL_CODE_NUM;
			break;
		}
		case 4:
		{
			PID4.PID_OutMax = MaxSpeed;
			PID4.State_RampOrNormal = Normal_e;
			PID4.PID_Target = AllTurns * WHEEL_CODE_NUM;
			PID4.RampTarget = (AllTurns - AdvanceStopTurns) * WHEEL_CODE_NUM;
			break;
		}		
	}
}

extern uint8_t UpdatePIDReality ;
void WRunPIDDebug(void)
{
	int32_t Temp1 = 0;

	WRun_SetWheelRunStep(1,10,90,3);
	WRun_SetWheelRunStep(2,10,90,3);
	WRun_SetWheelRunStep(3,10,90,3);
	WRun_SetWheelRunStep(4,10,90,3);
	
	while(1)
	{
		if(WRun_PID_Updata == 1)
		{
			
			//输出一个通道的实际值，目标值，输出值，
			printf("STO:%d,%.2f,%.2f\n\r",PositionRdcord.AllPulse_1,PID1.PID_Target,PID1.PID_Out);
//			printf("STO:%d,%.2f,%.2f\n\r",PositionRdcord.AllPulse_2,PID2.PID_Target,PID2.PID_Out);
			
			
			//输出4个轮子的实际转的圈数
//			printf("EOR:%d,%d,%d,%d\n\r",PositionRdcord.AllPulse_1,PositionRdcord.AllPulse_2,PositionRdcord.AllPulse_3,PositionRdcord.AllPulse_4);
						
			WRun_PID_Updata = 0;
			++ Temp1;
		}
		
//		if(UpdatePIDReality == 1)
//		{
//			//输出4个轮子的实时转速
//			printf("EOR:%.2f,%.2f,%.2f,%.2f\n\r",SpeedC_GetRealSpeed(1),SpeedC_GetRealSpeed(2),SpeedC_GetRealSpeed(3),SpeedC_GetRealSpeed(4));
//			
//			//输出4个轮子转速的目标值
//			printf("EOR:%.2f,%.2f,%.2f,%.2f\n\r",SpeedC_GetTarget(1),SpeedC_GetTarget(2),SpeedC_GetTarget(3),SpeedC_GetTarget(4));
//			UpdatePIDReality = 0;
//		}
		
	}
}


SpeedSolvingOut4_t SpeedSolvingOut4_2;

SpeedSolvingOut4_t SimpleSpeedSolving_4(float Speed_x,float Speed_y,float Speed_z)
{
	float Ramp1 = 0;
	float Ramp2 = 0;
	float TargetSpeed1 = 0;
	float TargetSpeed2 = 0;
	float StepTemp1 = 0;
	float StepTemp2 = 0;
	
	SpeedSolvingOut4_2.Speed_1 = Speed_y - Speed_x + Speed_z;
	SpeedSolvingOut4_2.Speed_2 = Speed_y + Speed_x - Speed_z;
	SpeedSolvingOut4_2.Speed_3 = Speed_y - Speed_x - Speed_z;
	SpeedSolvingOut4_2.Speed_4 = Speed_y + Speed_x + Speed_z;
	
	TargetSpeed1=SpeedC_GetTarget(1);
	TargetSpeed2=SpeedC_GetTarget(2);
	
	StepTemp1 = MyAbs(Speed_y - Speed_x - TargetSpeed1);
	StepTemp2 = MyAbs(Speed_y + Speed_x - TargetSpeed2);
	
	if(StepTemp1 < StepTemp2)
	{//说明1、3轮子目标速度比2、4轮子小，则1、3轮子的斜率应该变缓，斜坡单次时间应该为2、4轮子的单次时间的speed
		Ramp1 = StepTemp1 / StepTemp2 * PID_RAMP_STEP;
		Ramp2 = PID_RAMP_STEP;		
	}
	else if(StepTemp1 > StepTemp2)
	{//说明1、3轮子目标速度比2、4轮子大，则2、4轮子的斜率应该变缓，斜坡单次时间应该为2、4轮子的单次时间的speed
		Ramp2 =  StepTemp2 / StepTemp1 * PID_RAMP_STEP;
		Ramp1 = PID_RAMP_STEP;	
	}
	else 
	{
		Ramp1 = PID_RAMP_STEP;
		Ramp2 = PID_RAMP_STEP;
	}
	SpeedC_SetOneRamp(1,0,Ramp1);
	SpeedC_SetOneRamp(2,0,Ramp2);
	SpeedC_SetOneRamp(3,0,Ramp1);
	SpeedC_SetOneRamp(4,0,Ramp2);
	
	SpeedC_SetTarget(SpeedSolvingOut4_2.Speed_1,SpeedSolvingOut4_2.Speed_2,SpeedSolvingOut4_2.Speed_3,SpeedSolvingOut4_2.Speed_4);
}

void RunPointSimple(void)
{
	SimpleSpeedSolving_4(0,40,0);
//	SpeedSolving_4(-1,0,0);
//	SpeedSolving4_Set();
//	TCK_CountLine('F',5);
//	SimpleSpeedSolving_4(0,0,0);

//	SimpleSpeedSolving_4(-30,50,0);
//	delay_ms(3000);
//	SpeedSolving_4(0,0,180);
//	SpeedSolving4_Set();
//	delay_ms(1000);
//	SpeedSolving_4(0,0,0);
//	SpeedSolving4_Set();
}