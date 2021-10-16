/*
速度解算公式轮子位置不同得到的结果不同，严格对应后才能得出结果

*/
#include "SpeedSolving.h"
#include "usart1.h"


SpeedSolvingOut4_t SpeedSolvingOut4_1;
PositionSolving_t PositionSolving_1;
TargetSpeed_t TargetSpeed;



float MyAbs(float a)
{ 		   
	float temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
//
//float SpeedSolving_OToDis(float OToDis)
//{//输入角度，单位弧度，返回轮子走过的位移，单位为mm
//	static float Speed_Solving4_Data_OToDis = 30.0/3.1415926;//角速度转为转速要乘的比例，即N=W*30/pi=9.549*W
//	return (OToDis * Speed_Solving4_Data_OToDis);
//}
//
float SpeedSolving_WToN(float OMIGA)
{//输入角速度，单位弧度每秒，返回转速，单位转每分钟
	static float Speed_Solving4_Data_WToN = 30.0/3.1415926;//角速度转为转速要乘的比例，即N=W*30/pi=9.549*W
	return (OMIGA * Speed_Solving4_Data_WToN);
}
//

float SpeedSolving_PToW(float Pulse_B)
{//输入编码器脉冲，单位个，返回轮子转过的角度，单位弧度
	static float Speed_Solving4_Data_PToW = 2*3.1415926/SPEED_SOLVING4_DATA_ENCODE_PULSE_MAX;//编码器脉冲转为角度要乘的比例，
	//	即W=P/SPEED_SOLVING4_DATA_ENCODE_PULSE_MAX*2*pi
	return (Pulse_B*Speed_Solving4_Data_PToW);
}

float SpeedSolving_OToD(float OMIGA)
{//弧度转换为角度，输入弧度，输出角度
	static float Speed_Solving4_Data_OToD = 180/3.1415926;//D=O/3.1415926*180
	return (OMIGA * Speed_Solving4_Data_OToD);
}	
//
float SpeedSolving_DToO(float DEG)
{//角度转换为弧度，输入角度，输出弧度
	static float Speed_Solving4_Data_DToO = 3.1415926/180.0;
	return (DEG * Speed_Solving4_Data_DToO);
}
//


/*
* 输入形参：Speed_x、Speed_y单位均为m/s，Speed_z单位为度每秒，输出的结果是每10ms的编码器脉冲
*/
SpeedSolvingOut4_t SpeedSolving_4(float Speed_x,float Speed_y,float Speed_z)
{//逆解算
	static float Speed_Solving4_Data_1 = 1/(SPEED_SOLVING4_DATA_R_Y*SPEED_SOLVING4_DATA_TAN_A);//逆解算矩阵里面的第一列公算子
	static float Speed_Solving4_Data_2 = 1/SPEED_SOLVING4_DATA_R_X;//逆解算矩阵里面的第二列公算子
	static float Speed_Solving4_Data_3 = (SPEED_SOLVING4_DATA_TAN_A * SPEED_SOLVING4_DATA_L1 + SPEED_SOLVING4_DATA_L2)
										/(SPEED_SOLVING4_DATA_R_Z*SPEED_SOLVING4_DATA_TAN_A);//逆解算矩阵里面的第三列公算子	
	static float Speed_Solving4_Data_NToS = SPEED_SOLVING4_DATA_MOTOR_PULSE_N/SPEED_SOLVING4_DATA_MOTOR_N;
	//转速转换为编码器脉冲要乘的比例，即PWM=N*SPEED_SOLVING4_DATA_ENCODE_PULSE_MAX/SPEED_SOLVING4_DATA_MOTOR_N
	static float Speed_Solving4_Data_DToR = 3.1415926/180.0;//度转换为弧度
	static float Speed_Solving4_W1,Speed_Solving4_W2,Speed_Solving4_W3,Speed_Solving4_W4;//角速度
	static float Speed_Solving4_N1,Speed_Solving4_N2,Speed_Solving4_N3,Speed_Solving4_N4;//转速
	
	Speed_x = Speed_x * 1000 ;// 4.0;//单位从m/s转为mm/s
	Speed_y = Speed_y * 1000 ;// 4.0;//单位从m/s转为mm/s		
	Speed_z = Speed_z * Speed_Solving4_Data_DToR ;// 4.0; 	//度每秒转为弧度每秒
	
	Speed_Solving4_W4 = Speed_y * Speed_Solving4_Data_1 + Speed_x * Speed_Solving4_Data_2 + Speed_z * Speed_Solving4_Data_3;
	Speed_Solving4_W3 = -Speed_y * Speed_Solving4_Data_1 + Speed_x * Speed_Solving4_Data_2 + Speed_z * Speed_Solving4_Data_3;
	Speed_Solving4_W1 = -Speed_y * Speed_Solving4_Data_1 + Speed_x * Speed_Solving4_Data_2 - Speed_z * Speed_Solving4_Data_3;
	Speed_Solving4_W2 = Speed_y * Speed_Solving4_Data_1 + Speed_x * Speed_Solving4_Data_2 - Speed_z * Speed_Solving4_Data_3;
	//上面算出了角速度，单位是弧度每秒
	
	Speed_Solving4_N1 = SpeedSolving_WToN(Speed_Solving4_W1);
	Speed_Solving4_N2 = SpeedSolving_WToN(Speed_Solving4_W2);
	Speed_Solving4_N3 = SpeedSolving_WToN(Speed_Solving4_W3);
	Speed_Solving4_N4 = SpeedSolving_WToN(Speed_Solving4_W4);
	//把角速度转化为转速，单位是转/min
	
	SpeedSolvingOut4_1.Speed_1 = Speed_Solving4_N1 * Speed_Solving4_Data_NToS;//
	SpeedSolvingOut4_1.Speed_2 = Speed_Solving4_N2 * Speed_Solving4_Data_NToS;
	SpeedSolvingOut4_1.Speed_3 = Speed_Solving4_N3 * Speed_Solving4_Data_NToS;
	SpeedSolvingOut4_1.Speed_4 = Speed_Solving4_N4 * Speed_Solving4_Data_NToS;
	SpeedSolvingOut4_1.UpData = 1;
	
	return SpeedSolvingOut4_1;
}

/*
* 下面是正解算，输入每个轮子转的编码器脉冲，输出小车X、Y、Z方向转过的距离和角度
*/
PositionSolving_t PositionSolving_4(float Pulse_1,float Pulse_2,float Pulse_3,float Pulse_4)
{//正解算
	static float Position_Solving4_Angel_1,Position_Solving4_Angel_2,Position_Solving4_Angel_3,Position_Solving4_Angel_4;//角度，单位为弧度
	static float Position_Solving4_Data_1 = 0.25*SPEED_SOLVING4_DATA_R_Y*SPEED_SOLVING4_DATA_TAN_A;//正解算矩阵里面的第一行公算子，算了1/4
	static float Position_Solving4_Data_2 = SPEED_SOLVING4_DATA_R_X*0.25;//正解算矩阵里面的第二行公算子
	static float Position_Solving4_Data_3 = 0.25*SPEED_SOLVING4_DATA_R_Z*SPEED_SOLVING4_DATA_TAN_A
											 /(SPEED_SOLVING4_DATA_TAN_A * SPEED_SOLVING4_DATA_L1 + SPEED_SOLVING4_DATA_L2);//正解算矩阵里面的第三行公算子
	
	Position_Solving4_Angel_1 = SpeedSolving_PToW(Pulse_1);
	Position_Solving4_Angel_2 = SpeedSolving_PToW(Pulse_2);
	Position_Solving4_Angel_3 = SpeedSolving_PToW(Pulse_3);
	Position_Solving4_Angel_4 = SpeedSolving_PToW(Pulse_4);//把实际的1234转为矩阵里面的1234
	
	PositionSolving_1.PositionY_mm = (-Position_Solving4_Angel_1 + Position_Solving4_Angel_2
										- Position_Solving4_Angel_3 + Position_Solving4_Angel_4) * Position_Solving4_Data_1;
	PositionSolving_1.PositionX_mm = (Position_Solving4_Angel_1 + Position_Solving4_Angel_2
										+ Position_Solving4_Angel_3 + Position_Solving4_Angel_4) * Position_Solving4_Data_2;
	PositionSolving_1.PositionZ_D = (-Position_Solving4_Angel_1 - Position_Solving4_Angel_2
										+ Position_Solving4_Angel_3 + Position_Solving4_Angel_4) * Position_Solving4_Data_3;
	PositionSolving_1.PositionZ_D = SpeedSolving_OToD(PositionSolving_1.PositionZ_D);
	
	return PositionSolving_1;
}
//

/*
* 同时传入目标值
*/

void SpeedSolving4_Set(void)
{
	SpeedSolving4_SetWithTime(SPEED_SOLVING_RAMP_TIME,&SpeedSolvingOut4_1);
}


/*
* 在位置环计算出速度之后，还要精准控制时间，因此需要传进来一个斜坡函数的横距离time参数
*/
void SpeedSolving4_SetWithTime(float HowTime,SpeedSolvingOut4_t *TargetPulse)
{
	//将速度解算出的结果设置给速度环，需要考虑速度环里面的斜坡
	static float StepTemp1 = 0;
	static float StepTemp2 = 0;
	static float StepTemp3 = 0;
	static float StepTemp4 = 0;
	
	StepTemp1 = (TargetPulse->Speed_1 - SpeedC_GetTarget(1))/HowTime;
	StepTemp2 = (TargetPulse->Speed_2 - SpeedC_GetTarget(2))/HowTime;
	StepTemp3 = (TargetPulse->Speed_3 - SpeedC_GetTarget(3))/HowTime;
	StepTemp4 = (TargetPulse->Speed_4 - SpeedC_GetTarget(4))/HowTime;
		
	if(MyAbs(StepTemp1) < SPEED_SOLVING_RAMP_STEP_MIN)
		StepTemp1 = SPEED_SOLVING_RAMP_STEP_MIN;
	if(MyAbs(StepTemp2) < SPEED_SOLVING_RAMP_STEP_MIN)
		StepTemp2 = SPEED_SOLVING_RAMP_STEP_MIN;
	if(MyAbs(StepTemp3) < SPEED_SOLVING_RAMP_STEP_MIN)
		StepTemp3 = SPEED_SOLVING_RAMP_STEP_MIN;
	if(MyAbs(StepTemp4) < SPEED_SOLVING_RAMP_STEP_MIN)
		StepTemp4 = SPEED_SOLVING_RAMP_STEP_MIN;
	
	SpeedC_SetOneRamp(1,0,MyAbs(StepTemp1));
	SpeedC_SetOneRamp(2,0,MyAbs(StepTemp2));
	SpeedC_SetOneRamp(3,0,MyAbs(StepTemp3));
	SpeedC_SetOneRamp(4,0,MyAbs(StepTemp4));
	
	SpeedC_SetTarget(TargetPulse->Speed_1,TargetPulse->Speed_2,TargetPulse->Speed_3,TargetPulse->Speed_4);
	TargetPulse->UpData = 0;
}
//


void SpeedSolving_Report(void)
{
	float MyTemp1;
	
	MyTemp1 = SPEED_SOLVING4_DATA_MOTOR_N * 2.0f * 3.1415926f * SPEED_SOLVING4_DATA_R / 60000.0f;//每分钟最快转过的mm数/60000ms=m/s
	//得最快速度为1.69m/s。假设3s内达到最快速度不会打滑，则加速度最大为0.565m/s2，
	printf("最快速度%fm/s\r\n",MyTemp1);	
}


TargetSpeed_t SpeedSolving_GetTargetSpeed(void)
{
	return TargetSpeed;
}

