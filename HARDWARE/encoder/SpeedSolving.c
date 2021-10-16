/*
�ٶȽ��㹫ʽ����λ�ò�ͬ�õ��Ľ����ͬ���ϸ��Ӧ����ܵó����

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
//{//����Ƕȣ���λ���ȣ����������߹���λ�ƣ���λΪmm
//	static float Speed_Solving4_Data_OToDis = 30.0/3.1415926;//���ٶ�תΪת��Ҫ�˵ı�������N=W*30/pi=9.549*W
//	return (OToDis * Speed_Solving4_Data_OToDis);
//}
//
float SpeedSolving_WToN(float OMIGA)
{//������ٶȣ���λ����ÿ�룬����ת�٣���λתÿ����
	static float Speed_Solving4_Data_WToN = 30.0/3.1415926;//���ٶ�תΪת��Ҫ�˵ı�������N=W*30/pi=9.549*W
	return (OMIGA * Speed_Solving4_Data_WToN);
}
//

float SpeedSolving_PToW(float Pulse_B)
{//������������壬��λ������������ת���ĽǶȣ���λ����
	static float Speed_Solving4_Data_PToW = 2*3.1415926/SPEED_SOLVING4_DATA_ENCODE_PULSE_MAX;//����������תΪ�Ƕ�Ҫ�˵ı�����
	//	��W=P/SPEED_SOLVING4_DATA_ENCODE_PULSE_MAX*2*pi
	return (Pulse_B*Speed_Solving4_Data_PToW);
}

float SpeedSolving_OToD(float OMIGA)
{//����ת��Ϊ�Ƕȣ����뻡�ȣ�����Ƕ�
	static float Speed_Solving4_Data_OToD = 180/3.1415926;//D=O/3.1415926*180
	return (OMIGA * Speed_Solving4_Data_OToD);
}	
//
float SpeedSolving_DToO(float DEG)
{//�Ƕ�ת��Ϊ���ȣ�����Ƕȣ��������
	static float Speed_Solving4_Data_DToO = 3.1415926/180.0;
	return (DEG * Speed_Solving4_Data_DToO);
}
//


/*
* �����βΣ�Speed_x��Speed_y��λ��Ϊm/s��Speed_z��λΪ��ÿ�룬����Ľ����ÿ10ms�ı���������
*/
SpeedSolvingOut4_t SpeedSolving_4(float Speed_x,float Speed_y,float Speed_z)
{//�����
	static float Speed_Solving4_Data_1 = 1/(SPEED_SOLVING4_DATA_R_Y*SPEED_SOLVING4_DATA_TAN_A);//������������ĵ�һ�й�����
	static float Speed_Solving4_Data_2 = 1/SPEED_SOLVING4_DATA_R_X;//������������ĵڶ��й�����
	static float Speed_Solving4_Data_3 = (SPEED_SOLVING4_DATA_TAN_A * SPEED_SOLVING4_DATA_L1 + SPEED_SOLVING4_DATA_L2)
										/(SPEED_SOLVING4_DATA_R_Z*SPEED_SOLVING4_DATA_TAN_A);//������������ĵ����й�����	
	static float Speed_Solving4_Data_NToS = SPEED_SOLVING4_DATA_MOTOR_PULSE_N/SPEED_SOLVING4_DATA_MOTOR_N;
	//ת��ת��Ϊ����������Ҫ�˵ı�������PWM=N*SPEED_SOLVING4_DATA_ENCODE_PULSE_MAX/SPEED_SOLVING4_DATA_MOTOR_N
	static float Speed_Solving4_Data_DToR = 3.1415926/180.0;//��ת��Ϊ����
	static float Speed_Solving4_W1,Speed_Solving4_W2,Speed_Solving4_W3,Speed_Solving4_W4;//���ٶ�
	static float Speed_Solving4_N1,Speed_Solving4_N2,Speed_Solving4_N3,Speed_Solving4_N4;//ת��
	
	Speed_x = Speed_x * 1000 ;// 4.0;//��λ��m/sתΪmm/s
	Speed_y = Speed_y * 1000 ;// 4.0;//��λ��m/sתΪmm/s		
	Speed_z = Speed_z * Speed_Solving4_Data_DToR ;// 4.0; 	//��ÿ��תΪ����ÿ��
	
	Speed_Solving4_W4 = Speed_y * Speed_Solving4_Data_1 + Speed_x * Speed_Solving4_Data_2 + Speed_z * Speed_Solving4_Data_3;
	Speed_Solving4_W3 = -Speed_y * Speed_Solving4_Data_1 + Speed_x * Speed_Solving4_Data_2 + Speed_z * Speed_Solving4_Data_3;
	Speed_Solving4_W1 = -Speed_y * Speed_Solving4_Data_1 + Speed_x * Speed_Solving4_Data_2 - Speed_z * Speed_Solving4_Data_3;
	Speed_Solving4_W2 = Speed_y * Speed_Solving4_Data_1 + Speed_x * Speed_Solving4_Data_2 - Speed_z * Speed_Solving4_Data_3;
	//��������˽��ٶȣ���λ�ǻ���ÿ��
	
	Speed_Solving4_N1 = SpeedSolving_WToN(Speed_Solving4_W1);
	Speed_Solving4_N2 = SpeedSolving_WToN(Speed_Solving4_W2);
	Speed_Solving4_N3 = SpeedSolving_WToN(Speed_Solving4_W3);
	Speed_Solving4_N4 = SpeedSolving_WToN(Speed_Solving4_W4);
	//�ѽ��ٶ�ת��Ϊת�٣���λ��ת/min
	
	SpeedSolvingOut4_1.Speed_1 = Speed_Solving4_N1 * Speed_Solving4_Data_NToS;//
	SpeedSolvingOut4_1.Speed_2 = Speed_Solving4_N2 * Speed_Solving4_Data_NToS;
	SpeedSolvingOut4_1.Speed_3 = Speed_Solving4_N3 * Speed_Solving4_Data_NToS;
	SpeedSolvingOut4_1.Speed_4 = Speed_Solving4_N4 * Speed_Solving4_Data_NToS;
	SpeedSolvingOut4_1.UpData = 1;
	
	return SpeedSolvingOut4_1;
}

/*
* �����������㣬����ÿ������ת�ı��������壬���С��X��Y��Z����ת���ľ���ͽǶ�
*/
PositionSolving_t PositionSolving_4(float Pulse_1,float Pulse_2,float Pulse_3,float Pulse_4)
{//������
	static float Position_Solving4_Angel_1,Position_Solving4_Angel_2,Position_Solving4_Angel_3,Position_Solving4_Angel_4;//�Ƕȣ���λΪ����
	static float Position_Solving4_Data_1 = 0.25*SPEED_SOLVING4_DATA_R_Y*SPEED_SOLVING4_DATA_TAN_A;//�������������ĵ�һ�й����ӣ�����1/4
	static float Position_Solving4_Data_2 = SPEED_SOLVING4_DATA_R_X*0.25;//�������������ĵڶ��й�����
	static float Position_Solving4_Data_3 = 0.25*SPEED_SOLVING4_DATA_R_Z*SPEED_SOLVING4_DATA_TAN_A
											 /(SPEED_SOLVING4_DATA_TAN_A * SPEED_SOLVING4_DATA_L1 + SPEED_SOLVING4_DATA_L2);//�������������ĵ����й�����
	
	Position_Solving4_Angel_1 = SpeedSolving_PToW(Pulse_1);
	Position_Solving4_Angel_2 = SpeedSolving_PToW(Pulse_2);
	Position_Solving4_Angel_3 = SpeedSolving_PToW(Pulse_3);
	Position_Solving4_Angel_4 = SpeedSolving_PToW(Pulse_4);//��ʵ�ʵ�1234תΪ���������1234
	
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
* ͬʱ����Ŀ��ֵ
*/

void SpeedSolving4_Set(void)
{
	SpeedSolving4_SetWithTime(SPEED_SOLVING_RAMP_TIME,&SpeedSolvingOut4_1);
}


/*
* ��λ�û�������ٶ�֮�󣬻�Ҫ��׼����ʱ�䣬�����Ҫ������һ��б�º����ĺ����time����
*/
void SpeedSolving4_SetWithTime(float HowTime,SpeedSolvingOut4_t *TargetPulse)
{
	//���ٶȽ�����Ľ�����ø��ٶȻ�����Ҫ�����ٶȻ������б��
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
	
	MyTemp1 = SPEED_SOLVING4_DATA_MOTOR_N * 2.0f * 3.1415926f * SPEED_SOLVING4_DATA_R / 60000.0f;//ÿ�������ת����mm��/60000ms=m/s
	//������ٶ�Ϊ1.69m/s������3s�ڴﵽ����ٶȲ���򻬣�����ٶ����Ϊ0.565m/s2��
	printf("����ٶ�%fm/s\r\n",MyTemp1);	
}


TargetSpeed_t SpeedSolving_GetTargetSpeed(void)
{
	return TargetSpeed;
}

