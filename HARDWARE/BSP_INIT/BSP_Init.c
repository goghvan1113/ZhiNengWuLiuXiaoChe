#include "BSP_Init.h"
#include "delay.h"
#include "usart1.h"
#include "led.h"
#include "motor.h"
#include "encoder.h"
#include "key.h"
#include "SpeedControl.h"
#include "timer.h"
#include "gyroscope.h"
#include "tracking.h"
#include "CarRun.h"
#include "gm65.h"
#include "oled.h"

void BspInit(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);		      //��ʼ����ʱ����
	Usart1PrintInit();        //����1��ӡ��ʼ��
	TIM6_Init();                //PID�ٶȻ�λ�û�����,����DMA��ӡ��ʼ��
	delay_ms(5);
	printf("USART OK\r\n");
	LED_Init();		            //��ʼ��LED�˿�
	KEY_Init();               //������ʼ��
	WT931_Init();             //WT931�����ǳ�ʼ��
	QRCode_Uart4_Init(9600);  //��ά��ɨ�봮�ڳ�ʼ��
	OLED_Init();				      //��ʼ��OLED 

	MotorState_Init();        //��ʼ���������ת�˿�
	MotorPWM_Init();          //���ת��PWM��ʼ��
	Encoder_InitAll();        //�����������ʼ��
	TCK_Init();                 //Ѱ����ʼ��
	SpeedC_Init(PositionPID_e);  //�ٶȻ���ʼ��           
	PIDInit();                //λ�û���ʼ��

}


