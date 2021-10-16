#include "sys.h"
#include "BSP_Init.h"
#include "led.h"
#include "key.h"
#include "motor.h"
#include "SpeedControl.h"
#include "encoder.h"
#include "timer.h"
#include "usart1.h"
#include "gyroscope.h"
#include "CarRun.h"
#include "tracking.h"
#include "yaw.h"
#include "oled.h"
#include "gm65.h"

/*
PA0	TIM5�ı���������
PA1	TIM5�ı���������
PA2	 UART2_TX
PA3	 UART2_RX
PA4	
PA5	��ѭ������2��LED7
PA6	 LED2
PA7	 LED3
PA8	
PA9	  USART1_TX��������
PA10	USART1_RX��������
PA11	
PA12	
PA13	JTAG�����е�TMS��������10K�ӵ�
PA14	JTAG�����е�TCK����ֱ�ӽӵ�
PA15	TIM2�ı���������
PB0	TIM3�ı���������
PB1	TIM3�ı���������
PB2	BOOT1,�Ѿ�����100K�ӵ�
PB3	TIM2�ı���������
PB4	
PB5	
PB6	TIM4�ı���������
PB7	TIM4�ı���������
PB8	
PB9	
PB10	���4����ת����
PB11	��ѭ������4��LED5
PB12	��ѭ������4��LED6
PB13	��ѭ������4��LED7
PB14	���PWm
PB15	���PWM
PC0	��ѭ������2��LED3
PC1	��ѭ������2��LED4
PC2	��ѭ������2��LED5
PC3	��ѭ������2��LED6
PC4	��ѭ������4��LED1
PC5	��ѭ������4��LED2
PC6	��ʱ��8��TIM8PWM��
PC7	��ʱ��8��TIM9PWM��
PC8	��ʱ��8��TIM10PWM��
PC9	��ʱ��8��TIM11PWM��
PC10	UART4_TX   ɨ����
PC11	UART4_RX   ɨ����
PC12	UART5_TX
PC13	��ѭ������2��LED2
PC14	
PC15	
PD0	 
PD1	  ǰ��ѭ������1��LED_1
PD2	  ǰ��ѭ������1��LED_2
PD3	  ǰ��ѭ������1��LED_3
PD4	  ǰ��ѭ������1��LED_4
PD5	  ǰ��ѭ������1��LED_5
PD6	  ǰ��ѭ������1��LED_6
PD7	  ǰ��ѭ������1��LED_7
PD8	  �ҷ�ѭ������3��LED1
PD9	  �ҷ�ѭ������3��LED2
PD10	�ҷ�ѭ������3��LED3
PD11	�ҷ�ѭ������3��LED4
PD12	�ҷ�ѭ������3��LED5
PD13	�ҷ�ѭ������3��LED6
PD14	�ҷ�ѭ������3��LED7
PD15	
PE0	
PE1
PE2	 ��ѭ������3��LED1
PE3  ����0������ΪGND
PE4	 ����1������ΪGND
PE5	 ���PWM
PE6	 ���PWM
PE7		��ѭ������4��LED3
PE8	  ��ѭ������4��LED4
PE9	  ���1����ת����
PE10	���1����ת����
PE11	���2����ת����
PE12	���2����ת����
PE13	���3����ת����
PE14  ���3����ת����
PE15	���4����ת����

*/


int main(void)
{ 

	BspInit();                      //�����ʼ��
	printf("hello world\n");
	
	KeyDelay();      //��������
//	QRC_GetData();
//	TCK_Debug();
//	RunPointSimple();
//	RunPoint_Test();  //�ٶ�Ϊ30ʱ�ܺ�
//	YAW_Test();

	
//	YAW_RunWithSpeed(50,50);

//	SimpleSpeedSolving_4(0,0,60);
//	delay_ms(2000);
//	SimpleSpeedSolving_4(0,0,0);
//	SpeedSolving_4(0.5,0.5,0);
//	SpeedSolving4_Set();
//	delay_ms(2000);
//	SpeedSolving_4(0,0.5,0);
//	SpeedSolving4_Set();
//	WRunPIDDebug();
//	PID_DebugPID();
//	SpeedC_SetTarget(150,150,0,0);
//	SetWheelPulse(1000,1000,0,0);
	
//	SpeedSolving_4(0.5,0,0);//����200����5ms��ת���ĽǶ�
//	SpeedSolving4_Set();
//	SpeedSolving_Report(0);

	
}
