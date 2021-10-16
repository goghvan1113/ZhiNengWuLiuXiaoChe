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
PA0	TIM5的编码器功能
PA1	TIM5的编码器功能
PA2	 UART2_TX
PA3	 UART2_RX
PA4	
PA5	左方循迹灯条2，LED7
PA6	 LED2
PA7	 LED3
PA8	
PA9	  USART1_TX，调试用
PA10	USART1_RX，调试用
PA11	
PA12	
PA13	JTAG调试中的TMS，已下拉10K接地
PA14	JTAG调试中的TCK，已直接接地
PA15	TIM2的编码器功能
PB0	TIM3的编码器功能
PB1	TIM3的编码器功能
PB2	BOOT1,已经下拉100K接地
PB3	TIM2的编码器功能
PB4	
PB5	
PB6	TIM4的编码器功能
PB7	TIM4的编码器功能
PB8	
PB9	
PB10	电机4正反转引脚
PB11	后方循迹灯条4，LED5
PB12	后方循迹灯条4，LED6
PB13	后方循迹灯条4，LED7
PB14	舵机PWm
PB15	舵机PWM
PC0	左方循迹灯条2，LED3
PC1	左方循迹灯条2，LED4
PC2	左方循迹灯条2，LED5
PC3	左方循迹灯条2，LED6
PC4	后方循迹灯条4，LED1
PC5	后方循迹灯条4，LED2
PC6	定时器8，TIM8PWM波
PC7	定时器8，TIM9PWM波
PC8	定时器8，TIM10PWM波
PC9	定时器8，TIM11PWM波
PC10	UART4_TX   扫码器
PC11	UART4_RX   扫码器
PC12	UART5_TX
PC13	左方循迹灯条2，LED2
PC14	
PC15	
PD0	 
PD1	  前方循迹灯条1，LED_1
PD2	  前方循迹灯条1，LED_2
PD3	  前方循迹灯条1，LED_3
PD4	  前方循迹灯条1，LED_4
PD5	  前方循迹灯条1，LED_5
PD6	  前方循迹灯条1，LED_6
PD7	  前方循迹灯条1，LED_7
PD8	  右方循迹灯条3，LED1
PD9	  右方循迹灯条3，LED2
PD10	右方循迹灯条3，LED3
PD11	右方循迹灯条3，LED4
PD12	右方循迹灯条3，LED5
PD13	右方循迹灯条3，LED6
PD14	右方循迹灯条3，LED7
PD15	
PE0	
PE1
PE2	 左方循迹灯条3，LED1
PE3  按键0，按下为GND
PE4	 按键1，按下为GND
PE5	 舵机PWM
PE6	 舵机PWM
PE7		后方循迹灯条4，LED3
PE8	  后方循迹灯条4，LED4
PE9	  电机1正反转引脚
PE10	电机1正反转引脚
PE11	电机2正反转引脚
PE12	电机2正反转引脚
PE13	电机3正反转引脚
PE14  电机3正反转引脚
PE15	电机4正反转引脚

*/


int main(void)
{ 

	BspInit();                      //外设初始化
	printf("hello world\n");
	
	KeyDelay();      //按键启动
//	QRC_GetData();
//	TCK_Debug();
//	RunPointSimple();
//	RunPoint_Test();  //速度为30时很好
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
	
//	SpeedSolving_4(0.5,0,0);//除以200是在5ms内转过的角度
//	SpeedSolving4_Set();
//	SpeedSolving_Report(0);

	
}
