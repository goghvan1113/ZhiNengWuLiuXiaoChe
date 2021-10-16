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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);		      //初始化延时函数
	Usart1PrintInit();        //串口1打印初始化
	TIM6_Init();                //PID速度环位置环运行,串口DMA打印初始化
	delay_ms(5);
	printf("USART OK\r\n");
	LED_Init();		            //初始化LED端口
	KEY_Init();               //按键初始化
	WT931_Init();             //WT931陀螺仪初始化
	QRCode_Uart4_Init(9600);  //二维码扫码串口初始化
	OLED_Init();				      //初始化OLED 

	MotorState_Init();        //初始化电机正反转端口
	MotorPWM_Init();          //电机转速PWM初始化
	Encoder_InitAll();        //电机编码器初始化
	TCK_Init();                 //寻迹初始化
	SpeedC_Init(PositionPID_e);  //速度环初始化           
	PIDInit();                //位置环初始化

}


