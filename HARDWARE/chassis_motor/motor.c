#include "motor.h"
#include "delay.h"
#include "key.h"
#include "usart1.h"


#define MOT_MaxSpeedAll	345.0f			//电机的最快转速，以最慢的电机为准

#define MOT_MinSpeedPulse_1		200.0f  //		//输出脉宽小于这个值时，速度即为MOT_MinSpeed_1
#define MOT_MaxSpeedPulse_1		2000.0f	//电机最快转速时的脉冲数，要和转的最慢的电机一致
#define MOT_MinSpeed_1			0.0f		//
#define MOT_MAXSpeed_1			MOT_MaxSpeedAll		//

#define MOT_MinSpeedPulse_2		200.0f	
#define MOT_MaxSpeedPulse_2		2000.0f
#define MOT_MinSpeed_2			0.0f	
#define MOT_MAXSpeed_2			MOT_MaxSpeedAll

#define MOT_MinSpeedPulse_3		200.0f	
#define MOT_MaxSpeedPulse_3		2000.0f
#define MOT_MinSpeed_3			0.0f	
#define MOT_MAXSpeed_3			MOT_MaxSpeedAll	

#define MOT_MinSpeedPulse_4		200.0f	
#define MOT_MaxSpeedPulse_4		2000.0f
#define MOT_MinSpeed_4			0.0f	
#define MOT_MAXSpeed_4			MOT_MaxSpeedAll	


MotorCurrent_T	MotorCurrent = {0,0,0,0};


//////////////////////////
//设置四个电机控制正反转的引脚中，驱动桥每个通道的IN1为高电平向前走还是IN2为高电平向前走
//输入形参：Wheel_1为0表示输出GND刹车，1表示设置IN1为高IN2为低，2表示设置IN2为高IN1为低，3表示IN1和IN2都为1，
//默认Wheel_1、2、3、4分别为和PE9/PE11/PE13/PE14相配合的
void MotorSetPolarity(int8_t Wheel_1,int8_t Wheel_2,int8_t Wheel_3,int8_t Wheel_4)
{
	switch(Wheel_1)
	{
		case 0:
		{
			GPIO_ResetBits(GPIOE,GPIO_Pin_9);//IN1
			GPIO_ResetBits(GPIOE,GPIO_Pin_10);//IN2
			break;
		}
		case 1:
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_9);//IN1
			GPIO_ResetBits(GPIOE,GPIO_Pin_10);//IN2
			break;
		}
		case 2://正转
		{
			GPIO_ResetBits(GPIOE,GPIO_Pin_9);//IN1
			GPIO_SetBits(GPIOE,GPIO_Pin_10);//IN2
			break;
		}
		case 3:
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_9);//IN1
			GPIO_SetBits(GPIOE,GPIO_Pin_10);//IN2
			break;
		}
	}
	
	switch(Wheel_2)
	{
		case 0:
		{
			GPIO_ResetBits(GPIOE,GPIO_Pin_11);//IN1
			GPIO_ResetBits(GPIOE,GPIO_Pin_12);//IN2
			break;
		}
		case 1://正转
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_11);//IN1
			GPIO_ResetBits(GPIOE,GPIO_Pin_12);//IN2
			break;
		}
		case 2:
		{
			GPIO_ResetBits(GPIOE,GPIO_Pin_11);//IN1
			GPIO_SetBits(GPIOE,GPIO_Pin_12);//IN2
			break;
		}
		case 3:
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_11);//IN1
			GPIO_SetBits(GPIOE,GPIO_Pin_12);//IN2
			break;
		}
	}

	switch(Wheel_3)
	{
		case 0:
		{
			GPIO_ResetBits(GPIOE,GPIO_Pin_13);//IN1
			GPIO_ResetBits(GPIOE,GPIO_Pin_14);//IN2
			break;
		}
		case 1:
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_13);//IN1
			GPIO_ResetBits(GPIOE,GPIO_Pin_14);//IN2
			break;
		}
		case 2://正转
		{
			GPIO_ResetBits(GPIOE,GPIO_Pin_13);//IN1
			GPIO_SetBits(GPIOE,GPIO_Pin_14);//IN2
			break;
		}
		case 3:
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_13);//IN1
			GPIO_SetBits(GPIOE,GPIO_Pin_14);//IN2
			break;
		}
	}

	switch(Wheel_4)
	{
		case 0:
		{
			GPIO_ResetBits(GPIOE,GPIO_Pin_15);//IN1
			GPIO_ResetBits(GPIOB,GPIO_Pin_10);//IN2
			break;
		}
		case 1://正转
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_15);//IN1
			GPIO_ResetBits(GPIOB,GPIO_Pin_10);//IN2
			break;
		}
		case 2:
		{
			GPIO_ResetBits(GPIOE,GPIO_Pin_15);//IN1
			GPIO_SetBits(GPIOB,GPIO_Pin_10);//IN2
			break;
		}
		case 3:
		{
			GPIO_SetBits(GPIOE,GPIO_Pin_15);//IN1
			GPIO_SetBits(GPIOB,GPIO_Pin_10);//IN2
			break;
		}
	}	
}



/////////////////////////////////////
//设置占空比，输入值0-2000
void Motor_SetCompareNum(int16_t w_ch1,int16_t w_ch2,int16_t w_ch3, int16_t w_ch4)
{
	if( w_ch1 >=2000 ) w_ch1 = 2000;
	if( w_ch2 >=2000 ) w_ch2 = 2000;
	if( w_ch3 >=2000 ) w_ch3 = 2000;
	if( w_ch4 >=2000 ) w_ch4 = 2000;

	if( w_ch1 < 0 ) w_ch1 = 0;
	if( w_ch2 < 0 ) w_ch2 = 0;
	if( w_ch3 < 0 ) w_ch3 = 0;
	if( w_ch4 < 0 ) w_ch4 = 0;

	TIM_SetCompare1(TIM8,w_ch1);
	TIM_SetCompare2(TIM8,w_ch2);
	TIM_SetCompare3(TIM8,w_ch3);
	TIM_SetCompare4(TIM8,w_ch4);		
}


//设置定时器脉宽最大为2000
void SetWheelPulse(int16_t Wheel1, int16_t Wheel2,int16_t Wheel3,int16_t Wheel4)
{
  int8_t Temp1 = 1, Temp2 = 1, Temp3 = 1, Temp4 = 1;
  
  MotorCurrent.Wheel1 = Wheel1;
  MotorCurrent.Wheel2 = Wheel2;
  MotorCurrent.Wheel3 = Wheel3;
  MotorCurrent.Wheel4 = Wheel4;
  
	if( Wheel1 < 0) 
	{
		Wheel1 = -Wheel1;
		Temp1 = 1;
	}
	else if(Wheel1 > 0)
	{
		Wheel1 = Wheel1;
		Temp1 = 2;	  
	}
	else 
	{
		Wheel1 = 0;
		Temp1 = 0;	  
	}

	if( Wheel2 < 0) 
	{
		Wheel2 = -Wheel2;
		Temp2 = 1;
	}
	else if(Wheel2 > 0)
	{
		Wheel2 = Wheel2;
		Temp2 = 2;	  
	}
	else 
	{
		Wheel2 = 0;
		Temp2 = 0;	  
	}
	
	if( Wheel3 < 0) 
	{
		Wheel3 = -Wheel3;
		Temp3 = 2;
	}
	else if(Wheel3 > 0)
	{
		Wheel3 = Wheel3;
		Temp3 = 1;	  
	}
	else 
	{
		Wheel3 = 0;
		Temp3 = 0;	  
	}
	
	if( Wheel4 < 0) 
	{
		Wheel4 = -Wheel4;
		Temp4 = 2;
	}
	else if(Wheel4 > 0)
	{
		Wheel4 = Wheel4;
		Temp4 = 1;	  
	}
	else 
	{
		Wheel4 = 0;
		Temp4 = 0;	  
	}

	MotorSetPolarity( Temp1, Temp2, Temp3, Temp4 );    //
	Motor_SetCompareNum(Wheel1,Wheel2,Wheel3,Wheel4);   //
}

void SetWheelCurrent(float Wheel1, float Wheel2,float Wheel3,float Wheel4)
{
	static float YingShe_1 = 1.0,YingShe_2 = 1.0,YingShe_3 = 1.0,YingShe_4 = 1.0;
	static int8_t OneExecute = 0;
	static int16_t SetPulse_1,SetPulse_2,SetPulse_3,SetPulse_4;

	
	if(OneExecute == 0)
	{
		OneExecute = 1;
		YingShe_1 = (MOT_MaxSpeedPulse_1 - MOT_MinSpeedPulse_1) / (MOT_MAXSpeed_1 - MOT_MinSpeed_1);
		YingShe_2 = (MOT_MaxSpeedPulse_2 - MOT_MinSpeedPulse_2) / (MOT_MAXSpeed_2 - MOT_MinSpeed_2);
		YingShe_3 = (MOT_MaxSpeedPulse_3 - MOT_MinSpeedPulse_3) / (MOT_MAXSpeed_3 - MOT_MinSpeed_3);
		YingShe_4 = (MOT_MaxSpeedPulse_4 - MOT_MinSpeedPulse_4) / (MOT_MAXSpeed_4 - MOT_MinSpeed_4);
	}
	
	if(Wheel1 > MOT_MAXSpeed_1)
		Wheel1 = MOT_MAXSpeed_1;
	if(Wheel1 < -MOT_MAXSpeed_1)
		Wheel1 = -MOT_MAXSpeed_1;
	
	if(Wheel2 > MOT_MAXSpeed_2)
		Wheel2 = MOT_MAXSpeed_2;
	if(Wheel2 < -MOT_MAXSpeed_2)
		Wheel2 = -MOT_MAXSpeed_2;
	
	if(Wheel3 > MOT_MAXSpeed_3)
		Wheel3 = MOT_MAXSpeed_3;
	if(Wheel3 < -MOT_MAXSpeed_3)
		Wheel3 = -MOT_MAXSpeed_3;
	
	if(Wheel4 > MOT_MAXSpeed_4)
		Wheel4 = MOT_MAXSpeed_4;
	if(Wheel4 < -MOT_MAXSpeed_4)
		Wheel4 = -MOT_MAXSpeed_4;
	
	if(Wheel1 > 0)
	{
		SetPulse_1 = (Wheel1 - MOT_MinSpeed_1) * YingShe_1 + MOT_MinSpeedPulse_1;
	}
	else if(Wheel1 < 0)
	{
		SetPulse_1 = (Wheel1 + MOT_MinSpeed_1) * YingShe_1 - MOT_MinSpeedPulse_1;
	}
	else
	{
		SetPulse_1 = 0;
	}
	
	if(Wheel2 > 0)
	{
		SetPulse_2 = (Wheel2 - MOT_MinSpeed_2) * YingShe_2 + MOT_MinSpeedPulse_2;
	}
	else if(Wheel2 < 0)
	{
		SetPulse_2 = (Wheel2 + MOT_MinSpeed_2) * YingShe_2 - MOT_MinSpeedPulse_2;
	}
	else
	{
		SetPulse_2 = 0;
	}
	
	if(Wheel3 > 0)
	{
		SetPulse_3 = (Wheel3 - MOT_MinSpeed_3) * YingShe_3 + MOT_MinSpeedPulse_3;
	}
	else if(Wheel3 < 0)
	{
		SetPulse_3 = (Wheel3 + MOT_MinSpeed_3) * YingShe_3 - MOT_MinSpeedPulse_3;
	}
	else
	{
		SetPulse_3 = 0;
	}
	
	if(Wheel4 > 0)
	{
		SetPulse_4 = (Wheel4 - MOT_MinSpeed_4) * YingShe_4 + MOT_MinSpeedPulse_4;
	}
	else if(Wheel4 < 0)
	{
		SetPulse_4 = (Wheel4 + MOT_MinSpeed_4) * YingShe_4 - MOT_MinSpeedPulse_4;
	}
	else
	{
		SetPulse_4 = 0;
	}
	
//	printf("V:%d,%d,%d,%d\n",SetPulse_1,SetPulse_2,SetPulse_3,SetPulse_4);
	
	SetWheelPulse(SetPulse_1,SetPulse_2,SetPulse_3,SetPulse_4);
	
}
//输入的是电机转速，最大值345



/* 电机说明:最佳频率10KHz,PWM为0停转,越大转速越快 */
/* 电机pwm控制  */
void MotorPWM_Init(void)//TIM8 CH1、CH2、CH3、CH4
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);// 
 	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);  //使能GPIO外设时钟使能
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; //GPIOC6.7.8.9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;  			//复用输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽复用
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//速度100MHZ
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);					//初始化
	
	TIM_TimeBaseStructure.TIM_Prescaler=21-1;
//	//定时器1在APB2上，频率为168MHZ，定时器时钟==168MHZ/TIM_Prescaler+1，则在此配置下，计数周期为1/8000000秒，即0.125微秒
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=2000 - 1;   //自动重装载值，一次计数0.125微秒，则PWM周期为250us，频率为4KHZ，最小脉宽0.125微秒
//	//注意，驱动桥上说最小脉宽小于10微秒时驱动桥可能没输出，10us在此配置下为80个脉冲，可软件设置当占空比小于80时设置为0，也可以不用管
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //设置时钟分割，DIV1=0不延时。
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	//在空闲时输出，解析见：https://blog.csdn.net/gtkknd/article/details/52188266
	TIM_OCInitStructure.TIM_Pulse=0;//即初始化时的占空比，设置为0%，
	
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能		
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH4预装载使能	 
	
	TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIMx在ARR上的预装载寄存器
	
	TIM_Cmd(TIM8, ENABLE);  //使能TIM
	TIM_CtrlPWMOutputs(TIM8,ENABLE);	//MOE 主输出使能
	//并且要加在TIM_Cmd(TIM1, ENABLE);后面
	//见https://blog.csdn.net/oshan2012/article/details/79504995	
}



/* 电机状态控制 */
void MotorState_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	/////////////////////////////////////////////////////
	/* motor1(E9.10) - 低电平停止 */
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStruct);	
	
	/* motor2(E11.12) - 低电平停止 */
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_11|GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	/////////////////////////////////////////////////////
	
	/* motor3(E13.14) - 低电平停止 */
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	/* motor4(E15.B10) - 低电平停止 */
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	MOTOR1_OFF;
	MOTOR2_OFF;
	MOTOR3_OFF;
	MOTOR4_OFF;
	
}




