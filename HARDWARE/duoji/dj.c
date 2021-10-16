#include "dj.h"


_DJ_POSITION P0={450 ,2000,700 ,1500};  /* 初始姿态 */
_DJ_POSITION P1={930 ,1400,700 ,1200};  /*  识别区  */
/*--------------------------------------------------*/
_DJ_POSITION P2={930 ,2000,800 ,1200};  /* 提取区1  */
_DJ_POSITION P3={930 ,1100,1150,1200};	/*   夹取   */
_DJ_POSITION P4={930 ,1100,1150,1700};  /*   夹住   */
_DJ_POSITION P5={930 ,1800,1000,1700};  /*   缩回   */

_DJ_POSITION P6={2310,1100,1100,1700};  /* 扫点姿态 */
_DJ_POSITION P7={2395,900,1550,1700};   /*   放置   */
_DJ_POSITION P8={2395,900,1550,1200};   /*   松开   */
_DJ_POSITION P9={2395,1600,1500,1200};	/*   缩回   */
/*--------------------------------------------------*/
_DJ_POSITION P10={2350,1000,1700,1200}; /* 提取区2  */
_DJ_POSITION P11={2350,1000,1700,1200}; /*   夹取   */
_DJ_POSITION P12={2350,1000,1700,1700}; /*   夹住   */
_DJ_POSITION P13={2350,1800,1000,1700}; /*   缩回   */

_DJ_POSITION P14={1620,1100,1100,1700}; /* 扫点姿态 */
_DJ_POSITION P15={1730,900,1560,1700};  /*   放置   */
_DJ_POSITION P16={1730,900,1560,1200};  /*   松开   */
_DJ_POSITION P17={1730,1600,1500,1200};  /*   缩回   */
_DJ_POSITION P18={2350,1600,1500,1200};  /*   旋转   */
/*--------------------------------------------------*/
_DJ_POSITION P19={1730,2200,700 ,1200}; /*   完成   */


/* 50Hz */
/* TIM1-1.2.3.4  PA8.9.10.11  */
void dj_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); /* 168000000 */	/* PA8.9.10.11 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* TIM1-CH2,4 */
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8 , GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9 , GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);

	TIM_TimeBaseInitStruct.TIM_Period = TIM1_ARR; 
	TIM_TimeBaseInitStruct.TIM_Prescaler = TIM1_PRE;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
	TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;

	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	TIM_OC3Init(TIM1, &TIM_OCInitStruct);
	TIM_OC4Init(TIM1, &TIM_OCInitStruct);

	TIM_Cmd(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	dj_set(&P1);
}




/* 机械臂姿态设定 */
u8 dj_set(_DJ_POSITION *D)
{
	DJ1_PLUSE = D->D1;
	DJ2_PLUSE = D->D2;
	DJ3_PLUSE = D->D3;
	DJ4_PLUSE = D->D4;
	
	return 1 ;
}


/*end of dj.c*/


