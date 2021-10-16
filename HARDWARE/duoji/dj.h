/**
  *****************************************************************************
  * @file			dj.h
  * @author			WWJ
  * @version		v1.0
  * @date			2019/5/15
	* @environment	stm32f407
  * @brief 
  * @copyright		HUNAU  
  *****************************************************************************
**/





#ifndef __DJ_H
#define __DJ_H
#include "sys.h"




#define TIM1_ARR	20000
#define TIM1_PRE	168

#define TIM4_ARR	20000
#define TIM4_PRE	84

#define DJ4_PLUSE	TIM1->CCR1
#define DJ3_PLUSE	TIM1->CCR3
#define DJ2_PLUSE	TIM1->CCR2
#define DJ1_PLUSE	TIM1->CCR4





typedef struct
{
	uint16_t D1;
	uint16_t D2;
	uint16_t D3;
	uint16_t D4;
	
}_DJ_POSITION;




void dj_Init(void);
uint8_t dj_set(_DJ_POSITION *D);





#endif 




/*end of dj.h*/

