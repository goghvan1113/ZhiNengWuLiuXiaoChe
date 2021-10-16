#ifndef __LED_H
#define __LED_H
#include "sys.h"


//LED端口定义，位带操作
	 
#define LED2 PAout(6)	 
#define LED3 PAout(7)	  


//函数声明
void LED_Init(void);//初始化	
void LED_Toggle(void);//LED灯闪烁


#endif
