#ifndef __LED_H
#define __LED_H
#include "sys.h"


//LED�˿ڶ��壬λ������
	 
#define LED2 PAout(6)	 
#define LED3 PAout(7)	  


//��������
void LED_Init(void);//��ʼ��	
void LED_Toggle(void);//LED����˸


#endif
