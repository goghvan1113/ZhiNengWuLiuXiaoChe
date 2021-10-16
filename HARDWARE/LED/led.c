#include "led.h" 



//LED对应IO初始化
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOG时钟

  //PG13、PG14和PG15初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         //LED0、LED1和LED2对应IO口
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                   //上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);                         //初始化GPIO

  GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);  //GPIOG13,G14,G15设置高，灯灭

}


void LED_Toggle(void)
{
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6))
		GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	else
		GPIO_SetBits(GPIOA,GPIO_Pin_6);
	
	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7))
		GPIO_ResetBits(GPIOA,GPIO_Pin_7);
	else
		GPIO_SetBits(GPIOA,GPIO_Pin_7);
	
}




