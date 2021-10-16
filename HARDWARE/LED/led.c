#include "led.h" 



//LED��ӦIO��ʼ��
void LED_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOGʱ��

  //PG13��PG14��PG15��ʼ������
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;         //LED0��LED1��LED2��ӦIO��
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                  //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;                 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;             //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                   //����
  GPIO_Init(GPIOA, &GPIO_InitStructure);                         //��ʼ��GPIO

  GPIO_SetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);  //GPIOG13,G14,G15���øߣ�����

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




