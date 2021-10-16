#include "encoder.h"
#include "led.h"
#include "motor.h"
#include "usart1.h"
#include "SpeedControl.h"
#include "CarRun.h"


//������ٱ�Ϊ30����������������13��AB˫����ϵõ�4��Ƶ����ת1Ȧ����������Ϊ30*13*4=1560��
//���ת��=Encoder*100/1560 r/s   10ms��ȡһ�α���������ֵ



/*TIM2��ʼ��Ϊ�������ӿ�*/
void Encoder_Init_TIM2(void)//TIM2 CH1��CH2
{
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //ʹ�ܶ�ʱ��2��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PA�˿�ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PB�˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;			//GPIOA15
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//
	GPIO_Init(GPIOA, &GPIO_InitStructure);				//�����趨������ʼ��GPIOA
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;			//GPIOB3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//
	GPIO_Init(GPIOB, &GPIO_InitStructure);				//�����趨������ʼ��GPIOA
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2); 	//GPIOA15���ó�TIM2_CH1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2); 	//GPIOB3 ���ó�TIM2_CH2
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);		//
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 			// Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2, ENABLE); 
}

/*
PB0��1���ӱ�����ʱ��ȡ�������ݣ�ֻ�������ӵ�LED��PA6��7
*/
/*TIM3��ʼ��Ϊ�������ӿ�*/
//void Encoder_Init_TIM3(void)//TIM3 CH3��CH4 
//{
//TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//TIM_ICInitTypeDef TIM_ICInitStructure;  
//GPIO_InitTypeDef GPIO_InitStructure;

//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//ʹ�ܶ�ʱ��3��ʱ��
//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PA�˿�ʱ��	

//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;//GPIOB0��1
//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		//
//GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//
//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//
//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//
//GPIO_Init(GPIOB,&GPIO_InitStructure);               //��ʼ��

//GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3); 	//GPIOB0 ���ó�TIM3_CH3
//GPIO_PinAFConfig(GPIOB,GPIO_PinSource1,GPIO_AF_TIM3); 	//GPIOB1 ���ó�TIM3_CH4

//TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
//TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ65535
//TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
//TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
//TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

//TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
//TIM_ICStructInit(&TIM_ICInitStructure);
//TIM_ICInitStructure.TIM_ICFilter = 0;
//TIM_ICInit(TIM3, &TIM_ICInitStructure);

//TIM_ClearFlag(TIM3, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
//TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

//TIM_SetCounter(TIM3,0);
//TIM_Cmd(TIM3, ENABLE); 
//}

void Encoder_Init_TIM3(void)//TIM3 CH1��CH2 
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//ʹ�ܶ�ʱ��3��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PA�˿�ʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;//GPIOB0��1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//
	GPIO_Init(GPIOA,&GPIO_InitStructure);               //��ʼ��
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); 	//GPIOB0 ���ó�TIM3_CH3
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3); 	//GPIOB1 ���ó�TIM3_CH4
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ65535
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3, ENABLE); 
}


/*TIM4��ʼ��Ϊ�������ӿ�*/
void Encoder_Init_TIM4(void)//TIM4 CH1��CH2
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��4��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PB�˿�ʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;//GPIOB6.7
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//
	GPIO_Init(GPIOB,&GPIO_InitStructure);               //��ʼ��
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4); 	//GPIOB6.7 ���ó�TIM4_CH1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4); 	//GPIOB6.7 ���ó�TIM4_CH2
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ65535
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM4,0);
	TIM_Cmd(TIM4, ENABLE); 
}

/*TIM5��ʼ��Ϊ�������ӿ�*/
void Encoder_Init_TIM5(void)//TIM5 CH1 CH2
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;  
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);//ʹ�ܶ�ʱ��5��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PA�˿�ʱ��	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;//GPIOA.0.1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;		//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//
	GPIO_Init(GPIOA,&GPIO_InitStructure);               //��ʼ��
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5); 	//GPIOA0 ���ó�TIM5_CH1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); 	//GPIOA1 ���ó�TIM5_CH2
	
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ65535
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//ѡ��ʱ�ӷ�Ƶ������Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	
	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM5, &TIM_ICInitStructure);
	
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
	
	TIM_SetCounter(TIM5,0);
	TIM_Cmd(TIM5, ENABLE); 
}


void Encoder_InitAll(void)
{
	Encoder_Init_TIM2();            //=====��ʼ��������A
	Encoder_Init_TIM3();            //=====��ʼ��������B
	Encoder_Init_TIM4();            //=====��ʼ��������C
	Encoder_Init_TIM5();            //=====��ʼ��������D
	
}


/**************************************************************************
�������ܣ���λʱ���ȡ�����������������SpeedNow��

**************************************************************************/

uint8_t UpdatePIDReality = 0; //�ٶȻ������Ƿ���£�λ�û��������Ƿ����
void ReadEncoderLoop(void) //������Target�ӽ�30
{
	int16_t Speed1 = 0;
	int16_t Speed2 = 0;
	int16_t Speed3 = 0;
	int16_t Speed4 = 0;
	
	
	Speed1 = (short)TIM2 -> CNT;
		TIM2 -> CNT = 0;
	Speed2 = (short)TIM3 -> CNT;
		TIM3 -> CNT = 0;
	Speed3 = (short)TIM4 -> CNT;
		TIM4 -> CNT = 0;
	Speed4 = (short)TIM5 -> CNT;
		TIM5 -> CNT = 0;

//	printf("V:%.2f,%.2f,%.2f,%.2f",Speed1*1000.0f/1560,Speed2*1000.0f/1560,Speed3*1000.0f/1560,Speed4*1000.0f/1560);
	printf("V:%d,%d,%d,%d\n",-Speed1,Speed2,-Speed3,-Speed4);
	
	SpeedC_RealitySpeedUp(-Speed1, Speed2, -Speed3, -Speed4);
	WRun_Update(-Speed1, Speed2, -Speed3, -Speed4);
	UpdatePIDReality = 1;
}


/**************************************************************************
�������ܣ���ʱ���жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//����жϱ�־λ 	    
}

void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//����жϱ�־λ 	    
}

void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//����жϱ�־λ 	    
}

void TIM5_IRQHandler(void)
{ 		    		  			    
	if(TIM5->SR&0X0001)//����ж�
	{    				   				     	    	
	}				   
	TIM5->SR&=~(1<<0);//����жϱ�־λ 	    
}




