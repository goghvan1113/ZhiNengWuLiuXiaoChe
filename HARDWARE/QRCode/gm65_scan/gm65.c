#include "gm65.h"
#include "usart1.h"
#include "string.h"
#include "led.h"
#include "oled.h"


u8 i;
u8 flag=0; //��ȡ��ɱ�־

u8 uart4_readdata_send[]={0x7E,0x00,0x07,0x01,0x00,0x0A,0x01,0xAB,0xCD};   //����־λ����ָ��
u8 uart4_start_send[]={0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};   //�����ɨ��ָ��
u8 uart4_get[14];
u8 DATA_ASCII;

//����4  gm65
void QRCode_Uart4_Init(u32 bound)
{
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	//ʹ��USART4��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	//����4��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10����ΪUAR4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11����ΪUART4
  
	//UART4�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10��GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC10��PC11

  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  //Uart4 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
  //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(UART4, &USART_InitStructure); //��ʼ������4
  
  USART_Cmd(UART4, DISABLE);             //����2������ɨ��ָ�������  
}


void send_data2gm65(u8 *data)
{
		for(i=0;i<9;i++)
		{
			USART_SendData(UART4, data[i]);
			while(USART_GetFlagStatus(UART4,USART_FLAG_TC)!=SET);  //����Ƿ������
			memset(uart4_get,0,sizeof(uart4_get));		
		}
		
		if(i==9) i=0;
}

void UART4_IRQHandler(void)   //����4�жϷ������
{	
	if(USART_GetITStatus(UART4,USART_IT_RXNE) != RESET) 
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE); 
		DATA_ASCII = USART_ReceiveData(UART4);	
		uart4_get[i++]=DATA_ASCII;     //ǰ7���ֽ�Ϊ����ָ��ͺ󷵻صĻ�Ӧ��Ϣ����Ч
		if(i==14)          //���1���ֽ�Ϊ���з�����Ч
		{
			flag=1;          //��ȡ���
			USART_Cmd(UART4, DISABLE);    //�رմ���4 
		}
	}		
} 

void QRC_GetData(void)
{
	USART_Cmd(UART4, ENABLE);             //ʹ�ܴ���4
	send_data2gm65(uart4_readdata_send);  //���Ͷ�ά����Ϣ��������
	send_data2gm65(uart4_start_send);    //���ʹ�������ָ��

	if(flag==1)     //��ʾ������
	{	
		OLED_ShowString(20,18,uart4_get+7,24);
		OLED_Refresh();
		flag=0;
	}
	
}
