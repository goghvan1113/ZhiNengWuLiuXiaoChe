#include "gm65.h"
#include "usart1.h"
#include "string.h"
#include "led.h"
#include "oled.h"


u8 i;
u8 flag=0; //读取完成标志

u8 uart4_readdata_send[]={0x7E,0x00,0x07,0x01,0x00,0x0A,0x01,0xAB,0xCD};   //读标志位请求指令
u8 uart4_start_send[]={0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};   //命令触发扫描指令
u8 uart4_get[14];
u8 DATA_ASCII;

//串口4  gm65
void QRCode_Uart4_Init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	//使能USART4，GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	//串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10复用为UAR4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11复用为UART4
  
	//UART4端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC10与GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC10，PC11

  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启串口接受中断
  //Uart4 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
  //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(UART4, &USART_InitStructure); //初始化串口4
  
  USART_Cmd(UART4, DISABLE);             //串口2待定，扫码指令发出后开启  
}


void send_data2gm65(u8 *data)
{
		for(i=0;i<9;i++)
		{
			USART_SendData(UART4, data[i]);
			while(USART_GetFlagStatus(UART4,USART_FLAG_TC)!=SET);  //检测是否发送完毕
			memset(uart4_get,0,sizeof(uart4_get));		
		}
		
		if(i==9) i=0;
}

void UART4_IRQHandler(void)   //串口4中断服务程序
{	
	if(USART_GetITStatus(UART4,USART_IT_RXNE) != RESET) 
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE); 
		DATA_ASCII = USART_ReceiveData(UART4);	
		uart4_get[i++]=DATA_ASCII;     //前7个字节为触发指令发送后返回的回应信息，无效
		if(i==14)          //最后1个字节为换行符，无效
		{
			flag=1;          //读取完成
			USART_Cmd(UART4, DISABLE);    //关闭串口4 
		}
	}		
} 

void QRC_GetData(void)
{
	USART_Cmd(UART4, ENABLE);             //使能串口4
	send_data2gm65(uart4_readdata_send);  //发送二维码信息接收请求
	send_data2gm65(uart4_start_send);    //发送触发开启指令

	if(flag==1)     //显示任务码
	{	
		OLED_ShowString(20,18,uart4_get+7,24);
		OLED_Refresh();
		flag=0;
	}
	
}
