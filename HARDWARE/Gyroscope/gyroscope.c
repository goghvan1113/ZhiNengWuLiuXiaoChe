/************************************************************
* ���������ݼ�¼��
* �����Ƿ�������Ϊ11���ֽڣ���ͷ��0x55��β���к�У�飬���������RX_BUFFΪ11���ֽڡ�ÿ���յ�11���ֽڲ���У��ͨ����
* Ȼ��ִ�н�������������ݽ������Ӧ�洢ȡ
* Ĭ�����ý���Ϊ���õ�����������stm32f407��USART2��PA2��USART2_TX)��PA3��USART2_RX)�ڣ�
* USART2_RX�õ���DMA1��������5��ͨ��4
* 
************************************************************/

#include "gyroscope.h"
#include "usart1.h"
#include "stm32f4xx_dma.h"
#include "data_fifo.h"





#define GyroscopeBuffLength 20
static uint8_t GyroscopeBuff[GyroscopeBuffLength] = {0};


static int8_t MpuData[11] = {0};    //ÿ��֡�а����˸�����λ����һλΪ0x55����ʼλ���ڶ�λ0x53����ʾ�Ƕ�֡�����һλУ���
static uint8_t DMA_HadOpen = 0;    //DMAû��ʱΪ0��׼����ʱΪ1���򿪺�Ϊ2
static int8_t MpuCharNum = 0;  //��¼���յ��˼�λ����
static int8_t MpuYawWhichNew = 1; //��¼��һ�������µ���ֵ,0��û���µ���ֵ
static double MpuYaw1 = 0.0;
static double MpuYaw2 = 0.0;

static uint16_t MPU_UpdataRata = 0;//����ʱ��ӡ�����Ǹ�������
static uint8_t MPU_YawUpdated = 0;//��YAW���ݸ��º���λ1���������0
static uint8_t MPU_ErrState = 0;//0��û���µ�״̬��1�ǿ����յ����������ݵ�״̬��˵������û���⣬2�ǿ�����ȷ�յ����������ݣ�
//˵�������ǲ��ý�������

fifo_s_t Fifo_GyroscopeBuff;
fifo_s_t* PFifo_GyroscopeBuff = &Fifo_GyroscopeBuff;




void TestFIFO(void)
{
	uint8_t Temp = 0;
	uint8_t Temp1[11] = "1234567890";
	uint8_t Temp2[11] = "ABCDEFGHIJ";
	uint8_t Temp3[11];
	uint8_t Temp4 = 0;
	
	fifo_s_init(PFifo_GyroscopeBuff,GyroscopeBuff,GyroscopeBuffLength);
	

	++ Temp;
	fifo_s_puts(PFifo_GyroscopeBuff,Temp1,10);
	fifo_s_puts(PFifo_GyroscopeBuff,Temp2,10);
	delay_ms(100);
	printf("FIFO had:%d,%d\n\r",fifo_used_count(PFifo_GyroscopeBuff),fifo_free_count(PFifo_GyroscopeBuff) );
	
	Temp4 = fifo_s_gets(PFifo_GyroscopeBuff,Temp3,10);		
	Temp3[Temp4] = '\0';
	printf("%s\n\r",Temp3);
	Temp4 = fifo_s_gets(PFifo_GyroscopeBuff,Temp3,10);		
	Temp3[Temp4] = '\0';
	printf("%s\n\r",Temp3);
	
	while(1)
		delay_ms(100);
		

}


float ReadMpuYaw(void)
{
  MPU_YawUpdated = 0;
  	
  if(MpuYawWhichNew == 1)
    return MpuYaw1;
  
  if(MpuYawWhichNew == 2)
    return MpuYaw2;
  
  return 360;
}

uint8_t MPU_IsUpdated(void)
{
	if(MPU_YawUpdated != 0)
		return 1;
	else if(MPU_YawUpdated == 0)
		return 0;
	
	return 0;
}

void MPU_TestMPURate(void)
{
	while(1)
	{
		printf("F:%d\n\r",MPU_UpdataRata);
		MPU_UpdataRata = 0;
		delay_ms(1000);
	}
}

void MPU_PrintMPUYaw(void)
{
	//SetPIDTargetSpeed(-300,300,300,0);
	while(1)
	{
		if(MPU_YawUpdated == 1)
		{
			printf("X:%.5f\n\r",MPU_GetStartYaw());
			printf("Y:%.5f\n\r",ReadMpuYaw());
			delay_ms(1000);
		}
	}
}

static float MPU_StartYaw = 0;
//������ʼʱ�ĽǶȣ�����50�Σ�ð�������ȡ�м�30λ�Ƕȵ�ƽ��ֵ
void MPU_SetStartYaw(void)
{
	float PaiXu[50] = {0};
	uint8_t Count = 0;
	uint8_t Temp1 = 0;
	uint8_t Temp2 = 0;
	
	while(1)
	{
		if(MPU_YawUpdated == 1)
		{
			//printf("Y:%.5f\n\r",ReadMpuYaw());
			PaiXu[Count] = ReadMpuYaw();
			++ Count;			
		}
		
		if(Count >= 50)
			break;
	}
	
	//ð������
	for(Temp1 = 0; Temp1 < 50; ++ Temp1)
	{
		for(Temp2 = Temp1 + 1;Temp2 < 50 - Temp1;++ Temp2)
		{
			if(PaiXu[Temp1] < PaiXu[Temp2])
			{
				PaiXu[Temp1] = PaiXu[Temp2];
			}
		}		
	}
	
	//���м�30����9-39���ĺ�
	for(Temp1 = 9; Temp1 < 40; ++ Temp1)
	{
		PaiXu[0] +=  PaiXu[Temp1];
	}

	//��ƽ��ֵ
	
	MPU_StartYaw =  PaiXu[0] / 30.0;
}

float MPU_GetStartYaw(void)
{
	return MPU_StartYaw;
}

void MPU_UserSetStartYaw(float Yaw)
{
	
	MPU_StartYaw = ReadMpuYaw();
}



/***********************************************************************
* ����DMA���յ����ݾͲ���������жϺ���UART8_IRQHandler�ˣ�
* ÿ�յ�һ������Ҳ������жϣ������յ�DMA_InitStructure.DMA_BufferSize = 5;//���ݴ��������ֽں󣬻��һ��DMA1_Stream6_IRQHandler
***********************************************************************/
void SetMpuToDMA(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef	DMA_InitStructure;
	
//	USART_DeInit(USART2);  //��λ���ڣ���λ��ͰѴ����жϹر���
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1, ENABLE);		//ʹ��DMA1ʱ��
 
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); 
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
//	GPIO_Init(GPIOA,&GPIO_InitStructure);

//	USART_InitStructure.USART_BaudRate = 460800;//����������
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
//	USART_Init(USART2, &USART_InitStructure);	
//	USART_Cmd(USART2, ENABLE); 
//	USART_ClearFlag(USART2, USART_FLAG_TC);
	
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);//��������DMA
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	DMA_DeInit(DMA1_Stream5);
	
	while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE)
	{
		//�ȴ�DMA������ 
	}
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;//ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);//DMA�����ַ����DR���ж������൱�ڽ��գ���DR����д�����൱�ڷ���
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)GyroscopeBuff;//DMA�洢����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�洢ģʽ
	DMA_InitStructure.DMA_BufferSize = 11;//���ݴ�����
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ��ȣ�8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ��ȣ�8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//ʹ��ѭ��ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//������ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;//��ʹ��FIFO
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//����FIFO��ֵ��Ч
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;//����DMA�洢��ͻ��ģʽΪ����ģʽ
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA1_Stream5,&DMA_InitStructure);
		
	DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream5,ENABLE);
  
}
//
void WT931_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint16_t DMA_PreStart = 0;
	
	DMA_HadOpen = 0;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 460800;//460800;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx; //| USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);	
	USART_Cmd(USART2, ENABLE); 
	USART_ClearFlag(USART2, USART_FLAG_TC);
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�
	
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	SetMpuToDMA();
	
//	while(MPU_ErrState != 2)
//	{
//		++ DMA_PreStart;
//		if(DMA_PreStart >= 1000)
//		{//����1000ms��û��ȷ����������������
//			DMA_PreStart = 0;
//			if(MPU_ErrState == 0)
//			{
//				printf("MPU LINE ERR�����ǽ��ߴ���%d\n\r",MPU_ErrState);
////				while(1)
////					;
//			}
//			if(MPU_ErrState == 1)
//			{//�����ǲ��������ô�����Ҫ��������������
//				printf("MPU LINE ERR���������ô���%d\n\r",MPU_ErrState);
////				while(1)
////					;
//			}			
//			
//		}
//		delay_ms(1);//�ȴ�DMA׼���򿪣������������óɹ�
//	}
	
//	Lcd_Clear(GRAY0); //���� 
	

	delay_ms(200);
	MPU_SetStartYaw();
//	Gui_DrawFont_GBK24(0,30,BLUE,GRAY0,"MPU_OK");
//	Gui_DrawFont_GBK24(0,60,BLUE,GRAY0,"YAW_INIT");
//	Gui_DrawFont_Num32(0,90,BLUE,GRAY0,MPU_GetStartYaw());	
	
//	MPU_PrintMPUYaw();
//	MPU_TestMPURate();
	
}


void DMA1_Stream5_IRQHandler(void)
{
	uint32_t Temp = 0;
	Temp = DMA1->HISR;
    if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
	//if(DMA1->HISR & 0X0000C000)
	{//��ѯ������4�����Ƿ����
        DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
        DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
		if(GyroscopeBuff[0] == 0x55 && GyroscopeBuff[1] == 0x53)
		{     
			GyroscopeBuff[0] = GyroscopeBuff[9] + GyroscopeBuff[8] + GyroscopeBuff[7] + GyroscopeBuff[6] + \
			GyroscopeBuff[5] + GyroscopeBuff[4] + GyroscopeBuff[3] + GyroscopeBuff[2] + GyroscopeBuff[1] + GyroscopeBuff[0];
		  
			if( GyroscopeBuff[10] == GyroscopeBuff[0])
			{
				++ MPU_UpdataRata;
				MPU_YawUpdated = 1;
				GyroscopeBuff[1] = 0x00;
				if(MpuYawWhichNew == 1)
				{
					MpuYaw2 = (int16_t)( GyroscopeBuff[7]<<8 | GyroscopeBuff[8]) /32768.0 *180.0;					
					MpuYawWhichNew = 2;					
				}
				else if(MpuYawWhichNew == 2)
				{
					MpuYaw1 = (int16_t)( GyroscopeBuff[7]<<8 | GyroscopeBuff[8]) /32768.0 *180.0;
					MpuYawWhichNew = 1;
				}
			}			
		}
    }
}


void USART2_IRQHandler(void)
{	
	
  uint8_t USART2_Tmp1 = 0;    //����Ƿ�ֹ����һֱ��ѭ��������ط���
	
	if(MPU_ErrState == 0)
		MPU_ErrState = 1;//�����ǿ����յ����ݣ�֤������û��
  
  MpuData[0] =USART_ReceiveData(USART2);
  
  if( MpuData[0] == 0x55)
  {
      ++MpuCharNum;
      while(MpuCharNum <=10 && USART2_Tmp1 <= 30)    
      {
            if(USART_GetITStatus(USART2, USART_IT_RXNE) == 1)
            {
              MpuData[ MpuCharNum ] =USART_ReceiveData(USART2);             
              ++MpuCharNum;
              USART2_Tmp1 = 0;
            }
            else
            {
                delay_us(1);
                ++ USART2_Tmp1;   //460800�������£�����һ���ֽ���ʱ21.7us���ң�����Ѿ�����30us��û�н��ܵ�һ�����ֽڣ����˳�
            }            
      }
    MpuCharNum = 0;
	if(MpuData[1] == 0x53)
	{     
	  MpuData[0] = MpuData[9] + MpuData[8] + MpuData[7] + MpuData[6] + MpuData[5] + MpuData[4] + MpuData[3] + MpuData[2] + MpuData[1] + MpuData[0];
	  
	  if( MpuData[10] == MpuData[0])
	  {
		  if(MPU_ErrState == 1)
			MPU_ErrState = 2;//�����ǿ����յ����ݣ�����ȷ����������֤��ͨ�Ų�����û��
		  MpuYaw1 = (int16_t)( MpuData[7]<<8 | MpuData[8]) /32768.0 *180;
		  MpuData[1] = 0x00;
		  SetMpuToDMA();
		  DMA_HadOpen = 1;
	  }			
	}    
  }
  if(USART_GetITStatus(USART2, USART_IT_RXNE) == 1)
  {
	  USART_ReceiveData(USART2);	 
  }  
}

