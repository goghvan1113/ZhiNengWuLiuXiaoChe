#include "usmart.h"
#include "usmart_str.h"
////////////////////////////�û�������///////////////////////////////////////////////
//������Ҫ�������õ��ĺ�����������ͷ�ļ�(�û��Լ����) 
#include "delay.h" 
#include "sys.h"
#include "led.h"
//#include "pid.h"
//#include "claw.h"

//�������б��ʼ��(�û��Լ����)
//�û�ֱ������������Ҫִ�еĺ�����������Ҵ�
struct _m_usmart_nametab usmart_nametab[]=
{
	#if USMART_USE_WRFUNS==1 	//���ʹ���˶�д����
	(void*)read_addr,"u32 read_addr(u32 addr)",
	(void*)write_addr,"void write_addr(u32 addr,u32 val)",	 
#endif		   
	(void*)delay_ms,"void delay_ms(u16 nms)",
 	(void*)delay_us,"void delay_us(u32 nus)",	 
	(void*)LED_AllOpen,"void LED_AllOpen(void)",
	(void*)LED_AllClose,"void LED_AllClose(void)",
	(void*)PID_IncrementOrPosition_Init,"void PID_IncrementOrPosition_Init(uint8_t PID_Type)",
	(void*)SetPIDTargetSpeed,"void SetPIDTargetSpeed(int16_t Wheel_1, int16_t Wheel_2, int16_t Wheel_3, int16_t Wheel_4)",
	(void*)SetPIDNumber,"void SetPIDNumber(uint8_t Wheel_x,uint16_t KP, uint16_t KI, uint16_t KD)",
	(void*)PID_SetRamp,"void PID_SetRamp(uint8_t Wheel_x, uint8_t RampTime, uint8_t RampStep)",
    (void*)Usart6_SetColor,"void Usart6_SetColor(char Color1,char Color2,char Color3)", 
};
///////////////////////////////////END///////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
//�������ƹ�������ʼ��
//�õ������ܿغ���������
//�õ�����������
struct _m_usmart_dev usmart_dev=
{
    usmart_nametab,
    usmart_init,
    usmart_cmd_rec,
    usmart_exe,
    usmart_scan,
    sizeof(usmart_nametab)/sizeof(struct _m_usmart_nametab),//��������
    0,          //��������
    0,         //����ID
    1,        //������ʾ����,0,10����;1,16����
    0,        //��������.bitx:,0,����;1,�ַ���
    0,          //ÿ�������ĳ����ݴ��,��ҪMAX_PARM��0��ʼ��
    0,        //�����Ĳ���,��ҪPARM_LEN��0��ʼ��
};

