

#ifndef __SPEED_SOLVING_H__
#define __SPEED_SOLVING_H__

#include "SpeedControl.h"


#define SPEED_SOLVING_RAMP_TIME	4.0f	//经4次使斜坡达到最终值
#define SPEED_SOLVING_RAMP_STEP_MIN	0.25f

//问一下汪院找这个资料


//四轮麦克纳姆轮
//四轮速度解算，参考：https://blog.csdn.net/banzhuan133/article/details/69229922
//四轮速度解算，参考https://zhuanlan.zhihu.com/p/20282234
//轮子安装方式，左前方轮子正转时，力矩向左前方，右前方轮子正转时，力矩向右前方。
//坐标定义：前方为Y轴正前方，垂直向右为X轴正前方。
// Speed_x表示 X 轴运动的速度，即左右方向，定义向右为正；
// Speed_y表示 Y 轴运动的速度，即前后方向，定义向前为正；
// Speed_z表示 yaw 轴自转的角速度，定义逆时针为正
//下面的值定义见参考资料里面的论文：陈博翁
#define SPEED_SOLVING4_DATA_L1	139.5f	//小车横向尺寸的半宽，即旋转中心到左边轮子的距离。单位是mm
#define SPEED_SOLVING4_DATA_L2	135.0f	//小车纵向尺寸的半长，即旋转中心到前边轮子的距离。单位是mm
#define SPEED_SOLVING4_DATA_R_Y	30.000000f	//麦轮半径(mm)，通过位移计算出的半径。这个半径适用于Y方向
#define SPEED_SOLVING4_DATA_R_X	30.000000f	//麦轮半径(mm)，通过位移计算出的半径。这个半径适用于X方向
#define SPEED_SOLVING4_DATA_R_Z	30.000000f	//麦轮半径(mm)，通过位移计算出的半径。这个半径适用于Z方向
#define SPEED_SOLVING4_DATA_R	30.000000f		//卖家给的麦轮半径。这个也是理论半径
#define SPEED_SOLVING4_DATA_TAN_A	1.0f	//tan(a)，及角度阿尔法的正切值
#define SPEED_SOLVING4_DATA_MOTOR_N	375.0f	//电机最快转速
#define SPEED_SOLVING4_DATA_MOTOR_PULSE_N	98.0f	//电机最快转速对应的编码器每10毫秒脉冲
#define SPEED_SOLVING4_DATA_MOTOR_PWM	2000.0f	//PWM最大值
#define SPEED_SOLVING4_DATA_ENCODE_PULSE_MAX	1560.0f	//轮子转一圈对应的编码器脉冲数



typedef struct
{
	float Speed_1;
	float Speed_2;
	float Speed_3;
	float Speed_4;
	uint8_t UpData;
}SpeedSolvingOut4_t;


typedef struct
{
	float PositionX_mm;
	float PositionY_mm;
	float PositionZ_D;//单位是度
}PositionSolving_t;//位置解算

typedef struct
{
	float TargetSpeed_X;
	float TargetSpeed_Y;
	float TargetSpeed_Z;
}TargetSpeed_t;


/*
* 输入形参：Speed_x、Speed_y单位均为m/s，Speed_z单位为度每秒，输出的结果是每ms的PWM脉冲
*/
SpeedSolvingOut4_t SpeedSolving_4(float Speed_x,float Speed_y,float Speed_z);/*小车
* 的坐标为1、2轮向前为Y轴正方向，垂直向右为X轴正方向。这个为逆解算，即解算出每个轮子的目标转速
*/


/*
* 下面是正解算，输入每个轮子转的编码器脉冲，输出小车X、Y、Z方向转过的距离和角度，单位是mm和度
*/
PositionSolving_t PositionSolving_4(float Pulse_1,float Pulse_2,float Pulse_3,float Pulse_4);


/*
* 同时传入目标值
*/
void SpeedSolving4_Set(void);


/*
* 在位置环计算出速度之后，还有精准控制时间，因此需要传进来一个斜坡函数的横距离time参数
* 又觉得这个限幅的环不应该放在这里
*/
void SpeedSolving4_SetWithTime(float HowTime,SpeedSolvingOut4_t *TargetPulse);
TargetSpeed_t SpeedSolving_GetTargetSpeed(void);
float MyAbs(float a);


#endif











