#include "FrictionMoterPWM.h"
#include "stm32f4xx.h"

void FrictionMoterPWM_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	


	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM2);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1, GPIO_AF_TIM2);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2, GPIO_AF_TIM2);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_TIM2);

//GPIO的复用
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
	GPIO_Init(GPIOA,&GPIO_InitStructure);  
/////////////////////////////////////////	

//定时器的初始化
	TIM_TimeBaseStructure.TIM_Prescaler = 900-1;  //0.1MHZ定时器分频技计数频率
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period = 2000-1;  //50HZ，脉冲周期
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
///////////////////////////////////////////

//PWM的配置
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM模式1还是模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //使能PWM
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//高电平有效还是低电平有效
//	TIM_OCInitStructure.TIM_Pulse = 100;//比较值，即CCR的值，用来设置占空比//也可用TIM_SetCompare1（）库函数
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);   
//	TIM_OC2Init(TIM2, &TIM_OCInitStructure); 
//	TIM_OC3Init(TIM2, &TIM_OCInitStructure);   
//	TIM_OC4Init(TIM2, &TIM_OCInitStructure);   
	///////////////////////////////////

//把舵机接的IO口和TIM2定时器的输出IO口一致，TIM2定时器的IO口就可以输出PWM信号，只要找到开启摩擦轮的开关进入摩擦轮任务就行（把控制舵机放在了摩擦轮任务里了）
//或把TIM_SetCompare1（）写在底盘里


	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable); 
//	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  
//	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable); 
//	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); 

	TIM_ARRPreloadConfig(TIM2,ENABLE);

	TIM_Cmd(TIM2, ENABLE);  	
}


