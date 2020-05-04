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

//GPIO�ĸ���
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
	GPIO_Init(GPIOA,&GPIO_InitStructure);  
/////////////////////////////////////////	

//��ʱ���ĳ�ʼ��
	TIM_TimeBaseStructure.TIM_Prescaler = 900-1;  //0.1MHZ��ʱ����Ƶ������Ƶ��
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period = 2000-1;  //50HZ����������
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
///////////////////////////////////////////

//PWM������
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWMģʽ1����ģʽ2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //ʹ��PWM
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//�ߵ�ƽ��Ч���ǵ͵�ƽ��Ч
//	TIM_OCInitStructure.TIM_Pulse = 100;//�Ƚ�ֵ����CCR��ֵ����������ռ�ձ�//Ҳ����TIM_SetCompare1�����⺯��
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);   
//	TIM_OC2Init(TIM2, &TIM_OCInitStructure); 
//	TIM_OC3Init(TIM2, &TIM_OCInitStructure);   
//	TIM_OC4Init(TIM2, &TIM_OCInitStructure);   
	///////////////////////////////////

//�Ѷ���ӵ�IO�ں�TIM2��ʱ�������IO��һ�£�TIM2��ʱ����IO�ھͿ������PWM�źţ�ֻҪ�ҵ�����Ħ���ֵĿ��ؽ���Ħ����������У��ѿ��ƶ��������Ħ�����������ˣ�
//���TIM_SetCompare1����д�ڵ�����


	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable); 
//	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);  
//	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable); 
//	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); 

	TIM_ARRPreloadConfig(TIM2,ENABLE);

	TIM_Cmd(TIM2, ENABLE);  	
}


