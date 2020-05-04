#include "mypwm.h"
#include "stm32f4xx.h"



//void my_PWM_Init(u16 a,u16 p)
//{
//  	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
//	
//		GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM2);
//	
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;         
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
//	GPIO_Init(GPIOA,&GPIO_InitStructure);  
//	
//		TIM_TimeBaseStructure.TIM_Prescaler = a; 
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
//	TIM_TimeBaseStructure.TIM_Period = p;  
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
//	
//		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
//	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
//	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OCInitStructure.TIM_Pulse = 100;
//		TIM_OC4Init(TIM2, &TIM_OCInitStructure);
//		
//		
//		TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable); 
//		
//			TIM_ARRPreloadConfig(TIM2,ENABLE);

//	TIM_Cmd(TIM2, ENABLE);  

//}

void myPWM_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOF时钟

  //GPIOF9,F10初始化设置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
	
	GPIO_SetBits(GPIOA,GPIO_Pin_0 );//GPIOF9,F10设置高，灯灭

}



						  
