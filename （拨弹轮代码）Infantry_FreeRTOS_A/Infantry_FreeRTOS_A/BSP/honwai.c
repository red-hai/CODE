#include "honwai.h"
#include "stm32f4xx.h"



void honwai_Init(void)
{
          GPIO_InitTypeDef  GPIO_InitStructure;
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
       GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
       GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;        
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
       GPIO_Init(GPIOF, &GPIO_InitStructure);
      
//   GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1);

}