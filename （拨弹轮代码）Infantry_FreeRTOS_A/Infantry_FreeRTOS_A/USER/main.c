#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "freertostask.h"   




int main()
{
	
	/*设置优先级分组4，即4位抢占优先级，0位相应优先级*/
      NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    delay_init(180);		  //初始化滴答定时器
	
	/*创建开始任务并开启调度器*/
	FreeRTOS_init();  
while(1)
{

}	

//} 

//int main()
//{
//		pid_t pid1;
//	pid1.p=1000;
//	pid1.i=0;
//	pid1.d=0;
//	pid1.max_out=8000;
//	pid1.pid_mode = POSITION_PID;
//	int n;
//	int16_t  cm[4];
//    honwai_Init();
//	/*Never arrive here*/
//	while(1)
//	{  
// 	    
//				for(char i = 0;i<4;i++)
//		{
//	      cm[i]   =  (int16_t)pid_calc(&pid1,6000,CM_Motor[0].getspeed);
//			
//		}
//			n=GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_10);
//		if(n)
//		{
//		   CanSendMess(CAN1,0x201,cm);
//		}

//   	}
//	
		
}

