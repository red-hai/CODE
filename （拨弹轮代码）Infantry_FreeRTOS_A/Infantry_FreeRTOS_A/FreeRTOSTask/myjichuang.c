#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "freertostask.h"
#include "BasicPeripherals.h"
#include "FrictionMoterPWM.h"
#include "RemotDbus.h"
#include "usart.h"
#include "MotorCAN.h"
#include "RemotDbus.h"
#include "imu_int.h"
#include "imu.h" 
#include "pid_modify.h"
#include "stdlib.h"
#include "MPU6500_driver.h"
#include "stmflash.h"
#include "string.h"
#include "myjichuang.h"
#include "math.h"
#include "InfantryConfig.h"
#include "delay.h"
#include "Auto_attackTask.h"
#include "MainFocus_Usart.h"
#include "Referee.h"

void kaweimotor(void)
{
  RemotDbus_Init();
	
}