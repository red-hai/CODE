/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能，其中射击的初始化，以及循环都是在云台任务中调用，故而此文件
  *             不是freeRTOS任务，射击分关闭状态，准备状态，射击状态，以及完成状态
  *             关闭状态是关闭摩擦轮以及激光，准备状态是将子弹拨到微型开关处，射击状
  *             态是将子弹射出，判断微型开关值，进入完成状态，完成状态通过判断一定时间
  *             微型开关无子弹认为已经将子弹射出。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "Shoot.h"
#include "MotorCan.h"
#include "gimbal_behaviour.h"
#include "Detect_Task.h"
#include "pid.h"
#include "BasicPeripherals.h"
#include "arm_math.h"
#include "user_lib.h"
#include "FrictionMoterPWM.h"

#include "stm32f4xx.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Referee_DispatchTask.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //摩擦轮1pwm宏定义
#define shoot_fric2_on(pwm) fric2_on((pwm)) //摩擦轮2pwm宏定义
#define shoot_fric_off() fric_off()         //关闭两个摩擦轮

#define shoot_laser_on() laser_on()   //激光开启宏定义
#define shoot_laser_off() laser_off() //激光关闭宏定义

static const RC_ctrl_t *shoot_rc; //遥控器指针

 PidTypeDef trigger_motor_pid;
 Shoot_Motor_t trigger_motor;          //射击数据
 shoot_mode_e shoot_mode = SHOOT_STOP; //射击状态机

float    freOfSmallBullet =  8.5;
int16_t  speedOfSmallBullet =  Fric_DOWN;


extern void getTriggerMotorMeasure(motor_measure_t *motor);
/**
  * @brief          射击数据更新
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void);

static void LevelToStopBulet(void);
/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{

    static const fp32 Trigger_speed_pid[3]= {TRIGGER_15_ANGLE_PID_KP,TRIGGER_15_ANGLE_PID_KI,TRIGGER_15_ANGLE_PID_KD};
												 											 
    //遥控器指针
    shoot_rc = get_remote_control_point();
    //电机指针
	trigger_motor.shoot_motor_measure = get_shoot_Motor_Measure_Point(2);
    trigger_motor.referee_data = GetRefereeDataPoint();//获取裁判系统数据	 	 
    //初始化PID
    PID_Init(&trigger_motor_pid, PID_POSITION, &Trigger_speed_pid[0], TRIGGER_15_BULLET_PID_MAX_OUT, TRIGGER_15_BULLET_PID_MAX_IOUT);
    //更新数据
    Shoot_Feedback_Update();
    //初始化斜坡参数 
	ramp_init(&trigger_motor.fric1_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF); 
    ramp_init(&trigger_motor.fric2_ramp, SHOOT_CONTROL_TIME * 0.001f, Fric_DOWN, Fric_OFF);
	//初始化电机参数
	trigger_motor.ecd_count = 0;
	trigger_motor.given_current = 0;
	trigger_motor.move_flag = 0;
	trigger_motor.set_angle = trigger_motor.angle;
	trigger_motor.speed = 0.0f;
	trigger_motor.speed_set = 0.0f;
	trigger_motor.BulletShootCnt = 0;	 
	trigger_motor.angle = trigger_motor.shoot_motor_measure->ecd * Motor_ECD_TO_ANGLE_36;
    trigger_motor.mainfocus_data =  GetAutoDataPoint();	
}
/**
  * @brief          射击循环
  * @author         RM
  * @param[in]      void
  * @retval         返回can控制值
  */
char auto_shoot = 0;
extern  char auto_flag ;
int16_t *shoot_control_loop(void)
{
   static  int16_t shoot_CAN_Set_Current; //返回的can值
   static int8_t last_s = RC_SW_UP;
	static uint16_t cnt = 0;

    Shoot_Feedback_Update(); //更新数据
	
//	if( auto_flag == 1)
//		{
//			cnt++;
//			if(cnt>= 400)
//			{
//				cnt = 0;
//				shoot_mode = SHOOT_READY;
//			}
//		}
//		else
//		{
//		   cnt = 0;
//		   shoot_mode = SHOOT_STOP;
//		}
	
	if(toe_is_error(DBUSTOE))
	{
		if( auto_flag == 1)
		{
			cnt++;
			if(cnt>= 400)
			{
				cnt = 0;
				shoot_mode = SHOOT_READY;
			}
		}
		else
		{
		   cnt = 0;
		   shoot_mode = SHOOT_STOP;
		}	
	}
	else
	{	
				//上拨判断， 一次开启，再次关闭
		if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP))
		{
			shoot_mode = SHOOT_READY;
		}
		else if ((switch_is_up(shoot_rc->rc.s[Shoot_RC_Channel]) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP) || (shoot_rc->key.v & SHOOT_OFF_KEYBOARD))
		{
			shoot_mode = SHOOT_STOP;
		}

		//处于中档， 可以使用键盘开启摩擦轮
		if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && (shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_mode == SHOOT_STOP)
		{
			shoot_mode = SHOOT_READY;
		}
		//处于中档， 可以使用键盘关闭摩擦轮
		else if (switch_is_mid(shoot_rc->rc.s[Shoot_RC_Channel]) && (shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_mode == SHOOT_READY)
		{
			shoot_mode = SHOOT_STOP;
		}
		
		last_s = shoot_rc->rc.s[Shoot_RC_Channel];
	}
	
	        //摩擦轮pwm
        static uint16_t fric_pwm1 = Fric_OFF;
        static uint16_t fric_pwm2 = Fric_OFF;
		

        shoot_laser_on();       //激光开启

        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
        ramp_calc(&trigger_motor.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

        if(trigger_motor.fric1_ramp.out == trigger_motor.fric1_ramp.max_value)
        {
            ramp_calc(&trigger_motor.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
        }

        if( trigger_motor.fric2_ramp.out != trigger_motor.fric2_ramp.max_value)
        {
            trigger_motor.speed_set = 0.0f;
        }


       trigger_motor.fric1_ramp.max_value = speedOfSmallBullet;//Fric_UP;
       trigger_motor.fric2_ramp.max_value = speedOfSmallBullet;//Fric_UP;


        fric_pwm1 = (uint16_t)(trigger_motor.fric1_ramp.out);
        fric_pwm2 = (uint16_t)(trigger_motor.fric2_ramp.out);

        shoot_fric1_on(fric_pwm1);
        shoot_fric2_on(fric_pwm2);


    if (shoot_mode == SHOOT_STOP)
    {

        shoot_CAN_Set_Current = 0;
		
	    trigger_motor.set_angle = trigger_motor.angle;
	    trigger_motor.move_flag = 0;
	    trigger_motor.speed_set = 0;
    }
    else
    {

//		if(trigger_motor.move_flag == 0)
//		{
//			trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Eigth);
//			trigger_motor.cmd_time  = xTaskGetTickCount();
//			trigger_motor.move_flag = 1;		
//		} 
//		
//		if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
//		{
//			//角度达到判断
			trigger_motor.speed_set = freOfSmallBullet;
		
					//堵转判断
			static uint16_t stall_cnt = 0,flag = 0;;
			if(trigger_motor.speed_set != 0 && fabs(trigger_motor.speed) < 0.5f && flag == 0 )
			{	
				stall_cnt++;
				if(stall_cnt >= 300)
				{
					stall_cnt = 0;
					flag  = 1;
				}
			}
			else
			{
				stall_cnt = 0;
			}
			
			if(flag == 1)
			{
				static uint16_t negetive_cnt = 0;
				
				trigger_motor.speed_set = -5.5f;
				negetive_cnt++;
				if(negetive_cnt >= 200)
				{
					flag = 0;
					negetive_cnt = 0;
				}
			}
//			trigger_motor.run_time  = xTaskGetTickCount();
//			//堵转判断
//			if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < 1000 + BLOCK_TIME && fabs(trigger_motor.speed) < (5.5f * 0.3f))
//			{
//				trigger_motor.speed_set = -10.0f;
//			}
//			else if (trigger_motor.run_time - trigger_motor.cmd_time > 1000 + BLOCK_TIME)
//			{
//				trigger_motor.cmd_time = xTaskGetTickCount();
//			}
//		}
//		else
//		{
//			trigger_motor.move_flag = 0;
//		}		       			
		
		//根据等级在准备超热量时停止拨弹轮
		LevelToStopBulet();
		
	    PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);

       
		trigger_motor.given_current = (int16_t)(trigger_motor_pid.out);
        shoot_CAN_Set_Current = trigger_motor.given_current;				
    }
    return &shoot_CAN_Set_Current;
}

/**
  * @brief          射击数据更新
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void)
{

    static fp32 speed_fliter_1 = {0.0f};
    static fp32 speed_fliter_2 = {0.0f};
    static fp32 speed_fliter_3 = {0.0f};

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波

		speed_fliter_1 = speed_fliter_2;
		speed_fliter_2 = speed_fliter_3;
		speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
		trigger_motor.speed = speed_fliter_3;	


		//电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
		if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd > Half_ecd_range)
		{
			trigger_motor.ecd_count--;
		}
		else if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd < -Half_ecd_range)
		{
			trigger_motor.ecd_count++;
		}

		//鼠标按键
		trigger_motor.last_press_l = trigger_motor.press_l;
		trigger_motor.last_press_r = trigger_motor.press_r;
		trigger_motor.press_l = shoot_rc->mouse.press_l;
		trigger_motor.press_r = shoot_rc->mouse.press_r;	

		//长按计时
		if (trigger_motor.press_l)
		{
			if (trigger_motor.press_l_time < PRESS_LONG_TIME)
			{
				trigger_motor.press_l_time++;
			}
		}
		else
		{
			trigger_motor.press_l_time = 0;
		}	

		if (trigger_motor.press_r)
		{
			if (trigger_motor.press_r_time < PRESS_LONG_TIME)
			{
				trigger_motor.press_r_time++;
			}
		}
		else
		{
			trigger_motor.press_r_time = 0;
		}
		//射击开关下档时间计时
		if (shoot_mode != SHOOT_STOP && switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
		{

			if (trigger_motor.rc_s_time < RC_S_LONG_TIME)
			{
				trigger_motor.rc_s_time++;
			}
		}
		else
		{
			trigger_motor.rc_s_time = 0;
		}
		
			//射击开关下档时间计时
		if (shoot_mode != SHOOT_STOP && switch_is_down(shoot_rc->rc.s[Shoot_RC_Channel]))
		{

			if (trigger_motor.rc_s_time < RC_S_LONG_TIME)
			{
				trigger_motor.rc_s_time++;
			}
		}
		else
		{
			trigger_motor.rc_s_time = 0;
		}	
	
		if (trigger_motor.ecd_count == FULL_COUNT_36)
		{
			trigger_motor.ecd_count = -(FULL_COUNT_36 - 1);
		}
		else if (trigger_motor.ecd_count == -FULL_COUNT_36)
		{
			trigger_motor.ecd_count = FULL_COUNT_36 - 1;
		}		
		trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + trigger_motor.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE_36;
}

static void LevelToStopBulet()
{
	 
		if(trigger_motor.referee_data->power_heat_data.shooter_heat0 > 480 -200)
		{
			trigger_motor.speed_set = 0;
		}
}

static void SetShootFreAndSpeed()
{
	//C键低速高射频模式
   if(shoot_rc->key.v == KEY_PRESSED_OFFSET_C)
   {
	   freOfSmallBullet   =  8.5f;
	   speedOfSmallBullet =  1150;
         
   }
   else if(shoot_rc->key.v == KEY_PRESSED_OFFSET_V)
   {
	   //V键中射速中射频模式
 	   freOfSmallBullet   =  6.5f;
	   speedOfSmallBullet =  1200;
  
   }
   else if(shoot_rc->key.v == KEY_PRESSED_OFFSET_B)
   {
	   //V键高射速低射频模式
 	   freOfSmallBullet   =  4.5f;
	   speedOfSmallBullet =  1300;	
   }
}
