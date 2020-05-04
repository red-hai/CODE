/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
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

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "buzzer.h"
#include "Detect_Task.h"

#include "user_lib.h"


static void gimbal_auto_aim_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
////云台校准蜂鸣器响声
//#define GIMBALWarnBuzzerOn() buzzer_on(31, 20000)
//#define GIMBALWarnBuzzerOFF() buzzer_off()

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
  * @author         RM
  * @param[in]      输入的遥控器值
  * @param[in]      输出的死区处理后遥控器值
  * @param[in]      死区值
  * @retval         返回空
  */
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
  * @brief          云台校准的通过判断角速度来判断云台是否到达极限位置
  * @author         RM
  * @param[in]      对应轴的角速度，单位rad/s
  * @param[in]      计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
  * @param[in]      记录的角度 rad
  * @param[in]      反馈的角度 rad
  * @param[in]      记录的编码值 raw
  * @param[in]      反馈的编码值 raw
  * @param[in]      校准的步骤 完成一次 加一
  * @retval         返回空
  */
#define GIMBAL_CALI_GYRO_JUDGE(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
  * @brief          云台行为状态机设置，因为在cali等模式下使用了return，故而再用了一个函数
  * @author         RM
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set);

/**
  * @brief          云台无力控制，在这个模式下发送的yaw，pitch 是电机控制原始值，云台电机发送can零控制量，使得云台无力
  * @author         RM
  * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);



/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);



/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);

//云台行为状态机
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;   //gimbal_behaviour_e为枚举类型，如果上一次是GIMBAL_ZERO_FORCE，下一次模式被修改那么变为GIMBAL_INIT

/**
  * @brief          云台行为状态机以及电机状态机设置
  * @author         RM
  * @param[in]      云台数据指针
  * @retval         返回空
  */

gimbal_motor_mode_e test = GIMBAL_MOTOR_ENCONDE;
void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //云台行为状态机设置，这个函数确定gimbal_behaviour
    gimbal_behavour_set(gimbal_mode_set);

    //根据云台行为状态机gimbal_behaviour设置电机状态机gimbal_mode_set
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)  //云台无力时，电机原始值控制
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)  //云台初始化的电机运动模式（云台上电时的云台校准），全局变量为static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE，如果上一次是GIMBAL_ZERO_FORCE，下一次模式被修改那么变为GIMBAL_INIT
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_CALI)  //云台上电后的由外部条件而触发的云台校准模式
    {//云台校准，raw控制，电机原始值控制
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;//电机原始值控制
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
		
		//控制车子运动（底盘跟随云台）都用这个gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)//控制车子运动（底盘跟随云台）和一键掉头用的模式
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO; 
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
		
		
		//控制车子运动（底盘和云天独立控制）都用这个gimbal_behaviour == GIMBAL_RELATIVE_ANGLE
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)   //编码器的绝对角度控制，控制车子运动（底盘和云台独立控制）都用这个gimbal_behaviour == GIMBAL_RELATIVE_ANGLE
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;//都用编码器控制
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
		
		
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS) //防止陀螺仪漂移模式
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode   = GIMBAL_MOTOR_ENCONDE;//编码器控制
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
	else if(gimbal_behaviour == GIMBAL_AUTO_AIM)  //辅助射击模式
	{
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode   = GIMBAL_MOTOR_GYRO;//陀螺仪角度控制
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;	//编码器控制
	}
}

/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @author         RM
  * @param[in]      设置的yaw角度增加值，单位 rad
  * @param[in]      设置的pitch角度增加值，单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static fp32 rc_add_yaw, rc_add_pit;
    static int16_t yaw_channel = 0, pitch_channel = 0;

    //将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_channel, RC_deadband);

				//这里是遥控和键盘的数据转换为对电机控制数据
    rc_add_yaw = yaw_channel * Yaw_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen;
    rc_add_pit = - pitch_channel * Pitch_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.y * Pitch_Mouse_Sen;

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
		//将增量置零
        gimbal_zero_force_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT) //全局变量为static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE，如果上一次是GIMBAL_ZERO_FORCE，下一次模式被修改那么变为GIMBAL_INIT，上电时进入这里进行云台初始化
    {
        gimbal_init_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
		
		
		//上电之后由外部触发的云台校准
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_cali_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);  //上电之后由外部触发的云台校准
    }
		
		////车子运动（底盘跟随云台）时是gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE	
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)//这个模式是控制车子运动（底盘跟随云台）和用来一键掉头的
    {
        gimbal_absolute_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);  //陀螺仪控制云台控制变量的处理
			/*
			说明：在这个函数里当按下键盘上TurnKeyBoard这个键时，就是处理一键掉头的数据并且把数据赋给rc_add_yaw和rc_add_pit
			      如果并没有按下这个TurnKeyBoard键，在这个函数里对遥控值rc_add_yaw和rc_add_pit不进行处理（就像编码器控制一样），即遥控给多少就是多少
			*/
    }
		
   
//车子运动时（底盘和云台独立控制）是gimbal_behaviour = GIMBAL_RELATIVE_ANGLE		
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);//用编码器控制时，不用处理，遥控给多少就是多少
    }
		
		
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set); //静默模式就是把控制变量清零，就不会对机器有控制
    }
	else if(gimbal_behaviour == GIMBAL_AUTO_AIM)
	{
		gimbal_auto_aim_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);  
	}
		
		
    //将控制增加量赋值，把经过处理后的控制变量赋值
    *add_yaw = rc_add_yaw;  
    *add_pitch = rc_add_pit;
}

/**
  * @brief          云台在某些行为下，需要底盘不动
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          云台在某些行为下，需要射击停止
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
/**
  * @brief          云台行为状态机设置，因为在cali等模式下使用了return，故而再用了一个函数
  * @author         RM
  * @param[in]      云台数据指针
  * @retval         返回空
  */
extern uint8_t isChassisSwing(void);
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set)
{
	//gimbal_behaviour 一开始的值为 GIMBAL_ZERO_FORCE

    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //校准行为，return 不会设置其他的模式，GIMBAL_CALI的校准行为是由上电后的外部条件触发
    if (gimbal_behaviour == GIMBAL_CALI && gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {		
        return;
    }
	else if(gimbal_behaviour == GIMBAL_CALI && gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
	{
	   gimbal_mode_set->gimbal_cali.step = 0;
	   gimbal_behaviour = GIMBAL_ZERO_FORCE;//底盘不动
	}
	
	
	//如何把gimbal_behaviour赋值为GIMBAL_CALI进入校准模式：1.在校准任务calibrate_Task里，进入RC_cmd_to_calibrate()函数，在里边通过上外八把cali_sensor[CALI_GIMBAL].cali_cmd置1，即cali_sensor[1].cali_cmd置1
  //                                                     2.在云台校准时if (cali_sensor[i].cali_cmd)成立
	//                                                     3.就调用函数cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON)，即云台校准调用cali_gimbal_hook（uint32_t *cali, bool_t cmd）函数，
	//                                                     4.云台校准时传进来的cmd为CALI_FUNC_CMD_ON，他的宏为1，即cmd=1，使得函数cali_gimbal_hook里的(cmd == CALI_FUNC_CMD_ON)成立，就会执行cmd_cali_gimbal_hook（）函数
	//                                                     5.在cmd_cali_gimbal_hook（）函数里会把gimbal_control.gimbal_cali.step 赋值为GIMBAL_CALI_START_STEP;
	//                                                     6.使得    if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP)成立
  //如果外部使得校准步骤从0 变成 start，则进入校准模式
    if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP)
    {
        gimbal_behaviour = GIMBAL_CALI;//进入校准模式，底盘不动
        return;
    }
	

     //这里的gimbal_behaviour赋值GIMBAL_INIT从此函数的接近结尾处得来
    //云台上电时的云台校准初始化模式判断是否到达中值位置，云台校准
		//这里是检查云台是否到了中值，实现云台上电初始化的函数是gimbal_init_control（）；
    if (gimbal_behaviour == GIMBAL_INIT)//全局变量为static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE，如果上一次是GIMBAL_ZERO_FORCE，下一次模式被修改那么变为GIMBAL_INIT
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;
        //到达中值 计时
        if ((fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
        {
            //到达初始化位置
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            //没有到达初始化位置，时间计时
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        //超过初始化最大时间，或者已经稳定到中值一段时间，退出初始化状态开关打下档，或者掉线
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]) && !toe_is_error(DBUSTOE))
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    //云台校准过后就是用遥控开关控制 云台状态
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;//下档，云台无力，底盘不动
    }
    else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
			//控制车子运动（底盘和云台独立控制）都是gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;//编码器绝对角度控制，车子的云台都是用编码器来控制，用编码器或陀螺仪做反馈进行PID计算
    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
			
			//控制车子运动（底盘跟随云台）都是gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;//陀螺仪绝对角度控制，这个模式是车子运动（底盘跟随云台）和用来一键掉头的，能用来控制车子的运动
    }
	
	if((gimbal_mode_set->gimbal_rc_ctrl->key.v & VisonONKeyBoard) != 0 )
	{
		//按下Q键且识别到目标时进入辅助射击模式
		if(gimbal_mode_set->autodata->PitchAxiaAngle!=0.0f || gimbal_mode_set->autodata->YawAxiaAngle!=0.0f)
			gimbal_mode_set->auto_aim_flag = 1;	
		else 
			gimbal_mode_set->auto_aim_flag = 0;	
	}
	else if((gimbal_mode_set->gimbal_rc_ctrl->key.v & VisonOFFKeyBoard) != 0 && gimbal_mode_set->auto_aim_flag == 1)
	{
		//按下E键强制退出辅助射击模式
	    gimbal_mode_set->auto_aim_flag = 0;
	}
	
	//辅助射击模式下 2s未识别到目标强制退出
	if(gimbal_mode_set->auto_aim_flag == 1 && gimbal_mode_set->autodata->PitchAxiaAngle ==0 && gimbal_mode_set->autodata->YawAxiaAngle ==0 )
	{
		static uint16_t no_targe_cnt = 0;
		no_targe_cnt++;
		if(no_targe_cnt >= 2000)
		{
			no_targe_cnt = 0;
		    gimbal_mode_set->auto_aim_flag = 0;
		}
	}
	
	if(gimbal_mode_set->auto_aim_flag == 1 && !switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
	{
	    gimbal_behaviour = GIMBAL_AUTO_AIM;
	}

    if( toe_is_error(DBUSTOE))
    {

        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
		

      //上电时不会由辅助射击，所以上电时代码的指向会直接跳过辅助射击的代码，就会进入云台初始化模式
    //判断进入init状态机
    {
			//云台行为状态机的初始值为GIMBAL_ZERO_FORCE，接下来就把gimbal_behaviour设为云台初始化模式，把gimbal_behaviour赋值为GIMBAL_INIT
        static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;  //
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
        }
        last_gimbal_behaviour = gimbal_behaviour;
    }

		
		//如果遥控长时间无操作，就进入静默模式，即把gimbal_behaviour赋值为GIMBAL_MOTIONLESS
    static uint16_t motionless_time = 0;
    if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE && isChassisSwing() == 0 && gimbal_mode_set->auto_aim_flag == 0)
    {
        //遥控器 键盘均无输入，且底盘不处于扭腰模式时进入motionless状态
        if (int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[0]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[1]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[2]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[3]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->mouse.x) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->mouse.y) < GIMBAL_MOTIONLESS_RC_DEADLINE && gimbal_mode_set->gimbal_rc_ctrl->key.v == 0 && gimbal_mode_set->gimbal_rc_ctrl->mouse.press_l == 0 && gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r == 0)
        {
            if (motionless_time < GIMBAL_MOTIONLESS_TIME_MAX)
            {
                motionless_time++;
            }
        }
        else
        {
            motionless_time = 0;
        }

        if (motionless_time == GIMBAL_MOTIONLESS_TIME_MAX )
        {
            gimbal_behaviour = GIMBAL_MOTIONLESS;
        }
    }
    else
    {
        motionless_time = 0;
    }


}

/**
  * @brief          云台无力控制，在这个模式下发送的yaw，pitch 是电机控制原始值，云台电机发送can零控制量，使得云台无力
  * @author         RM
  * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;  //把P，Y轴的的控制值都置0，来实现云台无力
}
/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //初始化状态控制量计算，
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.relative_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
			//计算P轴现在的角度和设定角度的差值赋给云台控制变量
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;  
        *yaw = 0.0f;
    }
    else
    {
		*pitch = 0;
        //*pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
			//计算Y轴现在的角度和设定角度的差值赋给云台控制变量
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}



/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值，用来进行另一个函数中的云台中值计算
  * @author         RM
  * @param[in]      发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

		//通过机械的限位来来进行云台的校准，记录下的数据传到cmd_cali_gimbal_hook（）函数
    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch =- GIMBAL_CALI_PITCH_MOTOR_SET;//把电流的原始值给P轴的控制变量，效果就是P轴会自己动，当到达最大的位置时，判断堵转的时间，超过一定的时间就任务转到了最大值，记录下来
        *yaw = 0;

        //判断陀螺仪数据， 并记录最大最小角度数据
        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = GIMBAL_CALI_PITCH_MOTOR_SET;//把电流的原始值给P轴的控制变量，效果就是P轴会自己动，当到达最大的位置时，判断堵转的时间，超过一定的时间就任务转到了最大值，记录下来
        *yaw = 0;

        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}
/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    {
        static uint16_t last_turn_keyboard = 0;
        static uint8_t gimbal_turn_flag = 0;
        static fp32 gimbal_end_angle = 0.0f;

        if ((gimbal_control_set->gimbal_rc_ctrl->key.v & TurnKeyBoard) && !(last_turn_keyboard & TurnKeyBoard))
        {
            if (gimbal_turn_flag == 0)
            {
                gimbal_turn_flag = 1;
                //保存掉头的目标值
                gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
            }
        }
        last_turn_keyboard = gimbal_control_set->gimbal_rc_ctrl->key.v ;

        if (gimbal_turn_flag)
        {
            //不断控制到掉头的目标值，正转，反装是随机
            if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)
            {
                *yaw += TurnSpeed;
            }
            else
            {
                *yaw -= TurnSpeed;
            }
        }
        //到达pi （180°）后停止
        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
        {
            gimbal_turn_flag = 0;
        }
    }
}
/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //不需要处理，
}
/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          云台进入辅助瞄准模式，电机是相对角度控制，
  * @author         RM
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */

float ration_y = 0.0009f,ration_p=0.0009f;
static void gimbal_auto_aim_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
//    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
//    {
//        return;
//    }
//    if(gimbal_control_set->autodata->YawAxiaAngle > 0.087f )
//    {
//	    *yaw = ration_y;
//	}
//	else if(gimbal_control_set->autodata->YawAxiaAngle < -0.087f)
//	{
//	   *yaw = -ration_y;
//	}
//	else
//	{
//	   *yaw = 0.0f;
//	}
//	
//	if(gimbal_control_set->autodata->PitchAxiaAngle > 0.087f )
//	{
//	    *pitch  = ration_p;
//	}
//	else if( gimbal_control_set->autodata->PitchAxiaAngle < -0.087f)
//	{
//		*pitch = -ration_p;
//	}
//	else
//	{
//	  *pitch  = 0.0f;
//	}
		
		
	*yaw   = 0;//gimbal_control_set->autodata->YawAxiaAngle * ration_y;
	*pitch = 0;// gimbal_control_set->autodata->PitchAxiaAngle * ration_p ;	
}

uint8_t IsGimbalMotionless()
{
   return gimbal_behaviour == GIMBAL_MOTIONLESS;
}
