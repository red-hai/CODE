#ifndef _INFANTRYCONFIG_H_
#define _INFANTRYCONFIG_H_

#include "pid_modify.h"
#include "freertostask.h"


#define ENCODER_MIDDLE       (23L * 180L)
#define ENCODER_PER_DEGREE   (8192L/360L)
#define USART_FOR_PRINT 2L    //���ô��ڴ�ӡʹ�õĴ��ڣ�2 --uart2  3 --uart3  other--uart6
#define IMU_MODE        1    //����ʹ�����û�����IMU   1--����IMU  2--����IMU

#define FORWARESPEED    5000L 
#define HORIZONTALSPEED 5500L 
#define MOUSE_LEFTRIGHT_SPEED 0.07F 
#define MOUSE_UPDOWN_SPEED    1.8F  

#define REMOT_HORIONTALSPEED   5  
#define REMOT_FORWARDSPEED     10 
#define REMOT_LEFTRIGHT_SPEED  5/* 0.003F */
#define REMOT_UPDOWN_SPEED     0.1F   

#define CM_FOLLOW_SPEED 110U
#define CMSWINGANGLE    50L   //���õ��̰ڶ��ĽǶ�         ��Χ 0 - 100
#define CMSWINGSPEED    60L   //���õ��̰ڶ����ٶ�         ��Χ 0 - 160

#define Pitch_LIMIT_ANGLE    (29L * (ENCODER_PER_DEGREE)) //������̨���°ڶ��Ƕ�


#define PID_Parms_Init() {\
   /*��ʼ��������PID����*/\
	                                                                /*�������  ��������   Kp       Ki      kd*/\
	   PID_struct_init(&FireMotor.pid,DELTA_PID,8000   ,    1000,      7.0f,   1.0f,   3.0f);\
   /*��ʼ������PID����*/\
	for(char i = 0;i<4;i++)\
	   PID_struct_init(&CM_Motor[i].pid,DELTA_PID,8000   , 1000,     2.0f,   0.1f,   1.0f);\
   /*��ʼ����̨PID����*/\
	PID_struct_init(&YawMotor.position.pid,POSITION_PID,4000   , 1000,     12.2f,   0      ,   -5.0f);\
	PID_struct_init(&YawMotor.speed.pid,POSITION_PID,   8000   , 4000,      37.0f,   1.0f   ,   0);\
    PID_struct_init(&PitchMotor.position.pid,POSITION_PID,4000   , 1000,    -7.10f,   -0.0001 ,   10.0f);\
	PID_struct_init(&PitchMotor.speed.pid,POSITION_PID,   4000  , 2000,    -45.5f,   -0.5      ,   4.0f);\
}
#define SetFireSpeed(b) {FRICTION_MOTER_L = FRICTION_MOTER_R = b; }

#endif

