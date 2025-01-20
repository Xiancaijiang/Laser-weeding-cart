/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_init.c
 *  Description  : All initialization threads
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:42:52
 *  LastEditTime : 2023-05-05 15:17:43
 */


#include "app_init.h"
#include "sys_softTimer.h"
#include "sys_dwt.h"
#include "alg_pid.h"
#include "util_can.h"
#include "periph_motor.h"
#include "periph_bmi088.h"
#include "periph_remote.h"
#include "periph_referee.h"
#include "periph_servo.h"
#include "module_gimbal.h"
#include "module_shoot.h"
#include "module_chassis.h"
#include "module_referee.h"
#include "app_ins.h"
#include "app_remote.h"
#include "protocol_common.h"


void Init_InitAll() {
    DWT_Init(168);

    BMI088_Init(0);
    Remote_InitRemote();

    // Ins init
    INS_Init();

    // Motor Group init
	Can_InitFilterAndStart(&hcan1);
	Can_InitFilterAndStart(&hcan2);
    Motor_InitAllMotors();

    Referee_InitReferee();
    Servo_InitAllServos();
	

  	Referee_Setup();  
	GimbalPitch_InitGimbalPitch();
	GimbalYaw_InitGimbalYaw();
    Shooter_InitShooter();
		Chassis_InitChassis();
    Remote_RemotrControlInit();
    Protocol_InitProtocol();
}


void Init_Task(void const * argument) {
    SoftTimer_StartAll();
    forever {
        vTaskSuspend(Init_TaskHandleHandle);
      osDelay(25);
    }
}
