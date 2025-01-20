/*
 *  Project      : Polaris
 * 
 *  file         : app_gimbal.c
 *  Description  : This file contains Gimbal Pitch control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 13:13:54
 */


#include "app_gimbal.h"
#include "module_gimbal.h"
#include "app_autoaim.h"

/**
  * @brief          Gimbal task
  * @param          NULL
  * @retval         NULL
  */
void Gimbal_Task(void const * argument) {

    for(;;) {
			  MyUART_Init();
			  SendVisionData(&visionDataSend);
			  Tidy_send_vision(&visionDataSend);
        GimbalPitch_Control();
			  GimbalYaw_Control();
        GimbalPitch_Output();
      osDelay(2);
    }
}
