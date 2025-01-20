/*
 *  Project      : Polaris
 * 
 *  file         : app_remote.c
 *  Description  : This file contains Remote control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-07 11:26:15
 */


#include "sys_const.h"
#include "protocol_common.h"
#include "app_remote.h"
#include "app_referee.h"
#include "module_shoot.h"
#include "module_gimbal.h"
#include "module_chassis.h"
#include "module_referee.h"
#include "periph_servo.h"
#include "cmsis_os.h"
#include "app_autoaim.h"
#define REMOTE_TASK_PERIOD  1
#define ENCODER_LIMIT 500
Remote_RemoteControlTypeDef Remote_remoteControlData;
Math_SlopeParamTypeDef Remote_ChassisFBSlope;
float last_encoder_angle=0;
float encoder_angle=0;
float get_abs(float a){
    if(a>=0){
		return a;
		}
		else return(0-a);

}
float limit_siqu(float last_angle,float angle){
       if(get_abs((int16_t)(angle-last_angle))<=ENCODER_LIMIT){ 
			 return last_angle;}
	     else{
			 return angle;
			 }

}

/**
  * @brief          Remote task
  * @param          NULL
  * @retval         NULL
  */
void Remote_Task(void const * argument) {

    forever {
        Remote_ControlCom();
      osDelay(REMOTE_TASK_PERIOD);
    }
}


/**
  * @brief      Remote Control Init
  * @param      NULL
  * @retval     NULL
  */
void Remote_RemotrControlInit() {
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    
    Math_InitSlopeParam(&Remote_ChassisFBSlope, MOUSE_CHASSIS_ACCELERATE, MOUSE_CHASSIS_ACCELERATE);
}


/**
  * @brief      Gets the pointer to the remote control data object
  * @param      NULL
  * @retval     Pointer to remote control data object
  */
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr() {
    return &Remote_remoteControlData;
}


/**
* @brief      Remote control command
* @param      NULL
* @retval     NULL
*/
Remote_RemoteDataTypeDef *testdata;
void Remote_ControlCom() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
		testdata = data;
    control_data->pending = 1;

    switch (data->remote.s[0]) {
    /*      right switch control mode   */
        case Remote_SWITCH_UP: {
            /* right switch up is remote normal mode */
            Remote_RemoteProcess();
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            /* right switch mid is keymouse mode    */
            Remote_RemoteShooterModeSet();
            break;
        }
        case Remote_SWITCH_DOWN: {
            /* right switch down is auto aim mode   */
            Remote_KeyMouseProcess();
            Remote_MouseShooterModeSet();
            break;
        }
        default:
            break;
    }

    control_data->pending = 0;
}


/**
* @brief      Mouse shoot mode set
* @param      NULL
* @retval     NULL
*/
int test_count;
void Remote_MouseShooterModeSet() {
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();

    // Prevent launching without opening the friction wheel
//    if ((shooter->shooter_mode != Shoot_REFEREE) || (fabs(Motor_ShootLeftMotor.encoder.speed) <= 30) || (fabs(Motor_ShootRightMotor.encoder.speed) <= 30)) {
//        Shooter_ChangeFeederMode(Feeder_FINISH);
//        return;
//    }
    if ((fabs(Motor_ShootLeftMotor.encoder.speed) <= 30) || (fabs(Motor_ShootRightMotor.encoder.speed) <= 30)) {
        Shooter_ChangeFeederMode(Feeder_FINISH);
        return;
    }

    static int count_mouse_L = 0;
    if (data->mouse.l == 1) {
        count_mouse_L++;
        if (count_mouse_L >= 50) {
            Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
            count_mouse_L = 50;
        }
    }
    else {
        if (0 < count_mouse_L && count_mouse_L < 50) {
            Shooter_SingleShootReset();
            Shooter_ChangeFeederMode(Feeder_SINGLE);
        }
        else Shooter_ChangeFeederMode(Feeder_FINISH);
        count_mouse_L = 0;
    }
		
		test_count = count_mouse_L;
}


/**
* @brief      Remote shoot mode set
* @param      NULL
* @retval     NULL
*/
void Remote_RemoteShooterModeSet() {


		Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
		Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
		

    switch (data->remote.s[1]) {
    /*      left switch control mode   */
        case Remote_SWITCH_UP: {
            /* left switch up is fast shooting */
            Shooter_ChangeShooterMode(Shoot_NULL);
            Shooter_ChangeFeederMode(Feeder_FINISH);
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            /* left switch mid is stop shooting    */
            Shooter_ChangeShooterMode(Shoot_FAST);
            Shooter_ChangeFeederMode(Feeder_FINISH);
					  
            break;
        }
        case Remote_SWITCH_DOWN: {
            /* left switch down is slow shooting   */
            Shooter_ChangeShooterMode(Shoot_FAST);
            Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
//            if ((PID_GetPIDFdb(&shooter->shootLeftPID) >= 30) && (PID_GetPIDFdb(&shooter->shootRightPID) <= -30)) {
//                Shooter_ChangeFeederMode(Feeder_REFEREE);
//            }
//            else Shooter_ChangeFeederMode(Feeder_FINISH);
            break;
        }
        default:
            break;
    }
		
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
		gimbalpitch->output_state = 1;
		gimbalyaw->output_state = 1;
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
		GimbalYaw_SetYawRef(buscomm->yaw_ref);
    float pitch_ref;
    pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF+ (float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
    GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));
		
		Chassis_SetChassisMode(Chassis_SEP);
		Chassis_SetChassisRef((float)data->remote.ch[1]  , (float)data->remote.ch[0] , 0);
		
		
}


/**
* @brief      Remote control process
* @param      NULL
* @retval     NULL
*/
void Remote_RemoteProcess() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();

    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
		gimbalpitch->output_state = 0;
		gimbalyaw->output_state = 0;
		
    switch (data->remote.s[1]) {
    /*      left switch control mode   */
        case Remote_SWITCH_UP: {
	           Chassis_SetChassisMode(Chassis_SEP);
					   Chassis_SetChassisRef(get_target.vx,get_target.vy,get_target.vz);
					   //Chassis_SetChassisRef()
            break;
        }
        case Remote_SWITCH_MIDDLE: {
	          gimbalpitch->output_state = 1;
	          gimbalyaw->output_state = 1;
	          Chassis_SetChassisMode(Chassis_SEP);
					last_encoder_angle=encoder_angle;
					encoder_angle=(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET);
	          Chassis_SetChassisRef((float)data->remote.ch[1]  , (float)data->remote.ch[0] ,(float)data->remote.ch[2]); //(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));
            					  buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
		GimbalYaw_SetYawRef(buscomm->yaw_ref);
    float pitch_ref;
    pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF; //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
    GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));  
					break;
        }
        case Remote_SWITCH_DOWN: {
            /* left switch down is slow shooting   */
	          gimbalpitch->output_state = 1;
	          gimbalyaw->output_state = 1;
		        Chassis_SetChassisYawAngle(Motor_YawMotor.encoder.limited_angle,CHASSIS_YAW_ANGLE_OFFSET);
		        Chassis_SetChassisMode(Chassis_XTL);
		        Chassis_SetChassisRef((float)data->remote.ch[1]  , (float)data->remote.ch[0] , CHASSIS_XTL_WZ);
						buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
		GimbalYaw_SetYawRef(buscomm->yaw_ref);
    float pitch_ref;
    pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF; //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
    GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));
							
            break;
        }
        default:
            break;
    }
	
    
}


/**
* @brief      KeyMouse control process
* @param      NULL
* @retval     NULL
*/
void Remote_KeyMouseProcess() { 
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    GimbalPitch_GimbalPitchTypeDef *gimbal = GimbalPitch_GetGimbalPitchPtr();
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    
	
		//chassis control
		float chassis_vx;
		float chassis_vy;
		
		if(data->key.w == 1)
		{
			chassis_vx = 200.0f;
		}	
		else if(data->key.s == 1)
		{
			chassis_vx = -200.0f;
		}
		else
		{
			chassis_vx = 0.0f;
		}
		
		if(data->key.a == 1)
		{
			chassis_vy = -200.0f;
		}
		else if(data->key.d == 1)
		{
			chassis_vy = 200.0f;
		}
		else
		{
			chassis_vy = 0.0f;
		}
		
		if(data->key.shift == 1)
		{
		  Chassis_SetChassisYawAngle(Motor_YawMotor.encoder.limited_angle,CHASSIS_YAW_ANGLE_OFFSET);
		  Chassis_SetChassisMode(Chassis_XTL);
		  Chassis_SetChassisRef(chassis_vx  , chassis_vy , CHASSIS_XTL_WZ);
		}
		else if(data->key.shift == 0)
		{
			Chassis_SetChassisMode(Chassis_FOLLOW);
			Chassis_SetChassisRef(chassis_vx  , chassis_vy , (float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));	
		}

		if(data->key.q == 1)
		{
			Servo_SetServoAngle(&Servo_ammoContainerCapServo, 90);
		}
		else if(data->key.q == 0)
		{
			Servo_SetServoAngle(&Servo_ammoContainerCapServo, 0);
		}
	
		
		//autoaim control
		float autoaim_yaw;
		float autoaim_pitch;
		if(data->mouse.r == 1)
		{
			autoaim_yaw = (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f;
			autoaim_pitch = (float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
		}
		else if(data->mouse.r == 0)
		{
			autoaim_yaw = 0.0f;
			autoaim_pitch = 0.0f;
		}
		
		
		//gimbal control
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
		gimbalpitch->output_state = 1;
		gimbalyaw->output_state = 1;
    buscomm->yaw_ref += Gimbal_LimitYaw((float)data->mouse.x * -MOUSE_YAW_ANGLE_TO_FACT + autoaim_yaw);
		GimbalYaw_SetYawRef(buscomm->yaw_ref);
    float pitch_ref;
    pitch_ref = ((float)data->mouse.y * -MOUSE_PITCH_ANGLE_TO_FACT - autoaim_pitch);
    GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));

		//shoot control(fric)
    if (data->key.f == 1)      
        Shooter_ChangeShooterMode(Shoot_FAST);
    if (data->key.g == 1)      
        Shooter_ChangeShooterMode(Shoot_NULL);

}
