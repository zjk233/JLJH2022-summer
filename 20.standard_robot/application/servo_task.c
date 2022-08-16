/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"
#include "protocol_camp.h"

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500

#define PWM_DETAL_VALUE 10

#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Z
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V

#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT


extern servo_info_t servo_info; 
	
const RC_ctrl_t *servo_rc;
const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};
uint16_t servo_pwm[4] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          �������
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_task(void const * argument)
{
    //servo_rc = get_remote_control_point();

    while(1)
    {
        for(uint8_t i = 0; i < 3; i++)
        {

//            if( (servo_rc->key.v & SERVO_MINUS_PWM_KEY) && (servo_rc->key.v & servo_key[i]))
//            {
//                servo_pwm[i] -= PWM_DETAL_VALUE;
//            }
//            else if(servo_rc->key.v & servo_key[i])
//            {
//                servo_pwm[i] += PWM_DETAL_VALUE;
//            }

						 servo_pwm[0] = servo_info.pwm1; //进球舵机 500是初始 1400是下压
						 servo_pwm[1] = servo_info.pwm2;
             servo_pwm[2] = servo_info.pwm3; 
						
            //limit the pwm
            if(servo_pwm[i] < SERVO_MIN_PWM)
            {
                servo_pwm[i] = SERVO_MIN_PWM;
            }
            else if(servo_pwm[i] > SERVO_MAX_PWM)
            {
                servo_pwm[i] = SERVO_MAX_PWM;
            }

            servo_pwm_set(servo_pwm[i], i);
        }
//        servo_pwm_set(500,3);
//        osDelay(600);
//        servo_pwm_set(2500,3);
        osDelay(10);
    }
}


