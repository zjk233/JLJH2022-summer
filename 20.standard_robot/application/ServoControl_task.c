/**2022疆来计划_进阶二组_嵌入式程序
*功能：在standard_robot架构下实现舵机控制
	*编写者：我不是张锦昆 编写日期：2022.7.27
	*版本：V1.2
	*备注：ros上位机通过usb控制的代码会在2.0版本中加入
  */

#include "ServoControl_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"

#define SERVO_MIN_PWM   500
#define SERVO_MAX_PWM   2500


uint16_t servo_PwmOutput = 1500;


void ServoControl_task(void const * argument)
{


    while(1)
    {
        servo_pwm_set(servo_PwmOutput, 0);
        osDelay(500);
			  servo_pwm_set((servo_PwmOutput + 500),0); //通道0对应的C6
			  osDelay(500);
    }
}


