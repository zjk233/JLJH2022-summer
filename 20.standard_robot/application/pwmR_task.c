#include "bsp_servo_pwm.h"
#include "cmsis_os.h"
#include "pwmR_task.h"

void pwmR_task(void const * argument){
while(1){
servo_pwm_set(500,3);
osDelay(600);
servo_pwm_set(2500,3);
osDelay(600);
}


}