#include "RelayControl_task.h"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "tim.h"
#include "protocol_camp.h"
#include "CAN_receive.h"

int relay_output,relay_output_last;
extern TIM_HandleTypeDef htim1;
extern servo_info_t servo_info; 
extern const motor_measure_t *motor_measure_6020_2;
extern float e_angle_6020_2;
extern float angle_6020_2;

void RelayContorl_task(void const * argument){
  relay_output = 0;
  relay_output_last = 0;
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
  while(1){
  relay_output = servo_info.fpga;
  if(relay_output_last != servo_info.fpga){
  relay_output_last = relay_output;
  while(!(angle_6020_2 <= e_angle_6020_2+5 && angle_6020_2 >= e_angle_6020_2-5)){ //等电机转到位了以后电磁铁再弹出去
        osDelay(1);
        }
   __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 19999);   
   osDelay(200);
   __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0); 
  }
 } 
}

