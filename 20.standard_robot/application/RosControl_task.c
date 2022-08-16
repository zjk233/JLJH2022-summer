#include "RosControl_task.h"
#include "6020AngleControl.h"
#include "cmsis_os.h"
#include "protocol_camp.h"
#include "gpio.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "bsp_servo_pwm.h"
#include "CAN_receive.h"

//int cao;

extern TIM_HandleTypeDef htim1;
extern servo_info_t servo_info; 
extern const motor_measure_t *motor_measure_6020;
void MG6020_ros_control(){
  switch(servo_info.angle1){
  case 1:
  e_angle_6020 = 1525;
  break;
  case 2:
  e_angle_6020 = 3150;
  break;
  case 3:
  e_angle_6020 = -3500;
  break;
  case 4:
  e_angle_6020 = -1750;
  break;
  case 5:
  e_angle_6020 = 0;
  break;
  }
}

void MG6020_ros_control_2(){
  switch(servo_info.angle2){
  case 1:
  e_angle_6020_2 = 2400;
  break;
  case 2:
  e_angle_6020_2 = 3950;
  break;
  case 3:
  e_angle_6020_2 = -2600;
  break;
  case 4:
  e_angle_6020_2 = -950;
  break;
  case 5:
  e_angle_6020_2 = 700;
  break;
  }
}



void RosControl_task(void const * argument){

  GM6020_init();
  GM6020_init_2();
	
	while(1){
  MG6020_ros_control();
  MG6020_ros_control_2();

  motor_CanOutput_all();
//  if(motor_measure_6020->speed_rpm == 0 ){
//  
//  }else{
//  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 0);
//   }

  osDelay(5);
	}

}

