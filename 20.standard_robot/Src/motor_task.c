
#include "struct_typedef.h"
#include "main.h"
#include "cmsis_os.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "pid.h"
#include "CAN_Receive.h"


#define MOTOR_SPEED_PID_KP 1.0f
#define MOTOR_SPEED_PID_KI 0.0f
#define MOTOR_SPEED_PID_KD 0.0f

#define MOTOR_SPEED_PID_MAX_OUT  20000.0f
#define MOTOR_SPEED_PID_MAX_IOUT 5000.0f

#define MOTOR_ANGLE_PID_KP 1.0f
#define MOTOR_ANGLE_PID_KI 0.0f
#define MOTOR_ANGLE_PID_KD 0.0f

#define MOTOR_ANGLE_PID_MAX_OUT  20000.0f
#define MOTOR_ANGLE_PID_MAX_IOUT 5000.0f

const static fp32 motor_speed_pid[3] = { MOTOR_SPEED_PID_KP, MOTOR_SPEED_PID_KI ,MOTOR_SPEED_PID_KD};

const static fp32 motor_angle_pid[3] = { MOTOR_ANGLE_PID_KP, MOTOR_ANGLE_PID_KI ,MOTOR_ANGLE_PID_KD};

pid_type_def motorpid_angle  ;
pid_type_def motorpid_speed  ;

int16_t rpm =400;
				 
int16_t current = 0; 

int16_t gm6020_current = 10000;

extern const motor_measure_t *motor_measure;

const motor_measure_t *motor_measure2;

static int16_t motor_can_set_current = 0 ;

void Motor_task(void const * argument)
{
		motor_measure = get_chassis_motor_measure_point(0);

		motor_measure2 = get_yaw_gimbal_motor_measure_point();
	
		PID_init(&motorpid_angle,PID_POSITION,motor_angle_pid,MOTOR_ANGLE_PID_MAX_OUT,MOTOR_ANGLE_PID_MAX_IOUT);
		PID_init(&motorpid_speed,PID_POSITION,motor_speed_pid,MOTOR_SPEED_PID_MAX_OUT,MOTOR_SPEED_PID_MAX_IOUT);
	
	
    while(1)
    {
					
					//current = PID_calc(&motorpid_speed,motor_measure->speed_rpm,rpm);
					//motor_can_set_current =PID_calc(&motorpid_speed,motor_measure->speed_rpm,current);
					//CAN_cmd_gimbal(gm6020_current,gm6020_current,0,0);
			
			   //CAN_cmd_chassis(current,0,0,0);
			
				 osDelay(100);
			
	 
	
		}	
	

}
		


