#include "up_3508pid_task.h"
#include "pid.h"
#include "cmsis_os.h"
#include "CAN_Receive.h"
#include "main.h"
#include "6020AngleControl.h"

#define up3508_PID_KP 5.705f
#define up3508_PID_KI 0.8f
#define up3508_PID_KD 0.33f
#define up3508_PID_MaxOut 20000
#define up3508_PID_iMaxOut 16384

/**2022疆来计划_进阶二组_嵌入式程序
  *功能：在standard_robot架构下实现3508电机驱动及pid控速
	*编写者：我不是张锦昆 编写日期：2022.7.27
	*版本：V1.2
	*备注：3508的电调id应为0
  */

/**
  * @brief          3508 pid task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          3508 pid task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
int16_t rpm_3508 =2000;			 //定义转速
int16_t current_3508 = 0;    
const motor_measure_t *motor_measure; //声明一个指针指向can接收的那个结构体
int16_t debug_3508;


pid_type_def pid_3508;
const static fp32 pid3508[3] = { up3508_PID_KP, up3508_PID_KI ,up3508_PID_KD}; //pid初始化
	

	void up3508_pid_task(void const * argument){
		motor_measure = get_chassis_motor_measure_point(0);   //设置一个结构体获取can线上id0的电机参数		
		PID_init(&pid_3508, PID_POSITION, pid3508,up3508_PID_MaxOut, up3508_PID_iMaxOut); 

		while(1){
		current_3508 = PID_calc(&pid_3508,motor_measure->speed_rpm,rpm_3508); //第二个参数实际值为传指针
		debug_3508 = motor_measure->speed_rpm;
    CAN_cmd_chassis(current_3508,0,0,0);
		osDelay(5);
		}


}
