#include "6020AngleControl.h"
#include "CAN_receive.h"
#include "pid.h"
#include "cmsis_os.h"
#include "math.h"
#include "up_3508pid_task.h"


#define GM6020_PID_KP 0.15f
#define GM6020_PID_KI 0.02f
#define GM6020_PID_KD 0.0f
#define GM6020_PID_MaxOut 270
#define GM6020_PID_iMaxOut 270

#define GM6020_PID_KP_2 0.15f
#define GM6020_PID_KI_2 0.02f
#define GM6020_PID_KD_2 0.0f           //两个都已调好，勿动
#define GM6020_PID_MaxOut_2 300
#define GM6020_PID_iMaxOut_2 300

#define GM6020_PID_speed_KP 7.685f
#define GM6020_PID_speed_KI 0.0f
#define GM6020_PID_speed_KD 0.4745f
#define GM6020_PID_speed_MaxOut 30000
#define GM6020_PID_speed_iMaxOut 30000      //勿动，已调好

#define GM6020_PID_KP_speed_2 6.455f
#define GM6020_PID_KI_speed_2 0.0f
#define GM6020_PID_KD_speed_2 0.8425
#define GM6020_PID_MaxOut_speed_2 30000
#define GM6020_PID_iMaxOut_speed_2 30000  //电推杆6020，参数已调好

/**2022疆来计划_进阶二组_嵌入式程序
  *功能：在standard_robot架构下实现6020电机位置pid控制
	*编写者：我不是张锦昆 编写日期：2022.7.30
	*版本：V1.2
	*备注：6020的电调id应为2,双环pid很牛逼
  */

int voltage_6020 = 0;
const motor_measure_t *motor_measure_6020;
float e_angle_6020;
float angle_6020;//-4095到+4095
float e_speed_6020 = 100;

int voltage_6020_2 = 0;
const motor_measure_t *motor_measure_6020_2;
float e_angle_6020_2;
float angle_6020_2;//-4095到+4095
float e_speed_6020_2 = 100;

pid_type_def pid_6020;
const static fp32 pid6020[3] = { GM6020_PID_KP, GM6020_PID_KI ,GM6020_PID_KD}; //pid初始化
pid_type_def pid_6020_speed;
const static fp32 pid6020_speed[3] = { GM6020_PID_speed_KP, GM6020_PID_speed_KI ,GM6020_PID_speed_KD}; //pid初始化

pid_type_def pid_6020_2;
const static fp32 pid6020_2[3] = { GM6020_PID_KP_2, GM6020_PID_KI_2 ,GM6020_PID_KD_2}; //pid初始化
pid_type_def pid_6020_speed_2;
const static fp32 pid6020_speed_2[3] = { GM6020_PID_KP_speed_2, GM6020_PID_KI_speed_2 ,GM6020_PID_KD_speed_2}; //pid初始化

void GM6020_control(){
	if(motor_measure_6020->ecd > 4096){
	angle_6020 = motor_measure_6020->ecd - 8191;  //反转
	e_speed_6020 = PID_calc(&pid_6020,angle_6020,e_angle_6020);
//	CAN_cmd_gimbal(0,voltage_6020,0,0);
	}else{
	angle_6020 = motor_measure_6020->ecd;
	e_speed_6020 = PID_calc(&pid_6020,angle_6020,e_angle_6020);
//	CAN_cmd_gimbal(0,voltage_6020,0,0);
	}
}

void GM6020_control_2(){
	if(motor_measure_6020_2->ecd > 4096){
	angle_6020_2 = motor_measure_6020_2->ecd - 8191;  //反转
	e_speed_6020_2 = PID_calc(&pid_6020_2,angle_6020_2,e_angle_6020_2);
//	CAN_cmd_gimbal(0,0,voltage_6020_2,0);
	}else{
	angle_6020_2 = motor_measure_6020_2->ecd;
	e_speed_6020_2 = PID_calc(&pid_6020_2,angle_6020_2,e_angle_6020_2);
//	CAN_cmd_gimbal(0,0,voltage_6020_2,0);
	}
}
void GM6020_SpeedPID(){
voltage_6020 = PID_calc(&pid_6020_speed,motor_measure_6020->speed_rpm*10,e_speed_6020*20); 
voltage_6020_2 = PID_calc(&pid_6020_speed_2,motor_measure_6020_2->speed_rpm*10,e_speed_6020_2*20);   
}

void motor_CanOutput_all(){ //pid计算结果发送函数
GM6020_SpeedPID();
GM6020_control();
GM6020_control_2();
CAN_cmd_gimbal(0,voltage_6020,voltage_6020_2,0);
}

void GM6020_init(){
	motor_measure_6020 = get_pitch_gimbal_motor_measure_point();
  PID_init(&pid_6020, PID_POSITION, pid6020,GM6020_PID_MaxOut, GM6020_PID_iMaxOut); 
  PID_init(&pid_6020_speed, PID_POSITION, pid6020_speed,GM6020_PID_speed_MaxOut, GM6020_PID_speed_iMaxOut); 
}
void GM6020_init_2(){
	motor_measure_6020_2 = get_trigger_motor_measure_point();
  PID_init(&pid_6020_2, PID_POSITION, pid6020_2,GM6020_PID_MaxOut_2, GM6020_PID_iMaxOut_2); 
  PID_init(&pid_6020_speed_2, PID_POSITION, pid6020_speed_2,GM6020_PID_MaxOut_speed_2, GM6020_PID_iMaxOut_speed_2); 
}