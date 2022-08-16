#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

#include "struct_typedef.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#define END1_SOF 0x0D
#define END2_SOF 0x0A

#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

//RM协议内置命令码   //发送的ID号
typedef enum
{
  GAME_STATUS_CMD_ID = 0x0001,
  CHASSIS_ODOM_CMD_ID = 0x0101,
  CHASSIS_CTRL_CMD_ID = 0x0102,
	MESSAGE_ID = 0x0103,
	SERVO_ID = 0x0104,
	RC_ID  = 0x0105
	
} referee_data_cmd_id_tpye;

//RM协议帧头结构体
typedef  struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

//RM协议反序列化步骤枚举
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

//RM协议反序列化结构体
typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;

typedef struct
{
	uint8_t end1;
	uint8_t end2;
} taurus_end_info ;


/***************以下通信信息包多机协同课程暂未使用****************/
//RM夏令营比赛服务器信息包
typedef struct
{
  struct castle_energy
  {
    uint8_t energy[2];
  }castle_energy[7];

  struct region_occupy
  {
    uint8_t status : 2; // 0 = no one, 1 = weak, 2 = strong
    uint8_t belong : 2; // 0 = no one, 1 = red, 2 = blue
    uint8_t have_robot : 2; // same as belong // resv
    uint8_t resv : 2;  
  }region_occupy[9][7];

  uint8_t car_location[2];
  uint8_t round_remain_tick;
  uint8_t round_remain_cnt:7;
  uint8_t round_team : 1;
  int16_t realtime_score[2];
	
} summer_camp_info_t;

//底盘速度控制信息包
typedef struct
{
  float vx;
  float vy;
  float vw;
	
}  chassis_ctrl_info_t;

//里程计反馈数据
typedef struct
{
  
	float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vw;
	float gyro_z;
	float gyro_yaw;
	
}  chassis_odom_info_t;

//信息数据
typedef struct
{
	uint16_t num1;
  uint16_t num2;
	uint16_t num3;
	float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vw;
	float gyro_z;
	float gyro_yaw;
	
} message_info_t;

//舵机控制数据
typedef struct
{
	uint16_t pwm1;
	uint16_t pwm2;
  uint16_t pwm3;
  uint16_t angle1;
  uint16_t angle2;
  uint16_t fpga;
	
} servo_info_t;


//遥控控制数据
typedef struct
{
	
	int16_t ch[5];
	char s[2];
	
} rc_info_t;


#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
