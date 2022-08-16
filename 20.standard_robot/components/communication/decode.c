#include "decode_camp.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol_camp.h"
#include "fifo.h"
#include "cmsis_os.h"
#include "bsp_usart.h"

extern TIM_HandleTypeDef htim5;

//USB接收FIFO初始化
void usb_fifo_init(void);

void decode_task(void const * argument);
	
 extern chassis_ctrl_info_t chassis_ctrl;//底盘控制

 extern chassis_odom_info_t chassis_odom; //姿态信息
 
 
 message_info_t message_info;  //消息结构体
 
 servo_info_t servo_info;
 
 summer_camp_info_t summer_camp_info;//裁判系统消息
 
//USB FIFO控制结构体
fifo_s_t usb_fifo;
//USB FIFO环形缓存区
uint8_t usb_fifo_buf[512];
//RM协议解包控制结构体
unpack_data_t decode_unpack_obj;
//RM协议反序列化函数
void decode_unpack_fifo_data(void);

uint16_t decode_data_solve(uint8_t *frame);

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

//RM协议序列化函数
void encode_send_data(uint16_t cmd_id, void* buf, uint16_t len);

frame_header_struct_t decode_receive_header;


void decode_task(void const * argument);

//RM协议解析函数，系统自动调用
void decode_task(void const * argument)
{
    usb_fifo_init();
   //xTaskCreate((TaskFunction_t)referee_send_task, "send_task", 512, NULL, 0, &referee_send_handle);
   // referee_send_queue = xQueueCreate(5, REF_HEADER_CRC_CMDID_LEN + sizeof(communicate_class_output_data_t));

    while(1)
    {
      decode_unpack_fifo_data();
      osDelay(2);
    }
}


//USB FIFO初始化函数
void usb_fifo_init(void)
{
  fifo_s_init(&usb_fifo, usb_fifo_buf, 512);
}


//USB接收中断
void usb_receiver(uint8_t *buf, uint32_t len)
{
  fifo_s_puts(&usb_fifo, (char*)buf, len);
}


//RM协议反序列化
void decode_unpack_fifo_data(void)
{
  uint8_t byte = 0;
  uint8_t sof = HEADER_SOF;
  unpack_data_t *p_obj = &decode_unpack_obj;

  while ( fifo_s_used(&usb_fifo) )
  {
    byte = fifo_s_get(&usb_fifo);
    switch(p_obj->unpack_step)
    {
      //查找帧头
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
          p_obj->unpack_step = STEP_LENGTH_LOW;
          p_obj->protocol_packet[p_obj->index++] = byte;
        }
        else
        {
          p_obj->index = 0;
        }
      }break;
      
      //获取数据长度低字节
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      //获取数据长度高字节
      case STEP_LENGTH_HIGH:
      {
        p_obj->data_len |= (byte << 8);
        p_obj->protocol_packet[p_obj->index++] = byte;

        if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_obj->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;
        }
      }break;
    
      //记录协议包序列号
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      //校验帧头CRC8
      case STEP_HEADER_CRC8:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;

        if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
          {
            p_obj->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
          }
        }
      }break;  
      
      //校验整帧CRC16
      case STEP_DATA_CRC16:
      {
        if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
           p_obj->protocol_packet[p_obj->index++] = byte;  
        }
        if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
        {
          p_obj->unpack_step = STEP_HEADER_SOF;
          p_obj->index = 0;

          if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
          {
            //成功解到一个正确的信息包
            decode_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      //解包失败重新寻找帧头
      default:
      {
        p_obj->unpack_step = STEP_HEADER_SOF;
        p_obj->index = 0;
      }break;
    }
  }
}

uint16_t decode_data_solve(uint8_t *frame)
{
    uint8_t index = 0;
    uint16_t cmd_id = 0;

    memcpy(&decode_receive_header, frame, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);

    memcpy(&cmd_id, frame + index, sizeof(uint16_t));
    index += sizeof(uint16_t);
   
	switch (cmd_id)
    {
        //接收控制码对应信息包
        case CHASSIS_CTRL_CMD_ID:
        {
            memcpy(&chassis_ctrl, frame + index, sizeof(chassis_ctrl_info_t));
            // memcpy(&c, frame + index, sizeof(communicate_class_input_data_t));
            //communicate_class_solve();
            break;
        }
				case GAME_STATUS_CMD_ID:
				{
						memcpy(&summer_camp_info, frame + index, sizeof(summer_camp_info_t));
					  encode_send_data(GAME_STATUS_CMD_ID, &summer_camp_info, sizeof(summer_camp_info_t));
						break;
				}
				case MESSAGE_ID:
				{
					  memcpy(&message_info, frame + index, sizeof(message_info_t));
					
						break;
				}
				case SERVO_ID:
				{
					memcpy(&servo_info,frame + index,sizeof(servo_info_t));
				
					break;
				}
				
        default:
        {
            break;
        }
    }

		
    index += decode_receive_header.data_length + 2;
    return index;
}
