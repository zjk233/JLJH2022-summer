#include "decode_camp.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol_camp.h"
#include "fifo.h"
#include "cmsis_os.h"
#include "bsp_usart.h"

extern TIM_HandleTypeDef htim5;

//USB����FIFO��ʼ��
void usb_fifo_init(void);

void decode_task(void const * argument);
	
 extern chassis_ctrl_info_t chassis_ctrl;//���̿���

 extern chassis_odom_info_t chassis_odom; //��̬��Ϣ
 
 
 message_info_t message_info;  //��Ϣ�ṹ��
 
 servo_info_t servo_info;
 
 summer_camp_info_t summer_camp_info;//����ϵͳ��Ϣ
 
//USB FIFO���ƽṹ��
fifo_s_t usb_fifo;
//USB FIFO���λ�����
uint8_t usb_fifo_buf[512];
//RMЭ�������ƽṹ��
unpack_data_t decode_unpack_obj;
//RMЭ�鷴���л�����
void decode_unpack_fifo_data(void);

uint16_t decode_data_solve(uint8_t *frame);

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

//RMЭ�����л�����
void encode_send_data(uint16_t cmd_id, void* buf, uint16_t len);

frame_header_struct_t decode_receive_header;


void decode_task(void const * argument);

//RMЭ�����������ϵͳ�Զ�����
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


//USB FIFO��ʼ������
void usb_fifo_init(void)
{
  fifo_s_init(&usb_fifo, usb_fifo_buf, 512);
}


//USB�����ж�
void usb_receiver(uint8_t *buf, uint32_t len)
{
  fifo_s_puts(&usb_fifo, (char*)buf, len);
}


//RMЭ�鷴���л�
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
      //����֡ͷ
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
      
      //��ȡ���ݳ��ȵ��ֽ�
      case STEP_LENGTH_LOW:
      {
        p_obj->data_len = byte;
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_LENGTH_HIGH;
      }break;
      
      //��ȡ���ݳ��ȸ��ֽ�
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
    
      //��¼Э������к�
      case STEP_FRAME_SEQ:
      {
        p_obj->protocol_packet[p_obj->index++] = byte;
        p_obj->unpack_step = STEP_HEADER_CRC8;
      }break;

      //У��֡ͷCRC8
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
      
      //У����֡CRC16
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
            //�ɹ��⵽һ����ȷ����Ϣ��
            decode_data_solve(p_obj->protocol_packet);
          }
        }
      }break;

      //���ʧ������Ѱ��֡ͷ
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
        //���տ������Ӧ��Ϣ��
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
