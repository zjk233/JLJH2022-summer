#include "decode_camp.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol_camp.h"
#include "fifo.h"
#include "cmsis_os.h"

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
extern void append_CRC16_check_sum(uint8_t * pchMessage,uint32_t dwLength);
void encode_send_data(uint16_t cmd_id, void* buf, uint16_t len);
/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
extern void append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);
  
//USB�ײ㷢�ͺ�����ֱ�Ӳ���Ӳ��
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

chassis_ctrl_info_t chassis_ctrl;

chassis_odom_info_t chassis_odom; 

extern QueueHandle_t CDC_send_queue;

taurus_end_info taurus_end;

void rm_queue_data(uint16_t cmd_id,void* buf,uint16_t len ) //uint8_t queue_data[128];
{
   	
	  uint16_t index = 0;
	 uint8_t queue_data[128];
	  memcpy(queue_data,  (void*)&cmd_id, sizeof(uint16_t));
	  index +=sizeof(uint16_t);
	  
	  memcpy(queue_data + index, (void*)buf, len);
    index += len;
	
	  xQueueSend(CDC_send_queue, queue_data, 10);
}

void rm_dequeue_send_data(void* buf,uint16_t len)
{
	uint16_t cmd_id;
	memcpy(&cmd_id,buf,sizeof(uint16_t));
	
	switch(cmd_id)
	{
		case CHASSIS_ODOM_CMD_ID:  //��Ҫ���͵����ݰ�ID��
		{
			  encode_send_data(CHASSIS_ODOM_CMD_ID,((uint8_t*)buf+2),sizeof(chassis_odom_info_t));
		}
		break;
		case MESSAGE_ID:
		{
			  encode_send_data(MESSAGE_ID,((uint8_t*)buf+2),sizeof(message_info_t));
		}
		break;
		case RC_ID:
		{
			 encode_send_data(RC_ID,((uint8_t*)buf+2),sizeof(rc_info_t));
		}
		break;
	}
	
}

//ʵ��RMЭ������л�����
void encode_send_data(uint16_t cmd_id, void* buf, uint16_t len)
{
    
		taurus_end.end1 = END1_SOF;
		taurus_end.end2 = END2_SOF;
		//TODO ��������128�ֽڴ�С��������
    static uint8_t send_buf[128];
    uint16_t index = 0;
    //TODO ����֡ͷ�ṹ��
    frame_header_struct_t referee_send_header;
    
    //TODO ��ʼ����Ӧ֡ͷ�ṹ��
    referee_send_header.SOF = HEADER_SOF;
    referee_send_header.data_length = len;
    referee_send_header.seq++;
    
    //TODO ����CRC8У��
    append_CRC8_check_sum((uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
    
    memcpy(send_buf, (uint8_t*)&referee_send_header, sizeof(frame_header_struct_t));
    index += sizeof(frame_header_struct_t);

    memcpy(send_buf + index, (void*)&cmd_id, sizeof(uint16_t));
    index += sizeof(uint16_t);

    //TODO ������ݰ�
    memcpy(send_buf + index, (void*)buf, len);
    index += len;

    //TODO ����CRC16У��
    append_CRC16_check_sum(send_buf, REF_HEADER_CRC_CMDID_LEN + len);
    index += sizeof(uint16_t);
    
		memcpy(send_buf + index,(void*)&taurus_end,sizeof(taurus_end_info));
		index += sizeof(taurus_end_info);
		
   // xQueueSend(CDC_send_queue, send_buf, 10);
  
    //TODO ���õײ㷢�ͺ���

    CDC_Transmit_FS(send_buf, index);

}





