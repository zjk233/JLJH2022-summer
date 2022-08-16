/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       led_trigger_task.c/h
  * @brief      led RGB show.led RGBµÆÐ§¡£
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. rgb led
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "led_flow_task.h"
#include "bsp_led.h"
#include "cmsis_os.h"
#include "main.h"

#include "protocol_camp.h"

#define RGB_FLOW_COLOR_CHANGE_TIME  1000
#define RGB_FLOW_COLOR_LENGHT   6
//blue-> green(dark)-> red -> blue(dark) -> green(dark) -> red(dark) -> blue
//À¶ -> ÂÌ(Ãð) -> ºì -> À¶(Ãð) -> ÂÌ -> ºì(Ãð) -> À¶ 
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};


extern void encode_send_data(uint16_t cmd_id, void* buf, uint16_t len);

extern void rm_queue_data(uint16_t cmd_id,void* buf,uint16_t len ); //

extern TIM_HandleTypeDef htim5;

extern osTimerId Ts_myTimer01Handle;

/**
  * @brief          led rgb task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          led RGBÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void led_RGB_flow_task(void const * argument)
{
//    uint16_t i, j;
//    fp32 delta_alpha, delta_red, delta_green, delta_blue;
//    fp32 alpha,red,green,blue;
//    uint32_t aRGB;
	
	osTimerStart(Ts_myTimer01Handle,10);

    while(1)
    {
			 __HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 0);
			 osDelay(200);
			__HAL_TIM_SetCompare(&htim5, TIM_CHANNEL_2, 1200);
			 osDelay(200);
			
		}
}


