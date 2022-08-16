/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "calibrate_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "INS_task.h"
#include "led_flow_task.h"
#include "oled_task.h"
#include "referee_usart_task.h"
#include "usb_task.h"
#include "voltage_task.h"
#include "servo_task.h"
#include "protocol_camp.h"
#include "up_3508pid_task.h"
#include "ServoControl_task.h"
#include "RosControl_task.h"
#include "RelayControl_task.h"
#include "pwmR_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

osThreadId calibrate_tast_handle;
osThreadId chassisTaskHandle;
osThreadId detect_handle;
osThreadId gimbalTaskHandle;
osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;
osThreadId pwmR_task_handle;
osThreadId oled_handle;
osThreadId referee_usart_task_handle;
osThreadId usb_task_handle;
osThreadId battery_voltage_handle;
osThreadId servo_task_handle;
osThreadId RelayContorl_task_handle;
osThreadId decode_task_handle;
osThreadId rc_task_handle;
osThreadId up3508pid_task_handle;
osThreadId RosControl_task_handle;
osThreadId ServoControl_task_handle;
osThreadId  Motor_taskHandle; //µç»ú¿ØÖÆid

extern void decode_task(void const * argument);

extern void rc_task(void const * argument);

extern void Motor_task(void const * argument);

 QueueHandle_t CDC_send_queue; // ÐÂ½¨CDCÏûÏ¢¶ÓÁÐ
 
extern void rm_queue_data(uint16_t cmd_id,void* buf,uint16_t len ); //

message_info_t message_info2;

uint16_t msg_count = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId testHandle;
osTimerId Ts_myTimer01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);
void Ts_Callback01(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of Ts_myTimer01 */
  osTimerDef(Ts_myTimer01, Ts_Callback01);
  Ts_myTimer01Handle = osTimerCreate(osTimer(Ts_myTimer01), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of test */
  //osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  //testHandle = osThreadCreate(osThread(test), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
    //osThreadDef(cali, calibrate_task, osPriorityNormal, 0, 512);
    //calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);

    //osThreadDef(ChassisTask, chassis_task, osPriorityAboveNormal, 0, 512);
    //chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

    //osThreadDef(DETECT, detect_task, osPriorityNormal, 0, 256);
    //detect_handle = osThreadCreate(osThread(DETECT), NULL);

    //osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 512);
    //gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);

    //osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
    //imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

    osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 256);
    led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);

    osThreadDef(pwmR, pwmR_task, osPriorityNormal, 0, 128);
    pwmR_task_handle = osThreadCreate(osThread(pwmR), NULL);


    //osThreadDef(OLED, oled_task, osPriorityLow, 0, 256);
    //oled_handle = osThreadCreate(osThread(OLED), NULL);


    //osThreadDef(REFEREE, referee_usart_task, osPriorityNormal, 0, 128);
    //referee_usart_task_handle = osThreadCreate(osThread(REFEREE), NULL);


    osThreadDef(USBTask, usb_task, osPriorityHigh, 0, 128);
    usb_task_handle = osThreadCreate(osThread(USBTask), NULL);
		
	  osThreadDef(pid3508Task, up3508_pid_task, osPriorityHigh, 0, 256);
		up3508pid_task_handle = osThreadCreate(osThread(pid3508Task), NULL);

	  osThreadDef(RosControl_Task, RosControl_task, osPriorityHigh, 0, 256);
		RosControl_task_handle = osThreadCreate(osThread(RosControl_Task), NULL);
		
//    osThreadDef(ServoControlTask, ServoControl_task, osPriorityNormal, 0, 128);    //å¼ é”¦æ˜†å†™ èˆµæœºæŽ§åˆ¶
//		ServoControl_task_handle = osThreadCreate(osThread(ServoControlTask), NULL);
	
    osThreadDef(DecodeTask, decode_task, osPriorityHigh, 0, 128);
    decode_task_handle = osThreadCreate(osThread(DecodeTask), NULL);

    //osThreadDef(BATTERY_VOLTAGE, battery_voltage_task, osPriorityNormal, 0, 128);
    //battery_voltage_handle = osThreadCreate(osThread(BATTERY_VOLTAGE), NULL);
		
		//osThreadDef(Remotectrl_Task, rc_task, osPriorityNormal, 0, 128);
    //rc_task_handle = osThreadCreate(osThread(Remotectrl_Task), NULL);

    osThreadDef(SERVO, servo_task, osPriorityNormal, 0, 128);
    servo_task_handle = osThreadCreate(osThread(SERVO), NULL);  //rosä¸Šä½æœº
   
    osThreadDef(relay, RelayContorl_task, osPriorityNormal, 0, 128);
    servo_task_handle = osThreadCreate(osThread(relay), NULL);  //ç»§ç”µå™¨æŽ§åˆ¶çº¿ç¨‹
		
//		osThreadDef(motor_control_task, Motor_task, osPriorityHigh, 0, 256);
//    Motor_taskHandle = osThreadCreate(osThread(motor_control_task), NULL);   //·ÅÖÃ»úÐµ¿ØÖÆÈÎÎñ

		 CDC_send_queue = xQueueCreate(1, 128); // ´®¿ÚÏûÏ¢¶ÓÁÐ
		 
		 

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN test_task */
  /* Infinite loop */
  for(;;)
  {
		
		osDelay(1);
  }
  /* USER CODE END test_task */
}

/* Ts_Callback01 function */
void Ts_Callback01(void const * argument)
{
  /* USER CODE BEGIN Ts_Callback01 */
		message_info2.num1 = msg_count++;
	if(msg_count>10000)
	{
		msg_count=0;
		osTimerStop(Ts_myTimer01Handle);
	}
	rm_queue_data(MESSAGE_ID,&message_info2,sizeof(message_info_t));
	
  /* USER CODE END Ts_Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
