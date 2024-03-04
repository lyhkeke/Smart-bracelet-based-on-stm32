

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "./led/bsp_led.h"
#include <stdio.h>
#include <string.h> 
#include "./mpu6050/mpu6050.h"
#include "bsp_usart.h"
#include "dht11.h"
#include "bsp_esp8266.h"
#include "bsp_esp8266_test.h"
#include "bsp_SysTick.h"
#include "bsp_exti.h"
#include "rtc.h"
#define TASK_DELAY_NUM  2       //总任务个数，可以自己根据实际情况修改
#define TASK_DELAY_0    500   //任务0延时 1000*1 毫秒后执行：翻转LED
//#define TASK_DELAY_1    1000     //任务1延时 500*1 毫秒后执行：MPU6050任务
   


uint32_t Task_Delay_Group[TASK_DELAY_NUM];  //任务数组，用来计时、并判断是否执行对应任务



int read_dht11_finish;


extern DHT11_Data_TypeDef DHT11_Data;

int task_readdata_finish;

extern uint32_t TimeDisplay;
// 声明外部变量
extern short Accel[3];
extern short Gyro[3];



extern void TimingDelay_Decrement(void);

	uint8_t waring_tem=32;

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	int i;
  
  for(i=0; i<TASK_DELAY_NUM; i++)
  {
    Task_Delay_Group[i] ++;                   //任务计时，时间到后执行
  }
  
  /* 处理任务1 */
  if(Task_Delay_Group[0] >= TASK_DELAY_0)     //判断是否执行任务1
  {
    Task_Delay_Group[0] = 0;                  //置0重新计时
    
		
    /* 任务1：MPU6050任务 */
    if( ! task_readdata_finish )
    {
      MPU6050ReadAcc(Accel);
      MPU6050ReadGyro(Gyro);
     
      
      task_readdata_finish = 1; //标志位置1，表示需要在主循环处理MPU6050数据
    }
  }
//  if(Task_Delay_Group[1] >= TASK_DELAY_1)     //判断是否执行任务0
//  {
//    Task_Delay_Group[1] = 1;                  //置0重新计时
//    
//    /* 任务0：读取 DHT11 传感器数据 */
//    if( ! read_dht11_finish )
//    {
//      if ( DHT11_Read_TempAndHumidity ( & DHT11_Data ) == SUCCESS ) //读取 DHT11 温湿度信息
//      {
//        read_dht11_finish = 1; //读取完成
//      }
//      else
//      {
//        read_dht11_finish = -1; //读取错误
//      //}
//    }
//  }
//}
}
 
void RTC_IRQHandler(void)
{
	  if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
	  {
	    /* Clear the RTC Second interrupt */
	    RTC_ClearITPendingBit(RTC_IT_SEC);
	
	    /* Enable time update */
	    TimeDisplay = 1;
	
	    /* Wait until last write operation on RTC registers has finished */
	    RTC_WaitForLastTask();
	  }

}


/* 串口1接收中断 */
void DEBUG_USART_IRQHandler(void)
{
   uint8_t rec_cmd;
	 uint8_t ucCh;
	
	if(USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE) != RESET)
	{     
			rec_cmd = USART_ReceiveData(DEBUG_USARTx);
		  ucCh  = USART_ReceiveData( DEBUG_USARTx );
      //printf("%c \r\n", rec_cmd);
      if(rec_cmd == '\r' )  // 回车
      {
        //Soft_Reset();
      }
			if ( strUSART_Fram_Record .InfBit .FramLength < ( RX_BUF_MAX_LEN - 1 ) )                       //预留1个字节写结束符
			   strUSART_Fram_Record .Data_RX_BUF [ strUSART_Fram_Record .InfBit .FramLength ++ ]  = ucCh;
	}	 
	
	 	 
	if ( USART_GetITStatus( DEBUG_USARTx, USART_IT_IDLE ) == SET )                                         //数据帧接收完毕
	{
    strUSART_Fram_Record .InfBit .FramFinishFlag = 1;		
		
		ucCh = USART_ReceiveData( DEBUG_USARTx );                                                              //由软件序列清除中断标志位(先读USART_SR，然后读USART_DR)	
  }	
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles macESP8266_USARTx Handler.
  * @param  None
  * @retval None
  */
void macESP8266_USART_INT_FUN ( void )
{	
	uint8_t ucCh;
	
	if ( USART_GetITStatus ( macESP8266_USARTx, USART_IT_RXNE ) != RESET )
	{
		ucCh  = USART_ReceiveData( macESP8266_USARTx );
		
		if ( strEsp8266_Fram_Record .InfBit .FramLength < ( RX_BUF_MAX_LEN - 1 ) )                       //预留1个字节写结束符
			   strEsp8266_Fram_Record .Data_RX_BUF [ strEsp8266_Fram_Record .InfBit .FramLength ++ ]  = ucCh;

	}
	 	 
	if ( USART_GetITStatus( macESP8266_USARTx, USART_IT_IDLE ) == SET )                                         //数据帧接收完毕
	{
    strEsp8266_Fram_Record .InfBit .FramFinishFlag = 1;
		
		ucCh = USART_ReceiveData( macESP8266_USARTx );                                                              //由软件序列清除中断标志位(先读USART_SR，然后读USART_DR)
	
		ucTcpClosedFlag = strstr ( strEsp8266_Fram_Record .Data_RX_BUF, "CLOSED\r\n" ) ? 1 : 0;
		
  }	

}

 void KEY2_IRQHandler(void)
 {
 //确保是否产生了 EXTI Line 中断
 if (EXTI_GetITStatus(KEY2_INT_EXTI_LINE) != RESET) {
 // LED2 取反
 LED2_TOGGLE;
 //清除中断标志位
 EXTI_ClearITPendingBit(KEY2_INT_EXTI_LINE);
 }
 }

void KEY1_IRQHandler(void)
 {
 //确保是否产生了 EXTI Line 中断
 if (EXTI_GetITStatus(KEY1_INT_EXTI_LINE) != RESET) {
 struct rtc_time set_time;

				/*使用串口接收设置的时间，输入数字时注意末尾要加回车*/
				Time_Regulate_Get(&set_time);
				/*用接收到的时间设置RTC*/
				Time_Adjust(&set_time);
				
				//向备份寄存器写入标志
				BKP_WriteBackupRegister(RTC_BKP_DRX, RTC_BKP_DATA);
 //清除中断标志位
 EXTI_ClearITPendingBit(KEY1_INT_EXTI_LINE);
 }
 }
/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

