

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
#define TASK_DELAY_NUM  2       //����������������Լ�����ʵ������޸�
#define TASK_DELAY_0    500   //����0��ʱ 1000*1 �����ִ�У���תLED
//#define TASK_DELAY_1    1000     //����1��ʱ 500*1 �����ִ�У�MPU6050����
   


uint32_t Task_Delay_Group[TASK_DELAY_NUM];  //�������飬������ʱ�����ж��Ƿ�ִ�ж�Ӧ����



int read_dht11_finish;


extern DHT11_Data_TypeDef DHT11_Data;

int task_readdata_finish;

extern uint32_t TimeDisplay;
// �����ⲿ����
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
    Task_Delay_Group[i] ++;                   //�����ʱ��ʱ�䵽��ִ��
  }
  
  /* ��������1 */
  if(Task_Delay_Group[0] >= TASK_DELAY_0)     //�ж��Ƿ�ִ������1
  {
    Task_Delay_Group[0] = 0;                  //��0���¼�ʱ
    
		
    /* ����1��MPU6050���� */
    if( ! task_readdata_finish )
    {
      MPU6050ReadAcc(Accel);
      MPU6050ReadGyro(Gyro);
     
      
      task_readdata_finish = 1; //��־λ��1����ʾ��Ҫ����ѭ������MPU6050����
    }
  }
//  if(Task_Delay_Group[1] >= TASK_DELAY_1)     //�ж��Ƿ�ִ������0
//  {
//    Task_Delay_Group[1] = 1;                  //��0���¼�ʱ
//    
//    /* ����0����ȡ DHT11 ���������� */
//    if( ! read_dht11_finish )
//    {
//      if ( DHT11_Read_TempAndHumidity ( & DHT11_Data ) == SUCCESS ) //��ȡ DHT11 ��ʪ����Ϣ
//      {
//        read_dht11_finish = 1; //��ȡ���
//      }
//      else
//      {
//        read_dht11_finish = -1; //��ȡ����
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


/* ����1�����ж� */
void DEBUG_USART_IRQHandler(void)
{
   uint8_t rec_cmd;
	 uint8_t ucCh;
	
	if(USART_GetITStatus(DEBUG_USARTx, USART_IT_RXNE) != RESET)
	{     
			rec_cmd = USART_ReceiveData(DEBUG_USARTx);
		  ucCh  = USART_ReceiveData( DEBUG_USARTx );
      //printf("%c \r\n", rec_cmd);
      if(rec_cmd == '\r' )  // �س�
      {
        //Soft_Reset();
      }
			if ( strUSART_Fram_Record .InfBit .FramLength < ( RX_BUF_MAX_LEN - 1 ) )                       //Ԥ��1���ֽ�д������
			   strUSART_Fram_Record .Data_RX_BUF [ strUSART_Fram_Record .InfBit .FramLength ++ ]  = ucCh;
	}	 
	
	 	 
	if ( USART_GetITStatus( DEBUG_USARTx, USART_IT_IDLE ) == SET )                                         //����֡�������
	{
    strUSART_Fram_Record .InfBit .FramFinishFlag = 1;		
		
		ucCh = USART_ReceiveData( DEBUG_USARTx );                                                              //�������������жϱ�־λ(�ȶ�USART_SR��Ȼ���USART_DR)	
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
		
		if ( strEsp8266_Fram_Record .InfBit .FramLength < ( RX_BUF_MAX_LEN - 1 ) )                       //Ԥ��1���ֽ�д������
			   strEsp8266_Fram_Record .Data_RX_BUF [ strEsp8266_Fram_Record .InfBit .FramLength ++ ]  = ucCh;

	}
	 	 
	if ( USART_GetITStatus( macESP8266_USARTx, USART_IT_IDLE ) == SET )                                         //����֡�������
	{
    strEsp8266_Fram_Record .InfBit .FramFinishFlag = 1;
		
		ucCh = USART_ReceiveData( macESP8266_USARTx );                                                              //�������������жϱ�־λ(�ȶ�USART_SR��Ȼ���USART_DR)
	
		ucTcpClosedFlag = strstr ( strEsp8266_Fram_Record .Data_RX_BUF, "CLOSED\r\n" ) ? 1 : 0;
		
  }	

}

 void KEY2_IRQHandler(void)
 {
 //ȷ���Ƿ������ EXTI Line �ж�
 if (EXTI_GetITStatus(KEY2_INT_EXTI_LINE) != RESET) {
 // LED2 ȡ��
 LED2_TOGGLE;
 //����жϱ�־λ
 EXTI_ClearITPendingBit(KEY2_INT_EXTI_LINE);
 }
 }

void KEY1_IRQHandler(void)
 {
 //ȷ���Ƿ������ EXTI Line �ж�
 if (EXTI_GetITStatus(KEY1_INT_EXTI_LINE) != RESET) {
 struct rtc_time set_time;

				/*ʹ�ô��ڽ������õ�ʱ�䣬��������ʱע��ĩβҪ�ӻس�*/
				Time_Regulate_Get(&set_time);
				/*�ý��յ���ʱ������RTC*/
				Time_Adjust(&set_time);
				
				//�򱸷ݼĴ���д���־
				BKP_WriteBackupRegister(RTC_BKP_DRX, RTC_BKP_DATA);
 //����жϱ�־λ
 EXTI_ClearITPendingBit(KEY1_INT_EXTI_LINE);
 }
 }
/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

