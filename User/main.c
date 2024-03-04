#include "delay.h"   
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "bsp_SysTick.h"
#include "./led/bsp_led.h"
#include "./usart/bsp_usart.h"
#include "./mpu6050/mpu6050.h"
#include "./i2c/bsp_i2c.h"
#include "./lcd/bsp_ili9341_lcd.h"
#include "dht11.h"
#include "stdio.h"
#include "rtc.h"
#include "bsp_exti.h"
#include "./beep/bsp_beep.h" 
#include "date.h"
#include "./key/bsp_key.h"
#include "bsp_esp8266.h"
#include "bsp_esp8266_test.h"


extern uint8_t waring_tem;
struct rtc_time systmtime;
short Accel[3];
short Gyro[3];


struct rtc_time systmtime=
{
0,0,0,1,1,2000,0
};

extern __IO uint32_t TimeDisplay ;

  char waring_str[100];
	char dispBuff[100];

  char cStr [ 70 ];

int main(void)
{
	
  DHT11_Data_TypeDef DHT11_Data;
	ILI9341_Init ();         //LCD ��ʼ��
  ILI9341_GramScan ( 6 );

	
	ILI9341_Clear ( 0, 0, 240, 320);	


  LED_GPIO_Config();
  //��ʼ��systick
	SysTick_Init();
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;
	/* LED �˿ڳ�ʼ�� */
	LED_GPIO_Config();
	/* ����1ͨ�ų�ʼ�� */
	USART_Config();
 
	//I2C��ʼ��
	i2c_GPIO_Config();
  //MPU6050��ʼ��
	MPU6050_Init();
  EXTI_Key_Config();
  DHT11_Init ();
	CPU_TS_TmrInit();
	ESP8266_Init ();
	
	ESP8266_StaTcpClient_Unvarnish_ConfigTest();                          //��ESP8266��������

  
  
  

    BEEP_GPIO_Config();
    Key_GPIO_Config();
	  RTC_NVIC_Config();
	  RTC_CheckAndConfig(&systmtime);

 		if( MPU6050ReadID() == 0 )
	{
		printf("\r\nû�м�⵽MPU6050��������\r\n");
		LED_RED;
		while(1);	//��ⲻ��MPU6050 ������Ȼ����
	}
	
  while(1)
  {
		ILI9341_DispStringLine_EN_CH(LINE(0),"  QUT�����ֻ�");
		if (TimeDisplay == 1)
	    {
				/* ��ǰʱ�� */
	      Time_Display( RTC_GetCounter(),&systmtime); 		  
	      TimeDisplay = 0;
	    }
			

		
		if( DHT11_Read_TempAndHumidity ( & DHT11_Data ) == SUCCESS)
			{
		  	
        
        ILI9341_DispStringLine_EN_CH(LINE(6)," ��ʪ����ʾ");
        /* ��ʾ�¶� */
        sprintf(dispBuff," �¶� : %d.%d ",DHT11_Data.temp_int, DHT11_Data.temp_deci);
        LCD_ClearLine(LINE(7));	/* ����������� */
        ILI9341_DispStringLine_EN_CH(LINE(7),dispBuff);
        
        /* ��ʾʪ�� */
        sprintf(dispBuff," ʪ�� : %d.%d%% ",DHT11_Data.humi_int, DHT11_Data.humi_deci);
        LCD_ClearLine(LINE(8));	/* ����������� */
        ILI9341_DispStringLine_EN_CH(LINE(8),dispBuff);
				
				
			}		
			
		if(DHT11_Data.temp_int>=waring_tem)
		 {BEEP(K);
		   LCD_SetTextColor(RED);
			 LED_RED;
			 ILI9341_DispStringLine_EN_CH(LINE(14)," ����Ԥ���¶ȣ�");
		 }
		 else
		 { BEEP(G);
		  LCD_SetTextColor(BLACK);
			
		 }
	
		
    if( task_readdata_finish ) //task_readdata_finish = 1 ��ʾ��ȡMPU6050�������
    {
    

    
	    ILI9341_DispStringLine_EN_CH(LINE(10)," ���ٶ�");
      sprintf ( cStr, "%8d%8d%8d",Accel[0],Accel[1],Accel[2] );	//���ٶ�ԭʼ����
      ILI9341_DispStringLine_EN_CH(LINE(11),cStr);					

     	ILI9341_DispStringLine_EN_CH(LINE(12)," ���ٶ�");		
      sprintf ( cStr, "%8d%8d%8d",Gyro[0],Gyro[1],Gyro[2] );	//��ԭʼ����
      ILI9341_DispStringLine_EN_CH(LINE(13),cStr);	

				
      task_readdata_finish = 0; // �����־λ
    }
		
		if( read_dht11_finish ) // read_dht11_finish == 1 or read_dht11_finish == -1
    {
      
      ESP8266_SendDHT11DataTest(); // ESP8266 ����һ����ʪ������
      
      read_dht11_finish = 0;       // �����־λ
      

    }

	
  
}
}

  