#ifndef  __ESP8266_TEST_H
#define	 __ESP8266_TEST_H



#include "stm32f10x.h"



/********************************** 用户需要设置的参数**********************************/
 #define      macUser_ESP8266_ApSsid                       "dxwy"                //要连接的热点的名称
#define      macUser_ESP8266_ApPwd                        "77814041"           //要连接的热点的密钥

#define      macUser_ESP8266_TcpServer_IP                 "192.168.219.117"      //要连接的服务器的 IP
#define      macUser_ESP8266_TcpServer_Port               "8080"            //要连接的服务器的端口




/********************************** 外部全局变量 ***************************************/
extern volatile uint8_t ucTcpClosedFlag;
extern int read_dht11_finish;



/********************************** 测试函数声明 ***************************************/
void ESP8266_StaTcpClient_Unvarnish_ConfigTest(void);
void ESP8266_SendDHT11DataTest(void);

#endif

