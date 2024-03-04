#ifndef __CORE_DELAY_H
#define __CORE_DELAY_H

#include "stm32f10x.h"

#define USE_DWT_DELAY			1	/* ʹ��dwt�ں˾�ȷ��ʱ */

#if USE_DWT_DELAY
#define USE_TICK_DELAY		0		/* ��ʹ��SysTick��ʱ */
#else
#define USE_TICK_DELAY		1		/* ʹ��SysTick��ʱ */
#endif


/*���������*/
#define TASK_ENABLE 0
#define NumOfTask 3


#if USE_DWT_DELAY

#define Delay_ms(ms)  	CPU_TS_Tmr_Delay_MS(ms)
#define Delay_us(us)  	CPU_TS_Tmr_Delay_US(us)
///* �����ʱ 60s=2��32�η�/72000000 */
#define Delay_s(s)  	  CPU_TS_Tmr_Delay_S(s)

/* ��ȡ�ں�ʱ��Ƶ�� */
#define GET_CPU_ClkFreq()       (SystemCoreClock)
#define SysClockFreq            (SystemCoreClock)
/* Ϊ����ʹ�ã�����ʱ�����ڲ�����CPU_TS_TmrInit������ʼ��ʱ����Ĵ�����
   ����ÿ�ε��ú��������ʼ��һ�顣
   �ѱ���ֵ����Ϊ0��Ȼ����main����������ʱ����CPU_TS_TmrInit�ɱ���ÿ�ζ���ʼ�� */  

#define CPU_TS_INIT_IN_DELAY_FUNCTION   1


/*******************************************************************************
 * 							��������
 ******************************************************************************/
uint32_t CPU_TS_TmrRd(void);
void CPU_TS_TmrInit(void);

//ʹ�����º���ǰ�����ȵ���CPU_TS_TmrInit����ʹ�ܼ���������ʹ�ܺ�CPU_TS_INIT_IN_DELAY_FUNCTION
//�����ʱֵΪ60��
void CPU_TS_Tmr_Delay_US(uint32_t us);
#define CPU_TS_Tmr_Delay_MS(ms)     CPU_TS_Tmr_Delay_US(ms*1000)
#define CPU_TS_Tmr_Delay_S(s)       CPU_TS_Tmr_Delay_MS(s*1000)

#endif

#endif /* __CORE_DELAY_H */
