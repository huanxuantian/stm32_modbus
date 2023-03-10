#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 

//********************************************************************************
//V1.2修改说明
//修正了中断中调用出现死循环的错误
//防止延时不准确,采用do while结构!

//V1.3修改说明
//增加了对UCOSII延时的支持.
//如果使用ucosII,delay_init会自动设置SYSTICK的值,使之与ucos的TICKS_PER_SEC对应.
//delay_ms和delay_us也进行了针对ucos的改造.
//delay_us可以在ucos下使用,而且准确度很高,更重要的是没有占用额外的定时器.
//delay_ms在ucos下,可以当成OSTimeDly来用,在未启动ucos时,它采用delay_us实现,从而准确延时
//可以用来初始化外设,在启动了ucos之后delay_ms根据延时的长短,选择OSTimeDly实现或者delay_us实现.

//V1.4修改说明 20110929
//修改了使用ucos,但是ucos未启动的时候,delay_ms中中断无法响应的bug.
//V1.5修改说明 20120902
//在delay_us加入ucos上锁，防止由于ucos打断delay_us的执行，可能导致的延时不准。
////////////////////////////////////////////////////////////////////////////////// 



#define USTICK_SET (1)
#define PRE_MS_TICK	(uint32_t)(1000/USTICK_SET)

#define TIME_MS	(PRE_MS_TICK)
#define TIME_S	(1000*TIME_MS)

void SysTick_Init(void);	 
void delay_init(void);
void Delay_us(u32 nus);
void TimeDlay_ms__Decrement(void);
uint32_t getTickCount(void);
uint32_t  OSTimeGet (void);
uint32_t OSGetSecond(void);
#define Delay_ms(x) Delay_us(PRE_MS_TICK*x)//单位ms

#define delay_ms Delay_ms




#endif





























