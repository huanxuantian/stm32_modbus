/*-------------------------------------------------------------------------------
文件名称：SysTick.c
文件描述：SysTick 系统滴答时钟1us中断函数库,中断时间可自由配置，
          常用的有 1us 10us 1ms 中断。        
备    注：程序默认使用72M时钟，无分频 
---------------------------------------------------------------------------------*/
#include "delay.h"

static __IO u32 TimingDelay;
static __IO uint32_t SystemTickCount;
static uint32_t  __IO   interruptTickCount_have=0;

static __IO uint32_t SystemMsTickCount;

/*-------------------------------------------------------------------------------
程序名称：SysTick_Init
程序描述：启动系统滴答定时器 SysTick
输入参数：无
返回参数：无
备    注：使用此功能之前需要调用此函数，初始化寄存器等参数
---------------------------------------------------------------------------------*/
void SysTick_Init(void)
{
	 // SystemFrequency / 1000     1ms中断一次
	 // SystemFrequency / 100000	 10us中断一次
	 // SystemFrequency / 1000000  1us中断一次
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
//	SysTick_Config(SystemCoreClock / 1000*PRE_MS_TICK);
//	return;
	 
	if (SysTick_Config(SystemCoreClock / 1000000))	// 1us ST3.5.0库版本
	{ 
		/* Capture error */ 
		while (1);
	}
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
}


/*-------------------------------------------------------------------------------
程序名称：Delay_us
程序描述：延时nTime us  
输入参数：nTime
返回参数：无
备    注：无
---------------------------------------------------------------------------------*/
void Delay_us(__IO u32 nTime)
{ 
	TimingDelay = nTime;	
	if(getTickCount()>0)
	{
		while(TimingDelay != 0);
	}
	else{
		//
	}
}

/*-------------------------------------------------------------------------------
程序名称：TimingDelay_Decrement
程序描述：数字递减直到0  
输入参数：无
返回参数：无
备    注：需要在中断函数SysTick_Handler()中调用
---------------------------------------------------------------------------------*/
void TimingDelay_Decrement(void)
{
	SystemTickCount++;
	interruptTickCount_have=1;
	if (TimingDelay != 0x00)
	{ 
	TimingDelay--;
	}
}


void TimeDlay_ms__Decrement(void)
{
	SystemMsTickCount++;
	if (TimingDelay != 0x00)
	{ 
		if(TimingDelay>1000)
		{
			TimingDelay-=1000;
		}
		else
		{
			TimingDelay=0;
		}
	}
}

uint32_t getTickCount(void)
{
	int try_count=10000;
	uint32_t ttick;
	do{
		 interruptTickCount_have =0;
			ttick = SystemTickCount;
	}while(interruptTickCount_have&&(try_count--)>0);
	if(try_count<=0) return 0;
	return ttick;
}

uint32_t  OSTimeGet (void)
{
	  return getTickCount();
}

uint32_t OSGetSecond(void)
{
	return (OSTimeGet())/TIME_S;
}

/*----------------------德飞莱 技术论坛：www.doflye.net--------------------------*/

