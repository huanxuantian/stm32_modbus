/*-------------------------------------------------------------------------------
�ļ����ƣ�SysTick.c
�ļ�������SysTick ϵͳ�δ�ʱ��1us�жϺ�����,�ж�ʱ����������ã�
          ���õ��� 1us 10us 1ms �жϡ�        
��    ע������Ĭ��ʹ��72Mʱ�ӣ��޷�Ƶ 
---------------------------------------------------------------------------------*/
#include "delay.h"

static __IO u32 TimingDelay;
static __IO uint32_t SystemTickCount;
static uint32_t  __IO   interruptTickCount_have=0;

static __IO uint32_t SystemMsTickCount;

/*-------------------------------------------------------------------------------
�������ƣ�SysTick_Init
��������������ϵͳ�δ�ʱ�� SysTick
�����������
���ز�������
��    ע��ʹ�ô˹���֮ǰ��Ҫ���ô˺�������ʼ���Ĵ����Ȳ���
---------------------------------------------------------------------------------*/
void SysTick_Init(void)
{
	 // SystemFrequency / 1000     1ms�ж�һ��
	 // SystemFrequency / 100000	 10us�ж�һ��
	 // SystemFrequency / 1000000  1us�ж�һ��
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
//	SysTick_Config(SystemCoreClock / 1000*PRE_MS_TICK);
//	return;
	 
	if (SysTick_Config(SystemCoreClock / 1000000))	// 1us ST3.5.0��汾
	{ 
		/* Capture error */ 
		while (1);
	}
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
}


/*-------------------------------------------------------------------------------
�������ƣ�Delay_us
������������ʱnTime us  
���������nTime
���ز�������
��    ע����
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
�������ƣ�TimingDelay_Decrement
�������������ֵݼ�ֱ��0  
�����������
���ز�������
��    ע����Ҫ���жϺ���SysTick_Handler()�е���
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

/*----------------------�·��� ������̳��www.doflye.net--------------------------*/

