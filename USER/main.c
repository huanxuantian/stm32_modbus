#include <stdio.h>
#include "stm32f10x.h"
#include "mb.h"
#include "mbutils.h"
#include "delay.h"
#include "can.h"
#include "business.h"

u8 be_printf=0;

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
	
	
PUTCHAR_PROTOTYPE
{
 /* Place your implementation of fputc here */
 /* e.g. write a character to the USART */
	//be_printf =1;
  USART_SendData(USART2, (uint8_t) ch);
	//be_printf =0;
  /* 循环等待直到发送结束*/
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {}
	//be_printf =0;
  return ch;
}

void USART2_Int(int baud)
{

    GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	 
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能PORTA时钟
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //使能USART2
 	  USART_DeInit(USART2);  //复位串口2	  //USART2_TX   PA.2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA2
   
    //USART2_RX	  PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA3     
	
	  //Usart1 NVIC 配置
	
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
		
  /* USARTx configured as follow:
        - BaudRate = 9600 baud  波特率
        - Word Length = 8 Bits  数据长度
        - One Stop Bit          停止位
        - No parity             校验方式
        - Hardware flow control disabled (RTS and CTS signals) 硬件控制流
        - Receive and transmit enabled                         使能发送和接收
  */
  USART_InitStructure.USART_BaudRate = baud;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
  USART_Cmd(USART2, ENABLE);                    //使能串口  
}

void USART2_Config(int baud)
{
	USART_InitTypeDef USART_InitStructure;
	USART_Cmd(USART2, DISABLE);
	
	USART_InitStructure.USART_BaudRate =baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	
	USART_Cmd(USART2, ENABLE);
}

void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the TIM2 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    /* Enable the TIM2 gloabal Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}


/*-------------------------------------------------------------------------------
程序名称：TIM4_Base_Init
程序描述：定时器TIM4通用定时功能
输入参数：
返回参数：无
备    注：0.1khz
---------------------------------------------------------------------------------*/
void TIM4_Base_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = 10-1;  //重装值，
	TIM_TimeBaseStructure.TIM_Prescaler =7200-1; //分频系数，72M/7200=10KHz,其他依此类推
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);     //把上述数值写入对应寄存器
 
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);            //使能或者失能指定的TIM中断
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;            //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);                 //把上述数值写入对应寄存
	
	TIM_Cmd(TIM4, ENABLE);  //使能TIMx外设
	
							 
}

void TIM4_IRQHandler(void)   //TIM4中断
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//检查指定的TIM中断发生与否:TIM 中断源
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断 
		can_task();
		TimeDlay_ms__Decrement();
	}
}
void init_input(void);
void init_iomodule()
{
	    //下面是给各模块开启时钟
    //启动GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | \
                           RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO ,
                           ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//禁用JTAC模式，只开启SWD模式，PB3，PB4需要使用GPIO功能
		GPIO_InitTypeDef GPIO_InitStructure;

	//输出配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|//GPIO_Pin_2|GPIO_Pin_3|
	GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;  //???1  LED1  LED2
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA,GPIO_Pin_0);
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
	GPIO_SetBits(GPIOA,GPIO_Pin_2);
	GPIO_SetBits(GPIOA,GPIO_Pin_3);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	GPIO_SetBits(GPIOA,GPIO_Pin_7);
	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9| GPIO_Pin_10;  //??? ???3   ???2
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
	GPIO_SetBits(GPIOB,GPIO_Pin_8);
	GPIO_SetBits(GPIOB,GPIO_Pin_9);
	GPIO_SetBits(GPIOB,GPIO_Pin_10);
	//输入配置

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3|
		GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		GPIO_SetBits(GPIOC,GPIO_Pin_13);
		//读取IO外部初始值，之后定时更新
		init_input();
}
void init_regdata(void)
{
	
}
void handle_coils(void);//在主函数中处理即可
void handle_input(void);//根据配置在主程序处理或者定时处理更新到寄存器，或者中断触发后写入寄存器
void handle_regdata(void);//直接读写内存数据，
void handle_param(void);//通过特殊的寄存器地址写入特殊数据的处理，在主程序中处理,当参数需要保存时会启动保存
int main(void)
{
  SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_Configuration();
	//SysTick_Init();

	init_iomodule();
	init_regdata();
	
	USART2_Int(115200);
	printf("\r\nsystem startup now......\r\n");
		//初始化CAN
	//brp=4:500kpbs,8:250kpbs,16:125kpbs
	//CAN_Mode_Normal:正常模式，CAN_Mode_LoopBack：回环模式
	CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_11tq,4,CAN_Mode_Normal);
	//	CAN1_Config16BitFilter(0XFBE00,0xfd20);
	CAN1_Config16BitFilter(0XFD00,0xfd20); //不行就用这个ID 
	TIM4_Base_Init();	
	
	eMBInit( MB_RTU, 0x01, 0, 9600, MB_PAR_NONE );
	
	/* Enable the Modbus Protocol Stack. */
	eMBEnable(  );
	
	
	
	for( ;; )
	{
	    ( void )eMBPoll(  );
		handle_coils();
		handle_input();
		handle_param();
		//can_task();
		
	    /* Here we simply count the number of poll cycles. */
	}
}

void handle_param(void)
{
	
}

#define S_INPUTREG_START                  0
#define S_INPUTREG_NBit                 8

//Slave mode:Coils variables
USHORT   usInputregStart                                 = S_INPUTREG_START;
//输入寄存器，通过离散输入获取以及输入寄存器都是从这里读取数据
#if S_INPUTREG_NBit%8
#define S_INPUTREG_NRegs (S_INPUTREG_NBit/8+1)
#else
#define S_INPUTREG_NRegs (S_INPUTREG_NBit/8)
#endif
UCHAR    ucInputBuf[S_INPUTREG_NRegs];
#if S_INPUTREG_NBit%16
#define S_INPUTREG_WORD (S_INPUTREG_NBit/16+1)
#else
#define S_INPUTREG_WORD (S_INPUTREG_NBit/16)
#endif
//目前有效为低八位数据，分别对应PB0-PB7输入
void init_input(void)
{
	int last_check_id=0;
	uint8_t input_data=0;
	for(last_check_id=0;last_check_id<S_INPUTREG_NBit;last_check_id++)
		{
		if(last_check_id==0)input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
		else if(last_check_id==1)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1);
		else if(last_check_id==2)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2);
		else if(last_check_id==3)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3);
		else if(last_check_id==4)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4);
		else if(last_check_id==5)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5);
		else if(last_check_id==6)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
		else if(last_check_id==7)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
		if(last_check_id<S_INPUTREG_NBit)
		{
			xMBUtilSetBits(&ucInputBuf[last_check_id/8],last_check_id%8,1,input_data);
		}
	}
}

void handle_input(void)
{
	static int count=0;
	static int last_check_id=0;
	uint8_t input_data=0;
	last_check_id%=S_INPUTREG_NBit;
	if(count%2==0)
	{
		count=0;
		if(last_check_id==0)input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
		else if(last_check_id==1)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1);
		else if(last_check_id==2)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_2);
		else if(last_check_id==3)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3);
		else if(last_check_id==4)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4);
		else if(last_check_id==5)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5);
		else if(last_check_id==6)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
		else if(last_check_id==7)	input_data = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
		if(last_check_id<S_INPUTREG_NBit)
		{
			xMBUtilSetBits(&ucInputBuf[last_check_id/8],last_check_id%8,1,input_data);
		}
		last_check_id++;
	}
	else
	{
		count++;
	}
}

eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{	
	  eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iNReg;
    UCHAR *         pucDiscreteBuf;
    USHORT          INPUTREG_START;
    USHORT          INPUTREG_NWORD;
    USHORT          usInputStart;
    iNReg =  usNRegs;

    pucDiscreteBuf = ucInputBuf;
    INPUTREG_START = S_INPUTREG_START;
    INPUTREG_NWORD = S_INPUTREG_WORD;
    usInputStart = usInputregStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if( ( usAddress >= INPUTREG_START ) &&
        ( usAddress + usNRegs <= INPUTREG_START + INPUTREG_NWORD ) )
    {
        iRegIndex = (USHORT) (usAddress - usInputStart);
				while (iNReg > 0)
				{
						if(iRegIndex+1<=S_INPUTREG_NRegs)
						{
							*pucRegBuffer++ = pucDiscreteBuf[iRegIndex+1];
						}
						else
						{
							*pucRegBuffer++ = 0x00;
						}
						*pucRegBuffer++ = pucDiscreteBuf[iRegIndex];
						iRegIndex+=2;
						iNReg--;
				}
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;

}

/* add test */
/* holding register address */
#define REG_HOLDING_START 0x0000
/* number of regs */
#define REG_HOLDING_NREGS 8
/* content of the holding regs 
   the test data is from xukai's port */
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0xFFFF,0xFFFF,
0xFFFF,0xFFFF,0x1eb8,0x4055,0x147b,0x408e};

void handle_regdata(void)
{
	
}
eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    /* error state */
    eMBErrorCode    eStatus = MB_ENOERR;
    /* offset */
    int16_t iRegIndex;
    
    /* test if the reg is in the range */
    if (((int16_t)usAddress-1 >= REG_HOLDING_START) 
        && (usAddress-1 + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
    {
        /* compute the reg's offset */
        iRegIndex = (int16_t)(usAddress-1 - REG_HOLDING_START);
        switch (eMode)
        {
            case MB_REG_READ:
                while (usNRegs > 0)
                {
                    *pucRegBuffer++ = (uint8_t)( usRegHoldingBuf[iRegIndex] >> 8 );
                    *pucRegBuffer++ = (uint8_t)( usRegHoldingBuf[iRegIndex] & 0xff);
                    iRegIndex ++;
                    usNRegs --;
                }
                break;
            case MB_REG_WRITE:
                while (usNRegs > 0)
                {
                    usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                    usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                    iRegIndex ++;
                    usNRegs --;
                }
                break;
                
        }
    }
    else{
        eStatus = MB_ENOREG;
    }
    
    return eStatus;
    
}
#define S_COIL_START                  0
#define S_COIL_NCOILS                 11

//Slave mode:Coils variables
USHORT   usSCoilStart                                 = S_COIL_START;
#if S_COIL_NCOILS%8
UCHAR    ucSCoilBuf[S_COIL_NCOILS/8+1]                ;
#else
UCHAR    ucSCoilBuf[S_COIL_NCOILS/8]                  ;
#endif

void handle_coils(void)
{
	int i=0;
	static UCHAR last_coil_state[11]={0,0,0};
	UCHAR cur_bit;
	for(i=0;i<S_COIL_NCOILS;i++)
	{
		cur_bit = xMBUtilGetBits(ucSCoilBuf,i,1);
		if(cur_bit!=last_coil_state[i])
		{
			
			if(cur_bit)
			{
				if(i==0){ GPIO_ResetBits(GPIOA,GPIO_Pin_0);GPIO_ResetBits(GPIOC,GPIO_Pin_13);}
				else if(i==1) GPIO_ResetBits(GPIOA,GPIO_Pin_1);
				//else if(i==2) GPIO_ResetBits(GPIOA,GPIO_Pin_2);
				//else if(i==3) GPIO_ResetBits(GPIOA,GPIO_Pin_3);
				else if(i==4) GPIO_ResetBits(GPIOA,GPIO_Pin_4);
				else if(i==5) GPIO_ResetBits(GPIOA,GPIO_Pin_5);
				else if(i==6) GPIO_ResetBits(GPIOA,GPIO_Pin_6);
				else if(i==7) GPIO_ResetBits(GPIOA,GPIO_Pin_7);
				else if(i==8) GPIO_ResetBits(GPIOB,GPIO_Pin_8);
				else if(i==9) GPIO_ResetBits(GPIOB,GPIO_Pin_9);
				else if(i==10) GPIO_ResetBits(GPIOB,GPIO_Pin_10);
			}
			else
			{
				if(i==0){ GPIO_SetBits(GPIOA,GPIO_Pin_0);GPIO_SetBits(GPIOC,GPIO_Pin_13);}
				else if(i==1) GPIO_SetBits(GPIOA,GPIO_Pin_1);
				//else if(i==2) GPIO_SetBits(GPIOA,GPIO_Pin_2);
				//else if(i==3) GPIO_SetBits(GPIOA,GPIO_Pin_3);
				else if(i==4) GPIO_SetBits(GPIOA,GPIO_Pin_4);
				else if(i==5) GPIO_SetBits(GPIOA,GPIO_Pin_5);
				else if(i==6) GPIO_SetBits(GPIOA,GPIO_Pin_6);
				else if(i==7) GPIO_SetBits(GPIOA,GPIO_Pin_7);
				else if(i==8) GPIO_SetBits(GPIOB,GPIO_Pin_8);
				else if(i==9) GPIO_SetBits(GPIOB,GPIO_Pin_9);
				else if(i==10) GPIO_SetBits(GPIOB,GPIO_Pin_10);
			}
			last_coil_state[i]=cur_bit;
		}
	}
}

eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{

    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    UCHAR *         pucCoilBuf;
    USHORT          COIL_START;
    USHORT          COIL_NCOILS;
    USHORT          usCoilStart;
    iNReg =  usNCoils / 8 + 1;

    pucCoilBuf = ucSCoilBuf;
    COIL_START = S_COIL_START;
    COIL_NCOILS = S_COIL_NCOILS;
    usCoilStart = usSCoilStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if( ( usAddress >= COIL_START ) &&
        ( usAddress + usNCoils <= COIL_START + COIL_NCOILS ) )
    {
        iRegIndex = (USHORT) (usAddress - usCoilStart) / 8;
        iRegBitIndex = (USHORT) (usAddress - usCoilStart) % 8;
        switch ( eMode )
        {
        /* read current coil values from the protocol stack. */
        case MB_REG_READ:
            while (iNReg > 0)
            {
                *pucRegBuffer++ = xMBUtilGetBits(&pucCoilBuf[iRegIndex++],
                        iRegBitIndex, 8);
                iNReg--;
            }
            pucRegBuffer--;
            /* last coils */
            usNCoils = usNCoils % 8;
            /* filling zero to high bit */
            *pucRegBuffer = *pucRegBuffer << (8 - usNCoils);
            *pucRegBuffer = *pucRegBuffer >> (8 - usNCoils);
            break;

            /* write current coil values with new values from the protocol stack. */
        case MB_REG_WRITE:
            while (iNReg > 1)
            {
                xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, 8,
                        *pucRegBuffer++);
                iNReg--;
            }
            /* last coils */
            usNCoils = usNCoils % 8;
            /* xMBUtilSetBits has bug when ucNBits is zero */
            if (usNCoils != 0)
            {
                xMBUtilSetBits(&pucCoilBuf[iRegIndex++], iRegBitIndex, usNCoils,
                        *pucRegBuffer++);
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;

}

eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
	
    eMBErrorCode    eStatus = MB_ENOERR;
    USHORT          iRegIndex , iRegBitIndex , iNReg;
    UCHAR *         pucDiscreteBuf;
    USHORT          DISCRETE_START;
    USHORT          DISCRETE_NBit;
    USHORT          usDiscretStart;
    iNReg =  usNDiscrete / 8 + 1;

    pucDiscreteBuf = ucInputBuf;
    DISCRETE_START = S_INPUTREG_START;
    DISCRETE_NBit = S_INPUTREG_NBit;
    usDiscretStart = usInputregStart;

    /* it already plus one in modbus function method. */
    usAddress--;

    if( ( usAddress >= DISCRETE_START ) &&
        ( usAddress + usNDiscrete <= DISCRETE_START + DISCRETE_NBit ) )
    {
        iRegIndex = (USHORT) (usAddress - usDiscretStart) / 8;
        iRegBitIndex = (USHORT) (usAddress - usDiscretStart) % 8;
				while (iNReg > 0)
				{
						*pucRegBuffer++ = xMBUtilGetBits(&pucDiscreteBuf[iRegIndex++],
										iRegBitIndex, 8);
						iNReg--;
				}
				pucRegBuffer--;
				/* last coils */
				usNDiscrete = usNDiscrete % 8;
				/* filling zero to high bit */
				*pucRegBuffer = *pucRegBuffer << (8 - usNDiscrete);
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

