#include <stdio.h>
#include "stm32f10x.h"
#include "mb.h"
#include "mbutils.h"

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

void init_input(void);
void init_iomodule()
{
	    //下面是给各模块开启时钟
    //启动GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | \
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | \
						   RCC_APB2Periph_GPIOE ,
                           ENABLE);
		GPIO_InitTypeDef GPIO_InitStructure;

	//输出配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_Configuration();
	init_iomodule();
	init_regdata();

	
	eMBInit( MB_RTU, 0x01, 0, 9600, MB_PAR_NONE );
	
	/* Enable the Modbus Protocol Stack. */
	eMBEnable(  );
	
	for( ;; )
	{
	    ( void )eMBPoll(  );
		handle_coils();
		handle_input();
		handle_param();
		
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
			xMBUtilSetBits(&ucInputBuf[last_check_id/8],last_check_id,1,input_data);
		}
	}
}

void handle_input(void)
{
	static int count=0;
	static int last_check_id=0;
	uint8_t input_data=0;
	last_check_id%=S_INPUTREG_NBit;
	if(count%10==0)
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
			xMBUtilSetBits(&ucInputBuf[last_check_id/8],last_check_id,1,input_data);
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
	static uint8_t flag = 0;
	
	flag == 0 ? GPIO_SetBits(GPIOA,GPIO_Pin_1) : GPIO_ResetBits(GPIOA,GPIO_Pin_1);  
	flag ^= 1;
    return eStatus;
}

/* add test */
/* holding register address */
#define REG_HOLDING_START 0x0000
/* number of regs */
#define REG_HOLDING_NREGS 8
/* content of the holding regs 
   the test data is from xukai's port */
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0x147b,0x3f8e,
0x147b,0x400e,0x1eb8,0x4055,0x147b,0x408e};

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
/*
	flag == 0 ? GPIO_SetBits(GPIOA,GPIO_Pin_7) : GPIO_ResetBits(GPIOA,GPIO_Pin_7);  
	flag == 0 ? GPIO_SetBits(GPIOB,GPIO_Pin_8) : GPIO_ResetBits(GPIOB,GPIO_Pin_8);  
	flag == 0 ? GPIO_SetBits(GPIOB,GPIO_Pin_9) : GPIO_ResetBits(GPIOB,GPIO_Pin_9);
	flag ^= 1;
*/
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
				if(i==0) GPIO_ResetBits(GPIOA,GPIO_Pin_0);
				else if(i==1) GPIO_ResetBits(GPIOA,GPIO_Pin_1);
				else if(i==2) GPIO_ResetBits(GPIOA,GPIO_Pin_2);
				else if(i==3) GPIO_ResetBits(GPIOA,GPIO_Pin_3);
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
				if(i==0) GPIO_SetBits(GPIOA,GPIO_Pin_0);
				else if(i==1) GPIO_SetBits(GPIOA,GPIO_Pin_1);
				else if(i==2) GPIO_SetBits(GPIOA,GPIO_Pin_2);
				else if(i==3) GPIO_SetBits(GPIOA,GPIO_Pin_3);
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
    static uint8_t flag = 0;
	
	flag == 0 ? GPIO_SetBits(GPIOB,GPIO_Pin_8) : GPIO_ResetBits(GPIOB,GPIO_Pin_8);  
	flag ^= 1;
	
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

