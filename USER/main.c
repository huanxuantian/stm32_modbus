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

void init_output()
{
	    //下面是给各模块开启时钟
    //启动GPIO
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | \
                           RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | \
						   RCC_APB2Periph_GPIOE ,
                           ENABLE);
		GPIO_InitTypeDef GPIO_InitStructure;

	/***************????IO???*********************/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;  //???1  LED1  LED2
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
}
void handle_coils(void);

int main(void)
{
    SystemInit();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_Configuration();
	init_output();

	
	eMBInit( MB_RTU, 0x01, 0, 9600, MB_PAR_NONE );
	
	/* Enable the Modbus Protocol Stack. */
	eMBEnable(  );
	
	for( ;; )
	{
	    ( void )eMBPoll(  );
		handle_coils();
		
	    /* Here we simply count the number of poll cycles. */
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
	flag == 0 ? GPIO_SetBits(GPIOB,GPIO_Pin_9) : GPIO_ResetBits(GPIOB,GPIO_Pin_9);
	flag ^= 1;
	return MB_ENOERR;
}

