#ifndef __CAN_H
#define __CAN_H	 
#include "stm32f10x.h"   

#include "stm32f10x_can.h"

#define MAX(a,b) (a>b)?a:b
#define MIN(a,b) (a<b)?a:b


//CAN接收RX0中断使能
#define CAN_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化

void CAN1_Config16BitFilter(u16 id1, u16 id2);
 
u8 Can_Send_Msg(u32 id,u8* msg,u8 len);						//发送数据

u8 Can_Receive_Msg(u8 *buf);							//接收数据
void handle_recv_can(void);
#endif

/*----------------------德飞莱 技术论坛：www.doflye.net--------------------------*/















