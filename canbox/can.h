#ifndef __CAN_H
#define __CAN_H	 
#include "stm32f10x.h"   

#include "stm32f10x_can.h"

#define MAX(a,b) (a>b)?a:b
#define MIN(a,b) (a<b)?a:b


//CAN����RX0�ж�ʹ��
#define CAN_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    
										 							 				    
u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ��

void CAN1_Config16BitFilter(u16 id1, u16 id2);
 
u8 Can_Send_Msg(u32 id,u8* msg,u8 len);						//��������

u8 Can_Receive_Msg(u8 *buf);							//��������
void handle_recv_can(void);
#endif

/*----------------------�·��� ������̳��www.doflye.net--------------------------*/















