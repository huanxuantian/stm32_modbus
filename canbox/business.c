#include <stdio.h>
#include <math.h>
#include "stm32f10x.h"
#include "delay.h"
#include "business.h"
extern uint16_t usRegHoldingBuf[];
void update_param(u16 id,u32 value)
{
	char msg_string[150]={0};
	
	static uint32_t last_loop_tick=0;
		uint32_t cur_time_tick=0;
	cur_time_tick = getTickCount();
	if(last_loop_tick==0||cur_time_tick-last_loop_tick>10*TIME_S)
	{
		sprintf(msg_string,"{\"DATA\":\"CAN_PARAM\",\"ID\":%d,\"VALUE\":%d}",id,value);
		printf("AT+JSON:%s\r\n", msg_string);	
	}		
	if(id==10)
	{
		usRegHoldingBuf[0] = (value>>16)&0xFFFF;
		usRegHoldingBuf[1] = value&0xFFFF;
	}
	else if(id==11)
	{
		usRegHoldingBuf[2] = (value>>16)&0xFFFF;
		usRegHoldingBuf[3] = value&0xFFFF;
	}
}

u8  table_cmd[][8]= 
{ 
 {0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00}, //
 {0x02,0x01,0x20,0x00,0x00,0x00,0x00,0x00},	 //
 {0x02,0x01,0x40,0x00,0x00,0x00,0x00,0x00},	 //
 {0x02,0x01,0x01,0x00,0x00,0x00,0x00,0x00}, //
 {0x02,0x01,0x0C,0x00,0x00,0x00,0x00,0x00}, //
 {0x02,0x01,0x0D,0x00,0x00,0x00,0x00,0x00} //
 
}	;
u8  table_cmd21[][8]= 
{ 
 {0x21,0x4c,0x41,0x41,0x32,0x32,0x37,0x42}, //
 {0x22,0x30,0x30,0x35,0x34,0x33,0x39,0x34}, //
 {0x23,0x4c,0x41,0x41,0x32,0x32,0x37,0x42}, //
 {0x24,0x4c,0x41,0x41,0x32,0x32,0x37,0x42}, //
 {0x25,0x4c,0x41,0x41,0x32,0x32,0x37,0x42} //                    
}	;

#include "can.h"
//0:idel start req cmd0
static int8_t cur_req_state=0;
static int8_t cur_frame_id=-1;
static int8_t last_frame_id=0;
void set_cur_req_state(u8 state)
{
	cur_req_state=state;
}
											
static void try_send_cur_can_req(void)
{
	static u8 try_count=0;
	
	if(last_frame_id==0&&try_count<4)
	{
		cur_frame_id=0;
	}
	else{
		cur_frame_id++;
		try_count=0;
	}
	if(cur_frame_id>=sizeof(table_cmd)/8-1)
	{
		cur_frame_id=sizeof(table_cmd)/8-1;
		return;
	}
	
	if(cur_frame_id>=0&&cur_frame_id<sizeof(table_cmd)/8)
	{
		Can_Send_Msg(0xFBE0,table_cmd[cur_frame_id],8);
		try_count++;
		last_frame_id = cur_frame_id;
	}
}

static void try_restart_can_req(void)
{
	cur_frame_id=0;
	last_frame_id=0;
	set_cur_req_state(0);
	
}
unsigned char first_run=1;									
static uint32_t last_loop_tick=0;
static uint32_t last_task_tick=0;
uint32_t cur_time_tick=0;
void can_task(void)
{
		
	cur_time_tick+=1000;
	//cur_time_tick = getTickCount();
	
	if(first_run>0||cur_time_tick-last_task_tick>4*TIME_S)
	{
		try_restart_can_req();
		last_task_tick = cur_time_tick;//getTickCount();
	}
	
	if(first_run>0||cur_time_tick-last_loop_tick>100*TIME_MS)
	{
			try_send_cur_can_req();
		last_loop_tick = cur_time_tick;//getTickCount();
		handle_recv_can();
	}	
	if(first_run>0) first_run=0;
}
