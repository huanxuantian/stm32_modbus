#include <stdio.h>
#include <string.h>
#include "can.h"
#include "delay.h"
#include "business.h"
extern u8 be_printf;

//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:1~3; CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
//tbs2:时间段2的时间单元.范围:1~8;
//tbs1:时间段1的时间单元.范围:1~16;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024;(实际要加1,也就是1~1024) tq=(brp)*tpclk1
//注意以上参数任何一个都不能设为0,否则会乱.
//波特率=Fpclk1/((tsjw+tbs1+tbs2)*brp);
//mode:0,普通模式;1,回环模式;
//Fpclk1的时钟在初始化的时候设置为36M,如果设置CAN_Normal_Init(1,8,7,5,1);
//则波特率为:36M/((1+8+7)*5)=450Kbps
//返回值:0,初始化OK;
//    其他,初始化失败;


u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
 	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
    GPIO_Init(GPIOA, &GPIO_InitStructure);		//初始化IO
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化IO
	  
 	//CAN单元设置
 	CAN_InitStructure.CAN_TTCM=DISABLE;						 //非时间触发通信模式  //
 	CAN_InitStructure.CAN_ABOM=DISABLE;						 //软件自动离线管理	 //
  	CAN_InitStructure.CAN_AWUM=DISABLE;						 //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)//
  	CAN_InitStructure.CAN_NART=ENABLE;						 	//禁止报文自动传送 //
  	CAN_InitStructure.CAN_RFLM=DISABLE;						 //报文不锁定,新的覆盖旧的 // 
  	CAN_InitStructure.CAN_TXFP=DISABLE;						 //优先级由报文标识符决定 //
  	CAN_InitStructure.CAN_Mode= mode;	         //模式设置： mode:0,普通模式;1,回环模式; //
  	//设置波特率
  	CAN_InitStructure.CAN_SJW=tsjw;				//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1=tbs1+1个时间单位CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2=tbs2+1个时间单位CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;            //分频系数(Fdiv)为brp+1	//
  	CAN_Init(CAN1, &CAN_InitStructure);            // 初始化CAN1 

 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
 	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FilterFIFO0;//过滤器0关联到FIFO0//
 	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0

  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
#if CAN_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   


/*
*********************************************************************************************************
*                                          CAN1_Config32BitFilter()
*
* 功能   ： 设置CAN滤波器，过一个32位扩展帧ID
*
* 参数   ： id ：要过的32位为扩展帧ID
*
* 返回值 ： 无
*
* 注释   ： 无
*********************************************************************************************************
*/
void CAN1_Config32BitFilter(u32 id)
{
    u32 j=0xffffffff;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    
    CAN_FilterInitStructure.CAN_FilterNumber=1;
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh=id>>13;
    CAN_FilterInitStructure.CAN_FilterIdLow=(id<<3)|4;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=j>>13;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=(j<<3)|4;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
 
    CAN_FilterInit(&CAN_FilterInitStructure);
}

/*
*********************************************************************************************************
*                                          CAN1_Config16BitFilter()
*
* 功能   ： 设置CAN滤波器，过两个16位标准帧ID
*
* 参数   ： id1 ：要过的一个16位标准帧ID
*
*           id2 ：要过的另一个16位标准帧ID
*
* 返回值 ： 无
*
* 注释   ： 无
*********************************************************************************************************
*/
void CAN1_Config16BitFilter(u16 id1, u16 id2)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    
    CAN_FilterInitStructure.CAN_FilterNumber=1;          //指定初始化的过滤器1~13
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//标示符屏蔽位模式 可以过滤一组标示符
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;//2个16位过滤器
    CAN_FilterInitStructure.CAN_FilterIdHigh=id1;   //0x0000~0xffff  FC00>>7E0 
    CAN_FilterInitStructure.CAN_FilterIdLow=id2;    //0x0000~0xffff
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xffff;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
 
    CAN_FilterInit(&CAN_FilterInitStructure);
}


void parse_can_data(u8* can_data)
{
	u16 id=0;
	u32 value=0;
		static uint32_t last_loop_tick=0;
	static uint32_t last_loop_speed_tick=0;
		uint32_t cur_time_tick=0;
	cur_time_tick = getTickCount();

	if((can_data[1]==0x41)&&(can_data[2]==0x0c))
	{
		id=10;
		value=(can_data[3]*256+can_data[4])*16383/65535;
		if(last_loop_tick==0||cur_time_tick-last_loop_tick>TIME_S)
		{
			be_printf =1;
			printf("发动机转速=%d RPM\n",value);
			be_printf =0;
			last_loop_tick = cur_time_tick;
		}
	}
	else if((can_data[1]==0x41)&&(can_data[2]==0x0d))
	{
		id=11;
		value=can_data[3];
		if(last_loop_speed_tick==0||cur_time_tick-last_loop_speed_tick>TIME_S)
		{
			be_printf =1;
			printf("车辆速度=%d Km/h\n",value);
			be_printf =0;
			last_loop_speed_tick = cur_time_tick;
		}
	}
	if(id>0) update_param(id,value);
}

#if CAN_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  	CanRxMsg RxMessage;
	int i=0;
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	#if 0
	for(i=0;i<8;i++){
	printf("rxbuf[%d]:%d\r\n",i,RxMessage.Data[i]);
	}
	#endif
	if(RxMessage.IDE==0x00)//stcan
	{
		parse_can_data(RxMessage.Data);
	}
}
#endif

//can发送一组数据(固定格式:ID,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//		 其他,失败;
u8 Can_Send_Msg(u32 id,u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id/0x20;					 // 标准标识符为0
  TxMessage.ExtId=0;				 // 设置扩展标示符（29位）
  TxMessage.IDE=CAN_ID_STD;			 // 使用扩展标识符
  TxMessage.RTR=CAN_RTR_DATA;		 // 消息类型为数据帧，一帧8位
  TxMessage.DLC=8;							 // 发送两帧信息
	len = MIN(len,8);
  for(i=0;i<8;i++)
	{
		if(i<=len)
		{
			TxMessage.Data[i]=msg[i];				 // 第一帧信息 
		}
		else{
			TxMessage.Data[i]=0xAA;//默认以0xAA填充无效字节
		}
	}
	printf("send can req\r\n");	
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		

}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
u8 Can_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
    for(i=0;i<8;i++)
    buf[i]=RxMessage.Data[i]; 
		if(RxMessage.IDE==0x00)//stcan
		{
			parse_can_data(RxMessage.Data);
		}	
	return RxMessage.DLC;	
}

void handle_recv_can()
{
	u8 can_data[8];
	Can_Receive_Msg(can_data);
}
/*----------------------德飞莱 技术论坛：www.doflye.net--------------------------*/













