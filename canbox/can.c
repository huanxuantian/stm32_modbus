#include <stdio.h>
#include <string.h>
#include "can.h"
#include "delay.h"
#include "business.h"
extern u8 be_printf;

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:1~3; CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.��Χ:1~8;
//tbs1:ʱ���1��ʱ�䵥Ԫ.��Χ:1~16;	  CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024;(ʵ��Ҫ��1,Ҳ����1~1024) tq=(brp)*tpclk1
//ע�����ϲ����κ�һ����������Ϊ0,�������.
//������=Fpclk1/((tsjw+tbs1+tbs2)*brp);
//mode:0,��ͨģʽ;1,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,�������CAN_Normal_Init(1,8,7,5,1);
//������Ϊ:36M/((1+8+7)*5)=450Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��;


u8 CAN_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

	GPIO_InitTypeDef GPIO_InitStructure; 
	CAN_InitTypeDef        CAN_InitStructure;
 	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN_RX0_INT_ENABLE 
   	NVIC_InitTypeDef  NVIC_InitStructure;
#endif

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);		//��ʼ��IO
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��IO
	  
 	//CAN��Ԫ����
 	CAN_InitStructure.CAN_TTCM=DISABLE;						 //��ʱ�䴥��ͨ��ģʽ  //
 	CAN_InitStructure.CAN_ABOM=DISABLE;						 //����Զ����߹���	 //
  	CAN_InitStructure.CAN_AWUM=DISABLE;						 //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)//
  	CAN_InitStructure.CAN_NART=ENABLE;						 	//��ֹ�����Զ����� //
  	CAN_InitStructure.CAN_RFLM=DISABLE;						 //���Ĳ�����,�µĸ��Ǿɵ� // 
  	CAN_InitStructure.CAN_TXFP=DISABLE;						 //���ȼ��ɱ��ı�ʶ������ //
  	CAN_InitStructure.CAN_Mode= mode;	         //ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; //
  	//���ò�����
  	CAN_InitStructure.CAN_SJW=tsjw;				//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;            //��Ƶϵ��(Fdiv)Ϊbrp+1	//
  	CAN_Init(CAN1, &CAN_InitStructure);            // ��ʼ��CAN1 

 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
 	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FilterFIFO0;//������0������FIFO0//
 	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0

  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
#if CAN_RX0_INT_ENABLE
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
  	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
	return 0;
}   


/*
*********************************************************************************************************
*                                          CAN1_Config32BitFilter()
*
* ����   �� ����CAN�˲�������һ��32λ��չ֡ID
*
* ����   �� id ��Ҫ����32λΪ��չ֡ID
*
* ����ֵ �� ��
*
* ע��   �� ��
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
* ����   �� ����CAN�˲�����������16λ��׼֡ID
*
* ����   �� id1 ��Ҫ����һ��16λ��׼֡ID
*
*           id2 ��Ҫ������һ��16λ��׼֡ID
*
* ����ֵ �� ��
*
* ע��   �� ��
*********************************************************************************************************
*/
void CAN1_Config16BitFilter(u16 id1, u16 id2)
{
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    
    CAN_FilterInitStructure.CAN_FilterNumber=1;          //ָ����ʼ���Ĺ�����1~13
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//��ʾ������λģʽ ���Թ���һ���ʾ��
    CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_16bit;//2��16λ������
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
			printf("������ת��=%d RPM\n",value);
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
			printf("�����ٶ�=%d Km/h\n",value);
			be_printf =0;
			last_loop_speed_tick = cur_time_tick;
		}
	}
	if(id>0) update_param(id,value);
}

#if CAN_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
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

//can����һ������(�̶���ʽ:ID,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//		 ����,ʧ��;
u8 Can_Send_Msg(u32 id,u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id/0x20;					 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0;				 // ������չ��ʾ����29λ��
  TxMessage.IDE=CAN_ID_STD;			 // ʹ����չ��ʶ��
  TxMessage.RTR=CAN_RTR_DATA;		 // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=8;							 // ������֡��Ϣ
	len = MIN(len,8);
  for(i=0;i<8;i++)
	{
		if(i<=len)
		{
			TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ 
		}
		else{
			TxMessage.Data[i]=0xAA;//Ĭ����0xAA�����Ч�ֽ�
		}
	}
	printf("send can req\r\n");	
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		

}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 Can_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg RxMessage;
    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
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
/*----------------------�·��� ������̳��www.doflye.net--------------------------*/













