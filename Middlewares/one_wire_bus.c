
#include "one_wire_bus.h"

#include "tim.h"

/************************************************************************//**
复位信号拉低（480us）等待ack（240us内回复）； 数据帧通用60us；
  *****************************************************************************/

#define OWB_FRAME_HEADER '['
#define OWB_FRAME_END    ']'

#define  OWB_FRAME_RESET_TIM      	480 //us
#define  OWB_FRAME_WIDTH_TIM      	60 //60~120us，写帧间隔
#define  OWB_FRAME_START_TIM     	5 //帧起始信号>1us
#define  OWB_FRAME_READ_TIM    30 //us,读间隔
#define  OWB_FRAME_MINI_L_TIM     	15 //us,帧间隔最小低电平,15us后从机回复，等待释放总线
#define  OWB_FRAME_ACK_L_TIM      	240//60 //60~240us
#define  OWB_FRAME_MAX_DELAY_TIM  	485 //us

#define  OWB_DQ_OUT_H   HAL_GPIO_WritePin(FOOT_SWITCH_IN_GPIO_Port,FOOT_SWITCH_IN_Pin,GPIO_PIN_SET)
#define  OWB_DQ_OUT_L   HAL_GPIO_WritePin(FOOT_SWITCH_IN_GPIO_Port,FOOT_SWITCH_IN_Pin,GPIO_PIN_RESET)
#define  OWB_DQ_READ    HAL_GPIO_ReadPin(FOOT_SWITCH_IN_GPIO_Port, FOOT_SWITCH_IN_Pin)

typedef struct {
	unsigned char busStart;//0关闭，1启动
	unsigned char busIdleFlag;//0总线空闲,1 ack;2tx;3rx
	unsigned char busAck;//0,noACK;1等ack（低电平）
	unsigned short int txLength;//发送数据包长度
	unsigned short int rxLength;//接收数据包长度
	HAL_StatusTypeDef err;//总线错误
	unsigned int busTime;//总线时间
}ONE_BUS_FRAME;

static ONE_BUS_FRAME owb_frame;

#define OWB_MAX_FRAME_LENGTH 16
static unsigned char owb_rxBuff[OWB_MAX_FRAME_LENGTH+1]={0};
static unsigned char owb_txBuff[OWB_MAX_FRAME_LENGTH+1]={0};
#ifdef ONE_WIRE_BUS_JT_SLAVE 
//从机
/************************************************************************//**
  * @brief one_wire_bus_init
  * @param 
  * @note  
  * @retval  
  *****************************************************************************/
void one_wire_bus_init(void)
{
	/*
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOD_CLK_ENABLE();	
	GPIO_InitStruct.Pin = FOOT_SWITCH_IN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(FOOT_SWITCH_IN_GPIO_Port, &GPIO_InitStruct);	
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	*/
	owb_frame.busStart=1;
}
/************************************************************************//**
	* @brief receive data handlehandle
  * @param 
  * @note  
  * @retval  
  *****************************************************************************/
 unsigned short int  app_owb_get_receive_pack_len(void)
 {
	unsigned short int ret=owb_frame.rxLength;
	return ret;
 }
/************************************************************************//**
	* @brief receive data handlehandle
  * @param 
  * @note  采用定长数据，调用一次清空缓存
  * @retval  
  *****************************************************************************/
void  app_owb_receive_handle(unsigned char *pData,unsigned short int length)
 {
	unsigned short int  i=0;
	while(i+7<owb_frame.rxLength)
	{
		if(owb_rxBuff[i]=='['&&owb_rxBuff[i+7]==']')
		{
			memcpy(pData,&owb_rxBuff[i],8);	
			owb_frame.busIdleFlag=1;
			owb_frame.rxLength=0;
			break;
		}
		i++;		
	}
	owb_frame.rxLength=0;	
 }
/************************************************************************//**
	* @brief 数据接口边沿触发(上升沿)
  * @param 
  * @note  
  * @retval  
  *****************************************************************************/
void owb_dq_edge_callback(void)
{	
	if(owb_frame.busStart )
	{
		if(owb_frame.busIdleFlag==0)
		{
			HAL_TIM_Base_Stop_IT(&htim17);
			owb_frame.busIdleFlag=4;//reset丢弃第一包
			owb_frame.busTime=HAL_GetTick();
		}
		else if(owb_frame.busIdleFlag==1) 
		{	
			owb_frame.busTime=OWB_FRAME_RESET_TIM;//next bit						
			__HAL_TIM_SetAutoreload(&htim17,OWB_FRAME_RESET_TIM-100);
			HAL_TIM_Base_Start_IT(&htim17);	
		}
		else if(owb_frame.busIdleFlag==3) 
		{	
			owb_frame.busTime=OWB_FRAME_READ_TIM;//next bit	
			__HAL_TIM_SetAutoreload(&htim17,OWB_FRAME_READ_TIM-1);
			HAL_TIM_Base_Start_IT(&htim17);
		}
		else if(owb_frame.busIdleFlag==4)
		{
			if(HAL_GetTick()>owb_frame.busTime+50)
			{//start
				owb_frame.busIdleFlag=1;
				owb_frame.busTime=OWB_FRAME_RESET_TIM;//next bit						
				__HAL_TIM_SetAutoreload(&htim17,OWB_FRAME_RESET_TIM-100);
				HAL_TIM_Base_Start_IT(&htim17);		
			}
		}
	}
	else 
	{
		HAL_TIM_Base_Stop_IT(&htim17);
		owb_frame.busTime=0;
		owb_frame.busIdleFlag=0;		
	}
}
/************************************************************************//**
* @brief 写0
  * @param 
  * @note  
  * @retval  
  *****************************************************************************/
 void owb_tim_callback(unsigned int timeUs)
 {	
	static unsigned char txrxData,txrxBit=0;
	HAL_TIM_Base_Stop_IT(&htim17);
	if (owb_frame.busTime>OWB_FRAME_MAX_DELAY_TIM)//err
	{	
		owb_frame.busTime=0;						
		txrxBit=0;							
		txrxData=0;
		owb_frame.busIdleFlag=0;		
	}
	if(owb_frame.busStart==0)
	{
		owb_frame.busTime=0;						
		txrxBit=0;							
		txrxData=0;
		owb_frame.busIdleFlag=0;	
	}
	else 
	{
		switch(owb_frame.busIdleFlag)
		{	
			case 1://ack		
				if(owb_frame.busTime>=OWB_FRAME_RESET_TIM)//reset semr
				{
					if(OWB_DQ_READ==GPIO_PIN_RESET)
					{	
						owb_frame.busTime=0;
						owb_frame.busIdleFlag=3;
						txrxData=0;
						txrxBit=0;
					}
					else 
					{	
						txrxData=0;
						txrxBit=0;
						owb_frame.busTime=0;
						owb_frame.busIdleFlag = 0;	//err						
					}				
				}	
				break;	
			case 3://rx
				if(owb_frame.busTime>=OWB_FRAME_READ_TIM)//最少15us后读取
				{
					owb_frame.busTime=0;
					if(OWB_DQ_READ==GPIO_PIN_SET)	
					{
						txrxData|=(0x01<<txrxBit);	
					}	
					txrxBit+=1;	
					if(txrxBit>7) 
					{
						txrxBit=0;	
						owb_frame.rxLength%=(OWB_MAX_FRAME_LENGTH+1);
						owb_rxBuff[owb_frame.rxLength]=txrxData;			
						txrxData=0;										
						owb_frame.rxLength+=1;//=1;	
					}				
				}
				break;
			default :
				{	//接收结束	
					owb_frame.busTime=0;						
					txrxBit=0;							
					txrxData=0;
					owb_frame.busIdleFlag=0;
				}			
				break;
		}	
	} 	 
 }
#else 
//主机,只写

/************************************************************************//**
	* @brief 获取总线状态
  * @param 
  * @note  
  * @retval  
  *****************************************************************************/
 HAL_StatusTypeDef app_owb_write_status(void)
 {
	if(owb_frame.busIdleFlag!=0)
	{
		return  HAL_BUSY;
	}
    return  owb_frame.err;
 }
/************************************************************************//**
	* @brief 写数据
  * @param 
  * @note  
  * @retval  
  *****************************************************************************/
 void app_owb_write_bytes(unsigned char *pData,unsigned short int length)
{	
	ONE_BUS_FRAME *pFrame=&owb_frame;	
	if(pFrame->busIdleFlag!=0) pFrame->err=HAL_BUSY;
	else 
	{
		pFrame->busIdleFlag=1;
		pFrame->busAck=0;
		pFrame->busTime=0;
		pFrame->txLength=length;
		memcpy(owb_txBuff,pData,length);		
		OWB_DQ_OUT_H;
		HAL_TIM_Base_Start_IT(&htim17);	
	}
}	
/************************************************************************//**
	* @brief 主机写0
  * @param 
  * @note  busIdleFlag 0空闲;1；发送复位脉冲：2发送数据；
  * @retval  
  *****************************************************************************/
void owb_tim_callback(unsigned int timeUs)
{
	static unsigned char startFlag,txrxData,txrxBit,owb_txrxLength;
	owb_frame.busTime += timeUs;
	switch(owb_frame.busIdleFlag)
	{		
		case 1://ack
			if(owb_frame.busAck==0)
			{
				if(owb_frame.busTime>=OWB_FRAME_START_TIM)
				{
					OWB_DQ_OUT_L;
					owb_frame.busTime = 0;
					owb_frame.busAck = 1;	//wait ack	
				}
			}			
			else if(owb_frame.busAck==1)
			{
				if(owb_frame.busTime>=OWB_FRAME_RESET_TIM)
				{
					OWB_DQ_OUT_H;
					if(owb_frame.busTime>=OWB_FRAME_RESET_TIM+OWB_FRAME_START_TIM)
					{
						OWB_DQ_OUT_L;
						owb_frame.busTime = OWB_FRAME_MINI_L_TIM;//提前15us读
						owb_frame.busAck = 2;		
					}
				}
			}
			else if(owb_frame.busAck==2)
			{
				if(owb_frame.busTime>=OWB_FRAME_ACK_L_TIM)
				{
					if(OWB_DQ_READ==GPIO_PIN_RESET)
					{
						owb_frame.busAck = 3;//ok	
						owb_frame.busTime = 0;
					}
					else 
					{
						OWB_DQ_OUT_H;
						owb_frame.busTime = 0;
						owb_frame.busIdleFlag = 0;
						owb_frame.err = HAL_ERROR;
						HAL_TIM_Base_Stop_IT(&htim17);	
					}														
				}			
			}
			else if(owb_frame.busAck==3) 
			{
				if(owb_frame.busTime>=OWB_FRAME_RESET_TIM)//ok，tx
				{					
					OWB_DQ_OUT_H;
					if(owb_frame.busTime>=OWB_FRAME_RESET_TIM+OWB_FRAME_START_TIM)//start pulse
					{	
						OWB_DQ_OUT_L;
						owb_frame.busTime=0;
						owb_txrxLength=0;
						txrxBit=0;									
						startFlag=0;					
					}
				}
				 
			}
			else 
			{
				owb_frame.busIdleFlag=4;
			}
			break;
		case 2://tx	
			if(owb_txrxLength<owb_frame.txLength)
			{				
				if(owb_frame.busTime>=OWB_FRAME_START_TIM)
				{
					if(startFlag==0)
					{ 	
						startFlag=1;	
						if((owb_txBuff[owb_txrxLength]>>txrxBit)&0x01)
						{
							OWB_DQ_OUT_H;//写1
						}
						txrxBit++;
						txrxBit%=8;		
					}
					if(owb_frame.busTime>=OWB_FRAME_WIDTH_TIM)//one bit
					{
						if(owb_frame.busTime>=OWB_FRAME_WIDTH_TIM+OWB_FRAME_START_TIM)//start pulse HL
						{
							OWB_DQ_OUT_L;
							owb_frame.busTime=0;	
							startFlag=0;						
						}		
						else
						{
							OWB_DQ_OUT_H;	
							if(owb_txrxLength+1==owb_frame.txLength)
							{
								HAL_TIM_Base_Stop_IT(&htim17);
								OWB_DQ_OUT_H;							
								owb_frame.busIdleFlag=0;
								owb_frame.busAck=0;
								owb_frame.txLength=0;
								owb_txrxLength=0;
								owb_frame.err=HAL_OK;							
							}
							else 
							{
								owb_txrxLength++;	
							}
						}		
					}
					 							
				}
			}	
			else 
			{
				owb_frame.busIdleFlag=4;
			}							
			break;	
		case 3:			
			if(owb_frame.busTime>OWB_FRAME_START_TIM)//15uS内读取
			{	
				if(startFlag==0)
				{	
					startFlag=1;		
					if(OWB_DQ_READ)	
					{
						txrxData|=(0x01<<txrxBit);	
					}		
					txrxBit++;	
					if(txrxBit>=8) 
					{
						txrxBit=0;	
						owb_txrxLength%=OWB_MAX_FRAME_LENGTH;	
						owb_rxBuff[owb_txrxLength]=txrxBit;
						owb_txrxLength++;//=1;
					}
				}	
				if(owb_frame.busTime>=OWB_FRAME_WIDTH_TIM)
				{
					if(owb_frame.busTime>=OWB_FRAME_WIDTH_TIM+OWB_FRAME_START_TIM)//start pulse
					{
						OWB_DQ_OUT_L;
						owb_frame.busTime=0;		
					}		
					else
					{
						OWB_DQ_OUT_H;
						if(owb_txrxLength>=owb_frame.rxLength) 
						{
							owb_frame.busTime = 0;
							owb_frame.busIdleFlag = 0;
							owb_frame.err = HAL_OK;
							HAL_TIM_Base_Stop_IT(&htim17);	
						}
					}									
				}						
			}
			break;	
		default:
			{				
				owb_frame.busIdleFlag=0;
				owb_frame.busTime=0;
				startFlag=0;						
				txrxBit=0;
				owb_frame.err=HAL_TIMEOUT;
				HAL_TIM_Base_Stop_IT(&htim17);	
			}			
			break;
	}
}
#endif