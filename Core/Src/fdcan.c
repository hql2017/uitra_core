/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 
static void FDCAN1_filter_config(void);
static FDCAN_RxHeaderTypeDef RxHeader;

static  void FDCAN1_filter_config(void);
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */
  /***********RX FIFO0����64��8�ֽڶ��У�TX FIFO������oFDCAN1��RX FIFO1��ʱ�����oFDCAN2************/
  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 36;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 10;
  hfdcan1.Init.NominalTimeSeg2 = 9;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 64;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 64;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN1_filter_config();//filter config and start
  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */
    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void FDCAN1_filter_config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;
/*##-1 FDCAN1 filter########################################*/
	/*  FIFO 0 */
	#if 1
	sFilterConfig.IdType = FDCAN_STANDARD_ID; //ID
	sFilterConfig.FilterIndex = 0;         //
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;   //range ID
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;   // Rx FIFO 0
	sFilterConfig.FilterID1 = 0x0000;   //min ID
	sFilterConfig.FilterID2 = 0x07FF;   //max ID
	#else
	sFilterConfig.IdType = FDCAN_STANDARD_ID; //ID
	sFilterConfig.FilterIndex = 0;         //
	sFilterConfig.FilterType = FDCAN_FILTER_DUAL;   //DUAL ID
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;   // Rx FIFO 0
	sFilterConfig.FilterID1 = CAN_IDENTIFIER_POWER_ID;   //minID
	sFilterConfig.FilterID2 = CAN_MCU2_STATUS_ID;   //MCU ID
	#endif
	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)//config fdcan filter
	{
		DEBUG_PRINTF("Error_Handler()\n");
		Error_Handler();
	}
	/*##-2 start can ##############*/

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		DEBUG_PRINTF("Error_Handler()\n");
		Error_Handler();
	}	
	HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
}
/***************************************************************************//**
 * @brief 发送数据包
 * @param 
 * @note  
 * @return 返回ID
*******************************************************************************/
ErrorStatus FDCAN3_Send_Msg(uint8_t* msg,uint16_t targetID)
{	
	FDCAN_TxHeaderTypeDef TxHeader;	
	TxHeader.Identifier = targetID;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0x01;  
  if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&TxHeader,msg)!=HAL_OK)  return ERROR;
	return SUCCESS;	
}
extern   void app_canBbus_receive_semo(void);
/**
  * @brief  FDCAN1接收消息处理
  * @param  buf: 数据缓存
  * @param  Identifier: ID
  * @param  len: datalen
  * @retval 0:无数据
  *         DataLength: 数据包长度
  */
 uint8_t FDCAN1_Receive_Msg(uint8_t *buf, uint32_t *Identifier, uint8_t *len)
 {
    // check FIFO0 
    if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0) 
    {
      return 0; 
    }
    // copy data from  FIFO0 
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, buf) != HAL_OK)
    {			
      DEBUG_PRINTF("HAL_FDCAN_GetRxMessage---------------EEROR\n");
      return 0; 
    }	
    if(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)!=0)	
    {
      app_canBbus_receive_semo();//还有数据继续处理
    }//success
    *Identifier = RxHeader.Identifier;
    *len = RxHeader.DataLength; 
    return RxHeader.DataLength;
 }
 /**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	uint8_t canRxData[8];
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
  {		
		app_canBbus_receive_semo();
    /* Retrieve Rx messages from RX FIFO0 */    
//   if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, canRxData) != HAL_OK)
//   {
//      Error_Handler();
//		  DEBUG_PRINTF("HAL_FDCAN_GetRxMessage---------------EEROR\n");
//    }	
  }
}


/***************************************************************************//**


 * @brief 发送数据包
 * @param 
 * @note  该模块1秒间隔返回固定长度格式数据 包含，参考温度(f摄氏度);频率(mHz)；电容值（pF）
 * @return 返回ID
*******************************************************************************/
void APP_CAN_SEND_DATA(	uint8_t *data,uint8_t dataLen,uint16_t tartgetID)
{		
  ErrorStatus err;
	uint32_t timeOut=0;
	timeOut=0;
	if(dataLen>8)//需要分包
	{
		dataLen=8;//暂时不分
	}
	while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 32) 
	{
		timeOut+=100;
		HAL_Delay(50);
		if(timeOut>50000) 
		{
			HAL_FDCAN_Stop(&hfdcan1);
			break;
		}
	}
	if(timeOut<50000)	err=FDCAN3_Send_Msg(data,tartgetID);
	else 
	{//restart
		if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
		{
			DEBUG_PRINTF("Error_Handler()\n");
			Error_Handler();
		}	
	}	
	if(err!=SUCCESS) 
	{	
		DEBUG_PRINTF("can_tx  error\r\n"); 
	}
	//else DEBUG_PRINTF("can_send  succese\r\n");
}
/* USER CODE END 1 */
