/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
#define  CAN_BROADCAST_ID     0 //0x00高优先级广播；低优先级广播 0x7FF
#define  CAN_HMI_ID         0//0x101 //屏幕//0x052//CAN_TEST_ID//
#define  CAN_RTU_MASTER_ID  0x080 //本地 start ID
#define  CAN_RTU_SLAVE_ID   0x050 //L980 start ID

#define MAX_FDCAN_FRAME_DATALEN  312
/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle);
uint16_t FDCAN1_Receive_Msg(uint8_t *buf, uint32_t *Identifier, uint16_t *len);
void APP_CAN_SEND_DATA(	uint8_t *data,uint16_t dataLen,uint16_t targetID);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

