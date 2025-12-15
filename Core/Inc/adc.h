/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

/* USER CODE BEGIN Private defines */

#define AD1_NTC_INDEX          0//1//14 //PA2
#define AD1_OCP_Ibus_INDEX     1 // 2// 3// 16//PA0
#define AD1_24V_VBUS_INDEX    2//  3// 4//17  //PA1
#define AD1_AIR_PRESSER_INDEX 3//4// 5//18   //PA4

#define AD2_LASER_1064_INDEX   4// 1// 2//15//PA3

#define  MAX_AD2_ENERGE_BUFF_LENGTH 128//36//一个时间约6us=2.96*2(387.5cycle 2.96us)
#define  HALF_AD2_ENERGE_BUFF_LENGTH 36
/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

/* USER CODE BEGIN Prototypes */
void app_get_adc_value(unsigned char adChannel,float *vBuff);
void app_start_multi_channel_adc(void);
extern void pulse_adc_start(unsigned char Len);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

