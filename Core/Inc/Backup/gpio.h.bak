/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
typedef enum{ 
  In1_high_voltage_solenoid=0,//高压电磁阀
  In2_deflate_air_solenoid,//堵气电磁阀
  In3_chocke_air_solenoid,//泄气电磁阀
  In4_enviroment_tmprature_alert,//环境温度异常报警
  In5_h_air_error,//气泵电源异常
  In6_Hyperbaria_OFF_Signal,//气泵过压
  In7_water_ready_ok,//治疗水状态ok
  In8_water_circle_ok,//循环水状态OK
  IN_GENERATION_All
}genaration_IONum;



#define MAX_IO_KEY_NUM 2
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
//genaration IO out
 extern void app_high_voltage_solenoid(FunctionalState flag);
extern void app_mcu_power_switch(FunctionalState flag);
extern void app_lcd_power_12V_switch(FunctionalState flag);
extern void app_air_pump_switch( FunctionalState flag);
extern void app_circle_water_pump_switch( FunctionalState flag);
extern void app_PTC_en_switch(FunctionalState flag);
extern void app_laser1064_AD_sampling_sw(FunctionalState flag);
extern ErrorStatus app_get_io_status(genaration_IONum  IoNum);
extern unsigned int  app_IO_key_scan(unsigned short int timeMs);
extern app_key_message  app_key_value_analysis(unsigned  int keyValue);
extern   void app_circle_water_PTC_manage(float circleWaterTmprature,unsigned  int sysTime);
extern void app_intake_valve_air_solenoid(FunctionalState flag);
void app_deflate_air_solenoid(FunctionalState flag);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

