/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "fan_bsp.h"
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ADS1118_CS_out_Pin|LASER_PULSE_AD_RESET_out_Pin|Solenoid_EN2_Pin|Solenoid_EN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PWR_SYS_ON_Pin|EEROM_W_EN_out_Pin|S31FL3193_SDB_out_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RF24_SP6_CS_out_Pin|RF24_SDN_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RS485_DIR_out_Pin|H_AIR_EN_out_Pin|treatment_water_ready_ok_in_Pin|TMC2226_DIR_out_Pin
                          |circulating_water_pump_EN_Pin|PTC_EN_Pin|JDQ_STAND_Pin|JDQ_READY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_12V_ON_GPIO_Port, LCD_12V_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DAC_LD_Pin|DAC_CS_Pin|TMC2226_EN_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MCU_SYS_health_LED_GPIO_Port, MCU_SYS_health_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : ADS1118_CS_out_Pin LASER_PULSE_AD_RESET_out_Pin Solenoid_EN2_Pin Solenoid_EN3_Pin */
  GPIO_InitStruct.Pin = ADS1118_CS_out_Pin|LASER_PULSE_AD_RESET_out_Pin|Solenoid_EN2_Pin|Solenoid_EN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CIRCLE_WATER_DEPTH_in_Pin water_cycle_ok_Pin KEY_PWR_SWITCH_Pin */
  GPIO_InitStruct.Pin = CIRCLE_WATER_DEPTH_in_Pin|water_cycle_ok_Pin|KEY_PWR_SWITCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PWR_SYS_ON_Pin RF24_SP6_CS_out_Pin RF24_SDN_out_Pin EEROM_W_EN_out_Pin
                           S31FL3193_SDB_out_Pin */
  GPIO_InitStruct.Pin = PWR_SYS_ON_Pin|RF24_SP6_CS_out_Pin|RF24_SDN_out_Pin|EEROM_W_EN_out_Pin
                          |S31FL3193_SDB_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : FAN1_COUNT_in_Pin FAN2_COUNT_in_Pin */
  GPIO_InitStruct.Pin = FAN1_COUNT_in_Pin|FAN2_COUNT_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : H_AIR_ERROR_Pin */
  GPIO_InitStruct.Pin = H_AIR_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(H_AIR_ERROR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LASER_PULSE_COUNT_in_Pin */
  GPIO_InitStruct.Pin = LASER_PULSE_COUNT_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LASER_PULSE_COUNT_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TREATMENT_WATER_DEPTH_in_Pin REMOTE_ON_OFF_in_Pin Solenoid_STATUS2_Pin Solenoid_STATUS3_Pin
                           ADS1118_DRDY_in_Pin */
  GPIO_InitStruct.Pin = TREATMENT_WATER_DEPTH_in_Pin|REMOTE_ON_OFF_in_Pin|Solenoid_STATUS2_Pin|Solenoid_STATUS3_Pin
                          |ADS1118_DRDY_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : RS485_DIR_out_Pin H_AIR_EN_out_Pin treatment_water_ready_ok_in_Pin TMC2226_DIR_out_Pin
                           circulating_water_pump_EN_Pin PTC_EN_Pin JDQ_STAND_Pin JDQ_READY_Pin */
  GPIO_InitStruct.Pin = RS485_DIR_out_Pin|H_AIR_EN_out_Pin|treatment_water_ready_ok_in_Pin|TMC2226_DIR_out_Pin
                          |circulating_water_pump_EN_Pin|PTC_EN_Pin|JDQ_STAND_Pin|JDQ_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RF24_IRQ_in_Pin TMC2226_index_in_Pin TMC2226_ERROR_out_Pin Hyperbaria_OFF_Signal_Pin
                           EMERGENCY_LASER_STOP_STATUS_in_Pin circulating_water_pump_status_in_Pin */
  GPIO_InitStruct.Pin = RF24_IRQ_in_Pin|TMC2226_index_in_Pin|TMC2226_ERROR_out_Pin|Hyperbaria_OFF_Signal_Pin
                          |EMERGENCY_LASER_STOP_STATUS_in_Pin|circulating_water_pump_status_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : FOOT_SWITCH_IN_Pin */
  GPIO_InitStruct.Pin = FOOT_SWITCH_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FOOT_SWITCH_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_12V_ON_Pin */
  GPIO_InitStruct.Pin = LCD_12V_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_12V_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC_LD_Pin DAC_CS_Pin TMC2226_EN_out_Pin */
  GPIO_InitStruct.Pin = DAC_LD_Pin|DAC_CS_Pin|TMC2226_EN_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MCU_SYS_health_LED_Pin */
  GPIO_InitStruct.Pin = MCU_SYS_health_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MCU_SYS_health_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 2 */
//***************************OUT *******************************************/
  /************************************************************************//**
  * @brief 气泵开关
  * @param  flag-使能信号
  * @note   高电平有效
  * @retval None
  ****************************************************************************/
void app_air_pump_switch( FunctionalState flag)
{
  app_intake_valve_air_solenoid(flag);
  if(flag==DISABLE)  HAL_GPIO_WritePin(H_AIR_EN_out_GPIO_Port,H_AIR_EN_out_Pin,GPIO_PIN_RESET);
  else HAL_GPIO_WritePin(H_AIR_EN_out_GPIO_Port,H_AIR_EN_out_Pin,GPIO_PIN_SET);  
}
  /************************************************************************//**
  * @brief  循环水泵开关（冷却液）
  * @param  flag-使能信号         
  * @note   高电平有效
  * @retval None
  *****************************************************************************/
 void app_circle_water_pump_switch( FunctionalState flag)
 {
   if(flag==DISABLE)  HAL_GPIO_WritePin(circulating_water_pump_EN_GPIO_Port,circulating_water_pump_EN_Pin,GPIO_PIN_RESET);
   else HAL_GPIO_WritePin(circulating_water_pump_EN_GPIO_Port,circulating_water_pump_EN_Pin,GPIO_PIN_SET);
 }
  /************************************************************************//**
  * @brief 系统电源开关
  * @param flag-使能信号
  * @note   高电平有效
  * @retval None
  *****************************************************************************/
 void app_mcu_power_switch(FunctionalState flag)
 {
   if(flag==DISABLE)  HAL_GPIO_WritePin(PWR_SYS_ON_GPIO_Port,PWR_SYS_ON_Pin,GPIO_PIN_RESET);
   else HAL_GPIO_WritePin(PWR_SYS_ON_GPIO_Port,PWR_SYS_ON_Pin,GPIO_PIN_SET);
 }
  /************************************************************************//**
  * @brief  显示屏12V电源开关
  * @param flag-使能信号         
  * @note   高电平有效
  * @retval None
  ******************************************************************************/
 void app_lcd_power_12V_switch(FunctionalState flag)
 {
   if(flag==DISABLE)  HAL_GPIO_WritePin(LCD_12V_ON_GPIO_Port,LCD_12V_ON_Pin,GPIO_PIN_SET);
   else HAL_GPIO_WritePin(LCD_12V_ON_GPIO_Port,LCD_12V_ON_Pin,GPIO_PIN_RESET);
 }
  /************************************************************************//**
  * @brief 治疗水加热开关 treatment water 
  * @param flag-使能信号
  * @note  高电平有效  
  * @retval None
  *****************************************************************************/
 void app_PTC_en_switch(FunctionalState flag)
 {
   if(flag==DISABLE)  HAL_GPIO_WritePin(PTC_EN_GPIO_Port,PTC_EN_Pin,GPIO_PIN_RESET);
   else HAL_GPIO_WritePin(PTC_EN_GPIO_Port,PTC_EN_Pin,GPIO_PIN_SET);
 }
  /************************************************************************//**
  * @brief 循环水水加热管理  water 
  * @param circleWaterTmprature 冷却液温度
  * @note    冷却水需维持22~26摄氏度，最佳25℃, 1秒调用一次
  * @retval None
  *****************************************************************************/
void app_circle_water_PTC_manage(float circleWaterTmprature,unsigned  int sysTimeS)
{
  static unsigned  int ptcRunTime;
  static unsigned  char  PTC_flag;
  if(circleWaterTmprature>-40&&circleWaterTmprature<150)
  {  
    if(circleWaterTmprature<MIN_TEMPRATURE_LASER)  
    {
      if(PTC_flag==0)
      {
        fan_spd_set(FAN38_COMPRESSOR_NUM,2000);						
        PTC_flag=1;
        app_PTC_en_switch(ENABLE);
      }
    }
    else 
    {
      if(circleWaterTmprature>=MID_TEMPRATURE_LASER&&PTC_flag!=0) 
      {        
        PTC_flag=0;
        app_PTC_en_switch(DISABLE);
      }  
      else 
      {
        ptcRunTime++;
        if(ptcRunTime>1200) //升温超过20分钟，关闭一次
        {
          ptcRunTime=0;
          PTC_flag=0;          
          app_PTC_en_switch(DISABLE);
        }
      }       
    }        
  }
  else 
  {   //err
    if(PTC_flag!=0)
    {
      PTC_flag=0;
      app_PTC_en_switch(DISABLE);
    } 
  } 
}
  /************************************************************************//**
  * @brief l064激光 AD采集开关
  * @param flag-使能信号
  * @note   低电平有效
  * @retval None
  *****************************************************************************/
 void app_laser1064_AD_sampling_sw(FunctionalState flag)
 {
   if(flag==DISABLE)  HAL_GPIO_WritePin(LASER_PULSE_AD_RESET_out_GPIO_Port,LASER_PULSE_AD_RESET_out_Pin,GPIO_PIN_SET);
   else HAL_GPIO_WritePin(LASER_PULSE_AD_RESET_out_GPIO_Port,LASER_PULSE_AD_RESET_out_Pin,GPIO_PIN_RESET);
 }

  /************************************************************************//**
  * @brief 泄气阀（电磁阀）
  * @param flag-使能信号
  * @note   高电平有效
  * @retval None
  *****************************************************************************/
 void app_deflate_air_solenoid(FunctionalState flag)
 {
   if(flag==DISABLE)  HAL_GPIO_WritePin(Solenoid_EN2_GPIO_Port,Solenoid_EN2_Pin,GPIO_PIN_RESET);
   else HAL_GPIO_WritePin(Solenoid_EN2_GPIO_Port,Solenoid_EN2_Pin,GPIO_PIN_SET);
 }
  /************************************************************************//**
  * @brief 进气阀（电磁阀）
  * @param flag-使能信号
  * @note   高电平有效
  * @retval None
  *****************************************************************************/
 void app_intake_valve_air_solenoid(FunctionalState flag)
 {
   if(flag==DISABLE)  HAL_GPIO_WritePin(Solenoid_EN3_GPIO_Port,Solenoid_EN3_Pin,GPIO_PIN_RESET);
   else HAL_GPIO_WritePin(Solenoid_EN3_GPIO_Port,Solenoid_EN3_Pin,GPIO_PIN_SET);
 }
 //***************************IN IO*****************************************/
 /************************************************************************//**
  * @brief 遥控连锁
  * @param 
  * @note   高电平有效
  * @retval ErrorStatus
  *****************************************************************************/
 ErrorStatus app_remote_key_sta(void)
 {
    ErrorStatus err;
    err=SUCCESS;
    if(HAL_GPIO_ReadPin(REMOTE_ON_OFF_in_GPIO_Port,REMOTE_ON_OFF_in_Pin)==GPIO_PIN_RESET)
    {
      err=ERROR;//遥控按下
    }    
    return err;
 }
   /************************************************************************//**
  * @brief 按键值解析
  * @param 键值
  * @note   
  * @retval 按键消息
  *****************************************************************************/
 app_key_message  app_key_value_analysis(unsigned  int keyValue)
 {  
  static uint8_t flag;
  app_key_message  key_message;
  switch(keyValue)
  {
    case 0:
    key_message=NO_KEY_MESSAGE;
    break;
    case 0x01:
    key_message=key_jt_short_press;
    break;
    case 0x02:
    key_message=key_jt_long_press;
    //DEBUG_PRINTF("key_JT%d\r\n",keyValue); 
    break;
    case 0x03:
    key_message=key_jt_release;
    //DEBUG_PRINTF("JT relese%d\r\n",keyValue); 
    break;
    case 0x04:
    key_message=key_jt_no_connect;
    //DEBUG_PRINTF("JT relese%d\r\n",keyValue); 
    break;
    case 0x0100:
    if(flag) 
    { 
      app_laser1064_AD_sampling_sw(ENABLE);
      DEBUG_PRINTF("ad _low\r\n"); 
      flag=0;
    }
    else 
    {
      //app_laser1064_AD_sampling_sw(DISABLE);
      DEBUG_PRINTF("ad _high\r\n"); 
      flag=1;
    }
    key_message=key_pwr_short_press;
    break;
    case 0x0200:
    key_message=key_pwr_long_press;
    //DEBUG_PRINTF("key_PWR%d\r\n",keyValue); 
    break;
    case 0x0300:
    key_message=key_pwr_release;
    break;    
    default:
    if((keyValue&0xFF00)==0x0200)  //混合关机按键
    {
      key_message=key_pwr_long_press;
    }
    else key_message=NO_KEY_MESSAGE;
    break;
  }
  return key_message;
 }
 #ifdef ONE_WIRE_BUS_JT_SLAVE
 #include "one_wire_bus.h"
   /************************************************************************//**
  * @brief key 信号,按键扫描单总线
  * @param 按键扫描间隔时间ms
  * @note   4个字节，4个按键值，1byte一个按键
  * @retval 键值
  *****************************************************************************/
 unsigned int  app_owb_key_scan(unsigned short int timeMs)
 { 
  unsigned short int recLen,owb_key_value=0;
  unsigned char owb_buff[8];
  static unsigned  int historyKey=0;
  static unsigned  int timeout[2];
  recLen = app_owb_get_receive_pack_len();
  if(recLen>7)
  { 
    app_owb_receive_handle(owb_buff,recLen); 
    //DEBUG_PRINTF("JT owb recLen= %d %02x %02x %02x %02x %02x %02x %02x %02x \r\n",recLen,owb_buff[0],owb_buff[1],owb_buff[2],owb_buff[3],owb_buff[4],owb_buff[5],owb_buff[6],owb_buff[7]);
    if(owb_buff[0]=='[' && owb_buff[7]==']')
    { 
      DEBUG_PRINTF("jt own\r\n");
      sEnvParam.JT_ID=owb_buff[1]|(owb_buff[2]<<8)|(owb_buff[3]<<16)|(owb_buff[4]<<24);
      sEnvParam.JT_bat=owb_buff[6]; 
      timeout[1]=0;
      if(owb_buff[5]==KEY_LONG_PRESS)
      {
        timeout[0]+=100;
        timeout[1]=0;
        if(timeout[0]>=1000)
        {
          timeout[0]=0;						
          owb_key_value=owb_buff[5]; 
          historyKey=owb_buff[5];
        }
      }					
      else //if(owb_buff[5]==3)
      {  
        historyKey=KEY_LONG_RELEASE;
        owb_key_value=  KEY_LONG_RELEASE;
        timeout[0]=0;
      }					               
    }
    else
    {			
      timeout[0]=0;		
      owb_key_value=IO_KEY_IDLE;
      historyKey=IO_KEY_IDLE;
    }			
    recLen=0;
  }
  else
  {
    timeout[1]+=timeMs;
    if(timeout[1]>1000&&historyKey!=KEY_NO_CONNECT)
    {   //丢包
      timeout[1] = 0;
      timeout[0]=0;
      historyKey = KEY_NO_CONNECT;
      owb_key_value  = KEY_NO_CONNECT;
    }      
    else owb_key_value=IO_KEY_IDLE;
  }
  return   owb_key_value;
 }
 #endif
   /************************************************************************//**
  * @brief key IO信号,按键扫描
  * @param 按键扫描间隔时间ms
  * @note   4个字节，4个按键值，1byte一个按键
  * @retval 键值
  *****************************************************************************/
 unsigned int  app_IO_key_scan(unsigned short int timeMs)
 {  
  unsigned int retKeyValue;
  unsigned char keyValue[MAX_IO_KEY_NUM];
  static unsigned int keyTimeout[MAX_IO_KEY_NUM]={0};
  static unsigned char keyLongFlag[MAX_IO_KEY_NUM]={0};  
  #ifdef ONE_WIRE_BUS_JT_SLAVE
  keyValue[0]= app_owb_key_scan(timeMs);
  #else 
  if(HAL_GPIO_ReadPin(FOOT_SWITCH_IN_GPIO_Port,FOOT_SWITCH_IN_Pin)==GPIO_PIN_RESET)
  {
    keyTimeout[0]+=timeMs;
    if(keyTimeout[0]>=1000)  
    {
      keyTimeout[0]=0;
      keyValue[0]=KEY_LONG_PRESS;
      keyLongFlag[0]=1;
    }
  }
  else 
  {
    if(keyTimeout[0]>200&&keyLongFlag[0]==0) keyValue[0]=KEY_SHORT_PRESS;
    else 
    {     
      if(keyLongFlag[0]!=0)  
      {
        keyLongFlag[0]=0; 
        keyValue[0]=KEY_LONG_RELEASE;
      }
      else  keyValue[0]=IO_KEY_IDLE;  
    }  
    keyTimeout[0]=0;
  }
  #endif
  if(HAL_GPIO_ReadPin(KEY_PWR_SWITCH_GPIO_Port,KEY_PWR_SWITCH_Pin)==GPIO_PIN_RESET)
  {
    keyTimeout[1]+=timeMs;
    if(keyTimeout[1]>=1000) 
    {
      keyLongFlag[1]=1;
      keyTimeout[1]=0;
      keyValue[1]=KEY_LONG_PRESS;
    } 
  }
  else 
  {
    if(keyTimeout[1]>200&&keyLongFlag[1]==0)  keyValue[1]=KEY_SHORT_PRESS;
    else 
    {      
      if(keyLongFlag[1]!=0)  
      {
        keyLongFlag[1]=0; 
        keyValue[1]=KEY_LONG_RELEASE;
      }  
      else keyValue[1]=IO_KEY_IDLE; 
    }
    keyTimeout[1]=0;    
  }  
  retKeyValue =(keyValue[0]|(keyValue[1]<<8));
  return retKeyValue;
 }

 //***************************IN******************************************/
 
  /************************************************************************//**
  * @brief 获取IO信号
  * @param IoNum-IO编号：
  *   In1_high_voltage_solenoid=0,//高压电磁阀
      In2_deflate_air_solenoid,//堵气电磁阀
      In3_chocke_air_solenoid,//泄气电磁阀
      In4_enviroment_tmprature_alert,//环境温度异常报警
      In5_h_air_error,//气泵电源异常
      In6_Hyperbaria_OFF_Signal,//气泵过压
      In7_water_ready_ok,//治疗水状态ok
      In8_water_circle_ok,//循环水状态OK
  * @note   
  * @retval ErrorStatus 获取有效信号成功SUCCESS ，电平无效REROR
  *****************************************************************************/
 ErrorStatus app_get_io_status(genaration_IONum  IoNum)
 {  
  ErrorStatus err=ERROR;
  unsigned char DELAY_STATUS_VALUE=2;//防抖参数
  if(IoNum==In1_high_voltage_solenoid)
  {//低报警，高正常
//if(HAL_GPIO_ReadPin(Solenoid_STATUS1_GPIO_Port,Solenoid_STATUS1_Pin)==GPIO_PIN_SET)
    {
      err=SUCCESS;
    }   
  }
  else if(IoNum==In2_deflate_air_solenoid)
  {//低报警，高正常
    if(HAL_GPIO_ReadPin(Solenoid_STATUS2_GPIO_Port,Solenoid_STATUS2_Pin)==GPIO_PIN_SET)
    {
      err=SUCCESS;
    }
  }
  else if(IoNum==In3_chocke_air_solenoid)
  {//低报警，高正常
    if(HAL_GPIO_ReadPin(Solenoid_STATUS3_GPIO_Port,Solenoid_STATUS3_Pin)==GPIO_PIN_SET)
    {
      err=SUCCESS;
    }      
  }
  else if(IoNum==In4_enviroment_tmprature_alert)
  {//环境气压报警，高报警，低正常
    //high alert status (>Hth -or- <Lth)
//    if(HAL_GPIO_ReadPin(TEMPRATURE_ALERT_GPIO_Port,TEMPRATURE_ALERT_Pin)==GPIO_PIN_RESET)
    if(sEnvParam.enviroment_temprature>-40&&sEnvParam.enviroment_temprature<60)
    {
      err=SUCCESS;      
    }    
  }
  else if(IoNum==In5_h_air_error)//气泵电源异常报警，气泵打开状态下有效
  { //低报警，高正常
    if(HAL_GPIO_ReadPin(H_AIR_ERROR_GPIO_Port,H_AIR_ERROR_Pin)==GPIO_PIN_SET)
    {
      err=SUCCESS;
    }    
  }
  else if(IoNum==In6_Hyperbaria_OFF_Signal)//气泵压力过高
  {  //高报警，低正常
    if(HAL_GPIO_ReadPin(Hyperbaria_OFF_Signal_GPIO_Port,Hyperbaria_OFF_Signal_Pin)==GPIO_PIN_RESET)
    {
      err = SUCCESS;
    }    
  }  
  else if(IoNum==In7_water_ready_ok)//治疗水出口状态ok
  {//低ok有效
    if(HAL_GPIO_ReadPin(treatment_water_ready_ok_in_GPIO_Port,treatment_water_ready_ok_in_Pin)==GPIO_PIN_RESET)
    {
      err=SUCCESS;
    }    
  }
  else if(IoNum==In8_water_circle_ok)//水循环状态
  {//低ok有效
    if(HAL_GPIO_ReadPin(water_cycle_ok_GPIO_Port,water_cycle_ok_Pin)==GPIO_PIN_RESET)
    {
      err = SUCCESS;
    }   
  }  
  return err;  
 }

/* USER CODE END 2 */
