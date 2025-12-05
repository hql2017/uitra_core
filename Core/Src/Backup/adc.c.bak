/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "adc.h"

/* USER CODE BEGIN 0 */
#include "tim.h"

#define AD_VREF_VOLTAGE  3000

#define MAX_AD_BUFF_LENGTH  256//320
#define MAX_AD_BUFF_BYTES_LENGTH  512//640//MAX_AD_BUFF_LENGTH*2 //字节长度
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);
  /* USER CODE END ADC2_Init 2 */

}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED=0;

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_INP16
    PA1     ------> ADC1_INP17
    PA2     ------> ADC1_INP14
    PA4     ------> ADC1_INP18
    */
    GPIO_InitStruct.Pin = AD_OCP_CHANNEL_Pin|AD_VBUS_24V_CHANNEL_Pin|AD_TMC_NTC_CHANNEL_Pin|AD_AIR_PRESSURE_CHANNEL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Stream0;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_adc1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_adc1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_adc1.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    HAL_RCC_ADC12_CLK_ENABLED++;
    if(HAL_RCC_ADC12_CLK_ENABLED==1){
      __HAL_RCC_ADC12_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA3     ------> ADC2_INP15
    */
    GPIO_InitStruct.Pin = AD_LASER_CHANNEL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(AD_LASER_CHANNEL_GPIO_Port, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA2_Stream0;
    hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_CIRCULAR;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_adc2.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_adc2.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_adc2.Init.PeriphBurst = DMA_PBURST_SINGLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_INP16
    PA1     ------> ADC1_INP17
    PA2     ------> ADC1_INP14
    PA4     ------> ADC1_INP18
    */
    HAL_GPIO_DeInit(GPIOA, AD_OCP_CHANNEL_Pin|AD_VBUS_24V_CHANNEL_Pin|AD_TMC_NTC_CHANNEL_Pin|AD_AIR_PRESSURE_CHANNEL_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
  /* USER CODE BEGIN ADC1:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC1:ADC_IRQn disable */

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_ADC12_CLK_ENABLED--;
    if(HAL_RCC_ADC12_CLK_ENABLED==0){
      __HAL_RCC_ADC12_CLK_DISABLE();
    }

    /**ADC2 GPIO Configuration
    PA3     ------> ADC2_INP15
    */
    HAL_GPIO_DeInit(AD_LASER_CHANNEL_GPIO_Port, AD_LASER_CHANNEL_Pin);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC2 interrupt Deinit */
  /* USER CODE BEGIN ADC2:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC2:ADC_IRQn disable */

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief start_adc
  * @param void
  *         
  * @note   
  * @retval None
  */
#define  MAX_AD2_ENERGE_BUFF_LENGTH 36//一个时间约6us=2.96*2(387.5cycle 2.96us)
static  unsigned  short int adBuff[MAX_AD_BUFF_LENGTH];
static  unsigned short int advalue[4];
static  unsigned short int ad2Buff[MAX_AD2_ENERGE_BUFF_LENGTH];
static  unsigned short int ad2vale,ad2hle[16];

extern TIM_HandleTypeDef htim6;//hal tick timer
void app_start_multi_channel_adc(void)
{    
	tim_triger_ad(&htim6);//low power 
  HAL_ADC_Start_DMA(&hadc1,(unsigned int*)adBuff,MAX_AD_BUFF_LENGTH);  //4*64 
}
/**
  * @brief pulse_adc_start
  * @param void
  * @note   脉冲AD采集,tmeus =32*5=160us
  * @retval None
  */
void pulse_adc_start(void)
{  
	HAL_ADC_Start_DMA(&hadc2,(unsigned int*)ad2Buff,MAX_AD2_ENERGE_BUFF_LENGTH);  //5*64 
}
/**
  * @brief 
  * @param unsigned short int buff
  * @note   有效值计算
  * @retval None
  */
 unsigned short int ad_square_value(unsigned short int *enerAdBuff,unsigned char len)
 {
  unsigned short int ret,i,j=0;  
  long unsigned  int sum; 
  sum=0;
  for(i=0;i<len;i++)
  {
    if(enerAdBuff[i]>0)
    {
      sum+=(enerAdBuff[i]*enerAdBuff[i]);
      j++;
    }    
  }
  if(j>0)  ret = (unsigned short int)(sum/j);  
  return ret;
 } 
/**
  * @brief filter
  * @param void
  * @note   均值滤波
  * @retval None
  */
void filter_ad1(void)
{
  long unsigned int sum[4]={0};  	
	SCB_InvalidateDCache_by_Addr(adBuff, MAX_AD_BUFF_BYTES_LENGTH); 
  for(unsigned int i=0;i<64;i++)
  {   
    sum[0]+=adBuff[i*4];
    sum[1]+=adBuff[i*4+1];
    sum[2]+=adBuff[i*4+2];
    sum[3]+=adBuff[i*4+3];
  } 
  advalue[0]=(unsigned short int)(sum[0]>>6);        
  advalue[1]=(unsigned short int)(sum[1]>>6);  
  advalue[2]=(unsigned short int)(sum[2]>>6); 
  advalue[3]=(unsigned short int)(sum[3]>>6); 
} 
 
/**
  * @brief HAL_ADC_ConvCpltCallback
  * @param void
  * @note   
  * @retval None 
  */
void app_in_energe_adc_value(void);
void  HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{ 
  if(hadc->Instance==ADC1)
  {    
    filter_ad1();     
  }
  if(hadc->Instance==ADC2)
  { 
    app_in_energe_adc_value();
  }
}
/**
  * @brief app_get_energe_adc_value
  * @param void         
  * @note   
  * @retval None
  */
  void app_in_energe_adc_value(void)
  {
    SCB_InvalidateDCache_by_Addr(ad2Buff, MAX_AD2_ENERGE_BUFF_LENGTH);
    #if 0
    ad2vale = ad_square_value(ad2Buff,MAX_AD2_ENERGE_BUFF_LENGTH);
    #else
    long unsigned int sum=0;
    unsigned short int i=0;
    static unsigned char levelLen;
    for(i=10;i<20;i++)//MAX_AD2_ENERGE_BUFF_LENGTH;i++)//只取中间40us(8个)
    {   
      sum+=(ad2Buff[i]*ad2Buff[i]);  
    } 
    levelLen%=16;
    ad2hle[levelLen]=(unsigned short int)(sqrt(sum>>3));
    levelLen++;
    sum=0;
    for(i=0;i<16;i++)
    {
      sum+=ad2hle[levelLen];       
    }
    ad2vale=(sum>>4);     
    //kalman_filter_update(&kalmAd2, ad2vale);
    #endif 
    #if 0
   DEBUG_PRINTF("laserAD=\r\n");
   for(int i=0;i<MAX_AD2_ENERGE_BUFF_LENGTH;i++)
   {
    DEBUG_PRINTF(" %d",ad2Buff[i]);
   }
   DEBUG_PRINTF(" leve=%d ad2=%d\r\n",ad2vale,ad2hle);   
   #endif   
  }
/**
  * @brief NTC_T cal
  * @param void         
  * @note   B = (T1*T2)/(T2-T1) * ln(RT1/RT2)  ,T1=25+273.15;
  * 25/50:B=3380 ;                 25/80:B=3428 ; 25/85:B=3434 ; 25/100:B=3455 ;
  * -10:R=42.506k ;/50:R=4.917K ;  /80:R=1.669K ; /85:R=1.452k ;/100:R=0.974k ;
  * @retval T                
  */
 float  NTC_T_cal( unsigned short int voltage)
 { 
    float ret;    
    double  T0,Tt,R0,Rt,B; 
    T0=25+273.15;  
    R0=10;//10KΩ  
    //B=TtT0*log(Rt/R0)/(T0-Tt);
    //Tt = (1.0/(log(Rt/R0)/B + 1/T0) )-273.15;
    Rt=(3300*2.0/voltage)-2.00;//r=2k 
    if(Rt>4.917)      B=3380;//<50
    else if(Rt>1.669) B=3428;//<80
    else if(Rt>1.452) B=3434;//<85
    else if(Rt>0.974) B=3455;//<100
    else              B=3455;
    Tt = (1.0/(log(((3300*2.0/voltage)-2.00)/R0)/B + 1/T0) )-273.15;
    ret=Tt;
    return ret;    
 }
//15KPA~700kPA  adV 0.2V~4.7V    3v->441kPa
#define P_A_MIN 15.0 // The minimum range pressure value for example 15Kpa 
#define P_A_MAX 700.0 //The full scale pressure value, for example 700Kpa 
#define D_A_MIN 4369//3971.8 //AD value corresponding to The minimum range pressure, // for example 10%AD=2^16*0.2 
#define D_A_MAX 102673//93339.0 //AD Value Corresponding to The full scale pressure value, // for example 90%AD=2^16*4.7
//k=0.006569343  ,B=0.6138686131386861
//Vout=K*P+B -> P= (Vout-B)/k; 
/**
  * @brief app_get_adc_value
  * @param void         
  * @note   
  * @retval None
  */
void app_get_adc_value(unsigned char adChannel,float *vBuff)
{  
  unsigned int temp;	
  if(adChannel==AD1_NTC_INDEX)
  {
    temp=((advalue[AD1_NTC_INDEX]*AD_VREF_VOLTAGE)>>16);
    *vBuff=NTC_T_cal(temp);//NTC voltage VALUE
   //DEBUG_PRINTF("NTC=%.1f\r\n",*vBuff);
  }
  else  if(adChannel==AD2_LASER_1064_INDEX)
  {
    temp=(ad2vale*AD_VREF_VOLTAGE)>>16;  
    *vBuff= temp*1.0 ;//LASER064ADAD,   
   #if 0
   DEBUG_PRINTF("laserAD=");
   for(int i=0;i<MAX_AD2_ENERGE_BUFF_LENGTH;i++)
   {
    DEBUG_PRINTF(" %d",ad2Buff[i]);
   }
   DEBUG_PRINTF(" leve=%d ad2=%d\r\n",ad2vale,ad2hle);
   
   #endif 
  }
  else if(adChannel==AD1_OCP_Ibus_INDEX)
  {
    temp=(((advalue[AD1_OCP_Ibus_INDEX]*AD_VREF_VOLTAGE)>>16))*4;    
    *vBuff=temp*0.73076923076923;//0.657894736842105; //I_bus,I*0.005*50=Value  mV   :I=value/0.25   mA ;//calibration
    //DEBUG_PRINTF("iBus=%dmA\r\n", temp);
  }
  else  if(adChannel==AD1_24V_VBUS_INDEX)
  {
    temp=((advalue[AD1_24V_VBUS_INDEX]*AD_VREF_VOLTAGE)>>16)*11;
    *vBuff=temp*1.00359621978757;//v_bus
   //DEBUG_PRINTF("vBus=%dmV ad=%04x\r\n", temp,advalue[AD1_24V_VBUS_INDEX]);
  }
  else if(adChannel==AD1_AIR_PRESSER_INDEX)
  {//相对气压kPa    
    temp=((((advalue[AD1_AIR_PRESSER_INDEX]-D_A_MIN)*AD_VREF_VOLTAGE)>>16) +22);
    *vBuff=temp*0.1654131+P_A_MIN;//0.150723+P_A_MIN;//(441/3000);//air pressure  ,dio
    //DEBUG_PRINTF("air_pressure=%.2f v=%dmV ad=%x\r\n", *vBuff,temp,advalue[AD1_AIR_PRESSER_INDEX]);
  }  
}
/* USER CODE END 1 */
