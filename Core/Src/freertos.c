/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include <math.h>
#include "string.h"
#include "adc.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"
#include "fdcan.h"
#include "iwdg.h"
#include "air_gzp6816d_bsp.h"
#include "eeprom_bsp.h"
#include "m117Z_bsp.h"
#include "IS31FL3193_bsp.h" 
#include "ads1118_bsp.h"
#include "tmc2226_step_bsp.h"
#include "mer_mcp1081_bsp.h"
#include "ge2117_gp_bsp.h"
#include  "jdq_bsp.h"
#include "drv_RF24L01.h"
#include "user_can1.h"
#ifdef ONE_WIRE_BUS_SLAVE
#include "one_wire_bus.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

sys_genaration_status sGenSta;
sys_enviroment_assistant_param sEnvParam;
SYS_LOAD_STATUS sys_load_sta;//辅助系统加载状态
LASER_CONTROL_PARAM laser_ctr_param;
U_SYS_CONFIG_PARAM u_sys_param;
U_SYS_CONFIG_PARAM u_sys_default_param;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//auxStatusEvent01
#define EVENTS_AUX_STATUS_IO1_BIT  0x01
#define EVENTS_AUX_STATUS_IO2_BIT  0x01<<1
#define EVENTS_AUX_STATUS_IO3_BIT  0x01<<2
#define EVENTS_AUX_STATUS_IO4_BIT  0x01<<3
#define EVENTS_AUX_STATUS_IO5_BIT  0x01<<4
#define EVENTS_AUX_STATUS_IO6_BIT  0x01<<5
#define EVENTS_AUX_STATUS_IO7_BIT  0x01<<6//水雾准备
#define EVENTS_AUX_STATUS_IO8_BIT  0x01<<7                                            
#define EVENTS_AUX_STATUS_9_NTC_BIT                         0x0001<<8
#define EVENTS_AUX_STATUS_10_IBUS_BIT                       0x0001<<9
#define EVENTS_AUX_STATUS_11_VBUS_BIT                       0x0001<<10
#define EVENTS_AUX_STATUS_12_K1_TEMPRATURE_BIT           		0x0001<<11
#define EVENTS_AUX_STATUS_13_K2_TEMPRATURE_BIT           		0x0001<<12
#define EVENTS_AUX_STATUS_14_EMERGENCY_KEY_BIT              0x0001<<13
#define EVENTS_AUX_STATUS_15_JT_BIT                         0x0001<<14

#define EVENTS_AUX_STATUS_ALL_BITS     (EVENTS_AUX_STATUS_IO1_BIT|EVENTS_AUX_STATUS_IO2_BIT|EVENTS_AUX_STATUS_IO3_BIT|EVENTS_AUX_STATUS_IO4_BIT|EVENTS_AUX_STATUS_IO5_BIT\
			|EVENTS_AUX_STATUS_IO6_BIT|EVENTS_AUX_STATUS_IO7_BIT|EVENTS_AUX_STATUS_IO8_BIT|EVENTS_AUX_STATUS_9_NTC_BIT|EVENTS_AUX_STATUS_10_IBUS_BIT|EVENTS_AUX_STATUS_11_VBUS_BIT\
			|EVENTS_AUX_STATUS_12_K1_TEMPRATURE_BIT|EVENTS_AUX_STATUS_13_K2_TEMPRATURE_BIT|EVENTS_AUX_STATUS_14_EMERGENCY_KEY_BIT|EVENTS_AUX_STATUS_15_JT_BIT)


//laserEvent02Handle  
#define EVENTS_LASER_GX_TEST_PREPARE_OK_BIT  			0x01
//光纤激活
#define EVENTS_LASER_980_PREPARE_OK_BIT  					0x01<<1  //980laser
#define EVENTS_LASER_1064_PREPARE_OK_BIT        	0x01<<2   //1064laser
#define EVENTS_LASER_JT_ENABLE_BIT        	      0x01<<3   //脚踏开放

#define EVENTS_LASER_PREPARE_OK_ALL_BITS_MASK        	(EVENTS_LASER_JT_ENABLE_BIT|EVENTS_LASER_GX_TEST_PREPARE_OK_BIT|EVENTS_LASER_1064_PREPARE_OK_BIT|EVENTS_LASER_980_PREPARE_OK_BIT)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern osThreadId_t  canTaskHandle;

const osThreadAttr_t canTask_attributes = {
  .name = "canTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 640 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
uint32_t myTask03Buffer[ 128 ];
osStaticThreadDef_t myTask03ControlBlock;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .cb_mem = &myTask03ControlBlock,
  .cb_size = sizeof(myTask03ControlBlock),
  .stack_mem = &myTask03Buffer[0],
  .stack_size = sizeof(myTask03Buffer),
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal6,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for myTask07 */
osThreadId_t myTask07Handle;
const osThreadAttr_t myTask07_attributes = {
  .name = "myTask07",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for myTask08 */
osThreadId_t myTask08Handle;
const osThreadAttr_t myTask08_attributes = {
  .name = "myTask08",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal7,
};
/* Definitions for myTask09 */
osThreadId_t myTask09Handle;
const osThreadAttr_t myTask09_attributes = {
  .name = "myTask09",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal5,
};
/* Definitions for myTask10 */
osThreadId_t myTask10Handle;
const osThreadAttr_t myTask10_attributes = {
  .name = "myTask10",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for myTask11 */
osThreadId_t myTask11Handle;
const osThreadAttr_t myTask11_attributes = {
  .name = "myTask11",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for rgbQueue02 */
osMessageQueueId_t rgbQueue02Handle;
const osMessageQueueAttr_t rgbQueue02_attributes = {
  .name = "rgbQueue02"
};
/* Definitions for keyJTMessageQueue01 */
osMessageQueueId_t keyJTMessageQueue01Handle;
const osMessageQueueAttr_t keyJTMessageQueue01_attributes = {
  .name = "keyJTMessageQueue01"
};
/* Definitions for DitCleanQueue03 */
osMessageQueueId_t DitCleanQueue03Handle;
const osMessageQueueAttr_t DitCleanQueue03_attributes = {
  .name = "DitCleanQueue03"
};
/* Definitions for RF24_JT_BinarySem01 */
osSemaphoreId_t RF24_JT_BinarySem01Handle;
const osSemaphoreAttr_t RF24_JT_BinarySem01_attributes = {
  .name = "RF24_JT_BinarySem01"
};
/* Definitions for powerOffBinarySem02 */
osSemaphoreId_t powerOffBinarySem02Handle;
const osSemaphoreAttr_t powerOffBinarySem02_attributes = {
  .name = "powerOffBinarySem02"
};
/* Definitions for laserPrapareReqSem03 */
osSemaphoreId_t laserPrapareReqSem03Handle;
const osSemaphoreAttr_t laserPrapareReqSem03_attributes = {
  .name = "laserPrapareReqSem03"
};
/* Definitions for laserCloseSem05 */
osSemaphoreId_t laserCloseSem05Handle;
const osSemaphoreAttr_t laserCloseSem05_attributes = {
  .name = "laserCloseSem05"
};
/* Definitions for hmiCanBusIdleSem06 */
osSemaphoreId_t hmiCanBusIdleSem06Handle;
const osSemaphoreAttr_t hmiCanBusIdleSem06_attributes = {
  .name = "hmiCanBusIdleSem06"
};
/* Definitions for CANBusReceiveFrameSem07 */
osSemaphoreId_t CANBusReceiveFrameSem07Handle;
const osSemaphoreAttr_t CANBusReceiveFrameSem07_attributes = {
  .name = "CANBusReceiveFrameSem07"
};
/* Definitions for auxStatusEvent01 */
osEventFlagsId_t auxStatusEvent01Handle;
osStaticEventGroupDef_t auxStatusEvent01ControlBlock;
const osEventFlagsAttr_t auxStatusEvent01_attributes = {
  .name = "auxStatusEvent01",
  .cb_mem = &auxStatusEvent01ControlBlock,
  .cb_size = sizeof(auxStatusEvent01ControlBlock),
};
/* Definitions for laserEvent02 */
osEventFlagsId_t laserEvent02Handle;
const osEventFlagsAttr_t laserEvent02_attributes = {
  .name = "laserEvent02"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void app_set_default_sys_config_param(void);
void app_sys_genaration_status_manage(void);
void app_sys_param_load(void);
unsigned char app_sys_param_save_data(void);
void app_air_pump_manage(unsigned char air_level);
void app_pwr_gx_semo(unsigned char code);
void app_fresh_laser_status_param(void);
unsigned short int  app_laser_1064_energe_to_voltage(unsigned short int energe);
#ifdef ONE_WIRE_BUS_SLAVE
unsigned int  app_owb_key_scan(unsigned short int timeMs);
#endif
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void auxTask02(void *argument);
void keyScanTask03(void *argument);
void laserWorkTask04(void *argument);
void fastAuxTask05(void *argument);
void hmiAppTask06(void *argument);
void canReceiveTask07(void *argument);
void powerOffTask08(void *argument);
void laserProhotTask09(void *argument);
void ge2117ManageTask10(void *argument);
void ditCleanTask11(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of RF24_JT_BinarySem01 */
  RF24_JT_BinarySem01Handle = osSemaphoreNew(1, 0, &RF24_JT_BinarySem01_attributes);

  /* creation of powerOffBinarySem02 */
  powerOffBinarySem02Handle = osSemaphoreNew(1, 0, &powerOffBinarySem02_attributes);

  /* creation of laserPrapareReqSem03 */
  laserPrapareReqSem03Handle = osSemaphoreNew(1, 0, &laserPrapareReqSem03_attributes);

  /* creation of laserCloseSem05 */
  laserCloseSem05Handle = osSemaphoreNew(1, 0, &laserCloseSem05_attributes);

  /* creation of hmiCanBusIdleSem06 */
  hmiCanBusIdleSem06Handle = osSemaphoreNew(1, 0, &hmiCanBusIdleSem06_attributes);

  /* creation of CANBusReceiveFrameSem07 */
  CANBusReceiveFrameSem07Handle = osSemaphoreNew(1, 0, &CANBusReceiveFrameSem07_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of rgbQueue02 */
  rgbQueue02Handle = osMessageQueueNew (3, sizeof(uint16_t), &rgbQueue02_attributes);

  /* creation of keyJTMessageQueue01 */
  keyJTMessageQueue01Handle = osMessageQueueNew (2, sizeof(uint16_t), &keyJTMessageQueue01_attributes);

  /* creation of DitCleanQueue03 */
  DitCleanQueue03Handle = osMessageQueueNew (1, sizeof(uint16_t), &DitCleanQueue03_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(auxTask02, NULL, &myTask02_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(keyScanTask03, NULL, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(laserWorkTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(fastAuxTask05, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(hmiAppTask06, NULL, &myTask06_attributes);

  /* creation of myTask07 */
  myTask07Handle = osThreadNew(canReceiveTask07, NULL, &myTask07_attributes);

  /* creation of myTask08 */
  myTask08Handle = osThreadNew(powerOffTask08, NULL, &myTask08_attributes);

  /* creation of myTask09 */
  myTask09Handle = osThreadNew(laserProhotTask09, NULL, &myTask09_attributes);

  /* creation of myTask10 */
  myTask10Handle = osThreadNew(ge2117ManageTask10, NULL, &myTask10_attributes);

  /* creation of myTask11 */
  myTask11Handle = osThreadNew(ditCleanTask11, NULL, &myTask11_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */ 
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of auxStatusEvent01 */
  auxStatusEvent01Handle = osEventFlagsNew(&auxStatusEvent01_attributes);

  /* creation of laserEvent02 */
  laserEvent02Handle = osEventFlagsNew(&laserEvent02_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  *     typedef struct { 
      unsigned char systemParamFlag;                        //系统参数，0未加载；1待机；2开启；3异常；
      unsigned char jtFlag;                                 //脚踏输入，0未加载；1待机；2开启；3异常；
      unsigned char rgbFlag;                                //rgb氛围灯，0未加载；1待机；2开启；3异常；
      unsigned char auxiliary_bulbFlag;                     //激光指示灯，0未加载；1待机；2开启；3异常；
      unsigned char hmiLcdLoadFlag;                         //显示屏，0未加载；1待机；2开启；3异常；
      unsigned char coolWaterSystemLoadFlag;                //冷却水系统，0未加载；1待机；2开启；3异常；
      unsigned char treatmentWaterSystemLoadFlag;           //治疗水系统，0未加载；1待机；2开启；3异常；
      unsigned char tempratureSystemLoadFlag;               //温度系统，0未加载；1待机；2开启；3异常；
      unsigned char eTempratureAirpressureSystemLoadFlag;   //环境温度气压系统，0未加载；1待机；2开启；3异常；
      unsigned char airPressureSystemLoad
      Flag;              //气压系统，0未加载；1待机；2开启；3异常；
      unsigned char laserPowerSystemLoadFlag;               //激光电源系统，0未加载；1待机；2开启；3异常；
    }__attribute__ ((packed)) SYS_LOAD_STATUS;//辅助系统加载状态
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  uint32_t timeout;
  float treatmentWaterC; 
  app_mcu_power_switch(ENABLE);  
  app_laser1064_AD_sampling_sw(DISABLE);  
  for(;;)
  {  //load     
    uint8_t  err1;
    do
		{
			HAL_Delay(500);
			DEBUG_PRINTF("please release power key%d\r\n",HAL_GPIO_ReadPin(KEY_PWR_SWITCH_GPIO_Port,KEY_PWR_SWITCH_Pin));				 
		}while(HAL_GPIO_ReadPin(KEY_PWR_SWITCH_GPIO_Port,KEY_PWR_SWITCH_Pin)==GPIO_PIN_RESET);	
    if(sys_load_sta.hmiLcdLoadFlag==0)
    {      
      DEBUG_PRINTF("load hmi lcd drive...\r\n");  
      app_lcd_power_12V_switch(ENABLE);    
      sys_load_sta.hmiLcdLoadFlag=1;
    }
    if(sys_load_sta.systemParamFlag==0)
    {
      DEBUG_PRINTF("load system config param\r\n");
      EEPROM_M24C32_init();	
			app_sys_param_load();      
      sys_load_sta.systemParamFlag=1;      
    }
    sys_load_sta.jtFlag=1;
    if(sys_load_sta.rgbFlag==0)
    {
      DEBUG_PRINTF("load rgb...  \r\n");
      IS3_init();	
      sys_load_sta.rgbFlag=1;
    }
    if(sys_load_sta.auxiliary_bulbFlag==0)
    {
      DEBUG_PRINTF("load auxiliary_bulb...\r\n");     
      sys_load_sta.auxiliary_bulbFlag=1;
    }    
    if(sys_load_sta.coolWaterSystemLoadFlag==0)
    {
      DEBUG_PRINTF("load circle water system...\r\n");
      DEBUG_PRINTF("load compressor ...\r\n");
      ge2117_UART_Init(9600);
      DEBUG_PRINTF("load water pump ...\r\n");
      app_circle_water_pump_switch( ENABLE );     
      timeout=0;
      do
      { 
        HAL_Delay(20);
        timeout+=20; 
        if(timeout>2000)
        {
          app_circle_water_pump_switch( DISABLE );
          DEBUG_PRINTF("load circle load fail  \r\n");
          sys_load_sta.coolWaterSystemLoadFlag=3;// err
          break;
        }
      }while(app_get_io_status(In8_water_circle_ok)!=SUCCESS); //等待循环水ok   
      if( sys_load_sta.coolWaterSystemLoadFlag==0)  
      {
        DEBUG_PRINTF("load circle load ok  \r\n");
        sys_load_sta.coolWaterSystemLoadFlag=1;//ok
      }
    }  
    if(sys_load_sta.treatmentWaterSystemLoadFlag==0)
    {
      DEBUG_PRINTF("load treatment water system...\r\n");
      tmc2226_init();  
      if(app_get_io_status(In7_water_ready_ok)!=SUCCESS) 
      {
        tmc2226_start(1,3);
        timeout=0;
        do
        { 
          osDelay(50);
          timeout+=50; 
          if(timeout>5000)
          {
            DEBUG_PRINTF("treatment water load fail \r\n");
            osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO7_BIT);
            break;
          }
        }while(app_get_io_status(In7_water_ready_ok)!=SUCCESS); 
      }       
      if(sys_load_sta.treatmentWaterSystemLoadFlag==0)
      {     
        sys_load_sta.treatmentWaterSystemLoadFlag=1;
        DEBUG_PRINTF(" treatment water load ok \r\n");   
      }    
    }
    if(sys_load_sta.eTempratureAirpressureSystemLoadFlag==0)
    {
      DEBUG_PRINTF("load enciroment airpressure system...\r\n");
      GZP6816D_init();    
      GZP6816D_start_sampling(); 	
      timeout=0;
      do
      { 
        HAL_Delay(300);
        timeout+=300;   
        if(timeout>2000)
        { 
          DEBUG_PRINTF("eViromentAirPressure load fail \r\n"); 
          sEnvParam.air_gzp_enviroment_pressure_kpa=94.0;        
          sys_load_sta.eTempratureAirpressureSystemLoadFlag=3;//通信 err
          break;
        }          
      }while(GZP6816D_IsBusy()!=0);               
      if(sys_load_sta.eTempratureAirpressureSystemLoadFlag==0)  
      {
        GZP6816D_get_cal(&sEnvParam.air_gzp_enviroment_pressure_kpa,&sEnvParam.enviroment_temprature); 
        sys_load_sta.eTempratureAirpressureSystemLoadFlag=1;//standby  
        DEBUG_PRINTF("eViromentAirPressure load ok %.2f kPa \r\n",sEnvParam.air_gzp_enviroment_pressure_kpa); 
      }       
    }
    if(sys_load_sta.eTempratureSystemLoadFlag==0)
    {
      DEBUG_PRINTF("load enviroment temprature system...\r\n");
      #if 1
      M117Z_init();	
      timeout = 0;
      do
      {   
        HAL_Delay(250); 
        timeout+=250;
        if(timeout>2000)
        { 
          DEBUG_PRINTF("eviroment temprature  load failed \r\n");          
          sys_load_sta.eTempratureSystemLoadFlag=3;// err
          break;
        }        
        M117Z_start_sampling();	  
      }while(M117Z_get_temprature()==ERR_T_SHORT_INT_VALUE);      
      if(sys_load_sta.eTempratureSystemLoadFlag==0)  
      {
        sEnvParam.enviroment_temprature = M117Z_get_temprature()*1.0;
        sys_load_sta.eTempratureSystemLoadFlag=1; 
        DEBUG_PRINTF("eviroment temprature load ok %.1f ℃\r\n",sEnvParam.enviroment_temprature); 
      }
      #else
      sys_load_sta.tempratureSystemLoadFlag=1;
      #endif
    }
    if(sys_load_sta.tempratureSystemLoadFlag==0)
    {  
      DEBUG_PRINTF("load K thermocouple system...\r\n");    
	    app_ads1118_startup(); 
      HAL_Delay(ADS1118_DELEY_TIME_MS*4);   
      app_ads1118_channel_sampling_start(ADS1118_COOL_CHANNEL);	 
      HAL_Delay(ADS1118_DELEY_TIME_MS);
      float temp_t = app_ads1118_channel_get_value(ADS1118_COOL_CHANNEL);
      if(temp_t==ERR_T_FLOAT_VALUE)
      {
        sys_load_sta.tempratureSystemLoadFlag=3; 
      }
      app_ads1118_channel_sampling_start(ADS1118_K1_CHANNEL);	 
      HAL_Delay(ADS1118_DELEY_TIME_MS);
      temp_t = app_ads1118_channel_get_value(ADS1118_K1_CHANNEL);
      if(temp_t==ERR_T_FLOAT_VALUE)
      {
        sys_load_sta.tempratureSystemLoadFlag = 4;
      }
      else 
      {
        sEnvParam.eth_k1_temprature = temp_t;  
      } 
      app_ads1118_channel_sampling_start(ADS1118_K2_CHANNEL);	 
      HAL_Delay(ADS1118_DELEY_TIME_MS);
      temp_t=app_ads1118_channel_get_value(ADS1118_K2_CHANNEL);
      if(temp_t==ERR_T_FLOAT_VALUE)
      {
        sys_load_sta.tempratureSystemLoadFlag = 5;
      }
      else  sEnvParam.eth_k2_temprature = temp_t; 
      if(sys_load_sta.tempratureSystemLoadFlag==0) 
      {
        DEBUG_PRINTF("K_T load ok k1_TT=%.1f k2_TT=%.1f \r\n",sEnvParam.eth_k1_temprature,sEnvParam.eth_k2_temprature); 
        sys_load_sta.tempratureSystemLoadFlag=1;
      }
      else DEBUG_PRINTF("K_T load fail  \r\n");
    }
    /***********NTC,laser_energe,iBus,vBus,air_pump_pressure气泵气压，参数****************** */    
    app_start_multi_channel_adc();
    DEBUG_PRINTF("load adc sampling NTC ,laserenergetic,iBus,vbus,air pressure...\r\n");
    HAL_Delay(10); 
    app_get_adc_value(AD1_NTC_INDEX,&sEnvParam.NTC_temprature);
    app_get_adc_value(AD1_LASER_1064_INDEX,&sEnvParam.laser_1064_energy);
    app_get_adc_value(AD1_OCP_Ibus_INDEX,&sEnvParam.iBus);
    app_get_adc_value(AD1_24V_VBUS_INDEX,&sEnvParam.vBus);    
    app_get_adc_value(AD1_AIR_PRESSER_INDEX,&sEnvParam.air_pump_pressure);
    DEBUG_PRINTF("adc load:NTC=%.2f℃ laser_energe=%.1f iBus=%.1fmA ,vBus=%.1fmV,air_pump_pressure=%.2fkPa\r\n",sEnvParam.NTC_temprature,\
      sEnvParam.laser_1064_energy,sEnvParam.iBus,sEnvParam.vBus,sEnvParam.air_pump_pressure); 
    if(sys_load_sta.adNtCSystemLoadFlag==0)
    {     
      if(sEnvParam.NTC_temprature>-40&&sEnvParam.NTC_temprature<150)
      {
        sys_load_sta.adNtCSystemLoadFlag=1;
        DEBUG_PRINTF("adc load:NTC_ad_channel ok=%.2f℃\r\n",sEnvParam.NTC_temprature);         
      }
      else
      {
        DEBUG_PRINTF("adc load:NTC_ad_channel error\r\n"); 
        sys_load_sta.adNtCSystemLoadFlag=3; 
      } 
    }
    if(sys_load_sta.adLaserEnergeSystemLoadFlag==0)
    {
      if(sEnvParam.laser_1064_energy>50)
      {
        DEBUG_PRINTF("adc load:laser_1064_energy_ad_channel error\r\n");
        sys_load_sta.adLaserEnergeSystemLoadFlag=3;
      } 
      else 
      {
        DEBUG_PRINTF("adc load:laser_1064_energy_ad_channel ok=%.1f(mJ?)\r\n",sEnvParam.laser_1064_energy); 
        sys_load_sta.adLaserEnergeSystemLoadFlag=1;
      } 
    }
    if(sys_load_sta.adIBusSystemLoadFlag==0)
    {
      if(sEnvParam.iBus>MAX_IBUS_MA)
      {
        DEBUG_PRINTF("adc load:ibus_ad_channel error\r\n");
        sys_load_sta.adIBusSystemLoadFlag=3;
      }   
      else 
      {
        sys_load_sta.adIBusSystemLoadFlag=1;
        DEBUG_PRINTF("adc load:ibus_ad_channel ok=%.2fmA\r\n",sEnvParam.iBus); 
      }   
    } 
    if(sys_load_sta.adVBusSystemLoadFlag==0)
    {
      if(sEnvParam.vBus<MIN_VBUS_MV)
      {
        DEBUG_PRINTF("adc load:vbus_ad_channel error\r\n");
        sys_load_sta.adVBusSystemLoadFlag=3;
      }   
      else 
      {
        sys_load_sta.adVBusSystemLoadFlag=1;
        DEBUG_PRINTF("adc load:vbus_ad_channel ok=%.2fmV\r\n",sEnvParam.vBus); 
      }   
    } 
    if(sys_load_sta.adAirPressureSystemLoadFlag==0)
    {
      if(sys_load_sta.eTempratureAirpressureSystemLoadFlag==1)
      {
        if(sEnvParam.air_pump_pressure+20<sEnvParam.air_gzp_enviroment_pressure_kpa)
        {
          DEBUG_PRINTF("adc load:air_pressure_ad_channel error\r\n");
          sys_load_sta.adAirPressureSystemLoadFlag=3;
        }   
        else 
        {
          sys_load_sta.adAirPressureSystemLoadFlag=1;                    
          DEBUG_PRINTF("adc load:air_pressure_ad_channel ok=%.2fkPa\r\n",sEnvParam.air_pump_pressure); 
        }   
      }
      else 
      {
        sys_load_sta.adAirPressureSystemLoadFlag=1;       
        DEBUG_PRINTF("adc load:air_pressure_ad_channel ok=%.2fkPa\r\n",sEnvParam.air_pump_pressure); 
      }      
    }    
    if(sys_load_sta.laserPowerSystemLoadFlag==0)
    {
      float jdq_set_voltage,jdq_set_current;      
      DEBUG_PRINTF("load laser jdq power system ... \r\n");   
      jdq_init();
      HAL_Delay(JDQ_RS485_FRAME_MIN_MS);
      app_jdq_bus_voltage_set(LASER_JDQ_VOLTAGE_F);
      timeout=0;
			do
			{
        HAL_Delay(JDQ_RS485_FRAME_MIN_MS);         
        timeout+=JDQ_RS485_FRAME_MIN_MS;
        if(timeout>JDQ_RS485_FRAME_MAX_DELAY_MS)
        {				
          break;
        }
			}while(app_get_jdq_rs485_bus_statu()!=0);   
      app_jdq_bus_get_set_v_c(&jdq_set_voltage,&jdq_set_current);
      if(fabsf(jdq_set_voltage-LASER_JDQ_VOLTAGE_F)>5.0)
      {
          sys_load_sta.laserPowerSystemLoadFlag=3;
          DEBUG_PRINTF("laser jdq  load fail \r\n"); 
      }
      else
      {
        HAL_Delay(JDQ_RS485_FRAME_MIN_MS);
        app_jdq_bus_power_on_off(1);//打开
        timeout=0;
        do
        {
          HAL_Delay(JDQ_RS485_FRAME_MIN_MS);       
          timeout+=JDQ_RS485_FRAME_MIN_MS;
          if(timeout>JDQ_RS485_FRAME_MAX_DELAY_MS)
          {				
            break;
          }
        }while(app_get_jdq_rs485_bus_statu()!=0); 
        DEBUG_PRINTF("lasr jdq  load ok jdq_v=%.1f v \r\n",jdq_set_voltage); 
        sys_load_sta.laserPowerSystemLoadFlag = 1;
      } 
      app_laser_pulse_start(LASER_PULSE_STOP,20);  
    }   
    #ifdef IWDG_USED
    MX_IWDG1_Init(); 
    #endif	
    osThreadTerminate(defaultTaskHandle);    
   //osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_auxTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_auxTask02 */
void auxTask02(void *argument)
{
  /* USER CODE BEGIN auxTask02 */
  /* Infinite loop */
  uint32_t led_tick,timeout;
	uint16_t rgbRun=1;
  unsigned char local_water_air;
	led_tick=osKernelGetTickCount();
  for(;;)
  {
		if(osKernelGetTickCount()>led_tick+500)
		{
			led_tick=osKernelGetTickCount();			
			HAL_GPIO_TogglePin(MCU_SYS_health_LED_GPIO_Port,MCU_SYS_health_LED_Pin); 
		}	
		/*****************治疗水***********************/
		if(sGenSta.treatment_water_level_status!=	laser_ctr_param.treatmentWaterLevel)
		{
			sGenSta.treatment_water_level_status=laser_ctr_param.treatmentWaterLevel;
			app_tmc2226_sped_set(laser_ctr_param.treatmentWaterLevel);	//水
		}
 
    if(laser_ctr_param.air_water_prepare_ctr!=local_water_air)	
    {     
      local_water_air=laser_ctr_param.air_water_prepare_ctr;
      if(local_water_air!=0)
      {
        tmc2226_start(1,3);
        timeout=0;
        do
        { 
          osDelay(50);
          timeout+=50; 
          if(timeout>5000)
          {
            DEBUG_PRINTF("treatment water load fail \r\n");
            osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO7_BIT);
            break;
          }
        }while(app_get_io_status(In7_water_ready_ok)!=SUCCESS); 
        if(timeout<5000) 
        {
          osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO7_BIT);
          osDelay(1000);
        }         
        tmc2226_stop();
      }
    }
    /*****************气泵***********************/	
		if(sGenSta.air_level_status!=	laser_ctr_param.airPressureLevel)
		{
			sGenSta.air_level_status=laser_ctr_param.airPressureLevel;			
		}	
		/**********************RGB****************************/		
		if(osMessageQueueGet(rgbQueue02Handle,&rgbRun,0,0)==osOK)
		{
			if(rgbRun==3)
			{
				rgb_color_all(0);//先清除		
			}
			else if(rgbRun==2)
			{
				rgb_color_all(2);
			}
			else if(rgbRun==1)
			{
				rgb_color_all(1);
			}
			else //if(rgbRun==0)
			{
				rgb_color_all(0);
			}
		}
		if(rgbRun==3)
		{		     
			app_rgb_breath_ctl(laser_ctr_param.laserFreq,9);//tim callback
		}
		else if(rgbRun==1)
		{
			Green_Breath();
		}
    osDelay(1);
  }
  /* USER CODE END auxTask02 */
}

/* USER CODE BEGIN Header_keyScanTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_keyScanTask03 */
void keyScanTask03(void *argument)
{
  /* USER CODE BEGIN keyScanTask03 */
  unsigned int recKeyValue,rf24KeyValue;
	app_key_message key_message;   
	RF24_init();
  #ifdef ONE_WIRE_BUS_SLAVE
  one_wire_bus_intit();
  #endif
	/* Infinite loop */    
	for(;;)
	{  
    #ifdef IWDG_USED
    HAL_IWDG_Refresh(&hiwdg1); 
    #endif 
		osDelay(20); 
    #ifdef ONE_WIRE_BUS_SLAVE
    recKeyValue=  app_owb_key_scan(20);
    #else 
    recKeyValue = app_IO_key_scan(20); 
    #endif
    rf24KeyValue  = app_RF24_key_scan(20);		
		if((recKeyValue&0XFF)==IO_KEY_IDLE)
		{	
      recKeyValue|=rf24KeyValue;		
    }  
		key_message=app_key_value_analysis(recKeyValue);		
		if(key_message==key_pwr_long_press)
		{
			osSemaphoreRelease(powerOffBinarySem02Handle);
			key_message=NO_KEY_MESSAGE;			
		}
		else  
		{	
			if(key_message!=NO_KEY_MESSAGE)
			{	
				osStatus_t status = osMessageQueuePut(keyJTMessageQueue01Handle,&key_message,0,0);
				{          
				  if(status!=osOK )
					{ 
						DEBUG_PRINTF("key press too fast\r\n"); 
					}	
					else 
					{
						DEBUG_PRINTF("JT key press %d\r\n",key_message);										
					}	
				}
			}
		}		
	}
  /* USER CODE END keyScanTask03 */
}

/* USER CODE BEGIN Header_laserWorkTask04 */
/**
* @brief Function implementing the laserTask04 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_laserWorkTask04 */
void laserWorkTask04(void *argument)
{
  /* USER CODE BEGIN laserWorkTask04 */
  /* Infinite loop */

  uint8_t recKeyMessage;
	uint8_t reckey=0,pre_flag;
	uint16_t rgbMessage;	//0关闭//1待机绿色2：准备OK紫色常亮；3脉冲输出紫色呼吸
	uint32_t timeout=0,prepare_timeout;	
	LASER_CONTROL_PARAM *pLaserConfig;
	pLaserConfig = &laser_ctr_param;
	osEventFlagsClear(laserEvent02Handle,EVENTS_LASER_JT_ENABLE_BIT);	
  for(;;)
  {
    osStatus_t status1 = osMessageQueueGet(keyJTMessageQueue01Handle,&recKeyMessage,0,10);    	    
    uint32_t event1=osEventFlagsGet(laserEvent02Handle);
    if(sGenSta.laser_run_B6_t_water_prepare_status==0)		
    {
      if(laser_ctr_param.air_water_prepare_ctr!=0)
      {
        if(pre_flag!=0)	  prepare_timeout+=10;
        if(prepare_timeout>5000)	  
        {
          pre_flag=0;
          tmc2226_stop(); 
          sGenSta.laser_run_B6_t_water_prepare_status=1;
        }
        if(recKeyMessage==key_jt_long_press)
        {  
          if(pre_flag==0)	
          {	
            sGenSta.laser_param_B456_jt_status = key_jt_long_press;
            tmc2226_start(1,3); 
            prepare_timeout=0;
            pre_flag=1;		
          }         
        }
        else  //if (reckey==key_jt_release)
        {	          
          if(pre_flag!=0)	 
          {
            sGenSta.laser_param_B456_jt_status = key_jt_release;
            pre_flag=0;
            tmc2226_stop();
          }  
          sGenSta.laser_run_B6_t_water_prepare_status=1;
          DEBUG_PRINTF("treatment preapare success\r\n");
        }
      }
    }
    if(event1!=0)
    {
      if(recKeyMessage==key_jt_long_press)
      {  
        if(reckey==0)	
        {	
          reckey=1;							
          sGenSta.laser_param_B456_jt_status = key_jt_long_press;         
          if(pLaserConfig->treatmentWaterLevel!=0)
          {
            app_deflate_air_solenoid(ENABLE);
            tmc2226_start(1,laser_ctr_param.treatmentWaterLevel);            
          }
          if(sGenSta.laser_run_B0_pro_hot_status!=0)
          {	       
						sGenSta.laser_run_B1_laser_out_status=1;  
						rgbMessage = RGB_LASER_WORK_STATUS;
						osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);						
            app_laser_pulse_start(180,laser_ctr_param.laserFreq);  																					
            DEBUG_PRINTF("laser 1064 start=%d\r\n",recKeyMessage);
          }			
        }						
      }
      else  //if (reckey==key_jt_release)
      {	
        if(reckey!=0)
        {	
          reckey = 0;
          rgbMessage = RGB_LASER_PREPARE_OK;
          osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);
          sGenSta.laser_param_B456_jt_status=key_jt_release;	
					sGenSta.laser_run_B1_laser_out_status=0; 	
          app_laser_pulse_start(LASER_PULSE_STOP,laser_ctr_param.laserFreq);  
          tmc2226_start(0,laser_ctr_param.treatmentWaterLevel); 
          osDelay(120); 
          tmc2226_stop(); 
          app_deflate_air_solenoid(DISABLE);	          
          DEBUG_PRINTF("laser 1064 stop=%d\r\n",recKeyMessage);
       }       
      }   
    }	
    osStatus_t sta = osSemaphoreAcquire(laserCloseSem05Handle,1);//
    if(sta==osOK)
    { 
			if(reckey!=0)
			{
				reckey=0;
				sGenSta.laser_param_B456_jt_status=key_jt_release;	
				sGenSta.laser_run_B1_laser_out_status=0; 	
				app_laser_pulse_start(LASER_PULSE_STOP,laser_ctr_param.laserFreq);  
				tmc2226_start(0,laser_ctr_param.treatmentWaterLevel);  
				osDelay(120);
				tmc2226_stop();  
				app_deflate_air_solenoid(DISABLE);	 
			}
			AD5541A_SetVoltage(0, 4.096);
			osDelay(JDQ_RS485_FRAME_MIN_MS); 
			app_jdq_bus_power_on_off(0);       
			timeout=0;
			do
			{        
				osDelay(JDQ_RS485_FRAME_MIN_MS); 
				timeout+=JDQ_RS485_FRAME_MIN_MS;
				if(timeout>JDQ_RS485_FRAME_MAX_DELAY_MS)
				{           
					break;
				}        
			}while(app_get_jdq_rs485_bus_statu()!=0);
			if (app_jdq_get_vbus_sta()==0)

			{				
				DEBUG_PRINTF("laser close ok\r\n");
			}	
			else 	
			{				
				app_jdq_bus_power_on_off(0);	
			}				
			jdq_reley_charge(0);
			jdq_reley_charge_ready(0);				
			sGenSta.laser_run_B0_pro_hot_status=0;	
			rgbMessage=RGB_G_STANDBY;
			osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);
			osEventFlagsClear(laserEvent02Handle,EVENTS_LASER_PREPARE_OK_ALL_BITS_MASK);	//clear			
    }
    osDelay(1);
  }
  /* USER CODE END laserWorkTask04 */
}

/* USER CODE BEGIN Header_fastAuxTask05 */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fastAuxTask05 */
void fastAuxTask05(void *argument)
{
  /* USER CODE BEGIN fastAuxTask05 */
  /* Infinite loop */
  static uint32_t circleTick;    
  float  treatmentWaterC; 
  float  recVoltage,recCurrent,k0_cool_temprature; 
  for(;;)
  {
		/***********环境气压、温度监测*******************/ 
	 if(sys_load_sta.eTempratureAirpressureSystemLoadFlag==1) 
	 {
		 unsigned char  status=app_gzp6816d_listen(osKernelGetTickCount(),&sEnvParam.air_gzp_enviroment_pressure_kpa,&sEnvParam.enviroment_temprature);      
		 if(sEnvParam.air_gzp_enviroment_pressure_kpa<50&& sEnvParam.air_gzp_enviroment_pressure_kpa>100)//异常
		 {
			 sEnvParam.air_gzp_enviroment_pressure_kpa=94.0;
		 }	
	 }
	 else
	 {
		 sEnvParam.air_gzp_enviroment_pressure_kpa =94.0;
	 } 
  /******************治疗水位********环境温度************/  
    if(osKernelGetTickCount()>circleTick+1000)
    { 
      circleTick=osKernelGetTickCount();	
      //app_mcp61_get_singgle_c_value_req();//液位
      //DEBUG_PRINTF("C_water=%.3f\r\n",app_mcp61_c_value());
      // DEBUG_PRINTF("evnentBits=%04x\r\n",osEventFlagsGet(auxStatusEvent01Handle));
      //DEBUG_PRINTF("NTC=%.2f℃ laser_energe=%.1f iBus=%.1fmA ,vBus=%.1fmV,air_pump_pressure=%.2fkPa\r\n",sEnvParam.NTC_temprature,\
      sEnvParam.laser_1064_energy,sEnvParam.iBus,sEnvParam.vBus,sEnvParam.air_pump_pressure); 
      //DEBUG_PRINTF("ads1118: T1=%.1f℃ T2= %.1f℃ %.1f℃\r\n", sEnvParam.eth_k1_temprature,sEnvParam.eth_k2_temprature,k0_cool_temprature);      
      //DEBUG_PRINTF("gzp_enviroment_air_pressure: kpa=%.2fkPa temprature=%.1f ℃\r\n",sEnvParam.air_gzp_enviroment_pressure_kpa,sEnvParam.enviroment_temprature);
      //eventFlagError= osEventFlagsWait(status_io_updataEvent01Handle,EVENTS_STATUS_IO_ALL_BITS,osFlagsWaitAll,portMAX_DELAY);) 
      //DEBUG_PRINTF("NTC=%.2f℃ laser_energe=%.1f iBus=%.1fmA ,vBus=%.1fmV,air_pump_pressure=%.2fkPa\r\n",sEnvParam.NTC_temprature,\
        sEnvParam.laser_1064_energy,sEnvParam.iBus,sEnvParam.vBus,sEnvParam.air_pump_pressure); 
			/*****************环境温度监测*******************/
			#if 1 
			sEnvParam.enviroment_temprature = M117Z_get_temprature()*1.0; 
			M117Z_start_sampling();	
		  //DEBUG_PRINTF("enviroment_temprature: %.1f℃\r\n",sEnvParam.enviroment_temprature);	
			#endif
	 }
    /***********热电偶温度监测*******************/
    app_ads1118_channel_sampling_start(ADS1118_COOL_CHANNEL);	 
    osDelay(ADS1118_DELEY_TIME_MS);
    k0_cool_temprature= app_ads1118_channel_get_value(ADS1118_COOL_CHANNEL);
    app_ads1118_channel_sampling_start(ADS1118_K1_CHANNEL);	 
    osDelay(ADS1118_DELEY_TIME_MS);
    sEnvParam.eth_k1_temprature = app_ads1118_channel_get_value(ADS1118_K1_CHANNEL);   
    app_ads1118_channel_sampling_start(ADS1118_K2_CHANNEL);	 
    osDelay(ADS1118_DELEY_TIME_MS);
    sEnvParam.eth_k2_temprature = app_ads1118_channel_get_value(ADS1118_K2_CHANNEL);   
    //DEBUG_PRINTF("ads1118: T1=%.1f℃ T2=%.1f℃\r\n", sEnvParam.eth_k1_temprature,sEnvParam.eth_k2_temprature);
    /***********NTC,laser_energe,iB
     * us,vBus,air_pump_pressure气泵气压，参数****************** */     
    app_get_adc_value(AD1_NTC_INDEX,&sEnvParam.NTC_temprature);//蠕动泵，状态
    if(sEnvParam.NTC_temprature>sEnvParam.enviroment_temprature+40)//过热
    {
      osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_9_NTC_BIT);  
    }
    else
    {
      osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_9_NTC_BIT);
    }
    app_get_adc_value(AD1_LASER_1064_INDEX,&sEnvParam.laser_1064_energy);
    app_get_adc_value(AD1_OCP_Ibus_INDEX,&sEnvParam.iBus);
		if(sEnvParam.iBus<MAX_IBUS_MA)//<10A
    {
      osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_10_IBUS_BIT);
    }
    else
    {
      osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_10_IBUS_BIT);
    }   
    app_get_adc_value(AD1_24V_VBUS_INDEX,&sEnvParam.vBus); 
    if(sEnvParam.vBus>MIN_VBUS_MV)
    {
      osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_11_VBUS_BIT);
    }
    else
    {
      osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_11_VBUS_BIT);
    }  
    /***********气泵管理*******************/
		app_get_adc_value(AD1_AIR_PRESSER_INDEX,&sEnvParam.air_pump_pressure);	
		app_air_pump_manage(laser_ctr_param.airPressureLevel);    
		/***********aux genaration状态检查*******************/  
    app_sys_genaration_status_manage();	
		app_fresh_laser_status_param();			
    osDelay(3);
  }
  /* USER CODE END fastAuxTask05 */
}

/* USER CODE BEGIN Header_hmiAppTask06 */

/**
* @brief Function implementing the hmiTask06 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_hmiAppTask06 */
void hmiAppTask06(void *argument)
{
  /* USER CODE BEGIN hmiAppTask06 */
  /* Infinite loop */
  uint32_t timeout;
	uint8_t req_flag;
  for(;;)
  {	
		osStatus_t status = osSemaphoreAcquire(hmiCanBusIdleSem06Handle,HMI_CAN_FRAME_DELAY_TIME);
		if(status==pdTRUE)
		{
			osDelay(50);
		}
		else 
		{
			sGenSta.genaration_io_status=osEventFlagsGet(auxStatusEvent01Handle);
			if(sGenSta.laser_run_B0_pro_hot_status!=0&&sGenSta.laser_run_B1_laser_out_status!=0)
			{		
				req_flag++;
				if(req_flag>1) 
				{							
					app_hmi_report_status(&sGenSta);		
					req_flag=0;				
				}
				else app_hmi_report_pulse_count();						
			}		
			else 
			{
				app_hmi_report_status(&sGenSta);	
			}
		}			
		osDelay(1);
  }
  /* USER CODE END hmiAppTask06 */
}

/* USER CODE BEGIN Header_canReceiveTask07 */
/**
* @brief Function implementing the myTask07 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canReceiveTask07 */
void canReceiveTask07(void *argument)
{
  /* USER CODE BEGIN canReceiveTask07 */
  /* Infinite loop */
  uint8_t data_length;
	uint32_t identifier;	
 	uint8_t canRxData[8]={0};
  for(;;)
  {
    /*******************CAN��RX-DATA********************/	
		osStatus_t sta = osSemaphoreAcquire(CANBusReceiveFrameSem07Handle,portMAX_DELAY);	
    while(FDCAN1_Receive_Msg(canRxData, &identifier, &data_length))
		{						
			if(canRxData[0]==HMI_CAN_FRAME_HEADER)
			{        
				HMI_Parse_Data(canRxData,data_length);			
				/*****************激光指示灯***********************/	
				if(laser_ctr_param.ledLightLevel!=0) 
				{
					if(sGenSta.laser_run_B3_laser_pilot_lamp_status==0)
					{
						app_auxiliary_bulb_pwm(laser_ctr_param.ledLightLevel,ENABLE);
						sGenSta.laser_run_B3_laser_pilot_lamp_status=1;	
					}	
				}
				else
				{
					if(sGenSta.laser_run_B3_laser_pilot_lamp_status!=0)
					{
						app_auxiliary_bulb_pwm(0,DISABLE);
						sGenSta.laser_run_B3_laser_pilot_lamp_status=0;
					}					
				}									
				osSemaphoreRelease(hmiCanBusIdleSem06Handle);
			}
      osDelay(2);   
		}		
   	osDelay(1);   
  }
  /* USER CODE END canReceiveTask07 */
}

/* USER CODE BEGIN Header_powerOffTask08 */
/**
* @brief Function implementing the myTask08 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_powerOffTask08 */
void powerOffTask08(void *argument)
{
  /* USER CODE BEGIN powerOffTask08 */
  /* Infinite loop */
  for(;;)
  {
    osStatus_t status = osSemaphoreAcquire(powerOffBinarySem02Handle,portMAX_DELAY);
    ge2117_start_up_set(GE2117_STOP_CMD);
		if(app_sys_param_save_data()==0)
		{
			DEBUG_PRINTF(" sys  param save ok\r\n");
		}		    
    HAL_Delay(2000);
		app_mcu_power_switch(DISABLE);
		DEBUG_PRINTF("DEVICE POWER OFF\r\n"); 
		while(1)
		{					
			DEBUG_PRINTF("please release power key\r\n");
			HAL_Delay(1000);
      #ifdef IWDG_USED
      HAL_IWDG_Refresh(&hiwdg1); 
      #endif      	
		}	
   // osDelay(1);
  }
  /* USER CODE END powerOffTask08 */
}

/* USER CODE BEGIN Header_laserProhotTask09 */
/**
* @brief Function implementing the myTask09 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_laserProhotTask09 */
void laserProhotTask09(void *argument)
{
  /* USER CODE BEGIN laserProhotTask09 */
  /* Infinite loop */
  uint32_t timeout;
	uint16_t	rgbMessage=1;
  uint16_t laser_freq,laser_Voltage;					
	uint8_t local_proHotCtr=0;			
	float local_f=1.0,l_jdq_set_voltage,l_jdq_set_current;
  for(;;) 
  {
		osStatus_t sta = osSemaphoreAcquire(laserPrapareReqSem03Handle,portMAX_DELAY);//
		local_proHotCtr =	laser_ctr_param.proHotCtr;	
		laser_freq = laser_ctr_param.laserFreq;
		laser_Voltage = app_laser_1064_energe_to_voltage(laser_ctr_param.laserEnerge); 
    local_f =laser_Voltage*1.0;
    if(local_proHotCtr!=0)	
    {	
      app_jdq_bus_get_set_v_c(&l_jdq_set_voltage,&l_jdq_set_current);      
      if(fabsf(local_f - l_jdq_set_voltage)>5.0) 
      {
        osDelay(JDQ_RS485_FRAME_MIN_MS);   
        app_jdq_bus_voltage_set(LASER_JDQ_VOLTAGE_F);
				timeout=0;
				do
				{
					osDelay(JDQ_RS485_FRAME_MIN_MS); 
					timeout+=JDQ_RS485_FRAME_MIN_MS;
					if(timeout>JDQ_RS485_FRAME_MAX_DELAY_MS)
					{				
						break;
					}
				}while(app_get_jdq_rs485_bus_statu()!=0);
        app_jdq_bus_get_set_v_c(&l_jdq_set_voltage,&l_jdq_set_current);
      } 
      if(laser_ctr_param.laserEnerge>35)
      {
			  local_f=1.6+laser_ctr_param.laserEnerge*0.01;
      }
      else 
      {
        local_f=1.75+laser_ctr_param.laserEnerge*0.0013;        
      }
			if(local_f>4.0) local_f=4.0;//250mJ~4.0		
      AD5541A_SetVoltage(local_f, 4.096);
			osDelay(JDQ_RS485_FRAME_MIN_MS);   
      app_jdq_bus_power_on_off(1);
			timeout=0;
			do
			{
				osDelay(JDQ_RS485_FRAME_MIN_MS); 
				timeout+=JDQ_RS485_FRAME_MIN_MS;
				if(timeout>JDQ_RS485_FRAME_MAX_DELAY_MS)
				{				
					break;
				}
			}while(app_get_jdq_rs485_bus_statu()!=0);
      if(app_jdq_get_vbus_sta()!=0)
      {
        app_jdq_current_limit_charge();
        float outVoltage;
        timeout=0;
        do
        {
          timeout+=JDQ_RS485_FRAME_MIN_MS;
          osDelay(JDQ_RS485_FRAME_MIN_MS);
          if(timeout>LASER_JDQ_CHARGE_TIMEOUT_MS)//60s
          {
            sGenSta.laser_run_B0_pro_hot_status=0;
            break;
          }
          outVoltage = app_jdq_voltage_monitor();
          DEBUG_PRINTF("jdq—v%f %d\r\n",app_jdq_voltage_monitor(),laser_Voltage);
          if(outVoltage>80) app_jdq_direct_160v();//>80V,
        } while ( outVoltage+10<laser_Voltage);    //(90%)  
        if(timeout<=LASER_JDQ_CHARGE_TIMEOUT_MS)//ok
        {
          rgbMessage = RGB_LASER_PREPARE_OK;
          osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);        	      
          osEventFlagsSet(laserEvent02Handle,EVENTS_LASER_1064_PREPARE_OK_BIT|EVENTS_LASER_JT_ENABLE_BIT);
          sGenSta.laser_run_B0_pro_hot_status = 1;
        }  
      }    
    }		
    else 
    {
      if(sGenSta.laser_run_B0_pro_hot_status!=0) osSemaphoreRelease(laserCloseSem05Handle);			
    }		
		osDelay(1);	
  }
  /* USER CODE END laserProhotTask09 */
}

/* USER CODE BEGIN Header_ge2117ManageTask10 */
/**
* @brief Function implementing the myTask10 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ge2117ManageTask10 */
void ge2117ManageTask10(void *argument)
{
  /* USER CODE BEGIN ge2117ManageTask10 */
  /* Infinite loop */
  uint32_t local_timeS;
  local_timeS=osKernelGetTickCount()/1000;
  for(;;)
  {
    local_timeS++;
    osDelay(1000);
    app_circle_water_PTC_manage(sEnvParam.eth_k1_temprature,local_timeS);
    app_ge2117_gp_ctr(sEnvParam.eth_k1_temprature,local_timeS);
  }
  /* USER CODE END ge2117ManageTask10 */
}

/* USER CODE BEGIN Header_ditCleanTask11 */
/**
* @brief Function implementing the myTask11 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ditCleanTask11 */
void ditCleanTask11(void *argument)
{
  /* USER CODE BEGIN ditCleanTask11 */
  uint16_t rec_clean;
  uint32_t prepare_timeouts;
  /* Infinite loop */
  for(;;)
  {
    osStatus_t status1 = osMessageQueueGet(DitCleanQueue03Handle,&rec_clean,0,portMAX_DELAY);
    if(status1==osOK)
    {    
      if(rec_clean ==1)//dit,消毒用低速
      {
        tmc2226_start(1,1); 
        prepare_timeouts=0; 
        while(prepare_timeouts<u_sys_param.sys_config_param.laser_config_param.dit_time_min*60)
        {
          osDelay(1000);
          prepare_timeouts+=1;
          if(prepare_timeouts>1200)
          {
            break;
          }
        }   
        tmc2226_stop();
        rec_clean=0;
      }
      else if(rec_clean ==2)//clea
      {       
        tmc2226_start(1,3); 
        prepare_timeouts=0;      
        while(prepare_timeouts<u_sys_param.sys_config_param.laser_config_param.clean_time_min*60)
        {
          osDelay(1000);
          prepare_timeouts++;                  
          if(prepare_timeouts>1200)//20分钟
          {
            break;
          }
        }
        tmc2226_stop();
        rec_clean=0;
      } 
      else 
      {
        tmc2226_stop();
      }   
    }
    osDelay(1);
  }
  /* USER CODE END ditCleanTask11 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
  * @retval None
  */
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {  
  if(GPIO_Pin==LASER_1064_count_in_Pin)
  {
    u_sys_param.sys_config_param.laser_config_param.laser_pulse_count++;//光电管计数
  }
  if(GPIO_Pin==RF24_IRQ_in_Pin)
  {    
    osSemaphoreRelease(RF24_JT_BinarySem01Handle) ;  
  }
	#ifdef ONE_WIRE_BUS_SLAVE 
  if(GPIO_Pin==FOOT_SWITCH_IN_Pin)
  {    
    owb_dq_falling_callback();    
  }
	#endif
 }
 /************************************************************************//**
  * @brief 系统一般状态信号管理
  * @param 无
  * @note   周期性更新事件正常标志
  * @retval 无
  *****************************************************************************/
void app_sys_genaration_status_manage(void)
{ //高压电磁阀
  if(app_get_io_status(In1_high_voltage_solenoid)==SUCCESS)
  {      
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO1_BIT);//更新状态正常事件标志
  }
  else 
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO1_BIT);//清除正常标志 
  } //堵气阀
  if(app_get_io_status(In2_deflate_air_solenoid)==SUCCESS)
  {      
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO2_BIT);
  }
  else 
  {   
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO2_BIT);
  }//泄气阀
  if(app_get_io_status(In3_chocke_air_solenoid)==SUCCESS)
  {  
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO3_BIT);
  }
  else 
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO3_BIT);
  } //环境温度报警
  if(app_get_io_status(In4_enviroment_tmprature_alert)==SUCCESS)
  {  
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO4_BIT);
  }
  else 
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO4_BIT);
  }	//气泵过热报警
	if(app_get_io_status(In5_h_air_error)==SUCCESS)
	{  
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO5_BIT); 
	}
	else 
  {   
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO5_BIT);
  }//气泵气压过高信号报警
	if(app_get_io_status(In6_Hyperbaria_OFF_Signal)==SUCCESS)
	{ 
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO6_BIT);  
	}
	else
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO6_BIT);
  }
  //治疗水OK就绪信号
 
	if(app_get_io_status(In7_water_ready_ok)==SUCCESS&& sEnvParam.treatment_water_depth!=0)
	{  
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO7_BIT);
	}
	else 
  {  
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO7_BIT);
  }//水循环就绪信号
	if(app_get_io_status(In8_water_circle_ok)==SUCCESS)
	{  
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO8_BIT);
	}
	else 
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO8_BIT);
  }

  if(sEnvParam.eth_k1_temprature>ERR_LOW_TEMPRATURE_LASER&&sEnvParam.eth_k1_temprature<ERR_HIGH_TEMPRATURE_LASER)
  {
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_12_K1_TEMPRATURE_BIT);
  }  
  else
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_12_K1_TEMPRATURE_BIT);
  } 
  if(sEnvParam.eth_k2_temprature>ERR_LOW_TEMPRATURE_LASER&&sEnvParam.eth_k2_temprature<ERR_HIGH_TEMPRATURE_LASER)
  {
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_13_K2_TEMPRATURE_BIT);
  }  
  else
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_13_K2_TEMPRATURE_BIT);
  }   
  //DEBUG_PRINTF("IO8~1=%d%d%d%d%d%d%d%d\r\n" ,sGenSta.water_circle_ok_status,sGenSta.water_ready_ok_status,\
    sGenSta.Hyperbaria_OFF_Signal_staus,sGenSta.h_air_error_status ,sGenSta.enviroment_tmprature_alert_status,\
    sGenSta.chocke_air_solenoid_status, sGenSta.deflate_air_solenoid_status,sGenSta.high_voltage_solenoid_status);
}
/************************************************************************//**
  * @brief 使用默认系统参数
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
void app_set_default_sys_config_param(void)
{	
	u_sys_param.sys_config_param.synchronousFlag = EEROM_DATA_ERR_CHECK_FLAG;
	u_sys_param.sys_config_param.equipmentModel=1;//1064
	u_sys_param.sys_config_param.softVersion=250100;//25Ver1.0
	u_sys_param. sys_config_param.laser_config_param.equipmentId= DBGMCU->IDCODE;//HAL_GetDEVID();;//
  u_sys_param. sys_config_param.laser_config_param.jtId=0;
  u_sys_param. sys_config_param.laser_config_param.jt_status=0; 
  u_sys_param. sys_config_param.laser_config_param.cool_temprature_low=210;//21.0f
  u_sys_param. sys_config_param.laser_config_param.cool_temprature_target=230;//23.0f
  u_sys_param. sys_config_param.laser_config_param.cool_temprature_high=280;//28.0f
  u_sys_param. sys_config_param.laser_config_param.photodiod_low=51;//50mj
  u_sys_param. sys_config_param.laser_config_param.photodiod_mid=99;//100mJ
  u_sys_param. sys_config_param.laser_config_param.photodiod_high=200;//205mJ
  u_sys_param. sys_config_param.laser_config_param.laser_pulse_count=0;
  u_sys_param. sys_config_param.laser_config_param.laser_use_timeS=0;
  u_sys_param. sys_config_param.laser_config_param.RDB_use_timeS=0;
  u_sys_param. sys_config_param.laser_config_param.laser_pulse_width_us=180;//180us
  for(uint8_t i=0;i<36;i++)
  {  
    u_sys_param. sys_config_param.laser_config_param.e_cali[i].energe_dw=20+5*i;//20mJ
    u_sys_param. sys_config_param.laser_config_param.e_cali[i].energe_cali=256;
    u_sys_param. sys_config_param.laser_config_param.e_cali[i].power_cali=12;//
  }
  u_sys_param. sys_config_param.laser_config_param.treatment_water_depth_high=151;//15.1pf
  u_sys_param. sys_config_param.laser_config_param.treatment_water_depth_low=87;//8.7pF
  u_sys_param. sys_config_param.laser_config_param.cool_water_depth_high=167;//16.7pf
  u_sys_param. sys_config_param.laser_config_param.cool_water_depth_low=52;//
  u_sys_param. sys_config_param.laser_config_param.air_dw=3;
  u_sys_param. sys_config_param.laser_config_param.air_low_pressure=160;//160kPa
  u_sys_param. sys_config_param.laser_config_param.air_mid_pressure=180;//180kPa
  u_sys_param. sys_config_param.laser_config_param.air_high_pressure=200;//200kPa
  u_sys_param. sys_config_param.laser_config_param.t_water_dw=0;
  u_sys_param. sys_config_param.laser_config_param.t_water_low=15;//10ml/min
  u_sys_param. sys_config_param.laser_config_param.t_water_mid=25;
  u_sys_param. sys_config_param.laser_config_param.t_water_high=35;
  u_sys_param. sys_config_param.laser_config_param.dit_time_min=5;//5min
  u_sys_param. sys_config_param.laser_config_param.clean_time_min=5;
  u_sys_param. sys_config_param.laser_config_param.tec_switch=1;
  u_sys_param. sys_config_param.laser_config_param.low_freq=60;//MAX60Hz

  u_sys_param. sys_config_param.laser_config_param.charge_width_us=180;//180us
	u_sys_param. sys_config_param.laser_config_param.laser_pulse_count=0;
  u_sys_param. sys_config_param.laser_config_param.laser_use_timeS=0;
	u_sys_param. sys_config_param.laser_config_param.RDB_use_timeS=0;
	u_sys_param.sys_config_param.checkSum=sumCheck(u_sys_param.data,sizeof(SYS_CONFIG_PARAM)-4);	 
	DEBUG_PRINTF("sys param load failed! load defalut param\r\n"); 
}
/************************************************************************//**
  * @brief 加载本地系统参数
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
  void app_sys_param_load(void)
  {
    unsigned char flag;
    flag= EEPROM_M24C32_Read(EEROM_SYS_PARAM_SAVE_ADDR, u_sys_param.data, sizeof(SYS_CONFIG_PARAM));
    unsigned int sum=sumCheck(u_sys_param.data,sizeof(SYS_CONFIG_PARAM)-4);
    if(u_sys_param.sys_config_param.synchronousFlag!=EEROM_DATA_ERR_CHECK_FLAG||u_sys_param.sys_config_param.checkSum!=sum)//
    {
      app_set_default_sys_config_param();
      DEBUG_PRINTF("load default sys param\r\n");		 
    }
    else 
    {
      memcpy(u_sys_default_param.data,u_sys_param.data,sizeof(SYS_CONFIG_PARAM));
      DEBUG_PRINTF("sys param read ok= %d\r\n",u_sys_param.sys_config_param.laser1064PulseCount);
    }	  
  }
/************************************************************************//**
  * @brief 系统参数保存到本地
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 unsigned char app_sys_param_save_data(void)
 {
    unsigned char flag=1;	 
    if(compare_buff_no_change(u_sys_param.data,u_sys_default_param.data,sizeof(SYS_CONFIG_PARAM)))//有变化
    {
      u_sys_param.sys_config_param.checkSum=sumCheck(u_sys_param.data,sizeof(SYS_CONFIG_PARAM)-4);
      flag = EEPROM_M24C32_Write(EEROM_SYS_PARAM_SAVE_ADDR, u_sys_param.data, sizeof(SYS_CONFIG_PARAM));		
    }		
    return flag;
 }
 /************************************************************************//**
  * @brief 气泵管理
  * @param air_level 气泵气压等级
  * @note   
  * @retval 
  *****************************************************************************/
 void app_air_pump_manage(unsigned char air_level)
 {
		uint32_t eventFlag= osEventFlagsGet(auxStatusEvent01Handle);
		 float air_pressure=MID_AIR_PUMP_PRESSURE+sEnvParam.air_gzp_enviroment_pressure_kpa;
		if(air_level<=1)
    {
      air_pressure=u_sys_param.sys_config_param.laser_config_param.air_low_pressure+sEnvParam.air_gzp_enviroment_pressure_kpa;
      //air_pressure = MIN_AIR_PUMP_PRESSURE+sEnvParam.air_gzp_enviroment_pressure_kpa;
    }
    else  if(air_level==3)
    {
      air_pressure=u_sys_param.sys_config_param.laser_config_param.air_mid_pressure+sEnvParam.air_gzp_enviroment_pressure_kpa;
      //air_pressure=MAX_AIR_PUMP_PRESSURE+sEnvParam.air_gzp_enviroment_pressure_kpa;			
    }
    else //if(air_level==2)
    {
      air_pressure=u_sys_param.sys_config_param.laser_config_param.air_high_pressure+sEnvParam.air_gzp_enviroment_pressure_kpa;
      //air_pressure=MID_AIR_PUMP_PRESSURE+sEnvParam.air_gzp_enviroment_pressure_kpa;
    }
    if(((eventFlag&EVENTS_AUX_STATUS_IO6_BIT)!=EVENTS_AUX_STATUS_IO6_BIT)|| sEnvParam.air_pump_pressure>air_pressure+2 )//
    {			
      app_air_pump_switch(DISABLE);      
    }
		else 
		{
			if(sEnvParam.air_pump_pressure+2 < air_pressure)
			{				
				app_air_pump_switch(ENABLE);
			}
		} 
		if(sEnvParam.air_pump_pressure<(MIN_AIR_PUMP_PRESSURE + sEnvParam.air_gzp_enviroment_pressure_kpa) )	
		{
			sGenSta.laser_param_B23_air_pump_pressure_status = 0;
		}	
		else if(sEnvParam.air_pump_pressure > ( MAX_AIR_PUMP_PRESSURE + sEnvParam.air_gzp_enviroment_pressure_kpa) )	
		{
			sGenSta.laser_param_B23_air_pump_pressure_status = 2;
		}	
		else 
		{
			sGenSta.laser_param_B23_air_pump_pressure_status = 1;
		}			
 }
 /************************************************************************//**
  * @brief 更新状态参数
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
  void app_fresh_laser_status_param(void)
  {
    static float historyCircleWaterTemprature=0;
    if(historyCircleWaterTemprature == 0) historyCircleWaterTemprature = sEnvParam.enviroment_temprature;
    if(fabsf(historyCircleWaterTemprature - sEnvParam.eth_k1_temprature)>0.5)
    {
      historyCircleWaterTemprature=sEnvParam.eth_k1_temprature;      
    }
    sGenSta.circle_water_box_temprature = (char)((int)historyCircleWaterTemprature);
    sGenSta.treatment_water_level_status = laser_ctr_param.treatmentWaterLevel;
    sGenSta.air_level_status = laser_ctr_param.airPressureLevel;		 
    sGenSta.laser_run_B0_pro_hot_status = laser_ctr_param.proHotCtr;
    sGenSta.laser_run_B2_gx_test_status = laser_ctr_param.ctrTestMode;
    sGenSta.laser_param_B01_energe_status = 1;
    sGenSta.genaration_io_status = osEventFlagsGet(auxStatusEvent01Handle)&EVENTS_AUX_STATUS_ALL_BITS;	 
  }
  /************************************************************************//**
  * @brief laser
  * @param energe ,能量
  * @note   能量单位mJ
  * @retval  换算后电压100mV
  *****************************************************************************/
  unsigned short int  app_laser_1064_energe_to_voltage(unsigned short int energe)
  {
    unsigned short int ret_vol;
    /* HSM_III_lite
      ret_vol=energe*32+8000;
      if(ret_vol<LASER_1064_MIN_ENERGE_V) ret_vol=LASER_1064_MIN_ENERGE_V;
      if(ret_vol>LASER_1064_MAX_ENERGE_V) ret_vol=LASER_1064_MAX_ENERGE_V;
    */
    ret_vol= LASER_JDQ_VOLTAGE;
    return ret_vol;
  }
  #ifdef ONE_WIRE_BUS_SLAVE
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
  recLen= app_owb_get_receive_pack_len();
  if(recLen!=0)
  {    
    app_owb_receive_handle(owb_buff,recLen);     
   // DEBUG_PRINTF("JT owb recLen= %d %d %d %d %d %d",recLen,owb_buff[0],owb_buff[1],owb_buff[2],owb_buff[3],owb_buff[4]);
    if(owb_buff[0]=='[' && owb_buff[3]==']')
    {      
      if(owb_buff[1]==2)
      {
        timeout[0]+=100;
        timeout[1]=0;
        if(timeout[0]>=1000)//1s 时间不大准
        {
          timeout[0]=0;						
          owb_key_value=owb_buff[1]; 
          historyKey=owb_buff[1];
        }
      }					
      else //if(owb_buff[1]==3)
      {
        if(historyKey==2)//需要释放
        {
          historyKey=3;
          owb_key_value=3;
          timeout[0]=0;
        }
        else 
        {
          owb_key_value=0;
          timeout[0]=0;			
        }						
      }					               
    }
    else
    {			
      timeout[0]=0;		
      owb_key_value=0;
    }			
  }
  else
  {
    if(historyKey==2)//失联
    {
      timeout[1]+=timeMs;
      if(timeout[1]>2000)
      {
        timeout[1] = 0;
        timeout[0]=0;
        historyKey = 0;
        owb_key_value     = 3;
      }
      else owb_key_value=0;
    }			
    else owb_key_value=0;
  }
  return   owb_key_value;
 }
 #endif
 /***************************extern api**********************************************************/
 /************************************************************************//**
  * @brief 给出canBus数据接收信号量
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 void app_canBbus_receive_semo(void) 
 {
		osSemaphoreRelease( CANBusReceiveFrameSem07Handle);
 }
 /************************************************************************//**
  * @brief 给出激光准备信号量
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
  void app_laser_preapare_semo(void)
  {
    osSemaphoreRelease( laserPrapareReqSem03Handle); 
  }
  /************************************************************************//**
  * @brief 消毒、清洗指令
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 void app_treatment_water_cleaN_ctr(unsigned char *cleaflag)
 {
   osMessageQueuePut(DitCleanQueue03Handle,cleaflag,0,1);
 }
/* US
/* USER CODE END Application */

