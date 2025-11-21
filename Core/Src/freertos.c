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
#include "max31865atp_bsp.h"
#include "IS31FL3193_bsp.h" 
#include "tmc2226_step_bsp.h"
#include "mer_mcp1081_bsp.h"
#include "ge2117_gp_bsp.h"
#include  "jdq_bsp.h"
#include "drv_RF24L01.h"
#include "user_can1.h"
#include "fan_bsp.h"
#include "buzz_bsp.h"
#ifdef ONE_WIRE_BUS_JT_SLAVE
#include "one_wire_bus.h"
#endif

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */
sys_genaration_status sGenSta;
sys_enviroment_assistant_param sEnvParam;
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
#define EVENTS_AUX_STATUS_15_WATER_AIR_PREPARE_BIT          0x0001<<14
#define EVENTS_AUX_STATUS_16_CLEAN_BIT                      0x0001<<15

#define EVENTS_AUX_STATUS_ALL_BITS     (EVENTS_AUX_STATUS_IO1_BIT|EVENTS_AUX_STATUS_IO2_BIT|EVENTS_AUX_STATUS_IO3_BIT|EVENTS_AUX_STATUS_IO4_BIT|EVENTS_AUX_STATUS_IO5_BIT\
			|EVENTS_AUX_STATUS_IO6_BIT|EVENTS_AUX_STATUS_IO7_BIT|EVENTS_AUX_STATUS_IO8_BIT|EVENTS_AUX_STATUS_9_NTC_BIT|EVENTS_AUX_STATUS_10_IBUS_BIT|EVENTS_AUX_STATUS_11_VBUS_BIT\
			|EVENTS_AUX_STATUS_12_K1_TEMPRATURE_BIT|EVENTS_AUX_STATUS_13_K2_TEMPRATURE_BIT|EVENTS_AUX_STATUS_14_EMERGENCY_KEY_BIT| EVENTS_AUX_STATUS_15_WATER_AIR_PREPARE_BIT |EVENTS_AUX_STATUS_16_CLEAN_BIT\
    )
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
//static float voltage[16]={\
1.5,1.6,1.7,1.8,1.9,2.0,2.1,2.2,2.3,2.4,2.5,\
	2.7,2.9,3.1,3.5,3.8}
//static float jd_pulse_timeus[16]={\
	62,56,50,46.5,43,40,38,35.5,33.5,32,30.5,\
	28.5,26.5,25,22.5,20.5};
//1.5V~3.8V 0.1步进
//static unsigned short int  jdq_pulse_pro_timeUs[24]={\
  69,63,57,54,50,47,45,43,41,39,38,\
  37,36,34,33,32,31,31,30,30,30,29,28,27};
  //脉冲触发时间：电脉宽=脉冲触发时间+光脉宽+7(延长)
  //100us最低脉宽，初始能量值
  static unsigned short int  jdq_laser_voltage_energe[26]={//(1.5V~4.0V)//0~230mJ//
    5,15,20,27,32,37,42,47,52,57,66,
    72,79,86,93,99,106,113,120, 127,\
    134,141,148,155,162,169
  };
  
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
uint32_t myTask03Buffer[ 288 ];
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
  .stack_size = 320 * 4,
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
  .stack_size = 448 * 4,
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
/* Definitions for myTask12 */
osThreadId_t myTask12Handle;
const osThreadAttr_t myTask12_attributes = {
  .name = "myTask12",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for rgbQueue02 */
osMessageQueueId_t rgbQueue02Handle;
const osMessageQueueAttr_t rgbQueue02_attributes = {
  .name = "rgbQueue02"
};
/* Definitions for musicQueue03 */
osMessageQueueId_t musicQueue03Handle;
const osMessageQueueAttr_t musicQueue03_attributes = {
  .name = "musicQueue03"
};
/* Definitions for keyJTMessageQueue01 */
osMessageQueueId_t keyJTMessageQueue01Handle;
const osMessageQueueAttr_t keyJTMessageQueue01_attributes = {
  .name = "keyJTMessageQueue01"
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
osStaticEventGroupDef_t laserEvent02ControlBlock;
const osEventFlagsAttr_t laserEvent02_attributes = {
  .name = "laserEvent02",
  .cb_mem = &laserEvent02ControlBlock,
  .cb_size = sizeof(laserEvent02ControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
unsigned  int app_get_cali_devid(void);
void app_set_default_sys_config_param(void);
void app_sys_genaration_status_manage(void);
void app_sys_param_load(void);
unsigned char app_sys_param_save_data(void);
void app_air_pump_manage(unsigned char air_level);
void app_pwr_gx_semo(unsigned char code);
void app_fresh_laser_status_param(void);
unsigned short int  app_laser_1064_energe_to_voltage(unsigned short int energe);
unsigned short int app_hmi_package_check(unsigned char* pBuff,unsigned short int buffLen) ;
void app_t_clean_run_timer(unsigned char *runflag);
void app_treatment_water_prepare(unsigned char *ctrflag,unsigned int runtimeMs);
ErrorStatus app_laser_timer_ctr(unsigned char ctrTimeS, unsigned int TimeMs);
void app_buzz_music(music_type  music_num,unsigned char volume);
void app_jdq_restart(void);
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
void waterAirCleanTask11(void *argument);
void musicTask12(void *argument);

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

  /* creation of musicQueue03 */
  musicQueue03Handle = osMessageQueueNew (3, sizeof(uint16_t), &musicQueue03_attributes);

  /* creation of keyJTMessageQueue01 */
  keyJTMessageQueue01Handle = osMessageQueueNew (3, sizeof(uint16_t), &keyJTMessageQueue01_attributes);

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
  myTask11Handle = osThreadNew(waterAirCleanTask11, NULL, &myTask11_attributes);

  /* creation of myTask12 */
  myTask12Handle = osThreadNew(musicTask12, NULL, &myTask12_attributes);

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
  uint8_t load_sta;
  float treatmentWaterC; 
  app_mcu_power_switch(ENABLE);    
  app_lcd_power_12V_switch(ENABLE);
  for(;;)
  {  
    //Fan
    fan_init();
    //load 
    HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);    
    app_beep_pwm(music_tab_c[14],50);
    HAL_Delay(400);     
    app_beep_pwm(0,0);  
    do
		{
			HAL_Delay(500);//beep
			DEBUG_PRINTF("please release power key%d\r\n",HAL_GPIO_ReadPin(KEY_PWR_SWITCH_GPIO_Port,KEY_PWR_SWITCH_Pin));				 
		}while(HAL_GPIO_ReadPin(KEY_PWR_SWITCH_GPIO_Port,KEY_PWR_SWITCH_Pin)==GPIO_PIN_RESET); 
    
    DEBUG_PRINTF("load system config param\r\n");
    EEPROM_M24C32_init();	
    app_sys_param_load();       
    DEBUG_PRINTF("DEVICEID=%x\r\n",u_sys_param.sys_config_param.equipmentId);
    DEBUG_PRINTF("load hmi lcd power...\r\n");  
    app_lcd_power_12V_switch(ENABLE);   
    DEBUG_PRINTF("load rgb...  \r\n");
    IS3_init();   
    load_sta=0;
    DEBUG_PRINTF("load cool compressor ...\r\n");
    ge2117_UART_Init(9600);
    DEBUG_PRINTF("load circle water system...\r\n"); 
    app_circle_water_pump_switch( ENABLE );     
    timeout=0;
    do
    { 
      HAL_Delay(100);
      timeout+=100; 
      if(timeout>2000)
      {
        app_circle_water_pump_switch( DISABLE);
        DEBUG_PRINTF("load circle load fail  \r\n");
        load_sta=3;//err
        break;
      }
    }while(app_get_io_status(In8_water_circle_ok)!=SUCCESS);   
    if(load_sta==0)  
    {
      DEBUG_PRINTF("load circle load ok  \r\n");      
    }     
    load_sta = 0;
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
        load_sta=3;// err
        break;
      }          
    }while(GZP6816D_IsBusy()!=0);               
    if(load_sta==0)  
    {
      GZP6816D_get_cal(&sEnvParam.air_gzp_enviroment_pressure_kpa,&sEnvParam.enviroment_temprature);  
      DEBUG_PRINTF("eViromentAirPressure load ok %.2f kPa  eviroment temprature =%.1f ℃\r\n",sEnvParam.air_gzp_enviroment_pressure_kpa,sEnvParam.enviroment_temprature); 
    }  
    load_sta=0;  
    DEBUG_PRINTF("load cool tmprature  measure system...\r\n");   
		max_31865_pt1000();	
    HAL_Delay(300);//50HZ
    float temp_t_f=Get_pt_tempture();
		if(temp_t_f<60.0&&temp_t_f>-40.0)
		{
			sEnvParam.eth_k1_temprature= temp_t_f;
			sEnvParam.eth_k2_temprature = temp_t_f; 			
		}
		else 
		{
			sEnvParam.eth_k1_temprature= 25.5; //err
			sEnvParam.eth_k2_temprature = 25.0; 
			load_sta=1;			
		}
    if(load_sta==0) 
    {
      DEBUG_PRINTF("PT1000 load ok k1_T=%.1f k2_T=%.1f \r\n",sEnvParam.eth_k1_temprature,sEnvParam.eth_k2_temprature); 
    }
    else DEBUG_PRINTF("PT1000 load fail  \r\n");
    /***********NTC,laser_energe,iBus,vBus,air_pump_pressure气泵气压，参数****************** */    
    app_start_multi_channel_adc();
    DEBUG_PRINTF("load adc sampling NTC ,laserenergetic,iBus,vbus,air pressure...\r\n");
    HAL_Delay(100);//wait >64ms 
    app_get_adc_value(AD1_NTC_INDEX,&sEnvParam.NTC_temprature);
   // app_get_adc_value(AD2_LASER_1064_INDEX,&sEnvParam.laser_1064_energy);
    app_get_adc_value(AD1_OCP_Ibus_INDEX,&sEnvParam.iBus);
    app_get_adc_value(AD1_24V_VBUS_INDEX,&sEnvParam.vBus);    
    app_get_adc_value(AD1_AIR_PRESSER_INDEX,&sEnvParam.air_pump_pressure);
    DEBUG_PRINTF("adc load:NTC=%.2f℃ laser_energe=%.1f iBus=%.1fmA ,vBus=%.1fmV,air_pump_pressure=%.2fkPa\r\n",sEnvParam.NTC_temprature,\
      sEnvParam.laser_1064_energy,sEnvParam.iBus,sEnvParam.vBus,sEnvParam.air_pump_pressure); 
     
    if(sEnvParam.NTC_temprature>-40&&sEnvParam.NTC_temprature<150)
    {      
      DEBUG_PRINTF("adc load:NTC_ad_channel ok=%.2f℃\r\n",sEnvParam.NTC_temprature);         
    }
    else
    {
      DEBUG_PRINTF("adc load:NTC_ad_channel error\r\n"); 
    } 
    if(sEnvParam.laser_1064_energy>50)
    {
      DEBUG_PRINTF("adc load:laser_1064_energy_ad_channel error\r\n");    
    }     
    if(sEnvParam.iBus>MAX_IBUS_MA)
    {
      DEBUG_PRINTF("adc load:ibus_ad_channel error\r\n");      
    }   
    else 
    {
      DEBUG_PRINTF("adc load:ibus_ad_channel ok=%.2fmA\r\n",sEnvParam.iBus); 
    }       
    if(sEnvParam.vBus<MIN_VBUS_MV)
    {
      DEBUG_PRINTF("adc load:vbus_ad_channel error\r\n");
    }   
    else 
    {      
      DEBUG_PRINTF("adc load:vbus_ad_channel ok=%.2fmV\r\n",sEnvParam.vBus); 
    }  
    if(sEnvParam.air_pump_pressure+20<sEnvParam.air_gzp_enviroment_pressure_kpa)
    {
      DEBUG_PRINTF("adc load:air_pressure_ad_channel error\r\n");
    }   
    else 
    {                
      DEBUG_PRINTF("adc load:air_pressure_ad_channel ok=%.2fkPa\r\n",sEnvParam.air_pump_pressure); 
    }   
    load_sta=0;
    float s_jdq_set_voltage,s_jdq_set_current;      
    DEBUG_PRINTF("load laser jdq power system ... \r\n");  
    jdq_init(); 
  #ifdef JDQ_PWR_GWB_3200W   
    HAL_Delay(JDQ_RS485_FRAME_MAX_DELAY_MS);     
    app_jdq_bus_power_on_off(1);
    timeout = 0;
    do
    {
      HAL_Delay(JDQ_RS485_FRAME_MIN_MS);         
      timeout+=JDQ_RS485_FRAME_MIN_MS;    
    }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);    
    HAL_Delay(JDQ_RS485_FRAME_MIN_MS); 
    app_jdq_bus_vol_current_set(LASER_JDQ_VOLTAGE_F,LASER_JDQ_CURRENT_LIMIT_F);
    timeout = 0;
    do
    {
      HAL_Delay(JDQ_RS485_FRAME_MIN_MS);         
      timeout+=JDQ_RS485_FRAME_MIN_MS;    
    }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
    
    HAL_Delay(JDQ_RS485_FRAME_MAX_DELAY_MS); 
    app_jdq_bus_power_onoff_sta_req();
    timeout = 0;
    do
    {
      HAL_Delay(JDQ_RS485_FRAME_MIN_MS);         
      timeout+=JDQ_RS485_FRAME_MIN_MS;    
    }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
    if(app_jdq_get_vbus_sta()!=0)    
    {
      DEBUG_PRINTF("lasr jdq  load ok jdq_v=%.1f v \r\n",app_jdq_get_vbus_sta()*0.01);
      HAL_Delay(JDQ_RS485_FRAME_MAX_DELAY_MS);     
      app_jdq_bus_power_on_off(0);
      timeout = 0;
      do
      {
        HAL_Delay(JDQ_RS485_FRAME_MIN_MS);         
        timeout+=JDQ_RS485_FRAME_MIN_MS;    
      }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS); 
      load_sta = 1;
    }
    else 
    {
      load_sta=3;
      DEBUG_PRINTF("laser jdq  load fail \r\n");      
    } 
#else
    //HAL_Delay(JDQ_RS485_FRAME_MIN_MS);
    HAL_Delay(3000);
    app_jdq_bus_power_on_off(0);
    timeout = 0;
    do
    {
      HAL_Delay(JDQ_RS485_FRAME_MIN_MS);         
      timeout+=JDQ_RS485_FRAME_MIN_MS;    
    }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
    HAL_Delay(JDQ_RS485_FRAME_MIN_MS);   
    app_jdq_bus_get_v_c_req();
    timeout=0;
    do
    {
      HAL_Delay(JDQ_RS485_FRAME_MIN_MS); 
      timeout+=JDQ_RS485_FRAME_MIN_MS;           				
    }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
    app_jdq_bus_get_set_v_c(&s_jdq_set_voltage,&s_jdq_set_current);
    if(s_jdq_set_voltage!=LASER_JDQ_VOLTAGE_F)
    {
      HAL_Delay(JDQ_RS485_FRAME_MIN_MS);
      app_jdq_bus_voltage_set(LASER_JDQ_VOLTAGE_F);
      timeout = 0;
      do
      {
        HAL_Delay(JDQ_RS485_FRAME_MIN_MS);         
        timeout+=JDQ_RS485_FRAME_MIN_MS;    
      }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
    }
    if(s_jdq_set_current!=LASER_JDQ_CURRENT_LIMIT_F)    
    {
      HAL_Delay(JDQ_RS485_FRAME_MIN_MS);
      app_jdq_bus_current_set(LASER_JDQ_CURRENT_LIMIT_F);
      timeout = 0; 
      do
      {
        HAL_Delay(JDQ_RS485_FRAME_MIN_MS);         
        timeout+=JDQ_RS485_FRAME_MIN_MS;    
      }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS); 
    }
    HAL_Delay(JDQ_RS485_FRAME_MIN_MS);   
    app_jdq_bus_get_v_c_req();
    timeout=0;
    do
    {
      HAL_Delay(JDQ_RS485_FRAME_MIN_MS); 
      timeout+=JDQ_RS485_FRAME_MIN_MS;           				
    }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
    app_jdq_bus_get_set_v_c(&s_jdq_set_voltage,&s_jdq_set_current);
    if(s_jdq_set_voltage>LASER_JDQ_VOLTAGE_F+5.0||s_jdq_set_voltage+5.0<LASER_JDQ_VOLTAGE_F)
    {
      load_sta=3;
      DEBUG_PRINTF("laser jdq  load fail \r\n"); 
    }
    else
    {
      HAL_Delay(50);		
      DEBUG_PRINTF("lasr jdq  load ok jdq_v=%.1f v \r\n",s_jdq_set_voltage); 
      load_sta = 1;
    } 
  #endif   
    app_laser_pulse_start(LASER_PULSE_STOP,20,0);      
    load_sta=0;
    DEBUG_PRINTF("load treatment water system...\r\n");
    tmc2226_init();  
    if(app_get_io_status(In7_water_ready_ok)!=SUCCESS) 
    {
      tmc2226_start(1,3);
      timeout=0;
      do
      { 
        HAL_Delay(50);
        timeout+=50; 
        if(timeout>5000)
        {
          DEBUG_PRINTF("treatment water load fail \r\n");          
          load_sta=3;
          break;
        }
      }while(app_get_io_status(In7_water_ready_ok)!=SUCCESS); 
    }
    tmc2226_stop();
    if(load_sta==0)
    {     
      load_sta=1;
      DEBUG_PRINTF(" treatment water load ok \r\n");   
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
  uint32_t led_tick;
	uint16_t rgbRun=1;
	led_tick=osKernelGetTickCount();
  
  for(;;)
  {
		if(osKernelGetTickCount()>led_tick+1000)
		{      
			led_tick=osKernelGetTickCount();	
      app_fan_manage(led_tick);		
			HAL_GPIO_TogglePin(MCU_SYS_health_LED_GPIO_Port,MCU_SYS_health_LED_Pin); 
		}	
		/**********************RGB****************************/		
		osMessageQueueGet(rgbQueue02Handle,&rgbRun,0,0);		
    switch(rgbRun)
    {
      case 0:
        rgb_color_all(0);
      break;
      case 1:
        Green_Breath();
      break;
      case 2:
        rgb_color_all(2);
      break;
      case 3:
        app_rgb_breath_ctl(laser_ctr_param.laserFreq,9);
      break;
      default:
        rgb_color_all(0);
      break;
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
  static app_key_message history_key_message;   
	RF24_init();
  #ifdef ONE_WIRE_BUS_JT_SLAVE
  one_wire_bus_init();
  #endif  
	/* Infinite loop */    
	for(;;)
	{  
    #ifdef IWDG_USED
    HAL_IWDG_Refresh(&hiwdg1); 
    #endif 
		osDelay(50);
    recKeyValue = app_IO_key_scan(50); 
    rf24KeyValue  = app_RF24_key_scan(50);
    u_sys_param.sys_config_param.jt_status=1;
    if(u_sys_param.sys_config_param.jt_status!=0)
    {
      if((recKeyValue&0XFF)==IO_KEY_IDLE)
      {
        if(rf24KeyValue!=KEY_NO_CONNECT)
        {        
          recKeyValue|=rf24KeyValue;
        }      
      } 
      else if((recKeyValue&0XFF)==KEY_NO_CONNECT)
      {        
        recKeyValue&=0xFF00;
        recKeyValue|=rf24KeyValue;
      } 
    }
    else 
    {
      recKeyValue&=0xFF00;
    } 
		key_message=app_key_value_analysis(recKeyValue);		
		if(key_message==key_pwr_long_press)
		{
      sGenSta.laser_run_B6_close_device_status=1;
			osSemaphoreRelease(powerOffBinarySem02Handle);
			key_message = NO_KEY_MESSAGE;	
      history_key_message=key_message;
		}
		else  
		{	
			if(key_message!=NO_KEY_MESSAGE)
			{	
        if(app_remote_key_sta()==ERROR)
        {
          DEBUG_PRINTF("JT remote locked\r\n"); 
          if(history_key_message!=key_jt_release)
          {
            history_key_message = key_jt_release;
            osStatus_t status2  = osMessageQueuePut(keyJTMessageQueue01Handle,&history_key_message,0,0);
            {
              if(status2!=osOK) history_key_message=key_message; 
              else 
              { 
                key_message = NO_KEY_MESSAGE;                
              }            	
            }
          }
        }
        else 
        {
          if(osEventFlagsGet(laserEvent02Handle)!=0)
          {          
            osStatus_t status = osMessageQueuePut(keyJTMessageQueue01Handle,&key_message,0,0);
            {          
              if(status!=osOK)              
              {               
                DEBUG_PRINTF("key press too fast\r\n"); 
              }	
              else 
              {            
                DEBUG_PRINTF("JT key press %d\r\n",key_message);
                key_message =	NO_KEY_MESSAGE;
                history_key_message=key_message;									
              }
            }	            
          }
          else  key_message =	NO_KEY_MESSAGE;
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
	uint16_t rgbMessage;	//0关闭//1待机绿色2：准备OK紫色常亮；3脉冲输出紫色呼吸
	uint32_t timeout=0;
  osStatus_t statusJT;
  float local_f,load_buff_f;
	LASER_CONTROL_PARAM *pLaserConfig;
	pLaserConfig = &laser_ctr_param;
  uint32_t event;
  osStatus_t laser_close_sem;       
  uint16_t jdq_Volate_heart;
  for(;;)
  {  
    event=osEventFlagsWait(laserEvent02Handle,EVENTS_LASER_PREPARE_OK_ALL_BITS_MASK,osFlagsNoClear,portMAX_DELAY);
    laser_close_sem = osSemaphoreAcquire(laserCloseSem05Handle,10);    
    statusJT= osMessageQueueGet(keyJTMessageQueue01Handle,&recKeyMessage,0,10);
    if(event==(EVENTS_LASER_1064_PREPARE_OK_BIT|EVENTS_LASER_JT_ENABLE_BIT))
    {      
      if(laser_close_sem==osOK)
      {        	
        sGenSta.laser_run_B1_laser_out_status=0; 	
        app_laser_pulse_start(LASER_PULSE_STOP,laser_ctr_param.laserFreq,0);  
        tmc2226_start(0,laser_ctr_param.treatmentWaterLevel);  
        osDelay(120);
        tmc2226_stop();  
        app_deflate_air_solenoid(DISABLE);       
        AD5541A_SetVoltage(0, 4.096);
        osDelay(JDQ_RS485_FRAME_MIN_MS); 
        app_jdq_bus_power_on_off(0);       
        timeout=0;
        do
        {        
          osDelay(JDQ_RS485_FRAME_MIN_MS); 
          timeout+=JDQ_RS485_FRAME_MIN_MS;                   
        }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
        #ifdef JDQ_PWR_GWB_3200W
        osDelay(JDQ_RS485_FRAME_MIN_MS); 
        app_jdq_bus_power_onoff_sta_req();
        timeout = 0;
        do
        {
          osDelay(JDQ_RS485_FRAME_MIN_MS);         
          timeout+=JDQ_RS485_FRAME_MIN_MS;    
        }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
        //jdq_reley_charge(0);//close
        osDelay(500);
        jdq_reley_charge_ready(1);       
        DEBUG_PRINTF("laser close ok\r\n");
        #else
        if(app_jdq_get_vbus_sta()==0)
        {	
          DEBUG_PRINTF("laser close ok\r\n");
          jdq_reley_charge_ready(0); 
        }	
        else 	        
        {	
          app_jdq_bus_power_on_off(0);	          
          jdq_reley_charge(0);	
          osDelay(100);
          jdq_reley_charge_ready(1);
          osDelay(100);
        }  
        #endif            
        sGenSta.laser_run_B0_pro_hot_status=0;	
        rgbMessage=RGB_G_STANDBY;
        osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);
        osEventFlagsClear(laserEvent02Handle,EVENTS_LASER_PREPARE_OK_ALL_BITS_MASK);	//clear
      }         
      if(statusJT==osOK)          
      {    
        if(recKeyMessage==key_jt_long_press)
        { 
          if(pLaserConfig->proCali==0)
          {
            app_laser_timer_ctr(laser_ctr_param.timerCtr, 1000);
          }         
        }
        else  app_laser_timer_ctr(laser_ctr_param.timerCtr, 0);             
        if(recKeyMessage==key_jt_long_press&&sGenSta.laser_run_B5_timer_status==0)
        {
          if(sGenSta.laser_run_B1_laser_out_status==0)
          {  
            if(pLaserConfig->proCali==0)
            { 
              if(pLaserConfig->treatmentWaterLevel!=0)
              {
                app_deflate_air_solenoid(ENABLE);
                tmc2226_start(1,laser_ctr_param.treatmentWaterLevel); 
              }
              app_laser_timer_ctr(laser_ctr_param.timerCtr, 0);//restart
            }
            else
            {
              //local_f=local_f+u_sys_param.sys_config_param.e_cali[(laser_ctr_param.laserEnerge/5)-4].energe_cali*0.001;
            }   
            if(laser_ctr_param.airPressureLevel==0)
            {
              u_sys_param.sys_config_param.laser_pulse_width_us=80;
            }
            else u_sys_param.sys_config_param.laser_pulse_width_us=50+laser_ctr_param.airPressureLevel*50;
            //E=P*T: E=(Vdac*Vdac)*x*T;Vdac=
            laser_ctr_param.laserEnerge=5+laser_ctr_param.ledLightLevel*5;          
            if(laser_ctr_param.laserEnerge>200) laser_ctr_param.laserEnerge=200;
            DEBUG_PRINTF("laser 1064 start e=%d\r\n",laser_ctr_param.laserEnerge);             
            if(laser_ctr_param.laserEnerge>30)
            {              
              local_f=1.80+laser_ctr_param.laserEnerge*0.009;
            }
            else 
            {
              local_f=1.90+laser_ctr_param.laserEnerge*0.0048;        
            }
            local_f=sqrt(laser_ctr_param.laserEnerge*10.0/(u_sys_param.sys_config_param.laser_pulse_width_us));
            //local_f=1.5+laser_ctr_param.ledLightLevel*0.1;                  
            if(local_f>DAC_MAX_VOLTAGE_F) local_f=DAC_MAX_VOLTAGE_F;//250mJ~4.0
            if(local_f<DAC_MIN_VOLTAGE_F) local_f=DAC_MIN_VOLTAGE_F;//250mJ~4.0
            if(local_f<1.8) laser_ctr_param.lowEnergeMode=1;
            else laser_ctr_param.lowEnergeMode=0;
            AD5541A_SetVoltage(local_f,4.096);       
            sGenSta.laser_run_B1_laser_out_status=1; 
            rgbMessage = RGB_LASER_WORK_STATUS;
            osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);
             
            DEBUG_PRINTF("laser 1064 start pulse width=%.1f\r\n",u_sys_param.sys_config_param.laser_pulse_width_us);             
            if(pLaserConfig->proCali!=0)
            {
              app_laser_pulse_start(u_sys_param.sys_config_param.laser_pulse_width_us,10,local_f); 
            }
            else  app_laser_pulse_start(u_sys_param.sys_config_param.laser_pulse_width_us,laser_ctr_param.laserFreq,local_f);  
          }                     
        }
        else 
        {
          if(recKeyMessage!=key_jt_long_press&&sGenSta.laser_run_B5_timer_status!=0)
          {
            sGenSta.laser_run_B5_timer_status=0;
          }        
          if(sGenSta.laser_run_B1_laser_out_status!=0)
          {            
            app_laser_pulse_start(LASER_PULSE_STOP,laser_ctr_param.laserFreq,0); 
            AD5541A_SetVoltage(0, 4.096);
            sGenSta.laser_run_B1_laser_out_status=0; 
            if(pLaserConfig->proCali==0&&pLaserConfig->treatmentWaterLevel!=0)
            {   
              tmc2226_start(0,laser_ctr_param.treatmentWaterLevel);  
              osDelay(120);
              tmc2226_stop();  
              app_deflate_air_solenoid(DISABLE);   
            }
            rgbMessage = RGB_LASER_PREPARE_OK;
            osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);
            DEBUG_PRINTF("stop 1064 stop\r\n");          
            sGenSta.laser_param_B456_jt_status = recKeyMessage;
          }                      
        } 
        #if 0// cali        
        if (sGenSta.laser_run_B1_laser_out_status!=0) // cali
        {  
          app_get_adc_value( AD2_LASER_1064_INDEX,&sEnvParam.laser_1064_energy);
          DEBUG_PRINTF("energe=%.1f\r\n",sEnvParam.laser_1064_energy);
         //if(sEnvParam.laser_1064_energy<=targert_list)         
          if(sEnvParam.laser_1064_energy>260.0) 
          {
            load_buff_f=local_f*(260.0/sEnvParam.laser_1064_energy);
            if(load_buff_f>4.0) load_buff_f=4.0;//250mJ~4.0
            AD5541A_SetVoltage_noLoad(load_buff_f, 4.096); 
            AD5541A_SetVoltage_Load_enable();                
          } 
          else if(sEnvParam.laser_1064_energy+10<260.0) 
          {
            if(sEnvParam.laser_1064_energy!=0) load_buff_f=local_f*(260.0/sEnvParam.laser_1064_energy);
            
            AD5541A_SetVoltage_noLoad(load_buff_f, 4.096); 
            AD5541A_SetVoltage_Load_enable();
          } 
          else 
          {
            AD5541A_SetVoltage_Load_disable();
          }
        }
        #else 
        if(sGenSta.laser_run_B1_laser_out_status!=0) 
        {
          app_get_adc_value( AD2_LASER_1064_INDEX,&sEnvParam.laser_1064_energy);
          DEBUG_PRINTF("energe=%.1f\r\n",sEnvParam.laser_1064_energy);        
        }
          #ifdef JDQ_PWR_GWB_3200W
          osDelay(JDQ_RS485_FRAME_MIN_MS); 
          app_jdq_bus_power_onoff_sta_req();
          timeout = 0;
          do
          {
            osDelay(JDQ_RS485_FRAME_MIN_MS);         
            timeout+=JDQ_RS485_FRAME_MIN_MS;    
          }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
          if(app_jdq_get_vbus_sta()==JDQ_PWR_GWB_3200W_ERROR_FLAG)
          {//err

          }
          #endif
        #endif         
      }             
    }     
    else
    {
      if(recKeyMessage==key_jt_release)  sGenSta.laser_param_B456_jt_status=key_jt_release;      
    }   
    osDelay(5);  
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
  float  recVoltage,recCurrent; 
  for(;;)
  {
		/***********环境气压、温度监测*******************/ 	 
    unsigned char  status=app_gzp6816d_listen(osKernelGetTickCount(),&sEnvParam.air_gzp_enviroment_pressure_kpa,&sEnvParam.enviroment_temprature);    
    if(status!=0) sEnvParam.air_gzp_enviroment_pressure_kpa =94.0;//error 
  
    /***********NTC,laser_energe,iBus,Vbus,vBus,air_pump_pressure气泵气压，参数*******************/     
    app_get_adc_value(AD1_NTC_INDEX,&sEnvParam.NTC_temprature);//蠕动泵，状态   
    app_get_adc_value(AD1_OCP_Ibus_INDEX,&sEnvParam.iBus);		
    app_get_adc_value(AD1_24V_VBUS_INDEX,&sEnvParam.vBus);     
    /***********气泵管理*******************/
		app_get_adc_value(AD1_AIR_PRESSER_INDEX,&sEnvParam.air_pump_pressure);	
		app_air_pump_manage(laser_ctr_param.airPressureLevel);    
		/***********aux genaration状态检查*******************/  
    app_sys_genaration_status_manage();	
		app_fresh_laser_status_param();	
    /******************治疗水位********环境温度************/
    if(osKernelGetTickCount()>circleTick+1000)
    {         
      circleTick = osKernelGetTickCount();  
            //app_mcp61_get_singgle_c_value_req();
      //DEBUG_PRINTF("C_water=%.3f\r\n",app_mcp61_c_value());
      //DEBUG_PRINTF("evnentBits=%04x\r\n",osEventFlagsGet(auxStatusEvent01Handle));
     // DEBUG_PRINTF(" e_air_p=%.2f\r\n",sEnvParam.air_gzp_enviroment_pressure_kpa); 
     // DEBUG_PRINTF("ads1118: T1=%.1f℃ T2= %.1f℃ \r\n", sEnvParam.eth_k1_temprature,sEnvParam.eth_k2_temprature);      
      //DEBUG_PRINTF("gzp_enviroment_air_pressure: kpa=%.2fkPa temprature=%.1f ℃\r\n",sEnvParam.air_gzp_enviroment_pressure_kpa,sEnvParam.enviroment_temprature);
      //eventFlagError= osEventFlagsWait(status_io_updataEvent01Handle,EVENTS_STATUS_IO_ALL_BITS,osFlagsWaitAll,portMAX_DELAY);) 
      //DEBUG_PRINTF("NTC=%.2f℃ laser_energe=%.1f iBus=%.1fmA ,vBus=%.1fmV,air_pump_pressure=%.2fkPa\r\n",sEnvParam.NTC_temprature,\
        sEnvParam.laser_1064_energy,sEnvParam.iBus,sEnvParam.vBus,sEnvParam.air_pump_pressure);  
      //DEBUG_PRINTF("count=%d\r\n" ,u_sys_param.sys_config_param.laser_pulse_count);
	  }    		
    osDelay(1);
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
  osDelay(13000);
  uint16_t send_music_num = MUSIC_SYS_ON;
  osMessageQueuePut(musicQueue03Handle,&send_music_num,0,100);
  u_sys_param.sys_config_param.synchronousFlag=0; 
  for(;;)
  {    
    while(u_sys_param.sys_config_param.synchronousFlag!=3)
    {//test
      osDelay(1000);
      DEBUG_PRINTF("sync data...%d\r\n",u_sys_param.sys_config_param.synchronousFlag);
      app_hmi_sysnc_req(); 
          
    }    
    osStatus_t status = osSemaphoreAcquire(hmiCanBusIdleSem06Handle,HMI_CAN_FRAME_DELAY_TIME);
    if(status==pdTRUE)
    {
      osDelay(50);
    }		
    sGenSta.genaration_io_status = osEventFlagsGet(auxStatusEvent01Handle);		
    app_hmi_report_status(&sGenSta);    
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
  uint16_t packLen=0,readLen=0;
  for(;;)
  {
    /*******************CAN RX-DATA********************/	
		osStatus_t sta = osSemaphoreAcquire(CANBusReceiveFrameSem07Handle,portMAX_DELAY);      
    if(readLen<fd_canRxLen) 
    {
      packLen = fd_canRxLen-readLen;      
      #if 1        
      DEBUG_PRINTF("CAN_receive_pack:\r\n");
      for(int i=0;i<packLen;i++)
      {
        DEBUG_PRINTF(" %02x",fd_canRxBuff[i+readLen]);
      }
      DEBUG_PRINTF(" Len=%d\r\n",packLen);      
      #endif   
      packLen = app_hmi_package_check(&fd_canRxBuff[readLen],packLen);
      if(packLen!=0)
      {
        readLen+=packLen;
        if(readLen==fd_canRxLen)
        {
          readLen=0;
          fd_canRxLen=0;
        }
      }   
    }
    if(readLen>fd_canRxLen&&readLen!=0)//full
    {
      readLen=0;
      fd_canRxLen=0;
    }     
    osDelay(2);     
     
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
    sGenSta.genaration_io_status = osEventFlagsGet(auxStatusEvent01Handle);	
    sGenSta.laser_run_B6_close_device_status=1;
    app_hmi_report_status(&sGenSta);    
    ge2117_start_up_set(GE2117_STOP_CMD);
		if(app_sys_param_save_data()==0)
		{
			DEBUG_PRINTF(" sys param save ok\r\n");
		}	      
    app_beep_pwm(music_tab_c[1],50);
    HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
    HAL_Delay(500);     
    app_beep_pwm(0,0);
    HAL_Delay(1000);
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
   //osDelay(1);
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
	uint16_t rgbMessage=1;
  uint16_t laser_freq,laser_Voltage;					
	uint8_t local_proHotCtr=0;			
	float local_f = 1.0,l_jdq_set_voltage,l_jdq_set_current;
  float outVoltage;
  for(;;) 
  {
		osStatus_t sta = osSemaphoreAcquire(laserPrapareReqSem03Handle,portMAX_DELAY);
		local_proHotCtr =	laser_ctr_param.proHotCtr;	
		laser_freq = laser_ctr_param.laserFreq;
		laser_Voltage = app_laser_1064_energe_to_voltage(laser_ctr_param.laserEnerge); 
    local_f =laser_Voltage*1.0;
    if(laser_ctr_param.ctrTestMode!=0)
    { //加载固定能量参数,本机不支持光纤激活      
      rgbMessage = RGB_LASER_PREPARE_OK;
      osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);        	      
      osEventFlagsSet(laserEvent02Handle,EVENTS_LASER_GX_TEST_PREPARE_OK_BIT|EVENTS_LASER_JT_ENABLE_BIT);
      sGenSta.laser_run_B0_pro_hot_status = 1;      
    }
    else
    {
      if(local_proHotCtr!=0)	
      {	 
        #ifdef JDQ_PWR_GWB_3200W
        app_jdq_current_limit_charge();
        osDelay(JDQ_RS485_FRAME_MAX_DELAY_MS);         
        app_jdq_bus_power_on_off(1);
        timeout = 0;
        do
        {
          osDelay(JDQ_RS485_FRAME_MIN_MS);         
          timeout+=JDQ_RS485_FRAME_MIN_MS;    
        }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);    
        osDelay(JDQ_RS485_FRAME_MIN_MS); 
        app_jdq_bus_vol_current_set(local_f,LASER_JDQ_CHARGE_CURRENT_F);
        timeout = 0;
        do
        {
          osDelay(JDQ_RS485_FRAME_MIN_MS);         
          timeout+=JDQ_RS485_FRAME_MIN_MS;    
        }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
        //charge
        osDelay(JDQ_RS485_FRAME_MIN_MS);       
        timeout = 0;
        do
        {
          app_jdq_bus_power_onoff_sta_req();
          HAL_Delay(JDQ_RS485_FRAME_MAX_DELAY_MS);         
          timeout+=JDQ_RS485_FRAME_MAX_DELAY_MS;
          l_jdq_set_voltage= app_jdq_get_vbus_sta();         
        }while(l_jdq_set_voltage+5.0<local_f&&timeout<LASER_JDQ_CHARGE_TIMEOUT_MS);
        if(l_jdq_set_voltage!=JDQ_PWR_GWB_3200W_ERROR_FLAG&&l_jdq_set_voltage!=0)
        {
          timeout=0;
          do
          {  
            app_jdq_bus_power_onoff_sta_req();
            osDelay(JDQ_RS485_FRAME_MAX_DELAY_MS);
            timeout+=JDQ_RS485_FRAME_MAX_DELAY_MS;            
            outVoltage = app_jdq_voltage_monitor();                        
            if(laser_ctr_param.proHotCtr==0) 
            {
              timeout=LASER_JDQ_CHARGE_TIMEOUT_MS;
              break;
            }          
          } while (outVoltage+5.0<local_f&&timeout<LASER_JDQ_CHARGE_TIMEOUT_MS); //(90%)
          
        }
        /*
        osDelay(JDQ_RS485_FRAME_MIN_MS);         
        app_jdq_bus_power_on_off(0);
        timeout = 0;
        do
        {
          osDelay(JDQ_RS485_FRAME_MIN_MS);         
          timeout+=JDQ_RS485_FRAME_MIN_MS;    
        }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS); 
        osDelay(JDQ_RS485_FRAME_MIN_MS);         
        app_jdq_bus_power_on_off(1);
        timeout = 0;
        do
        {
          osDelay(JDQ_RS485_FRAME_MIN_MS);         
          timeout+=JDQ_RS485_FRAME_MIN_MS;    
        }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
        */ 
        osDelay(JDQ_RS485_FRAME_MIN_MS); 
        app_jdq_bus_vol_current_set(local_f,LASER_JDQ_CURRENT_LIMIT_F);
        timeout = 0;
        do
        {
          osDelay(JDQ_RS485_FRAME_MIN_MS);         
          timeout+=JDQ_RS485_FRAME_MIN_MS;    
        }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS); 
        #else
        app_jdq_current_limit_charge();
        osDelay(JDQ_RS485_FRAME_MIN_MS); 
        osDelay(JDQ_RS485_FRAME_MIN_MS);
        app_jdq_bus_power_on_off(0);
        timeout=0;
        do
        {
          osDelay(JDQ_RS485_FRAME_MIN_MS); 
          timeout+=JDQ_RS485_FRAME_MIN_MS;          
        }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);        
        outVoltage=app_jdq_voltage_monitor();
        if(outVoltage>local_f) 
        { 
          jdq_reley_charge(0);
          osDelay(50);		
          jdq_reley_charge_ready(1);
          timeout=0;
          do
          {  
            osDelay(JDQ_RS485_FRAME_MIN_MS);
            timeout+=JDQ_RS485_FRAME_MIN_MS;            
            outVoltage = app_jdq_voltage_monitor();  
          } while (outVoltage>local_f+5.0&&timeout<LASER_JDQ_CHARGE_TIMEOUT_MS);
        }
        osDelay(JDQ_RS485_FRAME_MIN_MS);
        app_jdq_bus_get_v_c_req();
        timeout=0;
        do
        {
          osDelay(JDQ_RS485_FRAME_MIN_MS); 
          timeout+=JDQ_RS485_FRAME_MIN_MS;           				
        }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);          
        app_jdq_bus_get_set_v_c(&l_jdq_set_voltage,&l_jdq_set_current); 
        if(local_f!= l_jdq_set_voltage)
        {
          osDelay(JDQ_RS485_FRAME_MIN_MS); 
          app_jdq_bus_voltage_set(LASER_JDQ_VOLTAGE_F);
          timeout=0;
          do
          {
            osDelay(JDQ_RS485_FRAME_MIN_MS); 
            timeout+=JDQ_RS485_FRAME_MIN_MS;           				
          }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
          app_jdq_bus_get_v_c_req();
          timeout=0;
          do
          {
            osDelay(JDQ_RS485_FRAME_MIN_MS); 
            timeout+=JDQ_RS485_FRAME_MIN_MS;           				
          }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);          
          app_jdq_bus_get_set_v_c(&l_jdq_set_voltage,&l_jdq_set_current);
                        
        } 
        jdq_reley_charge_ready(0);        
        osDelay(JDQ_RS485_FRAME_MIN_MS);   
        app_jdq_bus_power_on_off(1);
        timeout=0;
        do
        {
          osDelay(JDQ_RS485_FRAME_MIN_MS); 
          timeout+=JDQ_RS485_FRAME_MIN_MS;          
        }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
        outVoltage = app_jdq_voltage_monitor();
        timeout=0;
        do
        {  
          osDelay(JDQ_RS485_FRAME_MIN_MS);
          timeout+=JDQ_RS485_FRAME_MIN_MS;            
          outVoltage = app_jdq_voltage_monitor();                        
          if(laser_ctr_param.proHotCtr==0) 
          {
            timeout=LASER_JDQ_CHARGE_TIMEOUT_MS;
            break;
          }          
        } while (outVoltage+5.0<local_f&&timeout<LASER_JDQ_CHARGE_TIMEOUT_MS); //(90%)
        #endif        
          if(timeout>=LASER_JDQ_CHARGE_TIMEOUT_MS||outVoltage+10<local_f)//fail
          {
            DEBUG_PRINTF("charge fail jdq—v%f %d \r\n",outVoltage,laser_Voltage); 
            sGenSta.laser_run_B0_pro_hot_status = 0; 
            app_jdq_bus_power_on_off(0);
            timeout=0;
            do
            {
              osDelay(JDQ_RS485_FRAME_MIN_MS); 
              timeout+=JDQ_RS485_FRAME_MIN_MS;           				
            }while(app_get_jdq_rs485_bus_statu()!=0&&timeout<JDQ_RS485_FRAME_MAX_DELAY_MS);
            jdq_reley_charge(0);
            osDelay(JDQ_RS485_FRAME_MIN_MS);		
            jdq_reley_charge_ready(1);         
          }
          else  
          { 
            DEBUG_PRINTF("charge ok jdq—v%f %d timeMs=%d\r\n",outVoltage,laser_Voltage,timeout);            
            rgbMessage = RGB_LASER_PREPARE_OK;
            sGenSta.laser_run_B5_timer_status=1;
            osMessageQueuePut(rgbQueue02Handle,&rgbMessage,0,0);        	      
            osEventFlagsSet(laserEvent02Handle,EVENTS_LASER_1064_PREPARE_OK_BIT|EVENTS_LASER_JT_ENABLE_BIT);
            sGenSta.laser_run_B0_pro_hot_status = 1;            
          }  
      }		
      else 
      {
        if(sGenSta.laser_run_B0_pro_hot_status != 0)  osSemaphoreRelease(laserCloseSem05Handle);
      }	
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
  uint32_t local_time100mS,g_timeout,local_timeS;
  local_time100mS = 0;  
  local_timeS=0;//
  //float k0_cool_temprature;
  max_31865_pt1000(); 
  osDelay(200);
  for(;;)
  {  
    osDelay(100);//994+6=1000;
    local_time100mS++;
    if(local_time100mS>9) 
    {
      local_timeS++;
      local_time100mS=0;
    }    
    float temp_t_f=Get_pt_tempture();
    sEnvParam.eth_k1_temprature = temp_t_f;
    sEnvParam.eth_k2_temprature = temp_t_f;
    u_sys_param.sys_config_param.cool_temprature_target=240;
    if(sEnvParam.eth_k1_temprature>ERR_LOW_TEMPRATURE_LASER&&sEnvParam.eth_k1_temprature<ERR_HIGH_TEMPRATURE_LASER)
    {        
      app_ge2117_gp_ctr(sEnvParam.eth_k1_temprature,local_timeS);             
      app_circle_water_PTC_manage(sEnvParam.eth_k1_temprature,local_time100mS);
      if(sEnvParam.eth_k1_temprature<u_sys_param.sys_config_param.cool_temprature_low||sEnvParam.eth_k1_temprature>u_sys_param.sys_config_param.cool_temprature_high)
      {
        osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_12_K1_TEMPRATURE_BIT);
      }
      else osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_12_K1_TEMPRATURE_BIT);    
    }  
    else
    {      
      app_ge2117_gp_ctr(u_sys_param.sys_config_param.cool_temprature_target,local_timeS);//stop
      app_circle_water_PTC_manage(u_sys_param.sys_config_param.cool_temprature_target,local_time100mS);//stop
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
  }
  /* USER CODE END ge2117ManageTask10 */
}

/* USER CODE BEGIN Header_waterAirCleanTask11 */
/**
* @brief Function implementing the myTask11 thread.
* @param argument: Not used
* @retval None
*/        
/* USER CODE END Header_waterAirCleanTask11 */
void waterAirCleanTask11(void *argument)
{
  /* USER CODE BEGIN waterAirCleanTask11 */
  unsigned int local_timeS;
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    local_timeS+=100;
    if(local_timeS>=1000)
    {
      app_t_clean_run_timer(&laser_ctr_param.cleanCtr);
      local_timeS = 0;
    }    
    if(laser_ctr_param.cleanCtr==0) app_treatment_water_prepare(&laser_ctr_param.air_water_prepare_ctr,100); 
  }
  /* USER CODE END waterAirCleanTask11 */
}

/* USER CODE BEGIN Header_musicTask12 */
/**
* @brief Function implementing the myTask12 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_musicTask12 */
void musicTask12(void *argument)
{
  /* USER CODE BEGIN musicTask12 */
  /* Infinite loop */
  uint16_t music_num=0;  
  for(;;)
  {
    osMessageQueueGet(musicQueue03Handle,&music_num,0,portMAX_DELAY);	
    app_buzz_music(music_num,50);  
    osDelay(10);
  }
  /* USER CODE END musicTask12 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
  * @brief  EXTI line detection callback.
  * @param  GPIO_Pin: Specifies the port pin connected to correspon
  * 
  * ding EXTI line.
  * @retval None
  */
 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
 {  
  if(GPIO_Pin==LASER_PULSE_COUNT_in_Pin)
  {
    u_sys_param.sys_config_param.laser_pulse_count++;//光电管计数
  }
  if(GPIO_Pin==FAN1_COUNT_in_Pin)
  {
    app_fan_feed_count(1);
  }
  if(GPIO_Pin==FAN2_COUNT_in_Pin)
  {
    app_fan_feed_count(2);
  }
  if(GPIO_Pin==LASER_PULSE_COUNT_in_Pin)
  {
    u_sys_param.sys_config_param.laser_pulse_count++;
    if(laser_ctr_param.lowEnergeMode==0)   HAL_TIM_Base_Start_IT(&htim3);
    pulse_adc_start();
  }   
  #ifdef ONE_WIRE_BUS_JT_SLAVE 
  if(GPIO_Pin==FOOT_SWITCH_IN_Pin)
  {  
    owb_dq_edge_callback();    
  }
  #endif
 }
 /*************************************************************
  * ***********//**
  * @brief 系统一般状态信号管理
  * @param 无
  * @note   周期性更新事件正常标志
  * @retval 无
  *****************************************************************************/
void app_sys_genaration_status_manage(void)
{ //高压电磁阀
  if(app_get_io_status(In1_high_voltage_solenoid)==SUCCESS)
  {      
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO1_BIT);
  }
  else 
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO1_BIT); 
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
  //治疗水瓶液位 低有效
  if(HAL_GPIO_ReadPin(TREATMENT_WATER_DEPTH_in_GPIO_Port,TREATMENT_WATER_DEPTH_in_Pin)==GPIO_PIN_RESET)
  {
    sEnvParam.treatment_water_depth=1;
  }
  else sEnvParam.treatment_water_depth=0;
  //治疗水OK就绪信号 
	if(app_get_io_status(In7_water_ready_ok)==SUCCESS&&sEnvParam.treatment_water_depth!=0)
	{  
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO7_BIT|EVENTS_AUX_STATUS_15_WATER_AIR_PREPARE_BIT);
	}
	else 
  {  
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO7_BIT|EVENTS_AUX_STATUS_15_WATER_AIR_PREPARE_BIT);
  }//水循环就绪信号
	if(app_get_io_status(In8_water_circle_ok)==SUCCESS)
	{  
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO8_BIT);
	}
	else 
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_IO8_BIT);
  }
  if(sEnvParam.NTC_temprature>MAX_TMC2226_NTC_TEMPRATURE)//过热
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_9_NTC_BIT);  
  }
  else
  {
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_9_NTC_BIT);
  }
  if(sEnvParam.iBus<MAX_IBUS_MA)//<10A
  {
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_10_IBUS_BIT);    
  }
  else
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_10_IBUS_BIT);
  }  
  if(sEnvParam.vBus>MIN_VBUS_MV)
  {
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_11_VBUS_BIT);
  }
  else
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_11_VBUS_BIT);
  }    
  //紧急开关,低有效
  if(HAL_GPIO_ReadPin(EMERGENCY_LASER_STOP_STATUS_in_GPIO_Port,EMERGENCY_LASER_STOP_STATUS_in_Pin)==GPIO_PIN_RESET)
  {
    osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_14_EMERGENCY_KEY_BIT);//弹起
  }
  else 
  {
    osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_14_EMERGENCY_KEY_BIT);//按下
    osSemaphoreRelease(laserCloseSem05Handle);//关闭激光
  }
  //DEBUG_PRINTF("IO8~1=%d%d%d%d%d%d%d%d\r\n" ,sGenSta.water_circle_ok_status,sGenSta.water_ready_ok_status,\
    sGenSta.Hyperbaria_OFF_Signal_staus,sGenSta.h_air_error_status ,sGenSta.enviroment_tmprature_alert_status,\
    sGenSta.chocke_air_solenoid_status, sGenSta.deflate_air_solenoid_status,sGenSta.high_voltage_solenoid_status);
}
/************************************************************************//**
  * @brief 计算设备ID
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
unsigned  int app_get_cali_devid(void)
{
	unsigned  int id;
	unsigned char buf[12];//96bit	
	buf[0]=HAL_GetUIDw0()&0xFF;
	buf[1]=(HAL_GetUIDw0()>>8)&0xFF;
	buf[2]=(HAL_GetUIDw0()>>16)&0xFF;
	buf[3]=(HAL_GetUIDw0()>>24)&0xFF;

	buf[4]=HAL_GetUIDw1()&0xFF;
	buf[5]=(HAL_GetUIDw1()>>8)&0xFF;
	buf[6]=(HAL_GetUIDw1()>>16)&0xFF;
	buf[7]=(HAL_GetUIDw1()>>24)&0xFF;	

	buf[8]=HAL_GetUIDw2()&0xFF;
	buf[9]=(HAL_GetUIDw2()>>8)&0xFF;
	buf[10]=(HAL_GetUIDw2()>>16)&0xFF;
	buf[11]=(HAL_GetUIDw2()>>24)&0xFF;	
	id =  crc32_MPEG(buf,12);
	return id;
}	
/************************************************************************//**
  * @brief 使用默认系统参数
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
void app_set_default_sys_config_param(void)
{	
	u_sys_param. sys_config_param.synchronousFlag = 0;
  if(u_sys_param. sys_config_param.equipmentId==0||u_sys_param. sys_config_param.equipmentId==0xFFFFFFFF)
  {
	  u_sys_param. sys_config_param.equipmentId = app_get_cali_devid();//默认参数
  }
  u_sys_param. sys_config_param.jtId=0;
  u_sys_param. sys_config_param.jt_status=1; 
  u_sys_param. sys_config_param.cool_temprature_low=210;//21.0f
  u_sys_param. sys_config_param.cool_temprature_target=230;//23.0f
  u_sys_param. sys_config_param.cool_temprature_high=280;//28.0f
  u_sys_param. sys_config_param.photodiod_low=0;//50mj
  u_sys_param. sys_config_param.photodiod_mid=99;//100mJ
  u_sys_param. sys_config_param.photodiod_high=200;//205mJ
  u_sys_param. sys_config_param.laser_pulse_count=0;
  u_sys_param. sys_config_param.laser_use_timeS=0;
  u_sys_param. sys_config_param.RDB_use_timeS=0; 
  u_sys_param. sys_config_param.laser_pulse_width_us=120;
  u_sys_param. sys_config_param.treatment_water_depth_high=151;//15.1pf
  u_sys_param. sys_config_param.treatment_water_depth_low=87;//8.7pF
  u_sys_param. sys_config_param.cool_water_depth_high=167;//16.7pf
  u_sys_param. sys_config_param.cool_water_depth_low=52;//5.2 
  u_sys_param. sys_config_param.air_low_pressure=160;//160kPa
  u_sys_param. sys_config_param.air_mid_pressure=180;//180kPa
  u_sys_param. sys_config_param.air_high_pressure=200;//200kPa
  u_sys_param. sys_config_param.t_water_low=15;//10ml/min
  u_sys_param. sys_config_param.t_water_mid=25;
  u_sys_param. sys_config_param.t_water_high=35;
  u_sys_param. sys_config_param.dit_time_min=5;//5min
  u_sys_param. sys_config_param.clean_time_min=5;
  u_sys_param. sys_config_param.tec_switch=1;
  u_sys_param. sys_config_param.low_freq=60;//MAX60Hz 
  u_sys_param. sys_config_param.charge_width_us=180;//180us
//
  u_sys_param.sys_config_param.laser_led_light=50;            //）激光指示灯亮度
  u_sys_param.sys_config_param.rgb_light=50;                  //）rgb状态指示灯亮度
  u_sys_param.sys_config_param.beep=50;                       //）音量
  for(uint8_t i=0;i<40;i++)
  {  
    u_sys_param. sys_config_param.e_cali[i].energe_cali=2500;
    u_sys_param. sys_config_param.e_cali[i].power_cali=6;//
  }
  u_sys_param.sys_config_param.checkSum=sumCheck(u_sys_param.data,SYS_LASER_CONFIG_PARAM_LENGTH);//sizeof(SYS_CONFIG_PARAM)-4);	 
	DEBUG_PRINTF("sys param load failed! load defalut param\r\n");
  laser_ctr_param.airPressureLevel=1;
  laser_ctr_param.treatmentWaterLevel=0; 
  laser_ctr_param.ledLightLevel=20;
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
    flag = EEPROM_M24C32_Read(EEROM_SYS_PARAM_SAVE_ADDR, u_sys_param.data, sizeof(SYS_CONFIG_PARAM));
    unsigned int sum = sumCheck(u_sys_param.data,SYS_LASER_CONFIG_PARAM_LENGTH);   
    if(u_sys_param.sys_config_param.synchronousFlag==EEROM_DATA_ERR_CHECK_FLAG||u_sys_param.sys_config_param.checkSum!=sum)//
    {
      app_set_default_sys_config_param();
      DEBUG_PRINTF("load default sys param\r\n");		 
    }
    else 
    {
      //check param
      if(u_sys_param.sys_config_param.cool_temprature_target>280||u_sys_param.sys_config_param.cool_temprature_target<210)
      {
        u_sys_param.sys_config_param.cool_temprature_target=240;
      } 
      memcpy(u_sys_default_param.data,u_sys_param.data,sizeof(SYS_CONFIG_PARAM));      
      DEBUG_PRINTF("sys param read ok= %d\r\n",u_sys_param.sys_config_param.laser_pulse_count);
      laser_ctr_param.airPressureLevel=1;
      laser_ctr_param.treatmentWaterLevel=0; 
      laser_ctr_param.ledLightLevel=20;
      #if 1
      DEBUG_PRINTF("synchronousFlag=%d\r\n", u_sys_param. sys_config_param.synchronousFlag);
      DEBUG_PRINTF("equipmentId=%d\r\n",u_sys_param. sys_config_param.equipmentId);
      DEBUG_PRINTF("jtId=%d\r\n",u_sys_param. sys_config_param.jtId);
      DEBUG_PRINTF("jt_status=%d\r\n",u_sys_param. sys_config_param.jt_status);
      DEBUG_PRINTF("cool_temprature_low=%.1f℃\r\n",u_sys_param. sys_config_param.cool_temprature_low*0.1);
      DEBUG_PRINTF("cool_temprature_target=%.1f℃\r\n",u_sys_param. sys_config_param.cool_temprature_target*0.1);
      DEBUG_PRINTF("cool_temprature_high=%.1f℃\r\n",u_sys_param. sys_config_param.cool_temprature_high*0.1);

      DEBUG_PRINTF("photodiod_low=%d\r\n",u_sys_param. sys_config_param.photodiod_low);
      DEBUG_PRINTF("photodiod_mid=%d\r\n",u_sys_param. sys_config_param.photodiod_mid);
      DEBUG_PRINTF("photodiod_high=%d\r\n",u_sys_param. sys_config_param.photodiod_high); 
      DEBUG_PRINTF("laser_pulse_count=%d\r\n",u_sys_param. sys_config_param.laser_pulse_count);   
      DEBUG_PRINTF("laser_use_timeS=%d\r\n",u_sys_param. sys_config_param.laser_use_timeS);   
      DEBUG_PRINTF("RDB_use_timeS=%d\r\n",u_sys_param. sys_config_param.RDB_use_timeS);  
      DEBUG_PRINTF("laser_pulse_width_us=%d\r\n",u_sys_param. sys_config_param.laser_pulse_width_us);
      DEBUG_PRINTF("treatment_water_depth_high=%.1fr\n",u_sys_param. sys_config_param.treatment_water_depth_high*0.1);
      DEBUG_PRINTF("treatment_water_depth_low=%.1f\r\n",u_sys_param. sys_config_param.treatment_water_depth_low*0.1);
      DEBUG_PRINTF("cool_water_depth_high=%.1f \r\n",u_sys_param. sys_config_param.cool_water_depth_high*0.1);
      DEBUG_PRINTF("cool_water_depth_low=%.1f \r\n",u_sys_param. sys_config_param.cool_water_depth_low*0.1);
      DEBUG_PRINTF("air_low_pressure=%d kPa\r\n",u_sys_param. sys_config_param.air_low_pressure); 
      DEBUG_PRINTF("air_mid_pressure=%d kPa\r\n",u_sys_param. sys_config_param.air_mid_pressure);
      DEBUG_PRINTF("air_high_pressure=%d kPa\r\n",u_sys_param. sys_config_param.air_high_pressure);
      DEBUG_PRINTF("t_water_low=%d ml/min\r\n",u_sys_param. sys_config_param.t_water_low);
      DEBUG_PRINTF("t_water_mid=%d ml/min\r\n",u_sys_param. sys_config_param.t_water_mid);
      DEBUG_PRINTF("t_water_high=%d  ml/min\r\n",u_sys_param. sys_config_param.t_water_high);
      DEBUG_PRINTF("dit_time_min=%d  min\r\n",u_sys_param. sys_config_param.dit_time_min);
      DEBUG_PRINTF("clean_time_min=%d  min\r\n",u_sys_param. sys_config_param.clean_time_min);
      DEBUG_PRINTF("tec_switch=%d\r\n",u_sys_param. sys_config_param.tec_switch);
      DEBUG_PRINTF("low_freq=%d\r\n",u_sys_param. sys_config_param.low_freq);
      DEBUG_PRINTF("charge_width_us=%d\r\n",u_sys_param. sys_config_param.charge_width_us);  
      DEBUG_PRINTF("laser_led_light=%d\r\n",u_sys_param. sys_config_param.laser_led_light);
      DEBUG_PRINTF("rgb_light=%d\r\n",u_sys_param. sys_config_param.rgb_light);
      DEBUG_PRINTF("beep=%d\r\n",u_sys_param. sys_config_param.beep);
      #endif
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
    if(compare_buff_no_change(u_sys_param.data,u_sys_default_param.data,sizeof(SYS_CONFIG_PARAM)))
    {
      u_sys_param.sys_config_param.checkSum=sumCheck(u_sys_param.data,SYS_LASER_CONFIG_PARAM_LENGTH);
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
		if(air_level==1)
    {
     // air_pressure = u_sys_param.sys_config_param.air_low_pressure+sEnvParam.air_gzp_enviroment_pressure_kpa;
      air_pressure = MIN_AIR_PUMP_PRESSURE+sEnvParam.air_gzp_enviroment_pressure_kpa;
    }
    else if(air_level==3)
    {
      //air_pressure   =  u_sys_param.sys_config_param.air_mid_pressure+sEnvParam.air_gzp_enviroment_pressure_kpa;
      air_pressure = MAX_AIR_PUMP_PRESSURE+sEnvParam.air_gzp_enviroment_pressure_kpa;			
    }
    else if(air_level==2)
    {
      //air_pressure = u_sys_param.sys_config_param.air_high_pressure+sEnvParam.air_gzp_enviroment_pressure_kpa;
      air_pressure=MID_AIR_PUMP_PRESSURE+sEnvParam.air_gzp_enviroment_pressure_kpa;
    }    
    if(air_level==0||((eventFlag&EVENTS_AUX_STATUS_IO6_BIT)!= EVENTS_AUX_STATUS_IO6_BIT)|| sEnvParam.air_pump_pressure>air_pressure+3 )//
    {			
      app_air_pump_switch(DISABLE);     	     
    }
		else 
		{
			if(sEnvParam.air_pump_pressure+5 < air_pressure)
			{	
				app_air_pump_switch(ENABLE);
			}
		} 
		if(sEnvParam.air_pump_pressure+10<(MIN_AIR_PUMP_PRESSURE + sEnvParam.air_gzp_enviroment_pressure_kpa) )	
		{
			sGenSta.laser_param_B23_air_pump_pressure_status = 0;
		}	
		else if(sEnvParam.air_pump_pressure > ( MAX_AIR_PUMP_PRESSURE + sEnvParam.air_gzp_enviroment_pressure_kpa+10) )	
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
    sGenSta.circle_water_box_temprature = (char)((int)sEnvParam.eth_k1_temprature);
    sGenSta.treatment_water_level_status = laser_ctr_param.treatmentWaterLevel;
    sGenSta.air_level_status = laser_ctr_param.airPressureLevel;	
    sGenSta.laser_run_B2_gx_test_status = laser_ctr_param.ctrTestMode;
    sGenSta.laser_param_B01_energe_status = 1;
    sGenSta.genaration_io_status = osEventFlagsGet(auxStatusEvent01Handle);	 
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
  
 /***************************extern api**********************************************************/
 /************************************************************************//**
  * @brief 给出canBus数据接收信号量
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 void app_canBbus_receive_semo(void) 
 {  
	osSemaphoreRelease(CANBusReceiveFrameSem07Handle); 
  
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
  * @brief 倒计时
  * @param 无
  * @note   
  * @retval  ErrorStatus
  *****************************************************************************/
 ErrorStatus app_laser_timer_ctr(unsigned char ctrTimeS, unsigned int TimeMs)
 {  
  static unsigned int local_timeMS;   
  ErrorStatus ret=ERROR;
  if(laser_ctr_param.timerEnableFlag!=0&&ctrTimeS!=0&&TimeMs!=0)
  {
    local_timeMS += TimeMs;        
    if(local_timeMS>=ctrTimeS*1000)
    {      
      ret=SUCCESS;
      local_timeMS=ctrTimeS*1000;   
      if(sGenSta.laser_run_B5_timer_status==0)  
      {
        uint16_t send_music_num = MUSIC_SHORT_PROMT;
        osMessageQueuePut(musicQueue03Handle,&send_music_num,0,0);
        sGenSta.laser_run_B5_timer_status=1;
      } 
    }
  }
  else 
  {
   if(local_timeMS!=0) 
   {
      local_timeMS=0;
      sGenSta.laser_run_B5_timer_status = 0;
   } 
  }  
  return ret;
 }
/************************************************************************//**
  * @brief 水雾准备
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 void app_treatment_water_prepare(unsigned char *ctrflag,unsigned int runtimeMs)
 {
  static unsigned char local_tmc_flag; 
  static unsigned int timeout;    
  if(*ctrflag!=0)
  {
    if(local_tmc_flag==0)
    {
      tmc2226_start(1,3);
      timeout = 0;
      local_tmc_flag = 1;
    }
    else 
    {
      timeout+=runtimeMs; 
      if(timeout>20*SYS_1_MINUTES_TICK) //max20minutes 
      {
        *ctrflag = 0;   
        tmc2226_stop();
        local_tmc_flag = 0;
        timeout = 0;        
      } 
    }
  }
  else 
  {
    if(local_tmc_flag!=0)
    { 
      local_tmc_flag = 0;
      tmc2226_stop();
    }
    timeout=0;
  }
}
  /************************************************************************//**
  * @brief 消毒、清洗指令
  * @param 无
  * @note   
  * @retval 
  *****************************************************************************/
 void app_t_clean_run_timer(unsigned char *runflag)
 {
    static unsigned int runtimeS;
    static unsigned char local_flag;  
    if(local_flag==0)
    {
      if(*runflag!=0)
      {
        if(*runflag ==1) runtimeS = u_sys_param.sys_config_param.dit_time_min*60;
        else if(*runflag ==2) runtimeS = u_sys_param.sys_config_param.clean_time_min*60;
        osEventFlagsClear(auxStatusEvent01Handle,EVENTS_AUX_STATUS_16_CLEAN_BIT);
        if((sGenSta.genaration_io_status&EVENTS_AUX_STATUS_IO7_BIT)==0)  tmc2226_start(1,3);        
      }     
      else runtimeS=0;
      local_flag=*runflag; 
    }
    else //if(local_flag!=0) 
    {      
      if(runtimeS!=0)  runtimeS--; 
      if(runtimeS==0||*runflag==0||(sGenSta.genaration_io_status&EVENTS_AUX_STATUS_IO7_BIT)!=0)
      {
        if((sGenSta.genaration_io_status&EVENTS_AUX_STATUS_16_CLEAN_BIT)==0)
        {
          tmc2226_stop();  
          osEventFlagsSet(auxStatusEvent01Handle,EVENTS_AUX_STATUS_16_CLEAN_BIT);
          *runflag = 0;
        }
      }     
      local_flag = *runflag;  
    }    
 }
/***************************************************************************//**
 * @brief 数据包检查
 * @param 
 * @note  最小包长8字节
 * @return 
*******************************************************************************/
unsigned short int app_hmi_package_check(unsigned char* pBuff,unsigned short int buffLen) 
{
	unsigned short int retLen=0,i=0,pLen,crc;  
	while(i<buffLen)
	{   
		if(pBuff[i]==0x7E&&pBuff[i+1]==0x7E)
		{
			pLen=pBuff[i+2];       
      if(i+pLen>buffLen) 
      {
        retLen=i;
        break;
      } 
      else    
      { 
        //if(pBuff[i+pLen-2]==0x0D&&pBuff[i+pLen-1]==0x0A)
        crc=pBuff[pLen+i-4]|pBuff[pLen+i-3]<<8;      
        if(crc==crc16Num(pBuff+i,pLen-4))
        {
          HMI_Parse_Data(&pBuff[i], pLen);
          retLen = pLen+i; 
          i+=pLen;       
        } 
      } 
    }
		i++; 
	}
  if(i==buffLen) retLen=i;
	return retLen;
}
/************************************************************************//**
* @brief 蜂鸣器PWM
  * @param delayPai节拍
  * @note  
  * @retval  
  *****************************************************************************/
 void app_beep_pai_tim(unsigned char delayPai)
 {
  //1s一个音符，0.25秒一个节拍
  unsigned  int dp;
  dp = (delayPai-30)*75;//125 ; 
  osDelay(dp);
 }
/************************************************************************//**
* @brief 蜂鸣器music
  * @param volnum 音量0~100；
  * @note  
  * @retval  
  *****************************************************************************/
 void app_buzz_music(music_type  music_num,unsigned char volume)
 {
   unsigned int j, vol;  
   vol=1;//1低，2，中，3高
   if(music_num==MUSIC_SHORT_PROMT)
   {
      app_beep_pwm(music_tab_c[1],50);
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
      osDelay(300);     
      app_beep_pwm(0,0);
      osDelay(300);    
   }
   else if(music_num==MUSIC_LONG_PROMT)
   {
      app_beep_pwm(music_tab_c[16],50);
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1); 
      osDelay(1500);     
      app_beep_pwm(0,0);   
      osDelay(500); 
   }    
   else if(music_num==MUSIC_TWO_TIGER)
   {
      vol=1;
      app_beep_pwm(50,50);
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1); 
      for(j=0;j<32;j++)     
      {     
        app_beep_pwm(music_tab_c[music_two_tiger[j*2]*vol],50);
        app_beep_pai_tim(music_two_tiger[2*j+1]);	
      } 
      app_beep_pwm(0,50);   
   }
   else if(music_num==MUSIC_STAR)
   {   
      vol=1;
      app_beep_pwm(50,50);
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
      for(j=0;j<42;j++)     
      { 
        app_beep_pwm(music_tab_c[music_star[j*2]*vol],50);
        app_beep_pai_tim(music_star[2*j+1]);	
      }  
      app_beep_pwm(0,0);      
   }
   else if(music_num==MUSIC_HAPPY)
   {
      vol=2;      
      app_beep_pwm(50,50);
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);
      for(j=0;j<48;j++)     
      { 
        app_beep_pwm(music_tab_c[music_happy[2*j]*vol],50);
        app_beep_pai_tim(music_happy[2*j+1]);	
      } 
      app_beep_pwm(0,0);       
   }
   else if(music_num==MUSIC_SYS_ON)
   {  
      HAL_TIMEx_PWMN_Start(&htim15,TIM_CHANNEL_1);   
      app_beep_pwm(music_tab_c[6],50);
      osDelay(300);
      app_beep_pwm(music_tab_c[16],50);
      osDelay(300);
      app_beep_pwm(music_tab_c[21],50);
      osDelay(400);     
      app_beep_pwm(0,0);
   }
   else app_beep_pwm(0,0);  
 }
  /************************************************************************//**
  * @brief 激光能量反馈计算
  * @param 
  * @note  0.75mA/mW
  * @retval  能量值
  *****************************************************************************/
 unsigned short int app_energe_cali( unsigned int adVoltage )
 {
    static unsigned short int e_value;
    sEnvParam.laser_1064_energy = e_value;
    return e_value;
 }
  /***************************************************************************//**
 * @brief app_jdq_restart
 * @param 
 * @note 
 * @return 
*******************************************************************************/
 void app_jdq_restart(void)
 {	  	
	 jdq_reley_charge_ready(0);//负载断开
	 jdq_reley_charge(0);//
	 osDelay(500);		
	 jdq_reley_charge(1);//	
   osDelay(1000);		 	
 }
 #ifdef JDQ_PWR_GWB_3200W
 /***************************************************************************//**
 * @brief 状态管理
 * @param 
 * @note 
 * @return 
*******************************************************************************/
void app_jdq_gwb3200_status_manage_handle(unsigned  int timeMs)
{
	static unsigned  int heatTime;
  heatTime+=	timeMs;
	if(heatTime>SYS_1_MINUTES_TICK )	
	{
		app_jdq_restart();
		heatTime=0;
	}
	else
	{
		app_jdq_bus_get_v_c_req();
	}
}
#endif
/* USER CODE END Application */

